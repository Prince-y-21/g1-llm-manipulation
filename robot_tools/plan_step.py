"""Small Cartesian step using live joint feedback; sends only with --send."""
import argparse
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

p = argparse.ArgumentParser()
p.add_argument("dx", type=float)
p.add_argument("dy", type=float)
p.add_argument("dz", type=float)
p.add_argument("--duration", type=float, default=8)
p.add_argument("--send", action="store_true")
p.add_argument("--allow-fault-plan", action="store_true", help="Offline recovery planning only; cannot send")
args = p.parse_args()
work = Path("/tmp/codex-g1-observe")
status = json.loads((work / "guard_status.json").read_text())
assert not status["moving"]
assert not status["fault"] or (args.allow_fault_plan and not args.send)
q0 = np.array(status["q"][22:29])
bias = np.array(status["desired"][22:29]) - q0
root = ET.parse(work / "g1_29dof_rev_1_0.urdf").getroot()
names = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw"]
origins, axes, lower, upper = [], [], [], []


def mat(rot):
    return rot.as_matrix() if hasattr(rot, "as_matrix") else rot.as_dcm()


def rotation(m):
    return Rotation.from_matrix(m) if hasattr(Rotation, "from_matrix") else Rotation.from_dcm(m)


for name in names:
    j = root.find(f"joint[@name='right_{name}_joint']")
    o = j.find("origin")
    t = np.eye(4)
    t[:3, :3] = mat(Rotation.from_euler("xyz", np.fromstring(o.get("rpy", "0 0 0"), sep=" ")))
    t[:3, 3] = np.fromstring(o.get("xyz"), sep=" ")
    origins.append(t)
    axes.append(np.fromstring(j.find("axis").get("xyz"), sep=" "))
    lower.append(float(j.find("limit").get("lower")))
    upper.append(float(j.find("limit").get("upper")))


def fk(q):
    t = np.eye(4)
    for o, a, v in zip(origins, axes, q):
        r = np.eye(4)
        r[:3, :3] = mat(Rotation.from_rotvec(a * v))
        t = t @ o @ r
    return t


start = fk(q0)
target = start.copy()
target[:3, 3] += [args.dx, args.dy, args.dz]


def residual(q):
    pose = fk(q)
    return np.r_[(pose[:3, 3] - target[:3, 3]) * 100,
                 rotation(target[:3, :3].T @ pose[:3, :3]).as_rotvec() * 10,
                 (q - q0) * .02]


solution = least_squares(residual, q0, bounds=(lower, upper), xtol=1e-12, ftol=1e-12, gtol=1e-12)
q_target = solution.x + bias
delta = q_target - np.array(status["desired"][22:29])
error = np.linalg.norm(fk(solution.x)[:3, 3] - target[:3, 3])
assert error < .0001
assert np.max(np.abs(delta)) <= .12
assert np.max(np.abs(delta)) * 1.5 / args.duration <= .025
plan = dict(seq=status["seq"] + 1, requested_translation_m=[args.dx, args.dy, args.dz],
            observed_q=q0.tolist(), desired_q=q_target.tolist(), delta_q=delta.tolist(),
            start_wrist_xyz=start[:3, 3].tolist(), target_wrist_xyz=target[:3, 3].tolist(),
            solver_error_m=float(error), duration_s=args.duration)
print(json.dumps(plan, indent=2))
(work / f"plan_{plan['seq']}.json").write_text(json.dumps(plan, indent=2))
if args.send:
    # Re-check that the controller has not changed state while solving.
    latest = json.loads((work / "guard_status.json").read_text())
    assert latest["seq"] == status["seq"] and not latest["fault"] and not latest["moving"]
    (work / "arm_target.tmp").write_text(f"{plan['seq']} {args.duration} " + " ".join(map(str, q_target)) + "\n")
    os.replace(work / "arm_target.tmp", work / "arm_target.txt")
    print("SENT", plan["seq"])
