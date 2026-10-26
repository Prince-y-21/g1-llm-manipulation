"""Offline kinematic check. This script has no robot command interface."""
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def matrix(rotation):
    return rotation.as_matrix() if hasattr(rotation, "as_matrix") else rotation.as_dcm()


def rotation_from_matrix(value):
    return Rotation.from_matrix(value) if hasattr(Rotation, "from_matrix") else Rotation.from_dcm(value)


def transform(xyz, rpy):
    result = np.eye(4)
    result[:3, :3] = matrix(Rotation.from_euler("xyz", rpy))
    result[:3, 3] = xyz
    return result


model = ET.parse(sys.argv[1]).getroot()
names = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
         "wrist_roll", "wrist_pitch", "wrist_yaw"]
joints = [model.find(f"joint[@name='right_{name}_joint']") for name in names]
origins, axes, lower, upper = [], [], [], []
for joint in joints:
    origin = joint.find("origin")
    origins.append(transform(np.fromstring(origin.get("xyz"), sep=" "),
                             np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")))
    axes.append(np.fromstring(joint.find("axis").get("xyz"), sep=" "))
    lower.append(float(joint.find("limit").get("lower")))
    upper.append(float(joint.find("limit").get("upper")))


def forward(q):
    result = np.eye(4)
    for origin, axis, angle in zip(origins, axes, q):
        turn = np.eye(4)
        turn[:3, :3] = matrix(Rotation.from_rotvec(axis * angle))
        result = result @ origin @ turn
    return result


# Observation at 2026-10-16 19:11 Asia/Shanghai; planning reference only.
q0 = np.array([.0310391, -.000778975, -.000431432, .0384813,
               .00287621, .0119962, .000455401])
start = forward(q0)
target = start.copy()
target[:3, 3] += [.01, 0, .015]


def residual(q):
    pose = forward(q)
    position = (pose[:3, 3] - target[:3, 3]) * 100
    orientation = rotation_from_matrix(target[:3, :3].T @ pose[:3, :3]).as_rotvec() * 10
    return np.r_[position, orientation, (q - q0) * .005]


solution = least_squares(residual, q0, bounds=(lower, upper),
                         xtol=1e-12, ftol=1e-12, gtol=1e-12)
end = forward(solution.x)
result = {
    "status": "OFFLINE_ONLY_UNVALIDATED_ON_HARDWARE",
    "frame": "torso_link", "joint_indices": list(range(22, 29)),
    "reference_q_rad": q0.tolist(),
    "reference_wrist_xyz_m": start[:3, 3].tolist(),
    "requested_translation_m": [.01, 0, .015],
    "candidate_q_rad": solution.x.tolist(),
    "delta_q_rad": (solution.x - q0).tolist(),
    "position_error_m": float(np.linalg.norm(end[:3, 3] - target[:3, 3])),
    "orientation_error_rad": float(np.linalg.norm(rotation_from_matrix(
        target[:3, :3].T @ end[:3, :3]).as_rotvec())),
    "limitations": ["Camera mounting has not been verified against this model.",
                    "Dex1 mounting and collision geometry are not included.",
                    "Live joint state must replace the planning reference."]
}
print(json.dumps(result, indent=2))
Path(sys.argv[2]).write_text(json.dumps(result, indent=2))
