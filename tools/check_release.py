"""Offline-only release validation. Does not import or run robot programs."""
import ast
import hashlib
import json
from pathlib import Path
import re
import sys
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[1]
errors = []
counts = {"files": 0, "python": 0, "json": 0, "markdown_links": 0}
secret = re.compile(r"-----BEGIN [A-Z ]*PRIVATE KEY-----|ghp_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}|sk-(?:proj-)?[A-Za-z0-9_-]{25,}")
for path in ROOT.rglob("*"):
    if not path.is_file() or ".git" in path.relative_to(ROOT).parts or "__pycache__" in path.parts:
        continue
    counts["files"] += 1
    if path.suffix.lower() in {".mp4", ".jpg", ".png"}:
        continue
    relative = path.relative_to(ROOT).as_posix()
    try:
        text = path.read_text(encoding="utf-8-sig")
        if secret.search(text):
            errors.append(f"Credential-shaped content: {relative}")
        if path.suffix == ".py":
            ast.parse(text, filename=relative)
            counts["python"] += 1
        if path.suffix == ".json":
            json.loads(text)
            counts["json"] += 1
        if path.suffix == ".md":
            prose = re.sub(r"```.*?```", "", text, flags=re.S)
            prose = re.sub(r"`[^`\n]+`", "", prose)
            for match in re.finditer(r"\]\(([^)]+)\)", prose):
                target = match[1].strip().strip("<>").split("#", 1)[0]
                if not target or ":" in target or target.startswith("/"):
                    continue
                counts["markdown_links"] += 1
                if not (path.parent / unquote(target)).exists():
                    errors.append(f"Missing link in {relative}: {target}")
    except (UnicodeError, ValueError, SyntaxError) as error:
        errors.append(f"{relative}: {error}")

manifest = ROOT / "release_manifest.json"
if manifest.exists():
    for entry in json.loads(manifest.read_text(encoding="utf-8"))["files"]:
        path = ROOT / entry["path"]
        if not path.is_file():
            errors.append(f"Manifest missing file: {entry['path']}")
            continue
        with path.open("rb") as stream:
            digest = hashlib.file_digest(stream, "sha256").hexdigest()
        if digest != entry["sha256"]:
            errors.append(f"Manifest hash mismatch: {entry['path']}")
print(json.dumps({"scope": "offline_only_no_imports_no_hardware", "counts": counts,
                  "errors": errors, "passed": not errors}, ensure_ascii=False, indent=2))
sys.exit(bool(errors))
