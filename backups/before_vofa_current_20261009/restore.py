#!/usr/bin/env python3
"""Restore only this experiment's files; refuse to overwrite later edits."""
import argparse
import hashlib
import json
import shutil
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--check', action='store_true', help='Verify rollback without writing files')
args = parser.parse_args()
backup = Path(__file__).resolve().parent
root = backup.parent.parent
before = json.loads((backup / 'before_sha256.json').read_text())
after = json.loads((backup / 'after_sha256.json').read_text())
def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None

for name, expected in before.items():
    if digest(backup / 'files' / name) != expected:
        raise SystemExit(f'Backup checksum mismatch: {name}; nothing restored.')
    if digest(root / name) not in (expected, after[name]):
        raise SystemExit(f'Later edits detected: {name}; nothing restored. Request a selective rollback.')
if args.check:
    print('Rollback check passed; no files changed.')
else:
    for name, expected in before.items():
        shutil.copy2(backup / 'files' / name, root / name)
        assert digest(root / name) == expected
    print('Restored the five saved files byte-for-byte. Other files were not touched.')
    print('Run pio run, then upload the rebuilt firmware. The device is NOT restored by this script.')
