#!/usr/bin/env python3
"""Verify or restore this immutable source snapshot; never flash hardware."""
from pathlib import Path
import argparse, datetime, hashlib, json, shutil

EXCLUDED = {'.git', '.pio', '.pioenvs', '.piolibdeps', 'backups', '__pycache__'}
def scan(root):
    return {p.relative_to(root).as_posix(): p for p in root.rglob('*')
            if p.is_file() and not any(x in EXCLUDED for x in p.relative_to(root).parts)
            and p.suffix != '.pyc'}
def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()
def compare(root, manifest):
    current = scan(root)
    changed = [r for r, h in manifest.items() if r in current and digest(current[r]) != h]
    missing = sorted(set(manifest) - set(current))
    extra = sorted(set(current) - set(manifest))
    return current, changed, missing, extra

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--check', action='store_true')
    group.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    base = Path(__file__).resolve().parent
    root = base.parent.parent
    if base.parent.name != 'backups' or not (root / 'platformio.ini').exists():
        raise SystemExit('Refusing: script must remain in PROJECT/backups/BASELINE/')
    manifest = json.loads((base / 'manifest.json').read_text())
    for r, h in manifest.items():
        rel = Path(r)
        if rel.is_absolute() or '..' in rel.parts:
            raise SystemExit('Invalid manifest path')
        p = base / 'files' / r
        if not p.is_file() or digest(p) != h:
            raise SystemExit('Snapshot integrity failure: ' + r)
    current, changed, missing, extra = compare(root, manifest)
    for title, entries in [('CHANGED', changed), ('MISSING', missing), ('EXTRA', extra)]:
        print(title + ': ' + str(len(entries)))
        for r in entries: print('  ' + r)
    print('Snapshot integrity: OK (' + str(len(manifest)) + ' files)')
    if not args.apply or not (changed or missing or extra):
        return
    # Refuse symlinks before any write, including symlinked parent directories.
    for r in set(current) | set(manifest):
        p = root / r
        if any(q.is_symlink() for q in [p, *p.parents] if q != root.parent):
            raise SystemExit('Refusing symlink path: ' + r)
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    recovery = root / 'backups' / ('pre_restore_' + stamp)
    recovery.mkdir(parents=True, exist_ok=False)
    hashes = {}
    for r, p in current.items():
        dst = recovery / 'files' / r
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(p, dst)
        hashes[r] = digest(dst)
        if hashes[r] != digest(p):
            raise SystemExit('File changed during recovery backup: ' + r)
    (recovery / 'manifest.json').write_text(json.dumps(hashes, indent=2) + '\n')
    shutil.copy2(base / 'restore.py', recovery / 'restore.py')
    print('Recovery snapshot: ' + str(recovery), flush=True)
    for r in extra:
        (root / r).unlink()
    for r in manifest:
        dst = root / r
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(base / 'files' / r, dst)
    _, changed, missing, extra = compare(root, manifest)
    if changed or missing or extra:
        raise SystemExit('Restore verification failed; recovery retained at ' + str(recovery))
    print('Restore verified: all source files match baseline. Rebuild before flashing.')

if __name__ == '__main__':
    main()
