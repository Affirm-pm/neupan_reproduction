#!/usr/bin/env python3
"""Create a relocatable FineNav source archive; Python standard library only."""
import argparse
import hashlib
from pathlib import Path
import tarfile

ROOT = Path(__file__).resolve().parents[1]
SOURCES = (
    'README.md', '.gitignore', 'tools',
    'finenav_ws/src',
    'finenav_ws/neupan_dependencies/src',
    'finenav_ws/neupan_dependencies/COLCON_IGNORE',
    'finenav_ws/neupan_dependencies/README.md',
    'neupan_cpp_ws/src',
)
EXCLUDED = {'.git', '.agents', '.aws', '.codex', '.idea', '.vscode', '__pycache__', 'build', 'install', 'log', 'logs'}


def filter_entry(entry):
    if any(part in EXCLUDED for part in Path(entry.name).parts):
        return None
    if entry.name.endswith(('.pyc', '.pyo')):
        return None
    # Source-only bundles must not silently point back to a developer's checkout.
    if entry.issym() or entry.islnk():
        raise ValueError(f'Source link requires explicit review before packaging: {entry.name}')
    entry.uid = entry.gid = 0
    entry.uname = entry.gname = ''
    return entry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'dist/FineNav-source.tar.gz')
    args = parser.parse_args()
    output = args.output.expanduser().resolve()
    if output.exists():
        parser.error(f'Output already exists: {output}; select another --output path')
    for name in SOURCES:
        source = ROOT/name
        if not source.exists():
            parser.error(f'Missing required source: {source}')
        if output == source or source in output.parents:
            parser.error('Output must not be inside one of the packaged source directories')
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(output.name+'.partial')
    try:
        with tarfile.open(temporary, 'w:gz', dereference=False) as archive:
            for name in SOURCES:
                archive.add(ROOT/name, arcname='FineNav/'+name, filter=filter_entry)
        temporary.replace(output)
    finally:
        temporary.unlink(missing_ok=True)
    digest = hashlib.sha256()
    with output.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    checksum = output.with_name(output.name+'.sha256')
    checksum.write_text(f'{digest.hexdigest()}  {output.name}\n')
    print(f'Archive: {output}\nSize: {output.stat().st_size / 1024**2:.1f} MiB\nSHA256: {checksum}')


if __name__ == '__main__':
    main()
