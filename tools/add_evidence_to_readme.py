#!/usr/bin/env python3
"""Add an evidence section to repo README without overwriting existing text."""
from pathlib import Path
root=Path(__file__).resolve().parent.parent
p=root/'README.md'
insert=(root/'tools'/'README_EVIDENCE_INSERT.md').read_text()
if not p.exists(): raise SystemExit('Run inside your AMR-BIP repo; README.md missing')
s=p.read_text()
needle='## Verified Gazebo Behavior Tree evidence (18 October 2026)'
if needle in s:
    print('README evidence section already present; no change')
else:
    i=s.find('\n## ')
    if i==-1: s=s.rstrip()+'\n\n'+insert
    else: s=s[:i]+'\n'+insert+s[i:]
    p.write_text(s)
    print('README evidence section inserted')
