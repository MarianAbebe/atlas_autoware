#!/usr/bin/env python3
"""Summarize actual cache contents; never infer runtime success from a build."""
import json
from pathlib import Path

root = Path('/home/marian/roam/atlas_autoware')
evidence = root/'src/atlas_closed_loop_demo/results/current_audit'
closure = Path('/tmp/atlas_closed_loop_closure.tsv')
rows = [r.split('\t') for r in closure.read_text().splitlines()] if closure.exists() else []
cache = root/'.closed_loop_cache'
installed = [n for n, *_ in rows if (cache/'install'/n/'share'/n/'package.xml').exists()]
missing = [n for n, *_ in rows if n not in installed]
demo = [n for n in ['atlas_map_integration_demo','atlas_closed_loop_demo']
        if (cache/'demo_install'/n/'share'/n/'package.xml').exists()]
out = {'closure_packages': len(rows), 'installed_packages': installed,
       'not_installed_packages': missing, 'installed_demo_packages': demo,
       'build_complete': bool(rows and not missing and len(demo) == 2),
       'runtime_verified_this_session': False, 'system_installations': [],
       'cache_path': str(cache),
       'note': 'Installed manifests alone do not verify shared-library loading or runtime behavior.'}
evidence.mkdir(parents=True, exist_ok=True)
(evidence/'build_status.json').write_text(json.dumps(out, indent=2) + '\n')
print(json.dumps({'installed': len(installed), 'closure': len(rows), 'demo': demo,
                  'build_complete': out['build_complete']}))
