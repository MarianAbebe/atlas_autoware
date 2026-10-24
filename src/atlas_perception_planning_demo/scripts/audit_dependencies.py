#!/usr/bin/env python3
"""Audit complete production closure; optionally build only uncached source packages."""
import argparse
import json
import os
from pathlib import Path
import subprocess

from catkin_pkg.package import parse_package


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--build', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[3]
    ref = Path(os.environ.get('ATLAS_AUTOWARE_ROOT', root.parent / 'references/autoware'))
    output = root / 'src/atlas_perception_planning_demo/results'
    output.mkdir(parents=True, exist_ok=True)
    pins = {'.': '10718787ba6e28f038a0cb29ff99cc627b5abfd2',
            'src/core/autoware_core': 'f25f83c632c1984ec276c894c41857d4abc0dad8',
            'src/universe/autoware_universe': '6e477c645efec33f7909095eea684474e97f5e3d',
            'src/core/autoware_utils': '1a0ebbc8cb955e49ec28163a3a7953dc7fa4c312',
            'src/universe/external/managed_transform_buffer': 'b12c9c11bccdb0ea24ac3665c91f0933ba43752f',
            'src/core/autoware_simple_planning_simulator': '82d2c93a28a25f8802e8f5a21b4ec850beea9080'}
    audit = {}
    for relative, expected in pins.items():
        repo = ref / relative
        head = subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip()
        status = subprocess.check_output(['git', '-C', str(repo), 'status', '--porcelain'], text=True)
        audit[relative] = {'head': head, 'status': status}
        if head != expected or status:
            raise SystemExit(f'Unexpected pinned source state: {repo}')
    (output / 'pinned_sources.json').write_text(json.dumps(audit, indent=2))
    bases = [str(ref / 'src' / p) for p in ('core', 'universe', 'external')]
    rows = [r.split('\t') for r in subprocess.check_output(['colcon', 'list', '--base-paths', *bases], text=True).splitlines()]
    byname = {r[0]: r for r in rows}
    needed = set()

    def visit(name):
        if name in needed or name not in byname:
            return
        needed.add(name)
        pkg = parse_package(Path(byname[name][1]) / 'package.xml')
        pkg.evaluate_conditions(dict(os.environ))
        for tag in ('build_depends', 'buildtool_depends', 'build_export_depends', 'buildtool_export_depends', 'exec_depends'):
            for edge in getattr(pkg, tag):
                if edge.evaluated_condition:
                    visit(edge.name)

    for name in ('autoware_motion_velocity_planner', 'autoware_motion_velocity_obstacle_stop_module', 'autoware_ground_filter'):
        visit(name)
    rows = [r for r in rows if r[0] in needed]
    prefixes = os.environ.get('AMENT_PREFIX_PATH', '').split(':')
    missing = [r[0] for r in rows if not any(Path(p, 'share', r[0], 'package.xml').exists() for p in prefixes)]
    (output / 'closure.tsv').write_text('\n'.join('\t'.join(r) for r in rows)+'\n')
    (output / 'selection.json').write_text(json.dumps({'missing_source_builds': missing, 'total_closure': len(rows)}, indent=2))
    cmd = ['rosdep', 'check', '--rosdistro', 'humble', '--ignore-src']
    cmd += [f'--dependency-types={t}' for t in ('build', 'buildtool', 'build_export', 'buildtool_export', 'exec')]
    cmd += ['--from-paths', *[r[1] for r in rows], str(root/'src/atlas_perception_planning_demo')]
    # Local Atlas package is already built and sourced; rosdep --ignore-src must
    # also see its manifest without traversing any generated build trees.
    cmd += [str(root/'src/atlas_closed_loop_demo'), str(root/'src/atlas_map_integration_demo'), str(root/'src/atlas_obstacle_stopping_demo'), str(root/'src/atlas_sensor_kit'), str(root/'src/atlas_distortion_corrector_test')]
    result = subprocess.run(cmd, text=True, capture_output=True)
    (output/'dependency_audit.log').write_text(result.stdout+result.stderr)
    print(json.dumps({'closure': len(rows), 'missing_source_builds': missing}), flush=True)
    print(result.stdout+result.stderr, flush=True)
    if result.returncode:
        raise SystemExit('Dependencies missing. No installation attempted; obtain approval before installation.')
    if args.build and missing:
        cache = root/'.perception_cache'
        cache.mkdir(exist_ok=True)
        (cache/'.gitignore').write_text('*\n')
        subprocess.run(['python3', str(root/'src/atlas_closed_loop_demo/scripts/colcon_sync_io.py'),
                        '--log-base', str(cache/'log'), 'build', '--base-paths', *bases,
                        '--packages-select', *missing, '--packages-ignore', *sorted(set(byname)-needed),
                        '--executor', 'sequential', '--build-base', str(cache/'build'),
                        '--install-base', str(cache/'install'), '--event-handlers', 'console_direct+',
                        '--cmake-args', '-DBUILD_TESTING=OFF', '-DCMAKE_BUILD_TYPE=Release',
                        '-DCMAKE_SHARED_LINKER_FLAGS=-fuse-ld=gold', '-DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=gold'],
                       check=True, cwd=root)


if __name__ == '__main__':
    main()
