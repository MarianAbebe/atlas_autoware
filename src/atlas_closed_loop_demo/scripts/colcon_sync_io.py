#!/usr/bin/env python3
"""Local colcon runner for environments where asyncio pipe EOF stalls.
Uses standard blocking subprocess communication on the main thread; package
sources, compiler arguments, dependency ordering and installed colcon files remain unchanged.
Output callbacks run on the event loop after each subprocess completes.
"""
import subprocess
import colcon_core.subprocess as io

async def sync_io(args, stdout_callback, stderr_callback, *, use_pty=None, **kwargs):
    if kwargs.get('shell', False):
        args = ' '.join(io.escape_shell_argument(a) for a in args)
    result = subprocess.run(args, stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE, **kwargs)
    for callback, data in [(stdout_callback, result.stdout), (stderr_callback, result.stderr)]:
        if callable(callback):
            for line in data.splitlines(keepends=True): callback(line)
    return result.returncode, result.stdout, result.stderr

io._async_check_call = sync_io
if __name__ == '__main__':
    from colcon_core.command import main
    raise SystemExit(main())
