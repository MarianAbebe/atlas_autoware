#!/usr/bin/env python3
"""Use blocking subprocess I/O where this VM's asyncio pipe EOF can stall."""
import subprocess

import colcon_core.subprocess as colcon_io


async def sync_io(args, stdout_callback, stderr_callback, *, use_pty=None, **kwargs):
    if kwargs.get('shell', False):
        args = ' '.join(colcon_io.escape_shell_argument(value) for value in args)
    result = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, **kwargs)
    for callback, data in ((stdout_callback, result.stdout), (stderr_callback, result.stderr)):
        if callable(callback):
            for line in data.splitlines(keepends=True):
                callback(line)
    return result.returncode, result.stdout, result.stderr


colcon_io._async_check_call = sync_io

if __name__ == '__main__':
    from colcon_core.command import main
    raise SystemExit(main())
