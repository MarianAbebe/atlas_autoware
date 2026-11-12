"""Helpers for ensuring tests resolve their freshly built development overlay."""
import os
from pathlib import Path


def is_development_package_prefix(package_prefix, env=None):
    """Return whether *package_prefix* belongs to the selected overlay.

    ``ATLAS_NAV2_OVERLAY`` explicitly selects a colcon install prefix. Otherwise,
    the first ``COLCON_PREFIX_PATH`` entry is used, matching normal ROS overlay
    sourcing order. This avoids machine-specific workspace paths and rejects a
    package resolved only from the underlay.
    """
    env = os.environ if env is None else env
    package_prefix = Path(package_prefix).expanduser().resolve()
    explicit = env.get('ATLAS_NAV2_OVERLAY')
    if explicit:
        return package_prefix.parent == Path(explicit).expanduser().resolve()
    prefixes = [item for item in env.get('COLCON_PREFIX_PATH', '').split(os.pathsep)
                if item]
    return bool(prefixes and
                package_prefix.parent == Path(prefixes[0]).expanduser().resolve())
