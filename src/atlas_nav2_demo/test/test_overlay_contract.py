import tempfile
import unittest
from pathlib import Path

from atlas_nav2_demo.overlay_contract import is_development_package_prefix


class DevelopmentOverlayContractTest(unittest.TestCase):
    def test_explicit_overlay_must_own_resolved_package(self):
        with tempfile.TemporaryDirectory() as temp:
            install = Path(temp) / 'install'
            prefix = install / 'atlas_nav2_demo'
            self.assertTrue(is_development_package_prefix(
                prefix, {'ATLAS_NAV2_OVERLAY': str(install)}))
            self.assertFalse(is_development_package_prefix(
                Path(temp) / 'underlay' / 'atlas_nav2_demo',
                {'ATLAS_NAV2_OVERLAY': str(install)}))

    def test_first_colcon_prefix_is_the_development_overlay(self):
        with tempfile.TemporaryDirectory() as temp:
            install = Path(temp) / 'install'
            underlay = Path(temp) / 'underlay'
            package = install / 'atlas_nav2_demo'
            env = {'COLCON_PREFIX_PATH': f'{install}:{underlay}'}
            self.assertTrue(is_development_package_prefix(package, env))
            self.assertFalse(is_development_package_prefix(
                underlay / 'atlas_nav2_demo', env))

    def test_missing_overlay_context_fails_closed(self):
        self.assertFalse(is_development_package_prefix(
            '/opt/ros/humble/share/atlas_nav2_demo', {}))


if __name__ == '__main__':
    unittest.main()
