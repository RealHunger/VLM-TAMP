from __future__ import print_function

import unittest

from cogarch_tools.processes.pddlstream_agent import correct_home_path


class TestCorrectHomePath(unittest.TestCase):

    def test_uses_current_exp_dir_when_legacy_marker_is_absent(self):
        loaded_exp_dir = '/tmp/workspace/experiments/old_run'
        current_exp_dir = '/tmp/workspace/experiments/new_run'

        self.assertEqual(current_exp_dir, correct_home_path(loaded_exp_dir, current_exp_dir))


if __name__ == '__main__':
    unittest.main()
