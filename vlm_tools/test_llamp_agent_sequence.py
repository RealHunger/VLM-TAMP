import unittest
from unittest.mock import patch

import vlm_tools.llamp_agent as llamp_agent_module
from vlm_tools.llamp_agent import LLAMPAgent, get_progress_summary_from_time_log


class TestLLAMPAgentSequence(unittest.TestCase):

    def test_replan_returns_failed_plan_when_goal_sequence_is_empty(self):
        agent = LLAMPAgent.__new__(LLAMPAgent)
        agent.llamp_api = object()
        agent.goal_sequence = []
        agent.plan = ['stale-plan']

        result = agent.replan(observation=None)

        self.assertIsNone(result)
        self.assertIsNone(agent.plan)

    def test_update_obstacles_rebinds_inverse_kinematics_to_reduced_world(self):
        class FakeState:
            def __init__(self, world):
                self.world = world
                self.fixed = [42]

        def make_gen(*args, **kwargs):
            def gen(*_args, **_kwargs):
                if False:
                    yield None
            return gen

        def make_fn(*args, **kwargs):
            return lambda *_args, **_kwargs: None

        agent = LLAMPAgent.__new__(LLAMPAgent)
        agent.custom_limits = {}
        stream_map = {
            'inverse-reachability': object(),
            'inverse-reachability-rel': object(),
            'inverse-kinematics': object(),
            'inverse-kinematics-pull': object(),
            'inverse-kinematics-pull-with-link': object(),
        }
        old_ik_stream = stream_map['inverse-kinematics']

        with patch.object(llamp_agent_module, 'State', FakeState), \
                patch('pybullet_tools.mobile_streams.get_ik_gen_old', side_effect=make_gen), \
                patch('pybullet_tools.mobile_streams.get_ik_rel_gen_old', side_effect=make_gen), \
                patch('pybullet_tools.mobile_streams.get_ik_pull_gen', side_effect=make_gen), \
                patch('pybullet_tools.mobile_streams.get_ik_pull_with_link_gen', side_effect=make_gen), \
                patch('pybullet_tools.mobile_streams.get_ik_rel_fn_old', side_effect=make_fn), \
                patch('pybullet_tools.mobile_streams.get_ik_fn_old', side_effect=make_fn) as get_ik_fn_old:
            updated = agent._update_obstacles_in_stream_map(stream_map, world=object())

        self.assertIsNot(updated['inverse-kinematics'], old_ik_stream)
        get_ik_fn_old.assert_called_once()

    def test_update_obstacles_preserves_configured_ir_settings(self):
        class FakeState:
            def __init__(self, world):
                self.world = world
                self.fixed = [42]

        class FakeWorld:
            stream_kwargs = {'use_learned_ir': False, 'ir_max_attempts': 10}

        def make_gen(*args, **kwargs):
            def gen(*_args, **_kwargs):
                if False:
                    yield None
            return gen

        def make_fn(*args, **kwargs):
            return lambda *_args, **_kwargs: None

        agent = LLAMPAgent.__new__(LLAMPAgent)
        agent.custom_limits = {}
        stream_map = {}

        with patch.object(llamp_agent_module, 'State', FakeState), \
                patch('pybullet_tools.mobile_streams.get_ik_gen_old', side_effect=make_gen) as get_ik_gen_old, \
                patch('pybullet_tools.mobile_streams.get_ik_rel_gen_old', side_effect=make_gen) as get_ik_rel_gen_old, \
                patch('pybullet_tools.mobile_streams.get_ik_pull_gen', side_effect=make_gen), \
                patch('pybullet_tools.mobile_streams.get_ik_pull_with_link_gen', side_effect=make_gen), \
                patch('pybullet_tools.mobile_streams.get_ik_rel_fn_old', side_effect=make_fn), \
                patch('pybullet_tools.mobile_streams.get_ik_fn_old', side_effect=make_fn):
            agent._update_obstacles_in_stream_map(stream_map, world=FakeWorld())

        self.assertFalse(get_ik_gen_old.call_args.kwargs['learned'])
        self.assertEqual(10, get_ik_gen_old.call_args.kwargs['max_attempts'])
        self.assertFalse(get_ik_rel_gen_old.call_args.kwargs['learned'])
        self.assertEqual(10, get_ik_rel_gen_old.call_args.kwargs['max_attempts'])

    def test_holding_object_blocks_continuing_to_different_pick(self):
        agent = LLAMPAgent.__new__(LLAMPAgent)

        facts = [('atgrasp', 'left', '1|chicken-leg', 'g0')]

        self.assertTrue(agent._holding_blocks_next_pick(facts, ['picked', '12|salt-shaker']))
        self.assertFalse(agent._holding_blocks_next_pick(facts, ['picked', '1|chicken-leg']))
        self.assertFalse(agent._holding_blocks_next_pick(facts, ['in', '1|chicken-leg', '5|pot-body']))

    def test_progress_summary_counts_last_started_plan_as_completed(self):
        whole_goal_sequence = [
            [['picked', 1]],
            [['openedjoint', (6, 4)]],
        ]
        time_log = [
            {
                'goal': ['picked([1])'],
                'status': 'solved',
                'plan': ['pick'],
                'plan_len': 1,
                'planning': 1.0,
                'object_reducer': 'object-related',
                'last_node': "1_a_['picked', 'chicken leg']",
            },
            {
                'goal': ['openedjoint([(6, 4)])'],
                'status': 'started',
                'plan': ['move_base', 'grasp_pull_ungrasp_handle'],
                'plan_len': 2,
                'planning': 1.0,
                'object_reducer': 'object-related',
                'last_node': "2_a_['openedjoint', 'stove knob on the right']",
            },
        ]

        rows, summary = get_progress_summary_from_time_log(time_log, whole_goal_sequence)

        self.assertEqual(1, summary['task_success'])
        self.assertEqual(2, summary['num_completed_problems'])
        self.assertEqual(2, summary['num_success'])
        self.assertEqual('solved', rows[1][3])


if __name__ == '__main__':
    unittest.main()
