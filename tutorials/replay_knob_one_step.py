from __future__ import print_function

import argparse
from types import SimpleNamespace

from pybullet_tools.utils import set_joint_position
from world_builder.world import Observation
from tutorials.diagnose_lid_place_ik import _load_problem


DEFAULT_STATE = (
    '/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/'
    'experiments/diag_clean_after_pull_internal_diag/260706_002132_vlm-tamp/'
    'states/agent_state_16.pkl'
)


def replay_knob_one_step(state_path, joint_body, joint_index, predicate, joint_position, max_evaluation_plans,
                          downward_time, total_planning_timeout, evaluation_time):
    agent, problem = _load_problem(state_path)
    agent.object_reducer_state = 0
    agent.pddlstream_kwargs = dict(agent.pddlstream_kwargs)
    for key in ['max_planner_time', 'max_time']:
        agent.pddlstream_kwargs.pop(key, None)
    agent.pddlstream_kwargs.update({
        'max_evaluation_plans': max_evaluation_plans,
        'downward_time': downward_time,
        'total_planning_timeout': total_planning_timeout,
        'evaluation_time': evaluation_time,
        'debug': False,
    })

    joint = (joint_body, joint_index)
    if joint_position is not None:
        set_joint_position(joint_body, joint_index, joint_position)
    goals = [[predicate, joint]]
    agent.llamp_api = SimpleNamespace(planning_mode='sequence')
    agent._update_pddlstream_problem(problem.get_facts(), goals, reduce_objects=True)
    problem.robot.modify_pddl(agent.pddlstream_problem, remove_operators=['place'])

    agent.llamp_api = None
    if hasattr(agent, 'goal_sequence'):
        delattr(agent, 'goal_sequence')
    plan = agent.replan(Observation(problem))
    print('ONE_STEP_PLAN', plan)
    print('ONE_STEP_PLAN_LEN', None if plan is None else len(plan))
    return plan


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--state', default=DEFAULT_STATE)
    parser.add_argument('--joint_body', type=int, default=6)
    parser.add_argument('--joint_index', type=int, default=4)
    parser.add_argument('--predicate', default='openedjoint', choices=['openedjoint', 'closedjoint'])
    parser.add_argument('--joint_position', type=float, default=None)
    parser.add_argument('--max_evaluation_plans', type=int, default=24)
    parser.add_argument('--downward_time', type=int, default=60)
    parser.add_argument('--total_planning_timeout', type=int, default=300)
    parser.add_argument('--evaluation_time', type=int, default=60)
    args = parser.parse_args()

    plan = replay_knob_one_step(
        args.state,
        args.joint_body,
        args.joint_index,
        args.predicate,
        args.joint_position,
        args.max_evaluation_plans,
        args.downward_time,
        args.total_planning_timeout,
        args.evaluation_time,
    )
    if plan is None:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
