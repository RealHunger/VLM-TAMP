from __future__ import print_function

import argparse

from pybullet_tools.bullet_utils import nice
from pybullet_tools.general_streams import Position, get_handle_grasp_list_gen, sample_joint_position_gen
from pybullet_tools.mobile_streams import get_ik_pull_gen
from pybullet_tools.utils import WorldSaver, get_joint_limits, get_joint_position, set_joint_position
from tutorials.diagnose_lid_place_ik import ProblemView, _find_body, _load_problem


DEFAULT_STATE = (
    '/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/'
    'experiments/latest_run/states/agent_state_16.pkl'
)


class PullProblemView(ProblemView):

    def __init__(self, problem, fixed):
        super(PullProblemView, self).__init__(problem, fixed)
        self.ignored_pairs = getattr(problem, 'ignored_pairs', [])


def _find_right_stove_knob(world):
    knobs = list(world.cat_to_bodies('knob'))
    for knob in knobs:
        if knob == (6, 4):
            return knob
    for knob in knobs:
        name = world.get_name(knob).lower()
        if 'right' in name or 'knob_joint_2' in name:
            return knob
    if knobs:
        return knobs[0]
    raise RuntimeError('Could not find a knob body-joint')


def _sample_handle_grasps(problem, knob, max_grasps):
    grasp_gen = get_handle_grasp_list_gen(
        problem,
        collisions=True,
        num_samples=max_grasps,
        verbose=True,
        retain_all=True,
    )
    return [entry[0] for entry in grasp_gen(knob) if entry is not None]


def _sample_open_positions(problem, knob, pst1, max_positions):
    position_gen = sample_joint_position_gen(problem, num_samples=max_positions, verbose=True)
    return [entry[0] for entry in position_gen(knob, pst1) if entry is not None]


def _given_open_positions(knob, values):
    return [Position(knob, float(value)) for value in values]


def _run_scenario(problem, arm, knob, pst1, pst2s, grasps, obstacles, name, attempts, collisions):
    scenario_problem = PullProblemView(problem, obstacles)
    pull_gen = get_ik_pull_gen(
        scenario_problem,
        collisions=collisions,
        learned=False,
        custom_limits=problem.robot.custom_limits,
        max_attempts=attempts,
        num_intervals=30,
        verbose=True,
        visualize=False,
    )

    print('\n===== {} | obstacles={} ====='.format(name, obstacles))
    solved = []
    for pi, pst2 in enumerate(pst2s):
        for gi, grasp in enumerate(grasps):
            gen = pull_gen(arm, knob, pst1, pst2, grasp)
            count = 0
            result = None
            for _ in range(attempts):
                saver = WorldSaver(bodies=[problem.robot.body, knob[0]])
                try:
                    result = next(gen)
                except StopIteration:
                    saver.restore()
                    break
                count += 1
                saver.restore()
                if result is not None:
                    break
            if result is not None:
                print('  OK pst[{}]={} grasp[{}] {} after {} outputs'.format(
                    pi, nice(pst2.value), gi, nice(grasp.value), count))
                solved.append((pi, gi, result))
                break
            print('  FAIL pst[{}]={} grasp[{}] {} after {} outputs'.format(
                pi, nice(pst2.value), gi, nice(grasp.value), count))
    print('SUMMARY {}: {} / {} position-grasp pairs solved'.format(
        name, len(solved), len(pst2s) * len(grasps)))
    return solved


def diagnose(state_path, attempts, max_grasps, max_positions, collisions, positions=None,
             joint_body=None, joint_index=None, joint_position=None):
    agent, problem = _load_problem(state_path)
    world = problem.world
    robot = problem.robot
    arm = robot.arms[0]
    knob = (joint_body, joint_index) if joint_body is not None and joint_index is not None else _find_right_stove_knob(world)
    if joint_position is not None:
        set_joint_position(knob[0], knob[1], joint_position)
    lid = _find_body(world, ['braiserlid', 'pot lid', 'lid'], categories=['braiserlid'])
    pot = _find_body(world, ['braiserbody', 'pot body', 'pot'], categories=['space', 'region'])
    chicken = _find_body(world, ['chicken-leg', 'chicken'], categories=['food'])
    oven = _find_body(world, ['oven'], categories=['oven'])
    counter = _find_body(world, ['counter'], categories=['counter'])

    pst1 = Position(knob)
    grasps = _sample_handle_grasps(problem, knob, max_grasps)
    pst2s = _given_open_positions(knob, positions) if positions else _sample_open_positions(problem, knob, pst1, max_positions)
    base_obstacles = list(problem.fixed)

    print('state_path = {}'.format(state_path))
    print('knob = {} current={} limits={}'.format(
        world.get_name(knob), nice(get_joint_position(knob[0], knob[1])), nice(get_joint_limits(knob[0], knob[1]))))
    print('lid = {} | pot = {} | chicken = {} | oven = {} | counter = {}'.format(
        world.get_name(lid), world.get_name(pot), world.get_name(chicken), world.get_name(oven), world.get_name(counter)))
    print('obstacles = {}'.format(base_obstacles))
    print('testing {} positions and {} handle grasps'.format(len(pst2s), len(grasps)))

    scenarios = [
        ('original', base_obstacles),
        ('with_pot_lid', list(dict.fromkeys(base_obstacles + [pot, lid]))),
        ('without_lid', [o for o in base_obstacles if o != lid]),
        ('without_pot', [o for o in base_obstacles if o != pot]),
        ('without_pot_lid_chicken', [o for o in base_obstacles if o not in [pot, lid, chicken]]),
        ('without_counter', [o for o in base_obstacles if o != counter]),
        ('without_oven', [o for o in base_obstacles if o != oven]),
    ]
    for name, obstacles in scenarios:
        _run_scenario(problem, arm, knob, pst1, pst2s, grasps, obstacles, name, attempts, collisions)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--state', default=DEFAULT_STATE)
    parser.add_argument('--attempts', type=int, default=8)
    parser.add_argument('--max_grasps', type=int, default=4)
    parser.add_argument('--max_positions', type=int, default=3)
    parser.add_argument('--positions', nargs='*', type=float, default=None)
    parser.add_argument('--joint_body', type=int, default=None)
    parser.add_argument('--joint_index', type=int, default=None)
    parser.add_argument('--joint_position', type=float, default=None)
    parser.add_argument('--no_collisions', action='store_true')
    args = parser.parse_args()
    diagnose(args.state, args.attempts, args.max_grasps, args.max_positions, not args.no_collisions,
             args.positions, args.joint_body, args.joint_index, args.joint_position)


if __name__ == '__main__':
    main()
