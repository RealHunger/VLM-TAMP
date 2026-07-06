from __future__ import print_function

import argparse

from pybullet_tools.bullet_utils import nice
from pybullet_tools.mobile_streams import filter_grasp_obstacles_for_body, get_ik_fn_old, get_ir_sampler
from pybullet_tools.pr2_primitives import Pose
from pybullet_tools.utils import WorldSaver, get_pose, pairwise_collision
from tutorials.diagnose_lid_place_ik import ProblemView, _find_body, _load_problem, _sample_grasps


DEFAULT_STATE = (
    '/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/'
    'experiments/latest_run/states/agent_state_14.pkl'
)


def _current_support(problem, body):
    for key, attachment in getattr(problem.world, 'attachments', {}).items():
        child = getattr(attachment, 'child', None)
        key_body = getattr(key, 'body', key)
        child_body = getattr(child, 'body', child)
        if body in [key_body, child_body]:
            parent = getattr(attachment, 'parent', None)
            return getattr(parent, 'pybullet_name', getattr(parent, 'body', parent))
    return None


def _run_scenario(problem, arm, lid, pose, grasps, obstacles, name, ir_attempts, collisions,
                  ir_obstacles=None):
    ir_problem = ProblemView(problem, ir_obstacles if ir_obstacles is not None else obstacles)
    ik_problem = ProblemView(problem, obstacles)
    ir_sampler = get_ir_sampler(
        ir_problem, collisions=collisions, learned=False,
        custom_limits=problem.robot.custom_limits,
        max_attempts=ir_attempts, verbose=True, visualize=False)
    ik_fn = get_ik_fn_old(ik_problem, collisions=collisions, teleport=False,
                          custom_limits=problem.robot.custom_limits,
                          verbose=True, visualize=False)

    print('\n===== {} | obstacles={} ====='.format(name, obstacles))
    pose.assign()
    collisions_now = [o for o in obstacles if pairwise_collision(lid, o)]
    print('PICK pose support={} value={} collides={}'.format(
        getattr(pose, 'support', None), nice(pose.value), collisions_now))

    solved = []
    for gi, grasp in enumerate(grasps):
        base_gen = ir_sampler(arm, lid, pose, grasp)
        attempts = 0
        for _ in range(ir_attempts):
            saver = WorldSaver(bodies=[problem.robot.body, lid])
            try:
                bq_tuple = next(base_gen)
            except StopIteration:
                saver.restore()
                break
            if bq_tuple is None:
                saver.restore()
                break
            bq = bq_tuple[0]
            attempts += 1
            result = ik_fn(arm, lid, pose, grasp, bq)
            saver.restore()
            if result is not None:
                print('  OK grasp[{}] {} base={}'.format(gi, nice(grasp.value), nice(bq.values)))
                solved.append((gi, bq))
                break
        if not any(item[0] == gi for item in solved):
            print('  FAIL grasp[{}] {} after {} base attempts'.format(
                gi, nice(grasp.value), attempts))
    print('SUMMARY {}: {} / {} grasps solved'.format(name, len(solved), len(grasps)))
    return solved


def diagnose(state_path, ir_attempts, max_grasps, collisions):
    agent, problem = _load_problem(state_path)
    world = problem.world
    robot = problem.robot
    arm = robot.arms[0]
    lid = _find_body(world, ['braiserlid', 'pot lid', 'lid'], categories=['braiserlid'])
    pot = _find_body(world, ['braiserbody', 'pot body', 'pot'], categories=['space', 'region'])
    chicken = _find_body(world, ['chicken-leg', 'chicken'], categories=['food'])
    salt = _find_body(world, ['salt-shaker', 'salt'], categories=['condiment'])
    pepper = _find_body(world, ['pepper-shaker', 'pepper'], categories=['condiment'])
    counter = _find_body(world, ['counter'], categories=['counter'])
    oven = _find_body(world, ['oven'], categories=['oven'])

    support = _current_support(problem, lid)
    pose = Pose(lid, get_pose(lid), support=support)
    grasps = _sample_grasps(problem, lid, max_grasps)
    base_obstacles = list(problem.fixed)

    print('state_path = {}'.format(state_path))
    print('lid = {} pose={} support={}'.format(world.get_name(lid), nice(get_pose(lid)), support))
    print('pot = {} pose={}'.format(world.get_name(pot), nice(get_pose(pot))))
    print('chicken = {}'.format(world.get_name(chicken)))
    print('salt = {}'.format(world.get_name(salt)))
    print('pepper = {}'.format(world.get_name(pepper)))
    print('counter = {}'.format(world.get_name(counter)))
    print('oven = {}'.format(world.get_name(oven)))
    print('obstacles = {}'.format(base_obstacles))
    print('filtered obstacles for lid = {}'.format(
        filter_grasp_obstacles_for_body(world, lid, base_obstacles)))
    print('testing {} grasps'.format(len(grasps)))

    scenarios = [
        ('original', base_obstacles, None),
        ('full_reduced_world_like', list(dict.fromkeys(base_obstacles + [chicken, pot, salt, pepper])), None),
        ('without_pot', [o for o in base_obstacles if o != pot], None),
        ('without_pot_chicken', [o for o in base_obstacles if o not in [pot, chicken]], None),
        ('without_counter_pot_chicken', [o for o in base_obstacles if o not in [counter, pot, chicken]], None),
        ('without_oven', [o for o in base_obstacles if o != oven], None),
        ('ir_without_oven_only', base_obstacles, [o for o in base_obstacles if o != oven]),
    ]
    for name, obstacles, ir_obstacles in scenarios:
        _run_scenario(problem, arm, lid, pose, grasps, obstacles, name, ir_attempts, collisions,
                      ir_obstacles=ir_obstacles)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--state', default=DEFAULT_STATE)
    parser.add_argument('--ir_attempts', type=int, default=8)
    parser.add_argument('--max_grasps', type=int, default=4)
    parser.add_argument('--no_collisions', action='store_true')
    args = parser.parse_args()
    diagnose(args.state, args.ir_attempts, args.max_grasps, not args.no_collisions)


if __name__ == '__main__':
    main()
