from __future__ import print_function

import argparse
import sys
from os.path import basename, dirname, join

from pybullet_tools.bullet_utils import nice
from pybullet_tools.general_streams import get_grasp_list_gen, get_stable_gen
from pybullet_tools.mobile_streams import filter_grasp_obstacles_for_body, get_ik_fn_old, get_ir_sampler
from pybullet_tools.pr2_primitives import Pose
from pybullet_tools.utils import WorldSaver, get_pose, quat_from_euler

from cogarch_tools.cogarch_utils import (
    get_pddlstream_kwargs,
    get_pddlstream_problem,
    init_pybullet_client,
    parse_agent_args,
)
from cogarch_tools.processes.pddlstream_agent import PDDLStreamAgent
from tutorials.test_vlm_tamp import get_vlm_tamp_agent_parser_given_config
from vlm_tools import EXP_DIR, modify_agent_args_for_vlm_tamp, modify_world_builder_args_for_vlm_tamp
from vlm_tools.llamp_agent import LLAMPAgent, VLM_AGENT_CONFIG_ROOT
from vlm_tools.problems_vlm_tamp import vlm_tamp_problem_fn_from_name


DEFAULT_STATE = (
    '/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/'
    'experiments/latest_run/states/agent_state_3.pkl'
)

DEFAULT_REDUCED_OBSTACLES = [1, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14]
POT_BOTH_HANDLE_GRASPS = [
    [0.3334, 0.0815, 0.0067, -1.5708, -0.0, 1.5708],
    [-0.3352, 0.0815, 0.0067, 1.5708, -0.0, 1.5708],
    [0.3334, 0.0815, 0.0067, 1.5708, -0.0, -1.5708],
    [-0.3352, 0.0815, 0.0067, -1.5708, -0.0, -1.5708],
]


class ProblemView(object):

    def __init__(self, problem, fixed):
        self.problem = problem
        self.robot = problem.robot
        self.world = problem.world
        self.floors = problem.floors
        self.fixed = list(fixed)

    def get_gripper(self, *args, **kwargs):
        return self.problem.get_gripper(*args, **kwargs)


def _name_contains(world, text):
    text = text.lower()
    for body, obj in world.get_all_body_objects(False):
        name = getattr(obj, 'name', '') or ''
        debug_name = getattr(obj, 'debug_name', '') or ''
        if text in name.lower() or text in debug_name.lower():
            return body
    return None


def _find_body(world, names, categories=()):
    for name in names:
        body = world.name_to_body(name)
        if body is not None:
            return body
        body = _name_contains(world, name)
        if body is not None:
            return body
    for category in categories:
        bodies = world.cat_to_bodies(category)
        if bodies:
            return bodies[0]
    raise RuntimeError('Could not find body for names={} categories={}'.format(names, categories))


def _load_problem(state_path):
    old_argv = sys.argv[:]
    try:
        sys.argv = [old_argv[0]]
        args = parse_agent_args(
            config='config_nvidia_kitchen.yaml',
            config_root=VLM_AGENT_CONFIG_ROOT,
            get_agent_parser_given_config=get_vlm_tamp_agent_parser_given_config,
            modify_agent_args_fn=modify_agent_args_for_vlm_tamp,
            problem='test_kitchen_chicken_soup',
            open_goal='make chicken soup',
            planning_mode='sequence',
            api_class_name='gpt55',
            viewer=False,
            load_agent_state=state_path,
            exp_subdir='diag_pot_sink_ik_probe',
            scene_only=True,
            record_problem=False,
            save_initial_observation=False,
        )
    finally:
        sys.argv = old_argv
    args.problem = vlm_tamp_problem_fn_from_name(args.problem)
    world_builder_args = {'temp_dir': join(EXP_DIR, '_temp'), 'load_agent_state': state_path}
    world_builder_args = modify_world_builder_args_for_vlm_tamp(args, world_builder_args)

    init_pybullet_client(args)
    problem, exogenous, goals, problem_dict = get_pddlstream_problem(
        args,
        world_builder_args=world_builder_args,
        robot_builder_args=args.robot_builder_args,
    )
    pddlstream_problem = problem_dict['pddlstream_problem']
    solver_kwargs = get_pddlstream_kwargs(
        args,
        problem_dict['skeleton'],
        problem_dict['subgoals'],
        [problem, goals, pddlstream_problem.init],
    )

    agent = LLAMPAgent(problem.world, init=pddlstream_problem.init, goals=goals,
                       processes=exogenous, pddlstream_kwargs=solver_kwargs)
    agent.set_pddlstream_problem(problem_dict, problem)
    source_run_dir = dirname(dirname(state_path))
    agent.exp_dir = source_run_dir
    agent.timestamped_name = basename(source_run_dir)
    agent.domain_pddl = args.domain_pddl
    agent.stream_pddl = args.stream_pddl
    agent.custom_limits = problem.robot.custom_limits
    agent = PDDLStreamAgent.load_agent_state(agent, state_path)
    problem = agent.initial_state
    if problem is None:
        raise RuntimeError('Saved agent state does not contain initial_state')
    return agent, problem


def _sample_grasps(problem, pot, max_grasps):
    grasp_gen = get_grasp_list_gen(
        problem,
        collisions=True,
        use_all_grasps=True,
        num_samples=max_grasps,
        randomize=False,
        verbose=False,
    )
    return [entry[0] for entry in grasp_gen(pot) if entry is not None]


def _manual_both_handle_grasps(problem, pot):
    grasp_poses = [(tuple(g[:3]), quat_from_euler(g[3:])) for g in POT_BOTH_HANDLE_GRASPS]
    return problem.robot.make_grasps(
        'hand', problem.robot.arms[0], pot, grasp_poses, collisions=True)


def _run_scenario(problem, arm, pot, pot_pose, grasps, obstacles, name, ir_attempts, collisions):
    scenario_problem = ProblemView(problem, obstacles)
    ir_sampler = get_ir_sampler(
        scenario_problem, collisions=collisions, learned=False,
        custom_limits=problem.robot.custom_limits,
        max_attempts=ir_attempts, verbose=False, visualize=False)
    ik_fn = get_ik_fn_old(scenario_problem, collisions=collisions, teleport=False,
                          custom_limits=problem.robot.custom_limits,
                          verbose=False, visualize=False)
    print('\n===== {} | obstacles={} ====='.format(name, obstacles))
    solved = []
    for gi, grasp in enumerate(grasps):
        base_gen = ir_sampler(arm, pot, pot_pose, grasp)
        attempts = 0
        for _ in range(ir_attempts):
            saver = WorldSaver(bodies=[problem.robot.body, pot])
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
            result = ik_fn(arm, pot, pot_pose, grasp, bq)
            saver.restore()
            if result is not None:
                solved.append((gi, bq))
                print('  OK grasp[{}] {} base={}'.format(gi, nice(grasp.value), nice(bq.values)))
                break
        if not any(item[0] == gi for item in solved):
            print('  FAIL grasp[{}] {} after {} base attempts'.format(gi, nice(grasp.value), attempts))
    print('SUMMARY {}: {} / {} grasps solved'.format(name, len(solved), len(grasps)))
    return solved


def _sample_target_poses(problem, pot, target_surface, max_poses, collisions):
    stable_gen = get_stable_gen(
        problem,
        collisions=collisions,
        num_samples=max_poses,
        learned_sampling=True,
        relpose=False,
        verbose=False,
        visualize=False,
    )
    poses = []
    for entry in stable_gen(pot, target_surface):
        if entry is None:
            continue
        poses.append(entry[0])
        if len(poses) >= max_poses:
            break
    return poses


def diagnose(state_path, ir_attempts, max_grasps, max_target_poses, collisions, use_reduced_obstacles):
    agent, problem = _load_problem(state_path)
    world = problem.world
    robot = problem.robot
    arm = robot.arms[0]
    pot = _find_body(world, ['braiserbody', 'pot body', 'pot'], categories=['space', 'region'])
    chicken = _find_body(world, ['chicken-leg', 'chicken', 'meatturkeyleg'], categories=['food'])
    lid = _find_body(world, ['braiserlid', 'pot lid', 'lid'], categories=['braiserlid'])
    target_surface = _find_body(world, ['front_right_stove', 'stove on the right'])

    pot_pose = Pose(pot, get_pose(pot))
    grasps = _sample_grasps(problem, pot, max_grasps)
    both_handle_grasps = _manual_both_handle_grasps(problem, pot)
    if use_reduced_obstacles:
        base_obstacles = list(DEFAULT_REDUCED_OBSTACLES)
    else:
        base_obstacles = list(problem.fixed)

    print('state_path = {}'.format(state_path))
    print('pot = {} pose={}'.format(world.get_name(pot), nice(pot_pose.value)))
    print('chicken = {} | lid = {}'.format(world.get_name(chicken), world.get_name(lid)))
    print('target_surface = {}'.format(world.get_name(target_surface)))
    print('obstacles = {}'.format(base_obstacles))
    print('filtered obstacles for pot = {}'.format(
        filter_grasp_obstacles_for_body(world, pot, base_obstacles)))
    print('attachments detail:')
    for key, attachment in getattr(world, 'attachments', {}).items():
        parent = getattr(attachment, 'parent', None)
        child = getattr(attachment, 'child', None)
        print('  key={} key.body={} parent={} parent.body={} child={} child.body={}'.format(
            key, getattr(key, 'body', None), parent, getattr(parent, 'body', None), child, getattr(child, 'body', None)))
    print('testing {} pot grasps'.format(len(grasps)))

    scenarios = [
        ('original', base_obstacles),
        ('without_chicken', [o for o in base_obstacles if o != chicken]),
        ('without_chicken_lid', [o for o in base_obstacles if o not in [chicken, lid]]),
        ('without_movables_near_pot', [o for o in base_obstacles if o not in [chicken, lid, 12, 13]]),
    ]
    for name, obstacles in scenarios:
        _run_scenario(problem, arm, pot, pot_pose, grasps, obstacles, name, ir_attempts, collisions)
    _run_scenario(problem, arm, pot, pot_pose, both_handle_grasps, base_obstacles,
                  'manual_both_handles', ir_attempts, collisions)

    target_poses = _sample_target_poses(problem, pot, target_surface, max_target_poses, collisions)
    print('\ntesting {} sampled target poses on {}'.format(len(target_poses), world.get_name(target_surface)))
    for pi, target_pose in enumerate(target_poses):
        print('TARGET pose[{}] support={} value={}'.format(
            pi, getattr(target_pose, 'support', None), nice(target_pose.value)))
        for name, obstacles in scenarios:
            _run_scenario(problem, arm, pot, target_pose, grasps, obstacles,
                          'target_pose[{}]_{}'.format(pi, name), ir_attempts, collisions)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--state', default=DEFAULT_STATE)
    parser.add_argument('--ir_attempts', type=int, default=12)
    parser.add_argument('--max_grasps', type=int, default=8)
    parser.add_argument('--max_target_poses', type=int, default=2)
    parser.add_argument('--no_collisions', action='store_true')
    parser.add_argument('--problem_fixed_only', action='store_true')
    args = parser.parse_args()
    diagnose(args.state, args.ir_attempts, args.max_grasps, args.max_target_poses,
             not args.no_collisions, not args.problem_fixed_only)


if __name__ == '__main__':
    main()
