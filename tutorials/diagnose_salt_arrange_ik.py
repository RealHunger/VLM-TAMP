from __future__ import print_function

import argparse
import sys
from os.path import basename, dirname, join

from pybullet_tools.bullet_utils import nice
from pybullet_tools.general_streams import get_grasp_list_gen
from pybullet_tools.mobile_streams import filter_grasp_obstacles_for_body, get_ik_fn_old, get_ir_sampler
from pybullet_tools.pr2_primitives import Conf, Pose
from pybullet_tools.utils import WorldSaver, quat_from_euler, Euler

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
    'experiments/latest_run/states/agent_state_9.pkl'
)
DEFAULT_POSE = ((0.771, 7.071, 1.152), quat_from_euler(Euler(yaw=-3.141)))
DEFAULT_GRASP = (-0.0, 0.0, 0.124, -3.142, -0.0, 0.0)
DEFAULT_SUPPORT = (3, None, 0)
KNOWN_BASES = [
    (1.508, 7.218, 0.968, 1.918),
    (1.531, 7.116, 0.859, 1.972),
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


def _maybe_find_body(world, names, categories=()):
    try:
        return _find_body(world, names, categories=categories)
    except RuntimeError:
        return None


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
            exp_subdir='diag_salt_arrange_ik_probe',
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


def _sample_grasps(problem, body, max_grasps):
    grasp_gen = get_grasp_list_gen(
        problem,
        collisions=True,
        use_all_grasps=True,
        num_samples=max_grasps,
        randomize=False,
        verbose=False,
    )
    return [entry[0] for entry in grasp_gen(body) if entry is not None]


def _matching_grasp(grasps, target):
    for grasp in grasps:
        value = nice(grasp.value, round_to=6)
        if all(abs(a - b) < 1e-3 for a, b in zip(value, target)):
            return grasp
    raise RuntimeError('No grasp matched {}'.format(target))


def _current_atgrasp(agent, body, grasps):
    for fact in getattr(agent, 'state_facts', []):
        if str(fact[0]).lower() != 'atgrasp' or fact[2] != body:
            continue
        current = fact[3]
        current_value = nice(current.value, round_to=6) if hasattr(current, 'value') else nice(current, round_to=6)
        return _matching_grasp(grasps, current_value)
    return _matching_grasp(grasps, DEFAULT_GRASP)


def _run_scenario(problem, arm, salt, salt_pose, grasp, obstacles, name, ir_attempts, collisions):
    scenario_problem = ProblemView(problem, obstacles)
    ir_sampler = get_ir_sampler(
        scenario_problem, collisions=collisions, learned=False,
        custom_limits=problem.robot.custom_limits,
        max_attempts=ir_attempts, verbose=False, visualize=False)
    ik_fn = get_ik_fn_old(scenario_problem, collisions=collisions, teleport=False,
                          custom_limits=problem.robot.custom_limits,
                          verbose=False, visualize=False)
    print('\n===== {} | obstacles={} ====='.format(name, obstacles))
    base_gen = ir_sampler(arm, salt, salt_pose, grasp)
    probe_bq = None
    probe_saver = WorldSaver(bodies=[problem.robot.body, salt])
    try:
        probe_tuple = next(base_gen)
        if probe_tuple is not None:
            probe_bq = probe_tuple[0]
    except StopIteration:
        probe_bq = None
    probe_saver.restore()
    if probe_bq is not None:
        for values in KNOWN_BASES:
            saver = WorldSaver(bodies=[problem.robot.body, salt])
            bq = Conf(probe_bq.body, probe_bq.joints, values)
            result = ik_fn(arm, salt, salt_pose, grasp, bq)
            saver.restore()
            print('  KNOWN {} -> {}'.format(nice(values), 'OK' if result is not None else 'FAIL'))
        base_gen = ir_sampler(arm, salt, salt_pose, grasp)
    for attempt in range(ir_attempts):
        saver = WorldSaver(bodies=[problem.robot.body, salt])
        try:
            bq_tuple = next(base_gen)
        except StopIteration:
            saver.restore()
            print('  STOP after {} base attempts'.format(attempt))
            return False
        if bq_tuple is None:
            saver.restore()
            print('  NONE after {} base attempts'.format(attempt))
            return False
        bq = bq_tuple[0]
        result = ik_fn(arm, salt, salt_pose, grasp, bq)
        saver.restore()
        if result is not None:
            print('  OK attempt={} base={}'.format(attempt + 1, nice(bq.values)))
            return True
        print('  FAIL attempt={} base={}'.format(attempt + 1, nice(bq.values)))
    return False


def diagnose(state_path, ir_attempts, max_grasps, collisions, body_name):
    agent, problem = _load_problem(state_path)
    world = problem.world
    robot = problem.robot
    arm = robot.arms[0]

    salt = _find_body(world, ['salt-shaker', 'salt shaker'], categories=['sprinkler'])
    pepper = _maybe_find_body(world, ['pepper-shaker', 'pepper shaker'], categories=['sprinkler'])
    target = pepper if body_name == 'pepper' else salt
    pot = _maybe_find_body(world, ['braiserbody', 'pot body', 'pot'], categories=['space', 'region'])
    chicken = _maybe_find_body(world, ['chicken-leg', 'chicken'], categories=['food'])
    lid = _maybe_find_body(world, ['braiserlid', 'pot lid', 'lid'], categories=['braiserlid'])

    target_pose_value = DEFAULT_POSE if target == salt else ((0.764, 7.303, 1.164), quat_from_euler(Euler(yaw=-3.141)))
    target_pose = Pose(target, target_pose_value, DEFAULT_SUPPORT)
    grasps = _sample_grasps(problem, target, max_grasps)
    grasp = _current_atgrasp(agent, target, grasps)
    base_obstacles = [o for o in problem.fixed if o not in problem.floors]
    pddl_like_obstacles = base_obstacles + [o for o in [chicken, pot, lid, pepper] if o is not None and o != target]

    print('state_path = {}'.format(state_path))
    print('target = {} fixed_pose={} grasp={}'.format(world.get_name(target), nice(target_pose.value), nice(grasp.value)))
    print('pepper = {} | pot = {} | chicken = {} | lid = {}'.format(pepper, pot, chicken, lid))
    print('current atgrasp = {}'.format([
        fact for fact in getattr(agent, 'state_facts', [])
        if str(fact[0]).lower() == 'atgrasp'
    ]))
    print('base obstacles = {}'.format(base_obstacles))
    print('pddl-like obstacles = {}'.format(pddl_like_obstacles))
    print('filtered salt pddl-like obstacles = {}'.format(
        filter_grasp_obstacles_for_body(world, target, pddl_like_obstacles)))

    scenarios = [
        ('base_fixed', base_obstacles),
        ('pddl_like', pddl_like_obstacles),
        ('pddl_like_unique', list(dict.fromkeys(pddl_like_obstacles))),
        ('without_pepper', [o for o in pddl_like_obstacles if o != pepper]),
        ('without_pot', [o for o in pddl_like_obstacles if o != pot]),
        ('without_near_movables', base_obstacles),
    ]
    for name, obstacles in scenarios:
        _run_scenario(problem, arm, target, target_pose, grasp, obstacles, name, ir_attempts, collisions)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--state', default=DEFAULT_STATE)
    parser.add_argument('--ir_attempts', type=int, default=40)
    parser.add_argument('--max_grasps', type=int, default=15)
    parser.add_argument('--body', choices=['salt', 'pepper'], default='salt')
    parser.add_argument('--no_collisions', action='store_true')
    args = parser.parse_args()
    diagnose(args.state, args.ir_attempts, args.max_grasps, not args.no_collisions, args.body)


if __name__ == '__main__':
    main()
