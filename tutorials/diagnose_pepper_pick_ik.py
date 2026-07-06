from __future__ import print_function

import argparse
import sys
from os.path import basename, dirname, join

from pybullet_tools.bullet_utils import nice
from pybullet_tools.general_streams import get_grasp_list_gen
from pybullet_tools.mobile_streams import filter_grasp_obstacles_for_body, get_ik_fn_old, get_ir_sampler
from pybullet_tools.pr2_primitives import Pose
from pybullet_tools.utils import WorldSaver, get_pose

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
    'experiments/latest_run/states/agent_state_10b.pkl'
)


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
            exp_subdir='diag_pepper_pick_ik_probe',
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


def _run_scenario(problem, arm, pepper, pepper_pose, grasps, obstacles, name, ir_attempts, collisions):
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
        base_gen = ir_sampler(arm, pepper, pepper_pose, grasp)
        attempts = 0
        for _ in range(ir_attempts):
            saver = WorldSaver(bodies=[problem.robot.body, pepper])
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
            result = ik_fn(arm, pepper, pepper_pose, grasp, bq)
            saver.restore()
            if result is not None:
                solved.append((gi, bq))
                print('  OK grasp[{}] {} base={}'.format(gi, nice(grasp.value), nice(bq.values)))
                break
        if not any(item[0] == gi for item in solved):
            print('  FAIL grasp[{}] {} after {} base attempts'.format(gi, nice(grasp.value), attempts))
    print('SUMMARY {}: {} / {} grasps solved'.format(name, len(solved), len(grasps)))
    return solved


def diagnose(state_path, ir_attempts, max_grasps, collisions, body_name):
    agent, problem = _load_problem(state_path)
    world = problem.world
    robot = problem.robot
    arm = robot.arms[0]

    pepper = _find_body(world, ['pepper-shaker', 'pepper shaker'], categories=['sprinkler'])
    salt = _find_body(world, ['salt-shaker', 'salt shaker'], categories=['sprinkler'])
    pot = _find_body(world, ['braiserbody', 'pot body', 'pot'], categories=['space', 'region'])
    chicken = _find_body(world, ['chicken-leg', 'chicken'], categories=['food'])
    lid = _find_body(world, ['braiserlid', 'pot lid', 'lid'], categories=['braiserlid'])
    fork = _maybe_find_body(world, ['fork'], categories=['utensil'])

    target = salt if body_name == 'salt' else pepper
    other_condiment = pepper if body_name == 'salt' else salt
    target_pose = Pose(target, get_pose(target))
    grasps = _sample_grasps(problem, target, max_grasps)
    base_obstacles = [o for o in problem.fixed if o not in problem.floors]
    pddl_like_obstacles = base_obstacles + [chicken, pot, lid, other_condiment]

    print('state_path = {}'.format(state_path))
    print('target = {} pose={}'.format(world.get_name(target), nice(target_pose.value)))
    print('salt = {} | pot = {} | chicken = {} | lid = {} | fork = {}'.format(
        world.get_name(salt), world.get_name(pot), world.get_name(chicken),
        world.get_name(lid), world.get_name(fork) if fork is not None else None))
    print('current atgrasp = {}'.format([
        fact for fact in getattr(agent, 'state_facts', [])
        if str(fact[0]).lower() == 'atgrasp'
    ]))
    print('base obstacles = {}'.format(base_obstacles))
    print('pddl-like obstacles = {}'.format(pddl_like_obstacles))
    print('filtered target pddl-like obstacles = {}'.format(
        filter_grasp_obstacles_for_body(world, target, pddl_like_obstacles)))
    print('testing {} {} grasps'.format(len(grasps), body_name))

    scenarios = [
        ('base_fixed', base_obstacles),
        ('pddl_like', pddl_like_obstacles),
        ('pddl_like_unique', list(dict.fromkeys(pddl_like_obstacles))),
        ('without_other_condiment', [o for o in pddl_like_obstacles if o != other_condiment]),
        ('without_near_movables', [o for o in pddl_like_obstacles if o not in [other_condiment, pot, chicken, lid, fork]]),
    ]
    for name, obstacles in scenarios:
        _run_scenario(problem, arm, target, target_pose, grasps, obstacles, name, ir_attempts, collisions)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--state', default=DEFAULT_STATE)
    parser.add_argument('--ir_attempts', type=int, default=20)
    parser.add_argument('--max_grasps', type=int, default=8)
    parser.add_argument('--no_collisions', action='store_true')
    parser.add_argument('--body', choices=['pepper', 'salt'], default='pepper')
    args = parser.parse_args()
    diagnose(args.state, args.ir_attempts, args.max_grasps, not args.no_collisions, args.body)


if __name__ == '__main__':
    main()
