from __future__ import print_function

import argparse
import sys
from os.path import basename, dirname, join

from pybullet_tools.general_streams import get_contain_list_gen, get_grasp_list_gen
from pybullet_tools.mobile_streams import get_ik_fn_old, get_ir_sampler
from pybullet_tools.bullet_utils import nice
from pybullet_tools.pr2_utils import learned_pose_generator
from pybullet_tools.utils import all_between, get_custom_limits, get_joint_positions, uniform_pose_generator, WorldSaver
from cogarch_tools.cogarch_utils import (
    get_pddlstream_kwargs,
    get_pddlstream_problem,
    init_pybullet_client,
    parse_agent_args,
)
from tutorials.test_vlm_tamp import get_vlm_tamp_agent_parser_given_config
from vlm_tools import EXP_DIR, modify_agent_args_for_vlm_tamp, modify_world_builder_args_for_vlm_tamp
from vlm_tools.llamp_agent import LLAMPAgent, VLM_AGENT_CONFIG_ROOT
from vlm_tools.problems_vlm_tamp import vlm_tamp_problem_fn_from_name
from cogarch_tools.processes.pddlstream_agent import PDDLStreamAgent


DEFAULT_STATE = (
    '/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/'
    'pybullet_planning/../experiments/diag_place_chicken_use_all_grasps/'
    '260703_191641_vlm-tamp/states/agent_state_2.pkl'
)


def _name_contains(world, text):
    text = text.lower()
    for body, obj in world.get_all_body_objects(False):
        name = getattr(obj, 'name', '') or ''
        if text in name.lower():
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
            exp_subdir='diag_place_chicken_grasp_probe',
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
    # Restore commands relative to the source run, not a new diagnostic output dir.
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


def _sample_poses(problem, chicken, pot, pose_samples):
    pose_gen = get_contain_list_gen(problem, collisions=True, num_samples=pose_samples, verbose=False)
    poses = [entry[0] for entry in pose_gen(chicken, pot) if entry is not None]
    if not poses:
        raise RuntimeError('No contained poses sampled for chicken={} pot={}'.format(chicken, pot))
    return poses


def _sample_grasps(problem, chicken, max_grasps):
    grasp_gen = get_grasp_list_gen(
        problem,
        collisions=True,
        use_all_grasps=True,
        num_samples=max_grasps,
        randomize=False,
        verbose=False,
    )
    grasps = [entry[0] for entry in grasp_gen(chicken) if entry is not None]
    if not grasps:
        raise RuntimeError('No grasps sampled for chicken={}'.format(chicken))
    return grasps


def _check_grasps(problem, arm, chicken, poses, grasps, ir_attempts, learned, collisions):
    ir_sampler = get_ir_sampler(
        problem, collisions=collisions, learned=learned,
        custom_limits=problem.robot.custom_limits,
        max_attempts=ir_attempts, verbose=True, visualize=False)
    ik_fn = get_ik_fn_old(problem, collisions=collisions, teleport=False,
                          custom_limits=problem.robot.custom_limits,
                          verbose=False, visualize=False)

    print('\n===== {} IR | collisions={} ====='.format('learned' if learned else 'uniform', collisions))
    solved = []
    for gi, grasp in enumerate(grasps):
        attempts = 0
        success = None
        print('\nGRASP {} {}'.format(gi, nice(grasp.value)))
        for pi, pose in enumerate(poses):
            base_gen = ir_sampler(arm, chicken, pose, grasp)
            for _ in range(ir_attempts):
                saver = WorldSaver(bodies=[problem.robot.body, chicken])
                bq_tuple = next(base_gen)
                if bq_tuple is None:
                    saver.restore()
                    break
                bq = bq_tuple[0]
                attempts += 1
                result = ik_fn(arm, chicken, pose, grasp, bq)
                saver.restore()
                if result is not None:
                    success = (pi, bq, result)
                    solved.append((gi, pi, grasp, bq))
                    print('  OK pose[{}] base={}'.format(pi, nice(bq.values)))
                    break
            if success is not None:
                break
        if success is None:
            print('  FAIL after {} base attempts'.format(attempts))
    print('\nSUMMARY {} IR: {} / {} grasps place-compatible'.format(
        'learned' if learned else 'uniform', len(solved), len(grasps)))
    for gi, pi, grasp, bq in solved:
        print('  grasp[{}] pose[{}] grasp={} base={}'.format(gi, pi, nice(grasp.value), nice(bq.values)))
    return solved


def _print_base_probe(problem, arm, chicken, poses, grasps, learned, max_candidates=8):
    robot = problem.robot
    base_joints = robot.get_base_joints()
    lower_limits, upper_limits = get_custom_limits(robot, base_joints, robot.custom_limits)
    initial_torso = robot.get_base_positions()[2] if robot.use_torso else None
    print('\n===== raw base probe | {} IR ====='.format('learned' if learned else 'uniform'))
    print('base limits lower={} upper={} current_base={}'.format(
        nice(lower_limits), nice(upper_limits), nice(get_joint_positions(robot, base_joints))))
    for gi, grasp in enumerate(grasps[:3]):
        for pi, pose in enumerate(poses[:1]):
            pose.assign()
            gripper_pose = robot.get_grasp_pose(pose.value, grasp.value, arm, body=chicken)
            if learned:
                grasp_type = 'top' if grasp.grasp_type == 'hand' else grasp.grasp_type
                base_generator = learned_pose_generator(robot, gripper_pose, arm=arm, grasp_type=grasp_type)
            else:
                base_generator = uniform_pose_generator(robot, gripper_pose)
            print('grasp[{}] pose[{}] gripper_pose={}'.format(gi, pi, nice(gripper_pose)))
            for ci in range(max_candidates):
                base_conf = next(base_generator)
                raw_base_conf = base_conf
                if robot.use_torso:
                    x, y, theta = base_conf
                    base_conf = (x, y, initial_torso, theta)
                print('  cand[{}] raw={} with_torso={} in_limits={}'.format(
                    ci, nice(raw_base_conf), nice(base_conf), all_between(lower_limits, base_conf, upper_limits)))


def diagnose(state_path, pose_samples, ir_attempts, max_grasps, collisions):
    agent, problem = _load_problem(state_path)
    world = problem.world
    robot = problem.robot
    arm = robot.arms[0]
    chicken = _find_body(world, ['chicken-leg', 'chicken', 'meatturkeyleg'], categories=['food'])
    pot = _find_body(world, ['braiserbody', 'pot body', 'pot'], categories=['space', 'region'])

    print('state_path = {}'.format(state_path))
    print('agent = {} problem_count = {}'.format(type(agent).__name__, getattr(agent, 'problem_count', None)))
    print('chicken = {} | pot = {}'.format(world.get_name(chicken), world.get_name(pot)))
    print('current atgrasp = {}'.format([
        fact for fact in getattr(agent, 'state_facts', [])
        if str(fact[0]).lower() == 'atgrasp'
    ]))

    poses = _sample_poses(problem, chicken, pot, pose_samples)
    grasps = _sample_grasps(problem, chicken, max_grasps)
    print('sampled {} contained poses'.format(len(poses)))
    for i, pose in enumerate(poses):
        print('  pose[{}] = {}'.format(i, nice(pose.value)))
    print('sampled {} grasps from grasps_all'.format(len(grasps)))

    _print_base_probe(problem, arm, chicken, poses, grasps, learned=True)
    _print_base_probe(problem, arm, chicken, poses, grasps, learned=False)

    learned_solved = _check_grasps(problem, arm, chicken, poses, grasps, ir_attempts, learned=True, collisions=collisions)
    uniform_solved = _check_grasps(problem, arm, chicken, poses, grasps, ir_attempts, learned=False, collisions=collisions)
    return len(learned_solved) + len(uniform_solved)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--state', default=DEFAULT_STATE)
    parser.add_argument('--pose_samples', type=int, default=8)
    parser.add_argument('--ir_attempts', type=int, default=30)
    parser.add_argument('--max_grasps', type=int, default=20)
    parser.add_argument('--disable_collisions', action='store_true')
    args = parser.parse_args()
    diagnose(args.state, args.pose_samples, args.ir_attempts, args.max_grasps, collisions=not args.disable_collisions)


if __name__ == '__main__':
    main()
