from __future__ import print_function

import argparse
import sys
from os.path import basename, dirname, join

from pybullet_tools.bullet_utils import nice
from pybullet_tools.general_streams import get_above_pose_gen, get_grasp_list_gen
from pybullet_tools.mobile_streams import get_ik_fn_old, get_ir_sampler
from pybullet_tools.pr2_primitives import Pose
from pybullet_tools.utils import Euler, WorldSaver, get_pose, quat_from_euler

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
    'experiments/diag_place_chicken_uniform_ir_after_ir_fix/'
    '260703_203600_vlm-tamp/states/agent_state_8b.pkl'
)


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
            exp_subdir='diag_salt_sprinkle_ik_probe',
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


def _current_grasps(agent, sprinkler):
    grasps = []
    for fact in getattr(agent, 'state_facts', []):
        if len(fact) >= 4 and str(fact[0]).lower() == 'atgrasp' and fact[2] == sprinkler:
            grasps.append(fact[3])
    return grasps


def _sample_grasps(problem, sprinkler, max_grasps):
    grasp_gen = get_grasp_list_gen(
        problem,
        collisions=True,
        use_all_grasps=True,
        num_samples=max_grasps,
        randomize=False,
        verbose=False,
    )
    return [entry[0] for entry in grasp_gen(sprinkler) if entry is not None]


def _pose_variants(sprinkler, pose, z_offsets):
    point, quat = pose.value
    support = getattr(pose, 'support', None)
    for dz in z_offsets:
        yield Pose(sprinkler, ((point[0], point[1], point[2] + dz), quat), support=support)
    for dz in z_offsets:
        for roll in [0, 1.57079632679, 3.14159265359]:
            yield Pose(sprinkler, ((point[0], point[1], point[2] + dz), quat_from_euler(Euler(yaw=0, roll=roll))), support=support)


def _check(problem, arm, sprinkler, poses, grasps, ir_attempts, collisions):
    ir_sampler = get_ir_sampler(
        problem, collisions=collisions, learned=False,
        custom_limits=problem.robot.custom_limits,
        max_attempts=ir_attempts, verbose=True, visualize=False)
    ik_fn = get_ik_fn_old(problem, collisions=collisions, teleport=False,
                          custom_limits=problem.robot.custom_limits,
                          verbose=True, visualize=False)

    solved = []
    for gi, grasp in enumerate(grasps):
        print('\nGRASP {} {}'.format(gi, nice(grasp.value)))
        for pi, pose in enumerate(poses):
            attempts = 0
            base_gen = ir_sampler(arm, sprinkler, pose, grasp)
            for _ in range(ir_attempts):
                saver = WorldSaver(bodies=[problem.robot.body, sprinkler])
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
                result = ik_fn(arm, sprinkler, pose, grasp, bq)
                saver.restore()
                if result is not None:
                    print('  OK pose[{}]={} base={}'.format(pi, nice(pose.value), nice(bq.values)))
                    solved.append((gi, pi, pose, bq))
                    break
            if attempts and not any(item[0] == gi and item[1] == pi for item in solved):
                print('  FAIL pose[{}]={} after {} attempts'.format(pi, nice(pose.value), attempts))
    print('\nSUMMARY: {} / {} pose-grasp pairs solved'.format(len(solved), len(grasps) * len(poses)))
    return solved


def diagnose(state_path, ir_attempts, max_grasps, collisions, body):
    agent, problem = _load_problem(state_path)
    world = problem.world
    robot = problem.robot
    arm = robot.arms[0]
    body_names = {
        'salt': ['salt-shaker', 'salt shaker', 'salter'],
        'pepper': ['pepper-shaker', 'pepper shaker'],
    }
    sprinkler = _find_body(world, body_names[body], categories=['sprinkler'])
    pot = _find_body(world, ['braiserbody', 'pot body', 'pot'], categories=['space', 'region'])

    print('state_path = {}'.format(state_path))
    print('{} = {} pose={}'.format(body, world.get_name(sprinkler), nice(get_pose(sprinkler))))
    print('pot = {} pose={}'.format(world.get_name(pot), nice(get_pose(pot))))
    print('current atgrasp = {}'.format([
        fact for fact in getattr(agent, 'state_facts', [])
        if str(fact[0]).lower() == 'atgrasp'
    ]))

    region_pose = next(fact[2] for fact in agent.state_facts
                       if len(fact) >= 3 and str(fact[0]).lower() == 'pose' and fact[1] == pot)
    above_pose = next(get_above_pose_gen(problem, collisions=collisions)(pot, region_pose, sprinkler))[0]
    poses = list(_pose_variants(sprinkler, above_pose, [0, 0.05, 0.1, 0.2, 0.35, 0.5]))
    for i, pose in enumerate(poses):
        print('pose[{}] = {}'.format(i, nice(pose.value)))

    grasps = _current_grasps(agent, sprinkler) or _sample_grasps(problem, sprinkler, max_grasps)
    print('testing {} grasps'.format(len(grasps)))
    _check(problem, arm, sprinkler, poses, grasps[:max_grasps], ir_attempts, collisions)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--state', default=DEFAULT_STATE)
    parser.add_argument('--ir_attempts', type=int, default=8)
    parser.add_argument('--max_grasps', type=int, default=4)
    parser.add_argument('--body', choices=['salt', 'pepper'], default='salt')
    parser.add_argument('--no_collisions', action='store_true')
    args = parser.parse_args()
    diagnose(args.state, args.ir_attempts, args.max_grasps, not args.no_collisions, args.body)


if __name__ == '__main__':
    main()
