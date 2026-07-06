# Long-Horizon Manipulation Planning Toolbox

## GPT55 Chicken Soup 一键复刻

这个 fork 包含 2026 年 7 月 GPT55 VLM-TAMP chicken soup 任务的修复代码、验证结果和调试记录。

成功 run 的精简结果文件保存在：

```text
artifacts/gpt55_chicken_soup_success/
```

目录内容包括：

- `vlm-tamp.csv`：最终 planner 结果，最后一行是 `1.0 (17 / 17)`。
- `replay.mp4`：用保存下来的 commands 渲染出的 PyBullet 回放视频。
- `planning_config.json`：成功 run 保存下来的规划配置。
- `llm_memory.json`：成功 run 使用的 GPT55 子目标记忆。
- `agent_memory.json`：成功 run 使用的 agent 侧执行记忆。

### 一键回放

在包含 `pybullet_planning`、`kitchen-worlds`、`lisdf` 和 `motion_planners` 的 workspace 根目录下运行：

```bash
cd pybullet_planning
source ~/miniconda3/etc/profile.d/conda.sh
conda activate kitchen

PYTHONPATH="$(pwd)/../kitchen-worlds/pddlstream:$(pwd):$(pwd)/../kitchen-worlds:$(pwd)/../lisdf:$(pwd)/../kitchen-worlds/pybullet_planning/motion:$PYTHONPATH" python - <<'PY'
import sys
from os.path import abspath, join

R = abspath('.')
sys.path.extend([R, join(R, 'lisdf')])

from pigi_tools.replay_utils import run_replay, REPLAY_CONFIG_DEBUG, load_pigi_data
from world_builder import actions
from pybullet_tools.utils import set_color

run_dir = join(R, 'artifacts', 'gpt55_chicken_soup_success')

def load_without_plan(*args, **kwargs):
    world, problem, exp_dir, run_dir2, commands, plan, body_map = load_pigi_data(*args, **kwargs)
    return world, problem, exp_dir, run_dir2, commands, None, body_map

def tolerant_change_link_color_transition(self, state):
    set_color(self.body, self.color, self.link)
    for key, attachment in state.attachments.items():
        if not hasattr(key, 'body') or not hasattr(key, 'link'):
            continue
        if key.body == self.body and key.link == self.link:
            child = getattr(attachment, 'child', None)
            if child is not None and hasattr(child, 'body'):
                set_color(child.body, self.color, getattr(child, 'link', None))
    return state.new_state()

actions.ChangeLinkColorEvent.transition = tolerant_change_link_color_transition

run_replay(
    REPLAY_CONFIG_DEBUG,
    load_data_fn=load_without_plan,
    given_path=run_dir,
    use_gym=False,
    save_mp4=True,
    save_jpg=False,
    camera_point=(3.1, 7.8, 3.1),
    target_point=(0.5, 7.8, 1.0),
)
PY
```

回放视频会写到：

```text
artifacts/gpt55_chicken_soup_success/replay.mp4
```

### 一键验证

在 workspace 根目录运行下面的 focused regression suite。注意不要在 `pybullet_planning` 目录里面运行，否则 Python 可能会优先解析当前仓库自己的 `examples` 包，导致找不到正确的 `pddlstream/examples`。

```bash
cd ..
source ~/miniconda3/etc/profile.d/conda.sh
conda activate kitchen

PYTHONPATH="$(pwd)/kitchen-worlds/pddlstream:$(pwd)/pybullet_planning:$(pwd)/kitchen-worlds:$(pwd)/lisdf:$(pwd)/kitchen-worlds/pybullet_planning/motion:$PYTHONPATH" \
python -m unittest \
  pybullet_tools.test_general_streams.HandleGraspTests \
  pybullet_tools.test_mobile_streams \
  pybullet_tools.test_general_streams \
  leap_tools.test_object_reducers \
  vlm_tools.test_llamp_agent_sequence
```

预期结果：

```text
Ran 59 tests
OK
```

### 重新运行 memory-backed GPT55 policy

如果不只是回放保存好的 commands，而是想重新跑一次带 memory 的 GPT55 planning，需要先在本地配置 GPT55 key。不要把 key 提交到 GitHub。

```bash
mkdir -p ~/.config/vlm-tamp
printf 'YOUR_GPT55_KEY_HERE\n' > ~/.config/vlm-tamp/gpt55_api_key.txt
chmod 600 ~/.config/vlm-tamp/gpt55_api_key.txt
```

然后在 `pybullet_planning` 目录下运行：

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate kitchen

PYTHONPATH="$(pwd)/../kitchen-worlds/pddlstream:$(pwd):$(pwd)/../kitchen-worlds:$(pwd)/../lisdf:$(pwd)/../kitchen-worlds/pybullet_planning/motion:$PYTHONPATH" \
python tutorials/test_vlm_tamp.py \
  --problem test_kitchen_chicken_soup \
  --api_class_name gpt55 \
  --load_llm_memory artifacts/gpt55_chicken_soup_success \
  --exp_subdir reproduce_gpt55_chicken_soup
```

注意事项：

- `--load_llm_memory` 传的是包含 `llm_memory.json` 的目录，不是 JSON 文件本身。
- 不带 memory 的 fresh GPT55 run 可能生成不同的子目标序列，不保证和这个已验证成功 run 一致。
- 回放视频在少数柜门视角里可能看到视觉穿模，因为 replay 是按保存的运动学 commands 播放，不会重新运行物理接触仿真。

### 中文交接报告

中文交接文档在：

```text
docs/superpowers/reports/2026-07-06-gpt55-vlm-tamp-work-summary.md
docs/superpowers/reports/2026-07-06-vlm-tamp-source-fixes.md
docs/superpowers/reports/2026-07-06-vlm-tamp-debug-records.md
```

This toolbox helps you solve long-horizon mobile manipulation problems using planning or policies. 

It includes utility functions for
* procedurally generating scenes from `.urdf`, `.sdf`, `.obj`, and other mesh files.
  * output in `.lisdf` format that's an extension of `.sdf` format that supports including `.urdf` files and camera poses
  * support loading scenes in pybullet or in web front
* solving long-horizon problems using a task and motion planner `pddlstream`, including
  * samplers used by the planner for mobile manipulation and NAMO domains 
  * tools for speeding up planning based on
    * plan feasibility prediction ([PIGINet](https://piginet.github.io/) project)
    * vlm subgoal/action planning ([VLM-TAMP](https://zt-yang.github.io/vlm-tamp-robot) project)
    * state-space reduction (e.g., heuristic object reduction; identify frequently collided objects during planning)
    * action-space reduction (e.g., remove operators, axioms from pddl file; save databases of grasp, pose, configuration)
    * HPN-based (hierarchical planning in the now) hierarchical planning
  * scripts for generating images, animation, and videos from generated trajectories in pybullet and isaac gym 

We recommend that you use the [kitchen-world](https://github.com/Learning-and-Intelligent-Systems/kitchen-worlds/tree/main) repo, which includes this toolbox, if
* you are interested in procedural generation of kitchen scenes, because various assets and example layouts are provided there.
* you are interested in generating trajectories using motion planning or task and motion planning.

## Installation

The following is included in the kitchen-world installation guide if you took that route.

1. Clone and grab the submodules, may take a while

```shell
git clone --recurse-submodules git@github.com:zt-yang/pybullet_planning.git
cd pybullet_planning
```

2. Install dependencies

```shell
conda create -n pybullet python==3.8
pip install -r requirements.txt
conda activate pybullet
```

3. Build IK solvers

IKFast solver for PR2 arm planning (see [troubleshooting notes](pybullet_tools/ikfast/troubleshooting.md) if encountered error):

```shell
## sudo apt-get install python-dev
(cd pybullet_tools/ikfast/pr2; python setup.py)
```

If using Ubuntu, install TracIK for PR2 base, torso, and arm planning:

```shell
sudo apt-get install libeigen3-dev liborocos-kdl-dev libkdl-parser-dev liburdfdom-dev libnlopt-dev libnlopt-cxx-dev swig
pip install git+https://github.com/mjd3/tracikpy.git
```

Attempting to install tracikpy on MacOS:

```shell
brew install eigen orocos-kdl nlopt urdfdom
```

### Issue: `C++`

```shell
 xcrun: error: invalid active developer path (/Library/Developer/CommandLineTools), missing xcrun at: /Library/Developer/CommandLineTools/usr/bin/xcrun
```
solution, takes a while to install
```shell
xcode-select --install
```

## Issue: Eigen path not found

```shell
/usr/local/include/kdl/jacobian.hpp:26:10: fatal error: 'Eigen/Core' file not found
```

---
<!---
## Overview

Initially developed by Caelan for solving PDDLStream planning problems:
* `pybullet_tools`: basic Util functions for interfacing with pybullet and stream functions
* `databases`: saved grasps and other samples for faster debugging
* `images`: for visualization

Added by Yang for procedurally generating scenes and problems, solving partially-observable problems, and processing the data generated by planners for learning applications.
* `cogarch_tools`: for agents and processes planning and interacting with the world continuously 

---

## Tutorials - Procedural Scene Generation

The `/world_builder` directory includes functions for
* Building a `World` object, adding entities such as `Robot`, `Movable`, `Joint`, `Surface`, `Space`. For example, as shown in scripts `tutorials/test_assets.py`

```python
world = 
```
* Movable and articulated objects are usually sampled from assets of object categories, then randomly located in collision-free poses given supporting regions
* Procedurally generate scenes based on 
  * an `.svg` file that roughly lay out furniture types, locations; movable types and supporting regions
  * a function that 
* Initiating a scene from an `.svg` file that roughly lay out object types and locations

Run a flying panda gripper (feg) in kitchen simulation:
```shell
python tutorials/test_floating_gripper.py -t test_feg_pick
python tutorials/test_data_generation.py -c kitchen_full_feg.yaml
```

---

-->

## Trouble-Shooting 

See [trouble-shooting.md](trouble-shooting.md)

## Acknowledgements

* Developed based on Caelan Garrett's [pybullet_planning](https://github.com/caelan/pybullet-planning) utility functions for robotic motion planning, manipulation planning, and task and motion planning (TAMP).
* The development is partially performed during internship at NVIDIA Research, Seattle Robotics Lab.
