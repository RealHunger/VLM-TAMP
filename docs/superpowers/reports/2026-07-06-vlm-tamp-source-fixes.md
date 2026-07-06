# VLM-TAMP 源代码修复汇总

## 目的

本文集中记录 GPT55 chicken soup 长时程调试过程中发现的源代码问题、根因、修正内容和验证证据。它作为后续 review 或提交相关代码改动时的单一交接文档。

## 工作区边界

当前 worktree 有大量 dirty 和 untracked 文件。不要直接 stage 整个工作区。

本次修复的核心文件：

- `leap_tools/object_reducers.py`
- `leap_tools/test_object_reducers.py`
- `pybullet_tools/mobile_streams.py`
- `pybullet_tools/test_mobile_streams.py`
- `pybullet_tools/general_streams.py`
- `pybullet_tools/test_general_streams.py`
- `vlm_tools/llamp_agent.py`
- `vlm_tools/test_llamp_agent_sequence.py`
- `tutorials/replay_knob_one_step.py`
- `tutorials/diagnose_knob_pull_ik.py`
- `docs/superpowers/specs/2026-07-06-faucet-pull-design.md`
- `docs/superpowers/plans/2026-07-06-faucet-pull.md`
- `.superpowers/sdd/task-4-report.md`
- `.superpowers/sdd/task-4B-report.md`
- `.superpowers/sdd/task-5-report.md`

其他 dirty 文件可能来自更早的工作或更宽泛的诊断。提交前需要单独检查 diff，不能默认归入本次修复。

## 问题概述

目标任务是 VLM-TAMP GPT55 chicken soup 序列。流程前半段已经能推进到水龙头步骤，但规划器无法稳定完成 faucet handle 操作，随后又在 faucet close 上暴露出新的几何不可逆问题。

观察到的失败：

- 即使 stove knob `(6, 4)` 已修复，full policy 中 faucet 的 `openedjoint([(9, 3)])` 仍会失败。
- focused faucet replay 最初失败在 `inverse-kinematics-pull`；诊断显示 IR base 能被采样出来，但无法产生有效的 grasp/approach IK 候选。
- faucet open 修复后，full policy 会把 faucet 打开到约 `1.071`，随后 `closedjoint([(9, 3)])` 在 grasp IK 阶段失败，即使移除 obstacles 仍失败。
- 较小的几何开度，例如 `0.3`，可以反向关闭，但低于符号 open 阈值，不能满足 `IsOpenedPosition`。

## 根因

### 1. reduced-world planning 丢失相关 tuple object，并保留了无关动态事实

object reducer 需要保留目标中的 tuple object，例如 `(9, 3)` 和 `(6, 4)`。同时，knob-only reduced problem 不应该带入无关 movable dynamic facts，否则会扩大或扭曲 focused planning problem。

### 2. obstacle reduction 之后 reduced-world stream map 没有重新绑定

full LLAMP policy 会创建更小的 PDDLStream problem。部分 stream function 仍使用旧 world/obstacle 上下文，而不是 reduced world，导致 focused replay 和 full policy 行为不一致。

### 3. faucet manipulation 缺少目标专用几何支持

已有的稳定路径主要覆盖 stove knob，没有覆盖 faucet。faucet 需要自己的 base candidates、support/contact filtering、handle grasp variants 和 grasp ordering。通用 handle grasps 与 base sampling 会先尝试低质量候选，full policy 经常在到达可行 grasp/base 组合之前耗尽较小的 evaluation budget。

### 4. faucet open 采样到了不可逆目标

默认 open-position sampling 可能选择接近 `1.071` 的大开度。它满足 `IsOpenedPosition`，但对当前 PR2/faucet 几何来说无法从该角度关闭。证据：

- `1.071 -> 0.0` 即使 `obstacles=[]` 也失败。
- `0.45 -> 0.0` 在 IK/grasp 阶段失败。
- `0.4 -> 0.0` 成功，并且高于符号 open 阈值。
- open 阈值约为 `1.571 * 0.25 = 0.393`。

因此 faucet open 应采样 `0.4` 附近的小开度，而不是远端 open 默认值。

### 5. progress summary 可能把已有 plan 的成功项显示为 `STARTED`

当 time-log entry 的 `status == STARTED` 但 `plan` 非空时，summary 展示可能低估进度。这个问题不影响规划本身，但会干扰验证和结果阅读。

## 源代码修正

### `leap_tools/object_reducers.py`

- 从 goals 中保留 tuple goal objects，包括 `(9, 3)` 和 `(6, 4)` 这类 joint tuple。
- 对 joint-only reduced problems 过滤无关 movable dynamic facts。
- 在 `leap_tools/test_object_reducers.py` 中加入回归测试。

### `vlm_tools/llamp_agent.py`

- 在 `_update_obstacles_in_stream_map()` 中为 reduced world 重新绑定 stream map：
  - `inverse-reachability`
  - `inverse-kinematics`
  - `inverse-reachability-rel`
  - `inverse-kinematics-rel`
  - pull streams
- 重新绑定时保留已配置的 IR 选项，例如 `use_learned_ir` 和 `ir_max_attempts`。
- 对 summary 行做规范化：`STARTED` 且 `plan` 非空时应显示为 solved。
- 在 `vlm_tools/test_llamp_agent_sequence.py` 中加入回归测试。

### `pybullet_tools/mobile_streams.py`

- 增加 `_is_faucet_knob()` 用于识别 faucet knob。
- 增加 `_priority_faucet_knob_bases()`，提供 faucet 专用 priority bases。
- 在 `get_handle_motion_contact_bodies()` 中加入 faucet support/contact 处理，避免把有意的 faucet/basin 接触当成全局 collision relaxation。
- 将 `PULL_STREAM_DIAG` 扩展到 faucet `(9, 3)`，同时保持环境变量门控。
- 修复保持窄作用域：没有全局增加 planner budget，也没有大范围关闭 collision。
- 在 `pybullet_tools/test_mobile_streams.py` 中加入回归测试。

### `pybullet_tools/general_streams.py`

- 增加 faucet-specific handle grasp variants。
- 优先使用 replay 验证过的 faucet grasp：
  - `(0.011, 0.074, 0.0, PI, 0, PI/2)`
- 保留 close reverse grasp：
  - `(0.011, 0.074, 0.0, 0, 0, -PI/2)`
- 让 faucet grasp prioritization 同时支持 flat 6-tuples 和 pose tuple values。
- 将 faucet handle grasp 输出限制为一小组目标候选，避免 full policy 先消耗预算在低价值 grasp 上。
- 增加 faucet-only reversible open-position sampling：
  - 正向 limit faucet 优先 `0.4`，只采样 `0.4-0.42`。
  - 负向 reversed limit 使用对称的负区间。
- 在 `pybullet_tools/test_general_streams.py` 中加入回归测试。

### `tutorials/replay_knob_one_step.py`

- focused replay 支持 `--predicate openedjoint|closedjoint`。
- 增加 `--joint_position`，用于从指定 faucet 开度复现 close。这个参数是必要的，因为单独加载 `agent_state_5.pkl` 时 faucet 已经处于 closed 状态。
- 保持默认行为兼容原有 knob replay 用法。

### `tutorials/diagnose_knob_pull_ik.py`

- 增加 `--joint_body`、`--joint_index` 和 `--joint_position`，用于 faucet-specific pull IK probes。
- 用它区分 collision failure 和纯 grasp/IK reachability failure。

## 关键证据

grasp ordering 之后的 focused faucet open：

```text
/tmp/opencode/replay_faucet_after_pose_tuple_priority.log
PULL_DIAG start obj=(9, 3) ... grasp=(0.011, 0.074, 0.0, 3.142, 0.0, 1.571)
ONE_STEP_PLAN_LEN 2
```

实际采样开度下的 focused faucet close：

```text
/home/reality-hunger/.local/share/opencode/tool-output/tool_f37a3b3120011JDFJ66hIQ9X11
PULL_DIAG start obj=(9, 3) pst1=0.405 pst2=0.0
PULL_DIAG yield obj=(9, 3) pst2=0.0 base=(1.08, 5.7, 0.767, 1.6)
Solved: True
```

最新 memory-backed full policy 验证：

```text
/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/experiments/verify_full_faucet_open0405_close_260706/260706_213952_vlm-tamp
```

相关 CSV 行：

```csv
5,"['openedjoint([(9, 3)])']",5,solved,2,33.6029,1 (joint: 1),object-related,"5_a_['openedjoint', 'faucet handle']",agent_state_4.pkl
6,"['closedjoint([(9, 3)])']",6,solved,2,1.096,1 (joint: 1),object-related,"6_a_['closedjoint', 'faucet handle']",agent_state_5.pkl
18,"['openedjoint([(6, 4)])']",17,solved,2,1.095,1 (joint: 1),object-related,"17_a_['openedjoint', 'stove knob on the right']",agent_state_17.pkl
,1.0 (17 / 17),1.0 (17 / 17),1.0 (16 / 16),41,193.46,191.19 (effective time),2.27 (wasted time),,
```

最新单元测试证据：

```text
python -m unittest pybullet_tools.test_general_streams.HandleGraspTests pybullet_tools.test_mobile_streams pybullet_tools.test_general_streams leap_tools.test_object_reducers vlm_tools.test_llamp_agent_sequence
Ran 59 tests in 0.033s
OK
```

## 复现验证命令

单元测试从 `/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace` 运行：

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate kitchen && PYTHONPATH="/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds/pddlstream:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/pybullet_planning:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/lisdf:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds/pybullet_planning/motion" python -m unittest pybullet_tools.test_general_streams.HandleGraspTests pybullet_tools.test_mobile_streams pybullet_tools.test_general_streams leap_tools.test_object_reducers vlm_tools.test_llamp_agent_sequence
```

memory-backed full policy 从 `/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/pybullet_planning` 运行：

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate kitchen && python tutorials/test_vlm_tamp.py --problem test_kitchen_chicken_soup --api_class_name gpt55 --load_llm_memory /home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/experiments/verify_full_no_stream_diag_260706/260706_023820_vlm-tamp --exp_subdir verify_full_faucet_open0405_close_260706
```

## 提交分组建议

如果要准备 review，建议至少拆成两个 commit：

1. Reducer 与 LLAMP stream-map 正确性：
   - `leap_tools/object_reducers.py`
   - `leap_tools/test_object_reducers.py`
   - `vlm_tools/llamp_agent.py`
   - `vlm_tools/test_llamp_agent_sequence.py`

2. Faucet pull 几何支持与诊断：
   - `pybullet_tools/mobile_streams.py`
   - `pybullet_tools/test_mobile_streams.py`
   - `pybullet_tools/general_streams.py`
   - `pybullet_tools/test_general_streams.py`
   - `tutorials/replay_knob_one_step.py`
   - `tutorials/diagnose_knob_pull_ik.py`
   - 相关 faucet docs/reports

除非已经逐个 review diff 并确认属于本次范围，否则不要包含其他 dirty 文件。

## 当前状态

源代码层面的 faucet open/close blocker 已在 memory-backed full policy run 中验证解决。最终序列达到 `1.0 (17 / 17)` 子目标，相关回归测试通过。
