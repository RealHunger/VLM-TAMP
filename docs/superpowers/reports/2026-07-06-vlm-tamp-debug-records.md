# VLM-TAMP GPT55 Chicken Soup Debug 全记录

## 这份文档在说什么

这是本轮 debug 的完整记录。它不是只讲最后改了哪几行代码，而是按时间顺序说明：我们卡在哪里、怎么判断问题、试过哪些方向、哪些方向被排除、最后怎么跑通。

一句话总结：

> 我们把 GPT55 chicken soup 的长任务从“中途各种失败”推进到 memory-backed full policy 完整通过，最终 `17 / 17` 个子目标完成。

最终通过的实验目录：

```text
/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/experiments/verify_full_faucet_open0405_close_260706/260706_213952_vlm-tamp
```

更偏源码 review 的精简版在这里：

```text
docs/superpowers/reports/2026-07-06-vlm-tamp-source-fixes.md
```

## 最后结果

最终 full policy 跑通了整条 chicken soup 序列。

关键 CSV 结果：

```csv
5,"['openedjoint([(9, 3)])']",5,solved,2,33.6029,1 (joint: 1),object-related,"5_a_['openedjoint', 'faucet handle']",agent_state_4.pkl
6,"['closedjoint([(9, 3)])']",6,solved,2,1.096,1 (joint: 1),object-related,"6_a_['closedjoint', 'faucet handle']",agent_state_5.pkl
18,"['openedjoint([(6, 4)])']",17,solved,2,1.095,1 (joint: 1),object-related,"17_a_['openedjoint', 'stove knob on the right']",agent_state_17.pkl
,1.0 (17 / 17),1.0 (17 / 17),1.0 (16 / 16),41,193.46,191.19 (effective time),2.27 (wasted time),,
```

最终测试也通过：

```text
Ran 59 tests in 0.033s
OK
```

## 环境和命令上的坑

这些不是 planner 本身的问题，但不处理会让验证结果不可信。

- 要先进入环境：`source ~/miniconda3/etc/profile.d/conda.sh && conda activate kitchen`
- `pytest` 不可用，用 `python -m unittest ...`。
- VLM/probe 脚本最好从 workspace root 跑，避免 Python import 被 `pybullet_planning/examples` 误导。
- `--load_llm_memory` 要传 run directory，不是 `llm_memory.json` 文件。
- `--scene_only` 只建场景，不跑完整策略，所以不能用它验证 full policy。
- `--viewer false` 这类写法不对；false 就不要传这个 flag。
- 旧命令里的 `--domain_name`、`--world_builder_name`、`--llamp_api_name` 已经过时，要改成：

```bash
python tutorials/test_vlm_tamp.py --problem test_kitchen_chicken_soup --api_class_name gpt55 ...
```

## 我们最开始要解决什么

任务是让 GPT55 生成的 chicken soup 长计划真的跑完。计划大概是：

1. 打开冰箱，拿鸡腿，放进锅里。
2. 把锅放到水槽。
3. 打开水龙头，再关掉水龙头。
4. 把锅放回右侧台面。
5. 拿盐、撒盐、放回柜子。
6. 拿胡椒、撒胡椒、放回柜子。
7. 把锅放到右侧炉灶。
8. 拿锅盖，盖上锅。
9. 打开右侧炉灶旋钮。

真正难的是中间这些机器人动作不是纯符号规划，必须找到可达的 base、grasp、IK 和 collision-free motion。

## 第 1 阶段：先让系统能跑起来

一开始有一批基础问题会让任务还没到真正的几何难点就中断。

处理过的问题包括：

- GPT55 provider 接入和 scene-only 验证。
- grounding/body-ref 错误。
- logging import 过期。
- `focused.py` traceback。
- `max_evaluation_plans` 参数泄漏。
- marker `KeyError`。
- 已经 `picked` 的对象又被要求 pick。
- `goal_sequence` 为空。
- `sprinkledto` 里 pot body 被错误当成 movable。
- `Containble` 拼写错误。
- legacy path 和 headless verbose blocking。

这些修完后，系统才能稳定跑到更后面的真实失败点。

## 第 2 阶段：做 focused replay 工具

full policy 太慢，而且有随机性。直接反复跑完整任务很难定位问题。所以我们加了 focused replay 和诊断脚本，只复现某一个子目标。

关键脚本：

- `tutorials/replay_knob_one_step.py`
- `tutorials/diagnose_knob_pull_ik.py`
- `tutorials/diagnose_lid_pick_ik.py`
- `tutorials/diagnose_lid_place_ik.py`
- `tutorials/diagnose_salt_sprinkle_ik.py`

`replay_knob_one_step.py` 后来支持了：

- `--predicate openedjoint|closedjoint`
- `--joint_position <angle>`

这个很重要，因为 faucet close 的失败必须强制设置 faucet 当前角度才能复现。单独加载某些 saved state 时，faucet 已经又变成 closed 了。

## 第 3 阶段：reducer 和 stream map 问题

我们发现 focused replay 和 full policy 有时行为不一致。原因之一是 reduced world 的对象和 stream 没处理对。

简单说：

- full policy 会把大世界缩小成只和当前目标相关的小世界。
- knob/faucet 这种目标是 tuple object，比如 `(9, 3)`、`(6, 4)`。
- reducer 需要保留这些 tuple object。
- 小世界里的 stream 也要重新绑定到小世界的 obstacles，否则还会用旧上下文。

修正：

- `leap_tools/object_reducers.py` 保留 tuple goal objects，过滤无关 movable facts。
- `vlm_tools/llamp_agent.py` 在 reduced world 里重新绑定 IR/IK/pull streams。

验证：

```text
python -m unittest leap_tools.test_object_reducers vlm_tools.test_llamp_agent_sequence
Ran 7 tests ... OK
```

## 第 4 阶段：stove knob 修好

右侧炉灶旋钮 `(6, 4)` 曾经失败。它的问题和 faucet 类似，都是机器人要找到合适站位、抓法和旋转动作。

修正方向：

- 给 stove knob 加更靠谱的 priority base。
- 允许 knob 转动时和合理的接触物保持接触，不把这种有意接触当作失败。
- 保证 reduced-world stream map 正确。

结果：

- `openedjoint([(6, 4)])` 在 full policy 里 solved。
- 后续 faucet 修复都要保证不破坏这个结果。

## 第 5 阶段：summary 显示也有一个小问题

有些 time log 里明明有 plan，但状态显示还是 `STARTED`。这不是规划失败，但会误导我们看结果。

修正：

- 如果 `status == STARTED` 但 `plan` 非空，就按 solved 显示。

这只是报告修正，不是 motion planning 修正。

## 第 6 阶段：faucet open 最初失败

接下来主要 blocker 是水龙头 `(9, 3)`。

现象：

- full policy 里的 `openedjoint([(9, 3)])` failed。
- focused replay 里 `inverse-kinematics-pull` 找不到可行动作。

我们先怀疑 base 和 collision，所以给 faucet 加了 priority bases 和 contact filtering。但诊断显示：

- 有些 base 会撞冰箱或洗碗机。
- 有些 sink-side base 可以 yield 出来。
- 但最后还是没有可行 pull candidate。
- 更细的日志显示失败点是 `grasp_conf=None`，而且当时 `obstacles=[]`。

这说明：不是单纯 collision 问题，而是 faucet 的抓法和 approach IK 不对。

相关记录：

```text
.superpowers/sdd/task-4-report.md
```

## 第 7 阶段：faucet 抓法和 approach 修好

我们后来给 faucet 单独加了 handle grasp variants，并把已验证可行的抓法排到前面。

关键抓法：

```text
(0.011, 0.074, 0.0, PI, 0, PI/2)
```

同时做了几件事：

- faucet 用自己的 grasp ordering。
- faucet 的 handle grasp 输出限制成一小组高价值候选。
- faucet 用更短的 approach distance。
- `PULL_STREAM_DIAG` 能打印 faucet 的 pull 诊断。

focused faucet open 后来成功了：

```text
/tmp/opencode/replay_faucet_after_pose_tuple_priority.log
PULL_DIAG start obj=(9, 3) ... grasp=(0.011, 0.074, 0.0, 3.142, 0.0, 1.571)
ONE_STEP_PLAN_LEN 2
```

相关记录：

```text
.superpowers/sdd/task-4B-report.md
```

## 第 8 阶段：faucet open 过了，但 close 又失败

faucet open 修好后，full policy 能打开 faucet，但关 faucet 失败。

当时的 full run：

```text
/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/experiments/verify_full_faucet_pose_tuple_priority_260706/260706_141721_vlm-tamp
```

现象：

```text
openedjoint([(9, 3)]) solved
closedjoint([(9, 3)]) failed
closedjoint([(9, 3)]) failed
closedjoint([(9, 3)]) failed
```

我们排除了两个错误方向：

- 不是 fact sync 问题。close planning problem 里已经有 `AtPosition (9,3)=1.071` 和 `IsOpenedPosition`。
- 不是 collision filtering 问题。即使 `obstacles=[]`，从 `1.071 -> 0.0` 还是失败。

所以问题变成：faucet 被打开得太大，机器人几何上关不回去。

## 第 9 阶段：找能打开又能关回去的角度

我们测试了几个 faucet 当前角度：

- `1.071 -> 0.0`：失败。
- `0.45 -> 0.0`：失败。
- `0.3 -> 0.0`：几何上能关，但系统不认为它是 open。
- `0.317 -> 0.0`：也被系统认为是 closed。
- `0.4 -> 0.0`：成功，而且系统认为它是 open。
- `0.405 -> 0.0`：成功。

为什么 `0.3` 不行？

代码里 open threshold 大概是：

```text
1.571 * 0.25 = 0.393
```

所以 faucet 至少要打开到约 `0.393` 才算 open。`0.4` 刚好高于这个阈值，又还能关回来。

最终结论：

> faucet open 不应该采样到很大的角度。它应该采样到 `0.4-0.42` 这个小范围。

## 第 10 阶段：限制 faucet open 采样

我们在 `pybullet_tools/general_streams.py` 里加了 faucet-only reversible open-position sampling。

效果：

- 正向 faucet 优先采样 `0.4`，范围 `0.4-0.42`。
- 负向 reversed limit 用对称负区间。
- 只影响 faucet，不改变全局 joint sampling。

focused close 验证成功：

```text
/home/reality-hunger/.local/share/opencode/tool-output/tool_f37a3b3120011JDFJ66hIQ9X11
PULL_DIAG start obj=(9, 3) pst1=0.405 pst2=0.0
PULL_DIAG yield obj=(9, 3) pst2=0.0 base=(1.08, 5.7, 0.767, 1.6)
Solved: True
```

## 第 11 阶段：最终 full policy 跑通

最终命令：

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate kitchen && python tutorials/test_vlm_tamp.py --problem test_kitchen_chicken_soup --api_class_name gpt55 --load_llm_memory /home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/experiments/verify_full_no_stream_diag_260706/260706_023820_vlm-tamp --exp_subdir verify_full_faucet_open0405_close_260706
```

结果：

- 冰箱、鸡腿、锅、水槽步骤通过。
- faucet open solved。
- faucet close solved。
- 锅放回台面通过。
- 盐和胡椒相关步骤通过。
- 锅放到炉灶通过。
- 锅盖步骤通过。
- stove knob open solved。
- 总结是 `17 / 17`。

## 测试记录

最终测试命令：

```bash
source "$HOME/miniconda3/etc/profile.d/conda.sh" && conda activate kitchen && PYTHONPATH="/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds/pddlstream:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/pybullet_planning:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/lisdf:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds/pybullet_planning/motion" python -m unittest pybullet_tools.test_general_streams.HandleGraspTests pybullet_tools.test_mobile_streams pybullet_tools.test_general_streams leap_tools.test_object_reducers vlm_tools.test_llamp_agent_sequence
```

结果：

```text
Ran 59 tests in 0.033s
OK
```

## 重要日志索引

最终成功：

- final full policy output: `/home/reality-hunger/.local/share/opencode/tool-output/tool_f37a82fd80014u22NPbc044vdu`
- final full policy experiment: `/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/experiments/verify_full_faucet_open0405_close_260706/260706_213952_vlm-tamp`
- final CSV: `/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/experiments/verify_full_faucet_open0405_close_260706/260706_213952_vlm-tamp/vlm-tamp.csv`

faucet focused 成功：

- faucet open: `/tmp/opencode/replay_faucet_after_pose_tuple_priority.log`
- faucet close `0.405 -> 0.0`: `/home/reality-hunger/.local/share/opencode/tool-output/tool_f37a3b3120011JDFJ66hIQ9X11`

历史失败和诊断：

- clean full failure before reducer fix: `/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/experiments/verify_full_no_stream_diag_260706/260706_023820_vlm-tamp`
- earlier faucet full attempt: `/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/experiments/verify_full_faucet_pull_fix_260706/260706_134601_vlm-tamp`
- full policy before faucet close fix: `/tmp/opencode/full_policy_faucet_pose_tuple_priority.log`
- full policy experiment before close fix: `/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/experiments/verify_full_faucet_pose_tuple_priority_260706/260706_141721_vlm-tamp`
- full-open close no-collision diagnosis: `/home/reality-hunger/.local/share/opencode/tool-output/tool_f37959f490010bFaIUmB4l860X`
- close `0.45 -> 0.0`: `/home/reality-hunger/.local/share/opencode/tool-output/tool_f37a14eb0001HApsx9DFS0YgXI`
- close `0.4 -> 0.0`: `/home/reality-hunger/.local/share/opencode/tool-output/tool_f37a1f09c001nyASkiR47VEgzY`

## 改过/新增的主要文件

核心源码：

- `leap_tools/object_reducers.py`
- `vlm_tools/llamp_agent.py`
- `pybullet_tools/mobile_streams.py`
- `pybullet_tools/general_streams.py`

测试：

- `leap_tools/test_object_reducers.py`
- `vlm_tools/test_llamp_agent_sequence.py`
- `pybullet_tools/test_mobile_streams.py`
- `pybullet_tools/test_general_streams.py`

诊断脚本：

- `tutorials/replay_knob_one_step.py`
- `tutorials/diagnose_knob_pull_ik.py`
- `tutorials/diagnose_lid_pick_ik.py`
- `tutorials/diagnose_lid_place_ik.py`
- `tutorials/diagnose_salt_sprinkle_ik.py`

文档和报告：

- `docs/superpowers/specs/2026-07-06-faucet-pull-design.md`
- `docs/superpowers/plans/2026-07-06-faucet-pull.md`
- `.superpowers/sdd/task-4-report.md`
- `.superpowers/sdd/task-4B-report.md`
- `.superpowers/sdd/task-5-report.md`
- `docs/superpowers/reports/2026-07-06-vlm-tamp-source-fixes.md`

## 现在还要注意什么

- 当前 worktree 有很多 dirty/untracked 文件，不能直接全量提交。
- 如果要提交，先按源码汇总文档里的分组逐个 inspect diff。
- 以后再改这个任务，优先 focused replay 验证单个失败点，再跑 full policy。
- 不要用全局加预算或全局关 collision 来掩盖这类问题；这次能跑通靠的是更准确的目标几何和采样。
