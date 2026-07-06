# VLM-TAMP 调试与改动汇报

## 一、背景

我主要复现的是论文项目 `Guiding Long-Horizon Task and Motion Planning with Vision Language Models` 里的 open-world kitchen 示例，也就是让 VLM 把 `make chicken soup` 拆成一串中间子目标，再交给 PDDLStream 做任务与运动规划。

原 README 推荐的运行方式是：

```bash
python tutorials/test_vlm_tamp.py \
  --open_goal "make chicken soup" \
  --exp_subdir "test_fun" \
  --planning_mode "sequence"
```

我的主要变化是把原本的 GPT4/GPT4V API 换成了 `gpt55`。除了这个模型接口替换，整体运行方式基本还是按 README 的 VLM-TAMP 入口来跑。

## 二、为什么换模型后会暴露更多问题

这里不是简单“换个 LLM 名字”。VLM 在这个系统里负责生成中间子目标，例如：

```text
pick chicken leg
put chicken leg in pot
move pot to sink
turn on faucet
put pot on stove
sprinkle salt
turn on stove knob
```

不同模型即使用同一个 prompt，也可能生成不同的步骤顺序。步骤顺序一变，机器人面对的几何状态也会变。

例如 GPT55 有时会先把锅放进水槽，再打开水龙头。这样水龙头附近会多一个锅作为障碍物，IK、碰撞检测、base sampling 都会变难。原项目 README 示例并不保证所有 fresh LLM 生成的计划都能稳定跑通。

我另外重新 clone 了一个干净原版，只加 GPT55 接口，严格按 README 方式跑。结果原版在第 4 个子目标“把锅放到水槽”附近就失败，后面还触发了 `KeyError: 3`。这说明问题不是我当前改动引入的，而是原始研究代码在当前环境和 GPT55 fresh 输出下本来就不够稳。

## 三、我做的主要工作

## 1. 接入 GPT55

原版只支持 GPT/OpenAI 和 Claude 风格接口。我新增了 GPT55 provider：

```text
GPT55Api
GPT55PlanningApi
--api_class_name gpt55
```

作用是让原来的 VLM-TAMP 流程可以通过 `--api_class_name gpt55` 调用 GPT55，同时复用原来的 prompt 和解析逻辑。

这里要注意：我没有重写 prompt，GPT55 吃的仍然是原来的 `prompts_gpt4v.py` 模板。变化主要来自模型输出不同，而不是 prompt 文本变了。

## 2. 修复对象表示混用问题

原代码里对象有多种表示方式：

```text
数字 body id，例如 5
关节 tuple，例如 (6, 4)
带名字的字符串，例如 "(6, 4)|oven#1::knob_joint_2"
```

在简单步骤里这不一定会出错，但跑到长序列、reduced world、关节目标时，容易出现目标对象被过滤掉或匹配不上的问题。

我修了 object reducer 相关逻辑，让它能正确保留 tuple 类型的目标对象，尤其是炉灶旋钮、水龙头把手这类 joint object。否则 planner 会以为目标关节不重要，把它从 reduced problem 里删掉，后面就没法规划开关动作。

## 3. 修复 reduced world 下 stream 没更新的问题

LLAMP 每次规划子目标时会构造一个 reduced world，只保留和当前目标有关的对象。问题是原来的 stream map 有些地方还引用旧 world，导致 planner 看到的对象和 stream 实际用的对象不一致。

我在 `llamp_agent.py` 里重新绑定了关键 stream：

```text
inverse-reachability
inverse-kinematics
inverse-reachability-rel
inverse-kinematics-rel
inverse-kinematics-pull
inverse-kinematics-pull-with-link
```

通俗讲，就是每次换一个小规划问题，都要让 IK、IR、pull stream 使用当前这份小世界，而不是用旧环境里的对象。

## 4. 修复目标物体被误当成障碍物的问题

在调试过程中还遇到过一类碰撞检测问题：当前动作的目标物体，有时也会被放进 `obstacles` 列表里。

例如当前动作是：

```text
pick(chicken leg)
```

这时鸡腿是机器人要抓的目标，机械手必须允许接触鸡腿。如果它同时被当成普通障碍物，就会变成：

```text
机器人要抓鸡腿
但碰撞检测又要求机器人不能碰鸡腿
```

这样 IK 或 approach path 就会失败。

类似地，在：

```text
in(chicken leg, pot body)
```

里面，鸡腿要被放进锅里，鸡腿和锅体/锅底发生合理接触是动作本身的一部分。如果目标容器或目标支撑面被当成完全禁止接触的障碍，也会导致规划失败。

我做的修复不是关闭碰撞检测，而是按动作语义区分“合理接触”和“真实碰撞”：

```text
当前要抓的目标物体，不作为普通障碍物
当前要放置到的目标支撑面，允许发生合理接触
把手/旋钮操作时，允许抓手接触 handle 所属 body
其他无关物体仍然参与碰撞检测
```

通俗地说，就是把碰撞检测从“所有东西都不能碰”改成“该接触的可以接触，不该碰的仍然不能碰”。

## 5. 修复水龙头和炉灶旋钮的拉动/旋转规划

水龙头和炉灶旋钮不是普通 pick/place，它们需要 `pull` 或 `turn` 类动作。原版在这些动作上很容易失败，主要原因包括：

```text
机器人 base 采样不到合适位置
抓手姿态不适合把手
碰撞检测把合理接触也当成碰撞
水龙头打开角度太大，导致后续关不上
```

我做了几个针对性修复：

```text
给 faucet/stove knob 增加优先 base pose
给 faucet handle 增加更合适的 grasp/approach variant
过滤掉把手操作中允许接触的 body，避免合理接触被误判为碰撞
限制 faucet open position 到 0.4-0.42 左右，避免开太大后关不回来
```

这里不是全局放松碰撞，而是只对水龙头/旋钮这类动作做局部处理。这样风险比较小。

## 6. 修复执行状态和成功统计问题

有些子目标已经有 plan，但 summary 里会显示成 `STARTED`，看起来像没完成。我修了 progress summary 的状态归一化逻辑：

```text
如果 status 是 STARTED，但 plan 非空且不是 FAILED，就按 SOLVED 处理
```

这主要是为了让日志和 CSV 更真实地反映 planner 实际执行状态。

## 7. 增加 focused replay 和诊断脚本

长序列每次完整跑都很贵，而且 GPT 输出有随机性。所以我写了多个 focused diagnostic/replay 脚本，用来从某个保存的 agent state 直接复现单个失败子目标。

例如：

```text
replay_knob_one_step.py
```

这些脚本的作用是把“大任务失败”拆成“小问题复现”。比如只测“当前状态下能不能打开炉灶旋钮”，这样能更快定位到底是 grounding、reducer、IK、base sampling 还是 collision 的问题。

## 四、验证结果

最终我用 memory-backed 的 17-step chicken soup 序列完成了一次完整验证：

```text
结果：1.0 (17 / 17)
```

关键子目标都通过了：

```text
打开/关闭水龙头
移动锅到水槽和炉灶
撒盐、撒胡椒
处理锅盖
打开/关闭炉灶旋钮
```

同时跑过相关回归测试：

```text
Ran 59 tests
OK
```

这说明当前修复版对于已验证的 memory-backed 任务序列是能完整跑通的。

## 五、和原版对比

原版加最小 GPT55 接口后，按 README fresh run：

```text
第 1 步 fridge door already
第 2 步 picked chicken leg 先失败后成功
第 3 步 in chicken leg pot body 先失败后成功
第 4 步 on pot body kitchen sink 失败
随后触发 KeyError: 3
```

当前修复版至少能继续推进到更后面的水龙头、柜门、锅盖、炉灶旋钮等复杂步骤，并且 memory-backed 验证达到 `17 / 17`。

所以可以总结为：

```text
原版不是稳定可用的完整系统，更像论文研究代码。
我的工作是在保持原始 VLM-TAMP 框架的基础上，接入 GPT55，并修复一批长序列规划中暴露出的 grounding、reducer、IK、pull stream 和日志统计问题。
```

## 六、目前还没有完全解决的问题

fresh GPT55 仍然可能生成新的子目标序列，例如：

```text
先把锅放进水槽，再开水龙头
额外要求关冰箱门
额外要求关柜门
重新拿锅盖并放到锅上
```

这些会触发新的几何分支。当前修复版比原版稳定很多，但 fresh VLM 输出本身不受控，所以还不能保证“任意 GPT55 生成序列都 100% 成功”。

下一步如果继续做，可以考虑两条路线：

```text
路线 1：约束 GPT55 prompt，让它生成更接近已验证 memory-backed 的子目标顺序
路线 2：继续补 planner，使它能处理更多 fresh 生成的复杂几何状态
```

我目前更倾向先做路线 1，因为这更符合 VLM-TAMP 的设计：VLM 负责给 planner 一个合理、可执行的任务分解，而不是让 planner 被迫处理所有任意顺序。
