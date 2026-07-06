# Faucet Pull Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `openedjoint([(9, 3)])` for the faucet handle solve reliably without broad planner budget increases, global collision relaxation, or hiding the failure in summary metrics.

**Architecture:** Add a narrow faucet-specific path alongside the existing stove knob path in `pybullet_tools/mobile_streams.py`. The planner should get deterministic priority base candidates for the faucet and only ignore intentional faucet/basin contact during pull/ungrasp collision checks.

**Tech Stack:** Python 3.8, `unittest`, PyBullet planning utilities, PDDLStream, existing VLM-TAMP diagnostic scripts.

## Global Constraints

- Use `source ~/miniconda3/etc/profile.d/conda.sh && conda activate kitchen` for commands.
- VLM-related tests and scripts should run from `/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace` to avoid `examples.pybullet` import shadowing.
- Planner-focused commands can run from `/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/pybullet_planning` only if they do not import `vlm_tools` through the shadowed path.
- Use `unittest`; `pytest` is unavailable in the `kitchen` environment.
- Do not globally increase `ir_max_attempts`, `max_evaluation_plans`, or planning timeouts as the primary fix.
- Do not globally relax pull collisions.
- Do not mark faucet failures as successful at the summary layer.
- Preserve unrelated dirty worktree changes.

---

## File Structure

- Modify `pybullet_tools/mobile_streams.py`: faucet priority base helper, faucet classification helper, `PULL_STREAM_DIAG` gating for faucet, faucet contact filtering.
- Modify `pybullet_tools/test_mobile_streams.py`: unit tests for faucet helper behavior and contact bodies.
- Optionally modify `tutorials/replay_knob_one_step.py`: only if focused replay needs clearer faucet labels; keep its current generic `--joint_body` and `--joint_index` interface.

---

### Task 1: Add Faucet Unit Tests First

**Files:**
- Modify: `/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/pybullet_planning/pybullet_tools/test_mobile_streams.py`
- Test: `/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/pybullet_planning/pybullet_tools/test_mobile_streams.py`

**Interfaces:**
- Consumes: existing fake world/test helpers in `pybullet_tools/test_mobile_streams.py`.
- Produces: failing tests for `_priority_faucet_knob_bases()` and faucet contact bodies.

- [ ] **Step 1: Import the faucet helper in the test file**

Update the existing import at the top of `pybullet_tools/test_mobile_streams.py` to include `_priority_faucet_knob_bases`:

```python
from pybullet_tools.mobile_streams import get_ik_fn_old, sample_bconf, _priority_salt_sprinkle_bases, \
    _priority_salt_cabinet_bases, _priority_condiment_sprinkle_bases, _priority_condiment_cabinet_bases, \
    _priority_condiment_cabinet_pick_bases, _priority_pot_stove_bases, \
    _priority_pot_counter_pick_bases, _priority_lid_stove_pick_bases, _priority_stove_knob_bases, \
    _priority_faucet_knob_bases, _priority_lid_braiser_place_bases, get_handle_motion_contact_bodies, \
    get_target_support_bodies
```

- [ ] **Step 2: Add failing faucet priority base test**

Add this test near `test_priority_stove_knob_bases_only_for_right_knob`:

```python
    def test_priority_faucet_knob_bases_only_for_faucet_knob(self):
        world = FakeWorld()
        world.BODY_TO_OBJECT[(9, 3)] = FakeBodyInfo(categories=['knob', 'joint', 'faucet'])
        world.BODY_TO_OBJECT[(6, 4)] = FakeBodyInfo(categories=['knob', 'joint'])
        pose = FakePose(value=((0.265, 5.593, 1.012), (0, 0, 0, 1)))

        bases = _priority_faucet_knob_bases(
            world, (9, 3), pose, ((0.265, 5.45, 1.02), (0, 0, 0, 1)),
            [1, 3, 0, -3.2], [5, 10, 3, 3.2])

        self.assertGreaterEqual(len(bases), 3)
        self.assertEqual(bases[0], (1.0, 5.6, 0.58, 0.0))
        self.assertTrue(all(0 <= base[2] <= 3 for base in bases))
        self.assertEqual(_priority_faucet_knob_bases(
            world, (6, 4), pose, ((0.265, 5.45, 1.02), (0, 0, 0, 1)),
            [1, 3, 0, -3.2], [5, 10, 3, 3.2]), [])
```

- [ ] **Step 3: Add failing faucet contact body test**

Add this test near the existing handle motion contact body tests:

```python
    def test_handle_motion_contact_bodies_include_faucet_support_chain(self):
        world = FakeWorld()
        world.BODY_TO_OBJECT[8] = FakeBodyInfo(categories=['basin'])
        world.BODY_TO_OBJECT[9] = FakeBodyInfo(categories=['faucet'])
        world.BODY_TO_OBJECT[(8, None, 2)] = FakeBodyInfo(categories=['surface'])
        world.BODY_TO_OBJECT[12] = FakeBodyInfo(supporting_surface=FakeSurface((3, None, 0)))
        world.BODY_TO_OBJECT[(9, 3)] = FakeBodyInfo(categories=['knob', 'joint', 'faucet'])

        contacts = get_handle_motion_contact_bodies(world, (9, 3))

        self.assertIn(8, contacts)
        self.assertIn(9, contacts)
        self.assertIn((8, None, 2), contacts)
        self.assertNotIn(12, contacts)
```

- [ ] **Step 4: Run tests to verify they fail for the intended reason**

Run from workspace root:

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate kitchen && PYTHONPATH="/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds/pddlstream:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/pybullet_planning:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/lisdf:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds/pybullet_planning/motion" python -m unittest pybullet_tools.test_mobile_streams.TestMobileStreams.test_priority_faucet_knob_bases_only_for_faucet_knob pybullet_tools.test_mobile_streams.TestMobileStreams.test_handle_motion_contact_bodies_include_faucet_support_chain
```

Expected: fail because `_priority_faucet_knob_bases` does not exist or because contacts do not include faucet support chain yet.

---

### Task 2: Implement Faucet Priority Bases And Contact Filtering

**Files:**
- Modify: `/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/pybullet_planning/pybullet_tools/mobile_streams.py`
- Test: `/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/pybullet_planning/pybullet_tools/test_mobile_streams.py`

**Interfaces:**
- Consumes: tests from Task 1.
- Produces: `_priority_faucet_knob_bases(world, obj, pose, gripper_pose, lower_limits, upper_limits) -> list[tuple[float, float, float, float]]` and faucet-aware `get_handle_motion_contact_bodies(world, obj)`.

- [ ] **Step 1: Add faucet classification helper**

Add this helper above `_priority_stove_knob_bases()` in `mobile_streams.py`:

```python
def _is_faucet_knob(world, obj):
    if obj == (9, 3):
        return True
    categories = _object_categories(world, obj)
    if 'faucet' in categories and ('knob' in categories or 'joint' in categories):
        return True
    name = world.get_name(obj).lower() if hasattr(world, 'get_name') else ''
    if 'faucet' in name and ('knob' in name or 'joint' in name):
        return True
    if hasattr(world, 'body_to_name'):
        body_name = world.body_to_name.get(str(obj), world.body_to_name.get(obj, ''))
        if 'faucet' in str(body_name).lower() and ('knob' in str(body_name).lower() or 'joint' in str(body_name).lower()):
            return True
    return False
```

- [ ] **Step 2: Add faucet priority bases helper**

Add this function below `_is_faucet_knob()`:

```python
def _priority_faucet_knob_bases(world, obj, pose, gripper_pose, lower_limits, upper_limits):
    if not _is_faucet_knob(world, obj):
        return []

    gripper_z = gripper_pose[0][-1]
    low_torso = min(upper_limits[2], max(lower_limits[2], gripper_z - 0.56))
    mid_torso = min(upper_limits[2], max(lower_limits[2], gripper_z - 0.44))
    high_torso = min(upper_limits[2], max(lower_limits[2], gripper_z - 0.32))
    bases = [
        (1.000, 5.600, low_torso, 0.000),
        (0.900, 5.350, mid_torso, 0.420),
        (1.180, 5.350, mid_torso, -0.420),
        (1.050, 5.850, high_torso, -0.180),
        (0.820, 5.680, mid_torso, 0.240),
    ]
    return list(dict.fromkeys(tuple(round(value, 3) for value in base) for base in bases))
```

- [ ] **Step 3: Add faucet bases to the priority list**

In `get_ir_sampler()`, extend the existing `priority_list` expression so `_priority_faucet_knob_bases()` is included next to `_priority_stove_knob_bases()`:

```python
            _priority_pot_stove_bases(world, obj, pose, gripper_pose, lower_limits, upper_limits) +
            _priority_stove_knob_bases(world, original_obj, pose, gripper_pose, lower_limits, upper_limits) +
            _priority_faucet_knob_bases(world, original_obj, pose, gripper_pose, lower_limits, upper_limits))
```

- [ ] **Step 4: Extend contact filtering for faucet knobs**

In `get_handle_motion_contact_bodies()`, after `contacts = {obj[0]}` and knob detection, add a faucet branch before the stove cookware-specific logic:

```python
    if _is_faucet_knob(world, obj):
        contacts.add(8)
        contacts.add((8, None, 2))
        if hasattr(world, 'BODY_TO_OBJECT'):
            for body, body_info in world.BODY_TO_OBJECT.items():
                text = ' '.join(str(getattr(body_info, attr, '')) for attr in ['name', 'debug_name', 'lisdf_name'])
                categories = set(getattr(body_info, 'categories', []) or [])
                if 'faucet' in text.lower() or 'basin' in text.lower() or 'faucet' in categories or 'basin' in categories:
                    contacts.add(body)
        return contacts
```

- [ ] **Step 5: Run faucet unit tests**

Run the same command from Task 1 Step 4.

Expected: both tests pass.

- [ ] **Step 6: Run full mobile stream unit tests**

Run from workspace root:

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate kitchen && PYTHONPATH="/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds/pddlstream:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/pybullet_planning:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/lisdf:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds/pybullet_planning/motion" python -m unittest pybullet_tools.test_mobile_streams
```

Expected: all tests in `pybullet_tools.test_mobile_streams` pass.

---

### Task 3: Broaden Pull Diagnostics To Faucet

**Files:**
- Modify: `/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/pybullet_planning/pybullet_tools/mobile_streams.py`
- Test: focused replay command output.

**Interfaces:**
- Consumes: `_is_faucet_knob(world, obj)` from Task 2.
- Produces: `PULL_STREAM_DIAG=1` prints diagnostics for faucet `(9, 3)` as well as stove knob `(6, 4)`.

- [ ] **Step 1: Update pull diagnostic gating**

Change this line in `get_ik_pull_gen()`:

```python
        pull_diag = _pull_stream_diag_enabled() and o == (6, 4)
```

to:

```python
        pull_diag = _pull_stream_diag_enabled() and (o == (6, 4) or _is_faucet_knob(world, o))
```

- [ ] **Step 2: Run a focused faucet replay with diagnostics**

Run from workspace root:

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate kitchen && PULL_STREAM_DIAG=1 PYTHONPATH="/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds/pddlstream:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/pybullet_planning:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/lisdf:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds/pybullet_planning/motion" python pybullet_planning/tutorials/replay_knob_one_step.py --state /home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/experiments/verify_full_no_stream_diag_260706/260706_023820_vlm-tamp/states/agent_state_4.pkl --joint_body 9 --joint_index 3 --max_evaluation_plans 24
```

Expected: output includes `PULL_DIAG start obj=(9, 3)` and either yields a plan or shows candidate bases and collision outcomes for refinement.

---

### Task 4: Verify Focused Faucet Replay And Adjust Only Targeted Values If Needed

**Files:**
- Modify only if needed: `/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/pybullet_planning/pybullet_tools/mobile_streams.py`
- Test: focused replay command.

**Interfaces:**
- Consumes: Task 2 faucet helper and Task 3 diagnostics.
- Produces: focused faucet replay with non-empty plan.

- [ ] **Step 1: Run focused faucet replay without diagnostics**

Run from workspace root:

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate kitchen && PYTHONPATH="/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds/pddlstream:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/pybullet_planning:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/lisdf:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds/pybullet_planning/motion" python pybullet_planning/tutorials/replay_knob_one_step.py --state /home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/experiments/verify_full_no_stream_diag_260706/260706_023820_vlm-tamp/states/agent_state_4.pkl --joint_body 9 --joint_index 3 --max_evaluation_plans 24
```

Expected success output:

```text
ONE_STEP_PLAN_LEN 2
```

or another non-zero plan length.

- [ ] **Step 2: If replay still fails, tune only faucet priority bases**

Use the `PULL_DIAG candidate` and `PULL_DIAG pull-result` output from Task 3. Adjust only the list in `_priority_faucet_knob_bases()` by adding or reordering sink-region base tuples. Do not change global budgets or collision flags.

- [ ] **Step 3: Re-run unit and focused tests after any adjustment**

Run:

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate kitchen && PYTHONPATH="/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds/pddlstream:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/pybullet_planning:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/lisdf:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds/pybullet_planning/motion" python -m unittest pybullet_tools.test_mobile_streams
```

Then repeat Task 4 Step 1.

Expected: unit tests pass and focused faucet replay returns a non-empty plan.

---

### Task 4B: Diagnose And Add Faucet-Specific Handle Grasp Variant

**Files:**
- Modify: `/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/pybullet_planning/pybullet_tools/general_streams.py`
- Modify only if needed: `/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/pybullet_planning/pybullet_tools/mobile_streams.py`
- Test: focused faucet replay command.

**Interfaces:**
- Consumes: `_is_faucet_knob(world, obj)` and faucet priority bases from Task 2.
- Produces: faucet-specific handle grasps that can produce a non-`None` PR2 grasp IK configuration in focused replay.

- [ ] **Step 1: Confirm grasp IK is the active blocker**

Run focused replay with diagnostics from workspace root:

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate kitchen && PULL_STREAM_DIAG=1 PYTHONPATH="/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds/pddlstream:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/pybullet_planning:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/lisdf:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds/pybullet_planning/motion" python pybullet_planning/tutorials/replay_knob_one_step.py --state /home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/experiments/verify_full_no_stream_diag_260706/260706_023820_vlm-tamp/states/agent_state_4.pkl --joint_body 9 --joint_index 3 --max_evaluation_plans 24
```

Expected if still blocked: `PULL_DIAG start obj=(9, 3)` appears and no `PULL_DIAG candidate` yields a plan.

- [ ] **Step 2: Add a local faucet-knob classifier in `general_streams.py`**

Add this helper above `get_handle_grasp_list_gen()`:

```python
def _is_faucet_handle_joint(world, body_joint):
    if body_joint == (9, 3):
        return True
    categories = []
    if hasattr(world, 'BODY_TO_OBJECT') and body_joint in world.BODY_TO_OBJECT:
        categories = getattr(world.BODY_TO_OBJECT[body_joint], 'categories', []) or []
    if 'faucet' in categories and ('knob' in categories or 'joint' in categories):
        return True
    name = world.get_name(body_joint).lower() if hasattr(world, 'get_name') else ''
    return 'faucet' in name and ('knob' in name or 'joint' in name)
```

- [ ] **Step 3: Add faucet-specific grasp variants without changing global handle grasps**

In `get_handle_grasp_gen()`, after `grasps = get_hand_grasps(...)` and before randomization, prepend faucet-specific variants only when `_is_faucet_handle_joint(world, body_joint)` is true.

Use existing grasp values as the source and create small relative-pose variants by changing only translation/orientation offsets. Keep all variants wrapped as `HandleGrasp` exactly like existing grasps.

The implementation must preserve all existing non-faucet behavior.

- [ ] **Step 4: Run focused faucet replay**

Run from workspace root:

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate kitchen && PYTHONPATH="/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds/pddlstream:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/pybullet_planning:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/lisdf:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds/pybullet_planning/motion" python pybullet_planning/tutorials/replay_knob_one_step.py --state /home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/experiments/verify_full_no_stream_diag_260706/260706_023820_vlm-tamp/states/agent_state_4.pkl --joint_body 9 --joint_index 3 --max_evaluation_plans 24
```

Expected: `ONE_STEP_PLAN_LEN` is not `None`.

- [ ] **Step 5: Run regression tests**

Run from workspace root:

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate kitchen && PYTHONPATH="/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds/pddlstream:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/pybullet_planning:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/lisdf:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds/pybullet_planning/motion" python -m unittest pybullet_tools.test_mobile_streams pybullet_tools.test_general_streams
```

Expected: all tests pass.

---

### Task 5: Run Broader Regression And Full Policy Verification

**Files:**
- No code changes expected.
- Test logs under `/tmp/opencode/` and experiment output under `/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/experiments/`.

**Interfaces:**
- Consumes: successful focused replay from Task 4.
- Produces: final evidence for faucet and stove knob behavior.

- [ ] **Step 1: Run broader related unit suite**

Run from workspace root:

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate kitchen && PYTHONPATH="/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds/pddlstream:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/pybullet_planning:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/lisdf:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds/pybullet_planning/motion" python -m unittest vlm_tools.test_llamp_agent_sequence leap_tools.test_object_reducers pybullet_tools.test_pr2_streams pybullet_tools.test_general_streams pybullet_tools.test_pose_utils pybullet_tools.test_mobile_streams pybullet_tools.test_stream_agent world_builder.test_init_utils_subgoals
```

Expected: all listed tests pass.

- [ ] **Step 2: Run memory-backed full policy verification**

Run from `pybullet_planning` unless import shadowing appears; if it does, run the equivalent command from workspace root with `pybullet_planning/tutorials/test_vlm_tamp.py`.

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate kitchen && PYTHONPATH="/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds/pddlstream:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/pybullet_planning:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/lisdf:/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/kitchen-worlds/pybullet_planning/motion" python tutorials/test_vlm_tamp.py --domain_name kitchen --world_builder_name test_kitchen_chicken_soup --llamp_api_name gpt55 --load_llm_memory /home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/experiments/verify_full_no_stream_diag_260706/260706_023820_vlm-tamp --exp_subdir verify_full_faucet_pull_fix_260706
```

Expected:

```text
openedjoint([(9, 3)]) ... solved
openedjoint([(6, 4)]) ... solved
```

- [ ] **Step 3: Inspect final CSV/time summary**

Read the generated `vlm-tamp.csv` and `time.json` from the newest experiment under:

```text
/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/experiments/verify_full_faucet_pull_fix_260706/
```

Expected:

```text
['openedjoint([(9, 3)])'] status solved
['openedjoint([(6, 4)])'] status solved
```

If faucet is still failed but the final task reaches stove knob, do not mark the task complete; return to Task 4 diagnostics.

---

## Self-Review

- Spec coverage: The plan covers focused diagnostics, faucet priority bases, faucet contact filtering, unit tests, focused replay, and full policy verification.
- Placeholder scan: No `TODO`, `TBD`, or intentionally incomplete task remains.
- Type consistency: `_priority_faucet_knob_bases()` signature matches existing priority helper signatures; `_is_faucet_knob(world, obj)` is consumed by priority, contact filtering, and diagnostics.
