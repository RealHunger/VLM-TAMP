# Faucet Pull Planning Design

## Goal

Make `openedjoint([(9, 3)])` for the faucet handle solve reliably in the GPT55 chicken soup sequence without broad planner budget increases, global collision relaxation, or hiding the failure in summary metrics.

## Current Evidence

- The stove knob blocker is fixed: `openedjoint([(6, 4)])` solves in focused replay and memory-backed full policy verification.
- The remaining strict `task_success` failure comes from three failed faucet attempts at `openedjoint([(9, 3)])`.
- Focused faucet replay fails in `inverse-kinematics-pull` with `exceeding ir_max_attempts = 80`, indicating base/IK candidate generation fails before a valid pull plan is found.
- The faucet asset is `Faucet/104`; the target joint is `(9, 3)` with closed position `0.0` and open position near `1.571`.
- Existing stove knob robustness comes from targeted priority bases and intentional contact filtering. Faucet currently lacks an equivalent targeted path.

## Recommended Approach

Treat faucet pull as a narrow TAMP geometry/sampling issue and add targeted faucet support analogous to the stove knob fix.

Implementation should stay scoped to:

- Focused faucet diagnostics and replay.
- Faucet-specific priority base candidates.
- Faucet-specific intentional contact bodies for pull/ungrasp collision checks.
- Regression tests for the new targeting logic.

It should not:

- Globally increase `ir_max_attempts`, `max_evaluation_plans`, or planning timeouts as the primary fix.
- Globally relax pull collisions.
- Mark faucet failures as successful at the summary layer.

## Components

### Focused Replay And Diagnostics

Use `tutorials/replay_knob_one_step.py` with `--joint_body 9 --joint_index 3` against a saved faucet state such as `agent_state_4.pkl`.

Extend diagnostics only if needed to print faucet pull candidates, failed collision pairs, and sampled bases. `PULL_STREAM_DIAG` should include faucet `(9, 3)`, not only stove knob `(6, 4)`.

### Faucet Priority Bases

Add `_priority_faucet_knob_bases(world, obj, pose, gripper_pose, lower_limits, upper_limits)` in `pybullet_tools/mobile_streams.py`.

The helper should activate only for the target faucet joint or for a joint categorized/named as a faucet knob. It should return a small ordered list of base configurations near the sink/faucet stance region, with torso height derived from the gripper pose and clamped to robot limits.

This helper should be included in the existing priority list inside `get_ir_sampler()` near `_priority_stove_knob_bases()`.

### Faucet Contact Filtering

Extend `get_handle_motion_contact_bodies(world, obj)` for faucet knobs.

For faucet pull, intentional contact should include:

- the faucet body itself,
- the basin body,
- the faucet platform/support body,
- any directly attached faucet/support chain bodies needed by the asset model.

It should not include unrelated counter objects, condiments, pot, lid, chicken, or broad movable sets.

### Tests

Add unit tests in `pybullet_tools/test_mobile_streams.py` for:

- faucet priority bases are returned only for faucet target joints,
- faucet priority base torso values are clamped,
- faucet contact bodies include faucet and basin/support bodies,
- faucet contact bodies exclude unrelated movable/counter-supported objects.

Existing stove knob tests must continue to pass.

## Verification Plan

1. Run focused unit tests for `pybullet_tools.test_mobile_streams`.
2. Run the broader related suite that has been used for this debugging session.
3. Run focused faucet replay from `agent_state_4.pkl` with `--joint_body 9 --joint_index 3`.
4. If focused replay solves, run memory-backed full policy verification and confirm:
   - faucet `openedjoint([(9, 3)])` is `solved`,
   - stove knob `openedjoint([(6, 4)])` remains `solved`,
   - strict summary no longer fails because of faucet.

## Success Criteria

- Focused faucet replay returns a non-empty plan.
- The full memory-backed policy run reaches the final stove knob step with faucet recorded as solved or already achieved for a valid physical reason.
- No existing related unit tests regress.
- The fix is localized to faucet pull sampling/contact behavior and does not rely on global budget or collision relaxation.
