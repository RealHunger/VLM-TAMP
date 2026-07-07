# GPT55 Chicken Soup Success Artifact

This directory contains the compact verification artifact for the successful GPT55 VLM-TAMP chicken soup run.

- `vlm-tamp.csv`: final task-level planner result, ending with `1.0 (17 / 17)`.
- `replay.mp4`: PyBullet replay rendered from saved commands.
- `planning_config.json`: configuration saved with the successful run.
- `llm_memory.json`: saved GPT55 subgoal memory used by the successful run.
- `agent_memory.json`: saved agent-side execution memory for the successful run.
- `time.json`: detailed per-subgoal timing and plan skeleton records.
- `timing_analysis.md`: summarized timing table by subgoal and task stage.

Original local run directory:

```text
/home/reality-hunger/long-horizon-manipulation/external/vlm-tamp-workspace/experiments/verify_full_faucet_open0405_close_260706/260706_213952_vlm-tamp
```
