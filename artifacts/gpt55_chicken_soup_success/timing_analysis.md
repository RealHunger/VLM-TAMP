# GPT55 Chicken Soup 耗时统计

统计来源：

- `vlm-tamp.csv`：每个 subgoal 的状态、plan length、planning time。
- `time.json`：更完整的 subgoal planning 记录。

本次成功 run 的总规划时间为 `193.46s`，其中有效规划时间为 `191.19s`，失败重试浪费时间为 `2.27s`。最终完成度为 `1.0 (17 / 17)`。

## 每个 Subgoal 的耗时

| idx | task | status | plan_len | planning_s | part | planning_node |
|---:|---:|---|---:|---:|---|---|
| 1 | 1 | already | 0 | 0.0000 | 1. 取鸡腿并放入锅 | `1_a_['openedjoint', 'fridge door']` |
| 2 | 2 | solved | 2 | 10.9541 | 1. 取鸡腿并放入锅 | `2_a_['picked', 'chicken leg']` |
| 3 | 3 | solved | 5 | 45.7865 | 1. 取鸡腿并放入锅 | `3_a_['in', 'chicken leg', 'pot body']` |
| 4 | 4 | solved | 4 | 12.5842 | 2. 水槽加水并移回台面 | `4_a_['on', 'pot body', 'kitchen sink']` |
| 5 | 5 | solved | 2 | 33.6029 | 2. 水槽加水并移回台面 | `5_a_['openedjoint', 'faucet handle']` |
| 6 | 6 | solved | 2 | 1.0960 | 2. 水槽加水并移回台面 | `6_a_['closedjoint', 'faucet handle']` |
| 7 | 7 | failed | 0 | 2.2714 | 2. 水槽加水并移回台面 | `7_a_['on', 'pot body', 'counter top on the right']` |
| 8 | 7 | solved | 4 | 30.5807 | 2. 水槽加水并移回台面 | `7_a_['on', 'pot body', 'counter top on the right']` |
| 9 | 8 | solved | 2 | 5.0015 | 3. 加盐并收回盐罐 | `8_a_['picked', 'salt shaker']` |
| 10 | 9 | solved | 2 | 14.2910 | 3. 加盐并收回盐罐 | `9_a_['sprinkledto', 'salt shaker', 'pot body']` |
| 11 | 10 | solved | 2 | 6.5700 | 3. 加盐并收回盐罐 | `10_a_['in', 'salt shaker', 'cabinet']` |
| 12 | 11 | solved | 2 | 0.7771 | 4. 加胡椒并收回胡椒罐 | `11_a_['picked', 'pepper shaker']` |
| 13 | 12 | solved | 2 | 8.4883 | 4. 加胡椒并收回胡椒罐 | `12_a_['sprinkledto', 'pepper shaker', 'pot body']` |
| 14 | 13 | solved | 2 | 4.2709 | 4. 加胡椒并收回胡椒罐 | `13_a_['in', 'pepper shaker', 'cabinet']` |
| 15 | 14 | solved | 4 | 6.5114 | 5. 上炉、盖锅并开火 | `14_a_['on', 'pot body', 'stove on the right']` |
| 16 | 15 | solved | 2 | 5.4306 | 5. 上炉、盖锅并开火 | `15_a_['picked', 'pot lid']` |
| 17 | 16 | solved | 2 | 4.1522 | 5. 上炉、盖锅并开火 | `16_a_['on', 'pot lid', 'pot body']` |
| 18 | 17 | solved | 2 | 1.0950 | 5. 上炉、盖锅并开火 | `17_a_['openedjoint', 'stove knob on the right']` |

## 每个阶段的耗时

| part | attempts | solved | already | failed | plan_len_sum | total_s | effective_s | wasted_s | pct_total |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1. 取鸡腿并放入锅 | 3 | 2 | 1 | 0 | 7 | 56.7406 | 56.7406 | 0.0000 | 29.3% |
| 2. 水槽加水并移回台面 | 5 | 4 | 0 | 1 | 12 | 80.1352 | 77.8638 | 2.2714 | 41.4% |
| 3. 加盐并收回盐罐 | 3 | 3 | 0 | 0 | 6 | 25.8625 | 25.8625 | 0.0000 | 13.4% |
| 4. 加胡椒并收回胡椒罐 | 3 | 3 | 0 | 0 | 6 | 13.5363 | 13.5363 | 0.0000 | 7.0% |
| 5. 上炉、盖锅并开火 | 4 | 4 | 0 | 0 | 10 | 17.1892 | 17.1892 | 0.0000 | 8.9% |

## 主要观察

- 最耗时阶段是“水槽加水并移回台面”，共 `80.1352s`，占总规划时间约 `41.4%`。其中 faucet open 用时 `33.6029s`，把锅从水槽移回右侧台面成功尝试用时 `30.5807s`。
- 第二耗时阶段是“取鸡腿并放入锅”，共 `56.7406s`，占约 `29.3%`。其中 `in(chicken leg, pot body)` 用时 `45.7865s`，是单个 subgoal 里最慢的一步。
- 唯一失败重试发生在 `on(pot body, counter top on the right)`，失败尝试浪费 `2.2714s`，随后 custom add 路径成功。
- 后半段调味、盖锅和开火整体较快，说明这次成功 run 的主要瓶颈集中在早期容器/水槽/水龙头相关几何规划。
