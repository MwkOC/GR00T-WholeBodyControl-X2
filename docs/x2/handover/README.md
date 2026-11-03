# X2 遥操项目交接总览（2026-11-03）

> 这是两份真机指南的入口。先判断你要做哪类实验，再打开对应文档；**两条链路的按键语义相反，
> 不要混用**。

## 先读哪一篇？

| 你要做的事 | 用哪条链路 | 上手指南 |
|---|---|---|
| 让机器人**全身**跟随、走路、播动作，依赖 SONIC 策略自平衡 | **SONIC 全身遥操** | [`sonic_wbc_teleop_handover_CN.md`](sonic_wbc_teleop_handover_CN.md) |
| 只动**双臂/夹爪**，快速验证硬件或采数据；机器人吊架、`Develop_MC` | **上肢遥操（v4）** | [`x2_upper_body_teleop_handover_CN.md`](x2_upper_body_teleop_handover_CN.md) |

一句话区分：

- **SONIC**：Pico → sender `:5573` → PC2 **SONIC ONNX 策略** → 31 DOF；可站立、可走路。
- **上肢遥操**：Pico → sender `:5573` → PC2 **IK 节点直发 HAL 关节命令**；**必须吊架**，
  没有平衡策略。

## 两句话版本提醒

- SONIC：**L4 真机全身遥操已于 2026-10-30 验收通过**。正确进入 WHOLE_BODY 是
  **按住右扳机 + 按 B**（进入即 engage）；`grip` 只驱动手指，不是离合。
- 上肢：当前推荐 **v4 无 body tracker 3-point**；**grip = 手臂离合，trigger = 夹爪**。
  旧文档里的“扳机离合”是 v2，不要照抄到 v3/v4。

## 本机现状快照（2026-11-03）

| 项 | 值 |
|---|---|
| 源码根 | `~/Documents/czq/GR00T-WholeBodyControl-X2`（SONIC）/ `~/Documents/x2_sdk_v100`（SDK/上肢） |
| PC2 | `10.0.1.41`，ssh 用户 `run`，前缀 `/home/agi/gear-sonic` |
| PC1 EM | `10.0.1.40:50080`，app 名 `mc` |
| 机器人直连 | laptop `eth9` = `10.0.1.10/24`；**写这份总览时 eth9 未枚举，先恢复直连** |
| 上网 | 当前走 `eth8`（手机热点 172.20.10.8/28）；代理 `127.0.0.1:7897` 正常 |
| SONIC 代码 | `main = d116ec8`，比 `origin/main` 领先 50 个提交；`origin` 是上游，勿推 |
| 上肢 v3 bundle | 本机 dist `built_at=2026-10-30T10:29:42Z`、`source_dirty=yes`，部署前必须重建 |

## 第一次上机的最小顺序

1. 恢复 `eth9`：`ip -br addr show eth9` 有 `10.0.1.10/24`；`ping -c2 -I 10.0.1.10 10.0.1.41`。
2. 读对应指南的“安全须知”和“前置检查”。
3. SONIC：`check_environment.py` → PC2 preflight → ritual 点火 → sender → `RT+B`。
4. 上肢：吊架 + `Develop_MC` → PC2 v3 节点 dry-run → live → sender（v4 launcher）→ 捏 grip。
5. 收尾：SONIC 用 `x2_pc2_daemons.sh stop`（有零力矩窗口）；上肢 Ctrl-C sender → 节点 →
   `migrate_system_state Ready`。

## 交接待办（下一任操作者）

- [ ] 恢复 `eth9` 直连（USB 100M dongle 常掉载波），确认 `10.0.1.10 → 10.0.1.41` 通。
- [ ] 若继续上肢 v3/v4：用当前仓库 `build_bundle.sh` 重建 bundle，核对 PC2 的
      `SHA256SUMS`/`install` 副本（旧 dist 的 `source_dirty=yes`）。
- [ ] 新操作者/新握法：按 `PICO_LIVE_TELEOP_PITFALLS.md` §9 重标 ctrl lever，再上真机。
- [ ] SONIC 若沿用 shipped `x2_sonic_v16ft8_45000_dual` + `trained_gains_s0_inc_waistmc`，
      不需要改任何模型；换模型先过 `onnx_obs_contract.py --require body`。
- [ ] 每次上机前按对应指南的“前置检查”和“安全须知”过一遍；两条链路不要混用按键语义。

## 文档层级

- 本文件：只做导航和现状快照。
- 两份上手指南：意图、版本对比、详细步骤、排查表、已知限制。
- 仓库细节：SONIC `docs/x2/`（F01~F12、PICO_LIVE_TELEOP_PITFALLS）；上肢
  `x2_pc2/v3_upper_teleop/README.md` 与 `czq_test_v2_incremental.md`。

---

*整理：2026-11-03。若路径/地址变化，先改本文件，再同步两份指南。*
