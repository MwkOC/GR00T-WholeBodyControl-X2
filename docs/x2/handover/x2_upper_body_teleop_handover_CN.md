# X2 上肢遥操真机上手指南（v1 → v4）

> **适用对象**：接手 X2 上肢遥操（Pico 头显直驱双臂/腰/夹爪，**不走 SONIC 策略、无平衡**）的操作者。
> **最后更新**：2026-11-03 · **推荐入口**：**v4 无 tracker 3-point + v3 节点**。
> 本文是统一入口；v1/v2 的历史细节在 `czq_test.md`、`czq_test_v2_incremental.md`、
> `czq_quickstart_incremental_teleop.md`，v3 节点细节在仓库
> `x2_pc2/v3_upper_teleop/README.md`。
>
> 相邻项目（SONIC 全身遥操、策略自平衡）见
> [`sonic_wbc_teleop_handover_CN.md`](sonic_wbc_teleop_handover_CN.md)。
> **两条链路不要混着操作**：上肢遥操必须 `Develop_MC` + 吊架；SONIC 是策略闭环、可自平衡。

---

## 0. 30 秒认识这个项目

**项目意图**：在没有/不需要 SONIC 全身策略的情况下，用 Pico 头显把操作者的上肢动作
**直接映射成 X2 上半身关节命令**，用于：

- 快速验证手臂/腰/夹爪硬件与 HAL 链路；
- 采集真机操作数据、做遥操 demo；
- 作为 SONIC 策略不可用时的降级路径（吊架前提）。

数据流（三个窗口）：

```
Pico 4 Ultra 头显
   │ Wi-Fi / XRoboToolkit
   ▼
laptop（W-C）  run_pico_teleop.sh / run_pico_teleop_v4_real.sh
   │ PUB :5573 pico_intent   （50 Hz：SMPL 身体 + 双手柄 + trigger/grip；v3/v4 只消费手柄字段）
   │ PUB :5572 hand_finger_cmd（msgpack，10-DOF 手帧；仅手部通道开启时）
   ▼
PC2（W-A / W-B）
   ├─ incremental_upper_teleop_v3    SUB :5573 → /aima/hal/joint/{arm,waist,head,leg}/command
   └─ x2_hand_zmq_to_aimdk_bridge    SUB/BIND :5572 → /aima/hal/joint/hand/command（CLAW）
```

**与 SONIC 全身遥操的根本区别**：

| | 上肢遥操（本文） | SONIC 全身遥操 |
|---|---|---|
| 控制 | 闭式解 / IK 直接算关节角 | 神经网络策略输出 31 DOF |
| 平衡 | **无**；`Develop_MC` 零力矩 | 策略自平衡 + 安全栈 |
| 机器人状态 | 必须吊架/悬空 | 可站立，吊架仅首跑保险 |
| 代码 | `x2_sdk_v100` + `GR00T` 里的共享链/节点 | `GR00T` 全栈 + PC2 SONIC |
| 入口 | `ros2 run czq_examples ...` | ritual + `run_pico_teleop.sh --pc2-host` |

---

## 1. 版本谱系（先看这张表，别拿错文档）

| | **v1（方案 D）** | **v2（增量离合）** | **v3（4-DoF + 腕直填）** | **v4（3-point）** |
|---|---|---|---|---|
| 时间 | 2026-10-10 ~ 10-12 | 2026-10-12 ~ 10-14 | 2026-10-20 ~ 10-30 | 2026-10-27 ~ 10-30 |
| 真机节点 | `upper_body_teleop.py` | `incremental_upper_teleop.py` | `incremental_upper_teleop_v3.py` | **复用 v3 节点** |
| 共享链 | 无（单文件） | `incremental_chain.py` | `incremental_chain_v3base.py` + `incremental_chain_v3.py` | 同 v3 |
| **手臂离合** | B 键全身开关 | **扳机（trigger）** | **握把（grip）** | **握把（grip）** |
| **手指** | 未接 | 握把 → CLAW | **扳机 → CLAW** | **扳机 → CLAW** |
| 腰 | 固定 hold | `waist_yaw` 跟随，钳 ±0.6 rad | 固定 `--waist-stand`（默认 0） | 同 v3 |
| 头 | yaw 跟随 | 禁用（hold 0） | 禁用 | 禁用 |
| IK | 闭式解（analytical arms） | DLS 7-DoF | **4-DoF 位置 IK（肩 3 + 肘）+ 腕 WristClutch 直填**；`--full-pose-ik` 可回 7-DoF | 同 v3 |
| 后端 | numpy | DLS | **mink 默认**（PC2 已离线安装）；`--backend dls` 回退 | 同 v3 |
| 断流/松手 | **回中性** | **保持当前姿态** | 保持 | 保持 |
| 遥测 record | — | 82 列 | **85 列**（qpos 70:85 为 NaN） | 85 列 |
| 数据源 | SMPL 全身 + 手柄 | SMPL 全身 + 手柄（腰用 torso） | **手柄为主；SMPL 忽略**（body tracker 只影响 sender 的 dual/手通道模式） | **仅双手柄（无 body tracker）** |
| 状态 | 历史，保留可跑 | **legacy 回退** | 真机多轮验证 | **推荐默认** |

> **v4 不是新的节点版本**，而是 v3 链的**数据源**：`--tracking 3point` 用两个 controller 的
> pose 填满 v3 的 frame 契约（`smpl_joints` 完全被忽略）。PC2 节点、手桥、wire 都不变。
> v3 节点默认参数已按 2026-10-30 真机结论写死：`--operator-x-sign +1`、
> `--operator-y-sign +1`、腕轴序 `0,2,1`、腕 pitch 符号 `-1`、ctrl lever 内置。

### 1.1 各版本一句话

- **v1**：Pico 按 **B** 切 WHOLE_BODY，双臂闭式解跟随；松手/断流回中性；头只跟 yaw；
  无腰、无手。2026-10-12 真机三级全部通过。**不要再用于新实验**，只作历史对照。
- **v2**：每臂 **扳机** 离合；松开/断流**保持**；腰跟随 ±0.6 rad；头禁用；
  CLAW 夹爪走 `:5572` msgpack 桥。2026-10-14 真机全链通过。
  **PC2 上的 `incremental_chain.py` 仍是 v2 旧语义**（扳机离合、握把手），不要和 v3 链混用。
- **v3**：角色互换为 **grip = 离合，trigger = 手**；腕从 IK 里拿出来直接填；
  4-DoF 位置 IK + mink；加入 ctrl lever 补偿、位置死区、零空间姿态；腰固定、SMPL 忽略。
  PC2 bundle 8 个 `.py`，`--self-test` 8/8。**这是当前真机节点。**
- **v4**：不戴 body tracker，只戴头显 + 双手柄；启动时用双手柄标定一次 operator frame 后冻结；
  grip 离合、trigger 手；X/A 长按 3 s 回屈肘 neutral。**这是当前推荐上机路径。**

---

## 2. 代码地图（改哪里、推哪里）

### 2.1 laptop（本机）

| 路径 | 作用 |
|---|---|
| `~/Documents/x2_sdk_v100/src/py_examples/py_examples/upper_body_teleop.py` | **v1 节点**，同时是 v2/v3 共用的 HAL 表（关节名/限位/kp/kd/stand pose） |
| `~/Documents/x2_sdk_v100/src/py_examples/py_examples/incremental_upper_teleop.py` | **v2 真机节点** |
| `~/Documents/czq/GR00T-WholeBodyControl-X2/gear_sonic/utils/teleop/incremental_chain.py` | **v2 共享链**（PC2 部署副本仍是“扳机离合 / 握把手”的旧语义；仓库同名文件后来做过角色互换，别混用） |
| `.../gear_sonic/utils/teleop/incremental_chain_v3base.py` | **v3 基座链**（独立部署，不覆盖 v2 的 `incremental_chain.py`） |
| `.../gear_sonic/utils/teleop/incremental_chain_v3.py` | **v3 完整链**（WristClutch、ctrl lever、守卫） |
| `.../gear_sonic/utils/teleop/solver/arm/x2_arm_ik.py` | DLS / 4-DoF 位置 IK |
| `.../gear_sonic/utils/teleop/solver/arm/x2_arm_fk.py` | 臂 FK（含右腕几何修正） |
| `.../gear_sonic/utils/teleop/solver/arm/x2_arm_mink.py` | mink QP IK |
| `.../x2_pc2/v3_upper_teleop/` | **v3 真机节点源码 + bundle 构建 + dist + 详细 README** |
| `.../gear_sonic/scripts/run_pico_teleop.sh` | sender 总入口（live / tape / 3point） |
| `.../gear_sonic/scripts/run_pico_teleop_v4_real.sh` | **v4 真机 launcher**（预设 3point + trigger 手） |
| `.../gear_sonic/scripts/run_pico_teleop_v4_sim.sh` | **v4 仿真孪生**（MuJoCo harness + sender 一条命令） |
| `.../gear_sonic/scripts/arm_teleop_v3_mujoco.py` | v3/v4 MuJoCo harness（`--wire` 吃真实 sender） |
| `.../gear_sonic/scripts/pico_intent_sender.py` | Pico 意图发送（真实数据源） |
| `.../gear_sonic/scripts/live_pico_smpl_teleop.py` | SMPL 身体流 → intent |
| `.../GEAR.../tools/gen_v4_intent_tape.py` | 合成 v4 intent tape（离线门禁用） |
| `.../tools/compare_sim_node_command_stream.py` | sim vs 节点逐 tick 对齐门禁 |

### 2.2 PC2（机器人）

| 路径 | 作用 |
|---|---|
| `~/Workspaces/czq/ros2_ws/src/czq_examples/czq_examples/` | ros2 包源码（v2/v3 节点、链、solver、HAL 表） |
| `~/Workspaces/czq/ros2_ws/install/czq_examples/lib/python3.10/site-packages/czq_examples/` | **`ros2 run` 实际导入的副本**（colcon 拷贝，不是软链） |
| `~/Workspaces/czq/gear-sonic/gear_sonic_deploy/scripts/x2_hand_zmq_to_aimdk_bridge.py` | CLAW 手桥（**必须用这一份**，不是 `ws/src/x2/...` 里的旧 pose-only 副本） |
| `/home/agi/gear-sonic/venv/bin/python3` | 手桥解释器 |
| `/home/agi/gear-sonic/ws/install/setup.bash` | 手桥环境（不是 README 里旧的 `/home/run`） |

> **只 scp 到 `src` 不生效**：`ros2 run` 走 entry point，导入的是 `install/.../site-packages/czq_examples/`
> 里的副本。部署后必须 `colcon build --packages-select czq_examples`，或把同一批文件也拷进
> install 副本并删 `__pycache__`。

### 2.3 文档地图

| 文档 | 内容 |
|---|---|
| `czq_test.md` | v1（方案 D）任务回顾、闭式解、举不过头顶的定位 |
| `czq_test_v2_incremental.md` | v2 母文档：设计、八轮 live 修复、验收梯子、v1→v2 差异 |
| `czq_quickstart_incremental_teleop.md` | v2 快速命令单（给新人照抄） |
| `GR00T.../x2_pc2/v3_upper_teleop/README.md` | **v3 节点部署/真机/85 列判读/回退**（最详细） |
| `GR00T.../docs/x2/F03_pico_teleop.md` §“手臂 IK 后端 / v3 / v4” | v3/v4 的官方 runbook |
| `GR00T.../docs/x2/PICO_LIVE_TELEOP_PITFALLS.md` §9~§11 | lever 标定、腕映射、v4 踩坑 |
| **本文** | 统一入口、版本对比、两个项目的边界 |

---

## 3. 安全须知（比 SONIC 更严格：没有策略兜底）

1. **必须吊架/悬空，且现场负责人确认承重。** `Develop_MC` 是开发者直驱模式，腿/腰只发
   hold 形状命令，**没有平衡**。机器人一旦落地承重，可能瞬间失控或过流。
2. **起 v3/v4 节点就会从第一拍发 `ARM_NEUTRAL`（屈肘朝前）**，不是等你 engage 才动。
   起节点前必须吊架吊住、周围净空，不要假设手臂还停在 `Develop_MC` 的瘫软位置。
3. **`migrate_system_state Develop_MC` 会让机器人立即零力矩瘫软**，全程扶稳/吊住。
4. **绝不在机器人站着/吊着受力时 kill deploy 的 tmux 会话**；停节点用 Ctrl-C，按 §7 顺序。
5. **急停优先级**：
   1. **松开 grip** → 该臂立即冻结（最快）；
   2. Ctrl-C sender → 0.5 s stale → 双臂保持；
   3. 节点 Ctrl-C → 停发 HAL 命令，伺服保持最后位置；
   4. 硬件急停按钮；旁站人始终手扶吊架。
6. **第一次到这台机器人、或换人/换握法**：先做 §6.1 的离线自检 + §6.2 的 dry-run，
   再上真机；不要现场试错方向/符号。
7. **手桥日志一律 `grep -a` / `tail -a`，不要 `cat`**：rclpy 启动时 FastDDS 会往 stderr
   刷大量 discovery warning，`cat` 会卡死 ssh（2026-10-14 踩过）。

---

## 4. 前置检查（每次会话）

- [ ] 吊架就位、机器人悬空、周围净空；两人到位（一人守机器人，一人看 laptop）。
- [ ] 机器人网：`ip -br addr show eth9` 有 `10.0.1.10/24`；`ping -c2 -I 10.0.1.10 10.0.1.41` 通。
- [ ] laptop 没有本地 sim 栈抢端口：`./gear_sonic/scripts/simstack_local.sh --stop`（没起过可跳过）。
- [ ] laptop 无残留 sender：`pgrep -af pico_intent_sender`（应为空）。
- [ ] PC2 端口干净：`ss -tlnp | grep -E ':5573|:5572'`（起节点前 `:5573` 应为空；起桥前 `:5572` 应为空）。
- [ ] 头显有电、XRoboToolkit app 就绪；扳机/握把在 PICO 系统里读数正常。
- [ ] PC2 系统状态 = `Ready`（`ros2 run py_examples migrate_system_state` 系列可查）。
- [ ] **v3/v4 节点已部署且自检 8/8**（§6.1）；**v2 回退时**离线门禁 PASS（§6.3）。

### PC2 每个窗口先 source 三连

```bash
source /opt/ros/humble/setup.bash
source /home/agi/aimdk/install/setup.bash
source ~/Workspaces/czq/ros2_ws/install/setup.bash
```

ssh 姿势（别名按你的 ssh config；没有就用 `run@10.0.1.41`）：

```bash
ssh Agibot-X2-PC2-Orin-NX -t 'tmux new -A -s czq_v3'       # W-A 节点
ssh Agibot-X2-PC2-Orin-NX -t 'tmux new -A -s czq_v3_mon'   # W-B 监控/手桥
```

> tmux 会话名用 `czq_v3` / `czq_v3_mon`，**不要碰 deploy 的会话**；
> 机器人站着时绝不许 kill 任何 tmux。

---

## 5. 三个窗口

| 窗口 | 在哪 | 跑什么 |
|---|---|---|
| **W-A** | PC2 | v3 节点（全场唯一发 arm/waist/head/leg 命令的进程） |
| **W-B** | PC2 | 状态迁移、CLAW 手桥、端口/话题抽查 |
| **W-C** | laptop | sender（v4 real / v3 full / v2 / tape） |

启动顺序建议（v4/v3 通用）：

1. W-A：先 `--dry-run` 20 s 看命令流，再正式起节点；
2. W-B：起 CLAW 手桥（要夹爪才需要）；
3. W-C：起 sender；
4. 操作者戴头显，按 §7 操作卡执行。

---

## 6. 离线自检与一次性部署

### 6.1 v3/v4 节点自检（本机 + PC2）

**本机重建 bundle：**

```bash
cd /home/nvidia/Documents/czq/GR00T-WholeBodyControl-X2
PYTHON=$PWD/.venv/bin/python SELF_TEST=1 \
  x2_pc2/v3_upper_teleop/build_bundle.sh
# 产出 x2_pc2/v3_upper_teleop/dist/：8 个 .py + SHA256SUMS + bundle_info.txt
```

**本机自检：**

```bash
.venv/bin/python x2_pc2/v3_upper_teleop/dist/incremental_upper_teleop_v3.py \
  --self-test --backend mink
.venv/bin/python x2_pc2/v3_upper_teleop/dist/incremental_upper_teleop_v3.py \
  --self-test --backend dls
# 期望：[self-test] all checks: 8/8（两种后端都全绿）
```

**PC2 自检：**

```bash
# 先备份会被覆盖的三个文件（x2_arm_ik.py 也要备份，v2 也用它）
ssh Agibot-X2-PC2-Orin-NX \
  'D=~/Workspaces/czq/ros2_ws/src/czq_examples/czq_examples; \
   for f in upper_body_teleop.py x2_arm_fk.py x2_arm_ik.py; do \
     cp -a $D/$f $D/$f.bak.$(date +%s); done'

B=x2_pc2/v3_upper_teleop/dist
scp $B/incremental_upper_teleop_v3.py \
    $B/incremental_chain_v3base.py \
    $B/incremental_chain_v3.py \
    $B/x2_arm_ik.py \
    $B/x2_arm_fk.py \
    $B/x2_arm_mink.py \
    $B/x2_arm_kinematic.py \
    $B/upper_body_teleop.py \
    Agibot-X2-PC2-Orin-NX:~/Workspaces/czq/ros2_ws/src/czq_examples/czq_examples/
```

然后加 entry、colcon build、自检：

```bash
cd ~/Workspaces/czq/ros2_ws/src/czq_examples
grep -q incremental_upper_teleop_v3 setup.py || python3 - <<'EOF'
import pathlib
p = pathlib.Path("setup.py"); s = p.read_text()
anchor = '"incremental_upper_teleop = czq_examples.incremental_upper_teleop:main",'
assert anchor in s, "v2 entry not found -- edit setup.py by hand"
entry = '\n            "incremental_upper_teleop_v3 = czq_examples.incremental_upper_teleop_v3:main",'
if "incremental_upper_teleop_v3" not in s:
    p.write_text(s.replace(anchor, anchor + entry))
print("v3 entry added")
EOF

cd ~/Workspaces/czq/ros2_ws          # ★ 绝不在 ~/Workspaces/czq 顶层跑 colcon
colcon build --packages-select czq_examples

source /opt/ros/humble/setup.bash
source /home/agi/aimdk/install/setup.bash
source install/setup.bash
ros2 pkg executables czq_examples | grep incremental
ros2 run czq_examples incremental_upper_teleop_v3 --self-test --backend mink
ros2 run czq_examples incremental_upper_teleop_v3 --self-test --backend dls
```

**核对 src 与 install 副本一致：**

```bash
I=~/Workspaces/czq/ros2_ws/install/czq_examples/lib/python3.10/site-packages/czq_examples
ls -l $I/ | grep -E 'incremental_chain_v3|incremental_upper_teleop_v3'
# 大小/mtime 必须和刚 scp 的 src 一致；否则跑的还是旧链
```

> `x2_arm_kinematic.py` 是**生成文件**（由 `gen_arm_kinematic_model.py` 从 `x2_ultra.xml`
> 裁出 kinematics-only XML 内嵌），必须随 bundle 更新。**改了 MJCF 必须重新打包，不要手改。**
> 若 `SOURCE_SHA256` 校验失败，`build_bundle.sh` 会拒绝打包。

### 6.2 v4 无 sender 的 dry-run（PC2）

```bash
ros2 run czq_examples incremental_upper_teleop_v3 \
  --dry-run --record /tmp/v3_dry.npz --duration 20
```

期望：50 Hz 稳定、无异常；record 形状 `(N, 85)`，`rows[:,70:85]` 全 NaN。
正式起节点前，先确认 `--dry-run` 能跑满 20 s。

### 6.3 v2 离线门禁（只有回退到 v2 时才需要）

```bash
cd /home/nvidia/Documents/czq/GR00T-WholeBodyControl-X2
.venv/bin/python tools/gen_v4_intent_tape.py /tmp/v4_tape.npz
.venv/bin/python tools/compare_sim_node_command_stream.py --tape /tmp/v4_tape.npz
# 期望：LEVEL-0: PASS (bit-exact) + argparse defaults differ on: nothing
```

**不通过不上机器人。**

---

## 7. 真机操作（v4 推荐路径）

> v4 = 不戴 body tracker，只戴头显 + 双手柄；PC2 沿用 v3 节点和 CLAW 手桥。

### 7.1 当天顺序（照做，不跳步）

1. §4 检查清单过一遍；吊架吊住、悬空、净空。
2. **W-B**：确认系统状态 = `Ready`。
3. **W-B**：`ros2 run py_examples migrate_system_state Develop_MC`
   （★ 确认吊架承重后再执行，机器人立即零力矩瘫软）。
4. **W-A**：先 dry-run 20 s 看命令流，Ctrl-C；再正式起节点：

   ```bash
   ros2 run czq_examples incremental_upper_teleop_v3 \
     --dry-run --record /tmp/v3_dry.npz --duration 20
   ros2 run czq_examples incremental_upper_teleop_v3 --record /tmp/v3_live.npz
   ```

   启动横幅应出现（以实际为准）：

   ```
   [frame] operator_x_sign=+1 operator_y_sign=+1
   [wrist] left/right axis_order=0,2,1 signs=(+1,-1,+1)
   arm IK mink  4-DoF
   ```

5. **W-B**：抽查 HAL 与端口：

   ```bash
   ss -tlnp | grep :5573
   ros2 topic echo /aima/hal/joint/arm/command --once
   ```

6. **W-B**：起 CLAW 手桥（要夹爪才需要；起后静默待命）：

   ```bash
   source /opt/ros/humble/setup.bash
   export AMENT_PREFIX_PATH=/agibot/software/housekeeper/bin/aimdk_msgs:$AMENT_PREFIX_PATH
   export LD_LIBRARY_PATH=/agibot/software/housekeeper/bin/aimdk_msgs/lib:$LD_LIBRARY_PATH
   source /home/agi/gear-sonic/ws/install/setup.bash
   export LD_LIBRARY_PATH=/home/agi/gear-sonic/onnxruntime/lib:$LD_LIBRARY_PATH
   export PYTHONPATH=/agibot/software/housekeeper/bin/aimdk_msgs/local/lib/python3.10/dist-packages:/home/agi/gear-sonic/ws/src:${PYTHONPATH:-}

   setsid nohup /home/agi/gear-sonic/venv/bin/python3 \
     /home/agi/Workspaces/czq/gear-sonic/gear_sonic_deploy/scripts/x2_hand_zmq_to_aimdk_bridge.py \
     --wire-format msgpack --zmq-host '*' --zmq-port 5572 \
     --zmq-topic hand_finger_cmd --hand-mode claw --sides auto \
     > /tmp/hand_bridge.log 2>&1 < /dev/null &

   ss -ltnp | grep 5572                    # 必须看到桥在 BIND
   grep -a "hand_mode=claw" /tmp/hand_bridge.log | head -1
   ```

   - 收到第一帧 hand 数据前**完全静默**（不发任何 HAL 命令），提前起无风险。
   - 若刚跑过 `ros2 run py_examples hand_control`，**必须先 Ctrl-C 停掉**，否则两边
     50 Hz 抢写夹爪命令。

7. **W-C**：起 sender（v4 预设 = `${3point} + trigger 手`）：

   ```bash
   cd /home/nvidia/Documents/czq/GR00T-WholeBodyControl-X2
   ./gear_sonic/scripts/run_pico_teleop_v4_real.sh --pc2-host 10.0.1.41

   # 只动臂、不动手：
   # ./gear_sonic/scripts/run_pico_teleop_v4_real.sh --pc2-host 10.0.1.41 --no-hands
   ```

   launcher 会：

   - 固定 `--tracking 3point --hand-source trigger --hand-close-on-grip`；
   - 拒绝 loopback（`127.0.0.1` 是仿真，真机必须给 PC2 IP）；
   - 拒绝 `--tape-replay` / `--tape-once`（3point 是 live-only）；
   - 拒绝 PC2 节点侧参数（`--ctrl-lever-*`、`--wrist-*-sign-*`、`--operator-x-sign`、
     `--backend`、`--record`、`--duration`、`--self-test` 等）；这些必须写在 PC2 节点命令上；
   - 在 PC2 `:5573` / `:5572` 没监听时打印 WARNING（不会代起 PC2 进程）。

8. **标定（v4 第一次启动必做）**：
   双手各握手柄，自然举在胸前、**左右分开 ≥ 20 cm（README 最低 15 cm）、保持约 1 s**。
   看到 `[3pt] operator frame calibrated ...` 后进入 `stream live.`。
   失败时每 3 s 打印 `hold both controllers apart at chest height (>= 15 cm)` 并自动重试。

9. **确认关键横幅**：

   ```
   [frame] operator_x_sign=+1 operator_y_sign=+1
   [wrist] left/right axis_order=0,2,1 signs=(+1,-1,+1)
   hand_finger_cmd PUB -> tcp://10.0.1.41:5572
   [hand] grasp L ... R ...
   ```

   PC2 上 `ss -ltnp | grep 5572` 必须看到手桥在 BIND，否则臂动、手不动。

### 7.2 Pico 操作卡（v4 / v3 相同）

| 操作 | 效果 |
|---|---|
| **捏住握把（grip）** | 该侧手臂**咬合**，从当前姿态起增量跟随 |
| **松开握把** | 该臂**冻在当前位姿**，不回中性 |
| **扣扳机（trigger）** | CLAW 夹爪**闭合**；松开 = 张开（v4 预设 `--hand-close-on-grip`） |
| **左手柄 X 长按 ≥ 3 s** | 左臂以 0.5 rad/s 回 `ARM_NEUTRAL`（屈肘朝前） |
| **右手柄 A 长按 ≥ 3 s** | 右臂回 `ARM_NEUTRAL` |
| 快速反向甩手 | 可能触发冻结保护（正常）；恢复后继续 |
| 手推向躯干 | 被椭圆柱 keep-out 挡住，不穿模 |
| 手腕拧到极限 | 命令会平滑裁剪（腕 pitch 只有 ±32°，属关节限位） |

**不需要“进模式”**：v3/v4 的手臂/手部不依赖 OFF/LOCOMOTION/WHOLE_BODY；
`--no-locomotion` 关的是 planner/控制器层，**v4 3point 仍保留 `hand_finger_cmd` 到 `:5572`**。
`--hand-source off` 或 `--no-hands` 才是真的手臂-only。

**典型抓取动作**：捏 grip 把手移到位 → **松开 grip**（姿态冻住）→ 扣/松 trigger 开合夹爪
→ 需要再挪时重新捏 grip（从当前姿态继续）。

### 7.3 验收清单（v4 真机）

- [ ] sender 横幅有 `[3pt] operator frame calibrated` 和 `hand_finger_cmd PUB -> tcp://<PC2>:5572`。
- [ ] PC2 `ss -ltnp | grep 5572` 看到手桥在 BIND。
- [ ] 捏 grip → 对应臂状态 `eng=E`；只转手腕 → 肩 3 + 肘基本不动。
- [ ] 扣 trigger → `[hand] grasp L/R` 变化、CLAW 闭合；松开 → 张开。
- [ ] 平移方向：双手同时朝操作员**前方**平移，机器人腕 FK x 增大（看 §9 的 record 判读）。
- [ ] 头部隔离：手柄不动、只转头 5 s，手臂指令不随头动。
- [ ] 单侧遮挡手柄约 0.5 s → 该臂保持、该侧手指张开；恢复后不跳变。
- [ ] 长按 X/A 3 s → 对应臂回屈肘 neutral；仍握 grip 会在新姿态重新 anchor。
- [ ] Ctrl-C sender → 0.5 s stale → 双臂保持；Ctrl-C 节点 → 停发 HAL；迁回 `Ready`。

### 7.4 v3 全身路径（sender 走 dual、有 body tracker 时）

如果你确实要戴 body tracker 走 v3 的 sender dual 模式：

```bash
# PC2 节点同 §7.1（v3 节点）
# laptop sender：不要加 --no-locomotion；手要 trigger 驱动
cd /home/nvidia/Documents/czq/GR00T-WholeBodyControl-X2
./gear_sonic/scripts/run_pico_teleop.sh --pc2-host 10.0.1.41 \
  --hand-source trigger --hand-close-on-grip
```

- **v3 节点仍然只消费 controller pose**：`smpl_joints` 被忽略，waist 固定、头禁用。
  body tracker 不参与 v3 的控制律；它只影响 sender 是否走 dual 模式。
- **v3 full 不要加 `--no-locomotion`**：它会关掉 sender 的控制器层，`hand_sock` 不创建，
  真机手指永远不会动。
- `--hand-source off` 会让手部 socket 不创建，是“手指不动”的直接原因。
- 操作角色与 v4 相同：**grip = 离合，trigger = 手**。
- 想真正用 SMPL 身体/腰跟随，请回 v2（`waist_yaw ±0.6 rad`）；v3/v4 有意关掉了这条通道。

### 7.5 v2 回退路径（legacy）

只有 v3/v4 出问题、需要对照旧行为时才用：

```bash
# W-A
ros2 run czq_examples incremental_upper_teleop --dry-run --record /tmp/pc2_dry.npz --duration 20
ros2 run czq_examples incremental_upper_teleop

# W-B 手桥（v2 可用 --sides both，也可 auto）
setsid nohup python3 ~/Workspaces/czq/gear-sonic/gear_sonic_deploy/scripts/x2_hand_zmq_to_aimdk_bridge.py \
  --wire-format msgpack --zmq-host '*' --zmq-port 5572 --zmq-topic hand_finger_cmd \
  --hand-mode claw --sides both > /tmp/hand_bridge.log 2>&1 < /dev/null &

# W-C live
./gear_sonic/scripts/run_pico_teleop.sh --pc2-host 10.0.1.41
```

- v2 的语义：**扳机 = 手臂离合**，**握把 = 手**；松开/断流保持。
- v2 夹爪默认方向是 **松 = 合、捏 = 开**；要“握 = 合”在 sender 上加
  `--hand-close-on-grip`（方向由 sender 定，桥不用动）。
- **v2 live 绝不能加 `--no-locomotion`**：该 flag 让 `dual_mode=False`，扳机字段恒 0 上线，
  手臂永远不咬合（与节点/头显状态无关）。tape 回放可以加，因为 trigger 走 tape 录制字段。
- v2 的 `:5572` 手部通道会被 `--no-locomotion` 或 `--hand-source off` 任一静默废掉。
- v2 record 82 列；v3/v4 85 列，别拿错判读脚本。

---

## 8. 收尾（顺序不能乱）

1. **W-C**：Ctrl-C sender。
2. **W-A**：Ctrl-C 节点（停发 HAL 命令，伺服保持最后位置）。
3. **W-B**：手桥可留可停；停用 `pkill -f x2_hand_zmq_to_aimdk_bridge`。
4. **W-B**：`ros2 run py_examples migrate_system_state Ready` 恢复站位。
   **绝不在节点还在发命令时落地。**
5. 确认无残留：`pgrep -af incremental_upper_teleop`、`ss -tlnp | grep -E ':5573|:5572'`。
6. **绝不在机器人站着的时候 kill deploy 的 tmux 会话。**

---

## 9. record 判读（v3/v4 85 列）

```bash
scp Agibot-X2-PC2-Orin-NX:/tmp/v3_live.npz /tmp/
cd /home/nvidia/Documents/czq/GR00T-WholeBodyControl-X2
.venv/bin/python - <<'PY'
import numpy as np
rows = np.load("/tmp/v3_live.npz")["rows"]
print("rows", rows.shape)              # (N, 85)
for side, eng, qs, ws in (("L", 65, (35, 39), (49, 52)),
                          ("R", 66, (42, 46), (52, 55))):
    m = rows[:, eng] > 0.5             # ★ 65/66 = engaged；63/64 是冻结保护
    if not m.any():
        print(side, "never engaged"); continue
    seg = rows[m]
    seg = seg[min(int(0.4 / 0.02), len(seg) - 1):]   # 跳过咬合后 0.4 s 瞬态
    print(side, "engaged", int(m.sum()), "ticks",
          "q[sh3+elbow] p2p rad", np.round(np.ptp(seg[:, qs[0]:qs[1]], axis=0), 4),
          "wrist cmd p2p deg", np.round(np.degrees(np.ptp(seg[:, ws[0]:ws[1]], axis=0)), 1))
PY
```

列定义（关键）：

| 列 | 含义 |
|---|---|
| 0 | `t_wall` |
| 1 | `fresh`（wire 是否在流） |
| 2/3 | 原始 **trigger**（手输入） |
| 4/5 | LPF 后 **grip**（离合） |
| 7:14 / 14:21 | 左/右控制器原始 pose |
| 21:28 / 28:35 | 左/右 LPF 后 pose |
| 35:42 / 42:49 | 左/右 arm 命令 q（肩 3 + 肘 + 腕 3） |
| 49:52 / 52:55 | 左/右腕命令角（rad，不是末端位置） |
| 63/64 | 冻结保护（正常恒 0，**不要当离合窗口**） |
| **65/66** | **左/右 engaged** |
| 70:85 | qpos：真机节点为 NaN（没有 plant） |

判据：

- 纯腕旋转时，`q` 前 4 个（肩 3 + 肘）p2p 最好 **< 0.02 rad**；本机 wrist tape 基线 ≤ 0.0045 rad。
- 0.02~0.05 rad 可来自 posture/deadband 的慢漂，不应是随腕旋转的周期性联动。
- 若肩肘仍随腕旋转周期性动：优先怀疑 **ctrl lever**（重新标定），其次检查 sender 的控制器
  姿态是否在纯腕旋转时带了平移。
- **85 列没有 home 通道**：X/A 长按回位造成的差异不是代码 bug。

---

## 10. 故障排查

### 10.1 先分清是哪个版本

| 你看到的现象 | 先确认 |
|---|---|
| 扳机有值但臂不动 | v2 里扳机才是离合；v3/v4 里 **grip 才是离合** |
| 握把有值但手不动 | v2 里握把是手；v3/v4 里 **trigger 才是手** |
| record 是 82 列 | 你在跑 v2；85 列才是 v3/v4 |
| 启动横幅没有 `[frame]` | 你在跑 v2 或旧 bundle；v3/v4 应打印 `operator_x_sign/y_sign` |
| 节点名是 `incremental_upper_teleop` | v2；v3/v4 是 `incremental_upper_teleop_v3` |

### 10.2 症状表

| 现象 | 先看哪 | 处理 |
|---|---|---|
| **臂完全不动** | sender 是否带了 `--no-locomotion`（v2 致命）；节点状态行 `fresh=1?`、`age` 是否在涨；`ss -tn \| grep 63901` 头显在不在 | v2 去掉 `--no-locomotion`；v3/v4 检查 grip 是否捏住、节点是否 live |
| 状态行 `trig` 有值但臂不动 | v2：`trig` 需 ≥0.5 才咬合；确认扣的是**扳机**不是握把 | 换对按键；看 `eng=` 字段 |
| 臂动但手不动 | ① sender 启动行有没有 `hand_finger_cmd PUB`；② 桥是否活着；③ `:5572` 是否 BIND | 去掉 `--hand-source off`/`--no-hands`；起桥；`grep -a "stats:" /tmp/hand_bridge.log \| tail -3` 看 `zmq_rx`/`hand_present` 是否在涨、`clamp_hits=0` |
| 夹爪方向反了 | v4/v3：sender 是否带 `--hand-close-on-grip` | 不用改桥；改 sender 旗标重启 |
| 桥起不来 / 报 msgpack | PC2 是否装了 `msgpack` | `pip install --user msgpack`（已装 1.2.3；无外网用离线轮子） |
| 头显停顿越来越长 → `device missing` | **头显没电** | 先充电，别查软件 |
| 腰不跟（v2） | 坐姿会让 torso 长期超限（守卫在干活） | 站起来转身试 |
| `ros2 run` 改了代码不生效 | 只 scp 到了 `src`，`install` 副本没更新 | `colcon build --packages-select czq_examples`，或同步 install 副本并删 `__pycache__` |
| 节点第一条 tick 就 `TypeError: from_quat() got an unexpected keyword argument 'scalar_first'` | PC2 scipy 1.8.0 不支持 `scalar_first`（旧链） | 更新到含 `_rot_from_quat_wxyz` 的链（提交 `1611a9c` 之后） |
| 节点报缺 mink/mujoco/daqp | v3 PC2 默认 `--backend mink` | 装离线 wheels（`x2_pc2/v3_upper_teleop/README.md` §3.3），或 `--backend dls` 回退 |
| `x2_arm_kinematic.py` 校验失败 | MJCF 改了但没重新打包 | `build_bundle.sh` 重新生成，不要手改模块 |
| 方向反 / 镜像（前后、左右、画圆） | 当前真机默认 `x=+1, y=+1`；只有**面对面站位**才用 −1 | 先确认操作员与机器人**同向**；不要现场试符号，按 `F03`/`PITFALLS §11` 定位 |
| 只转手腕却拖肘/肩 | ctrl lever 与实际握持不符 | 按 `PITFALLS §9` 重新标 lever；临时可 `--ctrl-lever-left/right`；不要先改 IK |
| 前伸时手往机器人右侧偏 | reach 球以肩为球心，屈肘 neutral 只剩 ~14 cm 行程 | 沿锚点→目标射线截断已修；要完整行程用 `--position-scale 0.35` 或在伸出位重新起锚 |
| 前伸时 z 下落 | 操作员压低手柄（输入侧）+ reach 饱和（4.1% 拍），不是映射 bug | 保持手柄高度；需要更大行程时缩放/重锚；plant 关节反馈没录 |
| 腕只能动很小角度 | 腕 pitch 只有 ±32°，roll 左/右不对称 | 关节限位，不是 IK 失败；先小幅度确认方向再加大 |
| 单侧手柄被遮挡 | 该侧 `grip=0/trigger=0`，臂保持、手指张开 | 正常失效安全；恢复后重新捏 grip |
| 节点启动手臂就动 | v3/v4 从第一拍发 `ARM_NEUTRAL`（屈肘朝前） | 起节点前吊架/清场；这不是 bug |
| `--tracking 3point` 被 launcher 拒绝 | v4 预设自己拥有 `--tracking` | 不要重复传；用 `run_pico_teleop_v4_real.sh`/`_sim.sh` |
| v4 real launcher 拒绝 `--pc2-host 127.0.0.1` | 127.0.0.1 是仿真目标 | 仿真用 `run_pico_teleop_v4_sim.sh` |
| v4 launcher 拒绝 `--record`/`--duration` | 这些是 PC2 节点选项 | 写到 PC2 的 `ros2 run ... v3` 命令上 |
| ssh 进 PC2 后 `cat` 桥日志卡死 | FastDDS 启动洪水 | 一律 `grep -a` / `tail -a` |
| 头显连不上 PC Service | 候选 IP 选错网段（laptop 多网卡） | 按头显 Wi-Fi 页网段挑；`ss -tn \| grep 63901` 复核 |
| PC2 上 `:5572` 被 kplanner 占着 | `pc2_kplanner_onnx.py --arm-connect` 先占了 | 停 kplanner（v3 直连 HAL 不需要它），或让 kplanner 转发手部帧到 pose 桥 |
| 手桥用了旧副本，`--wire-format` 不识别 | `ws/src/x2/agi_x2_deploy_onnx_ref/scripts/` 那份是 pose-only | 用 `gear_sonic_deploy/scripts/x2_hand_zmq_to_aimdk_bridge.py` |
| PC2 bundle 与仓库不一致 | dist 是旧构建 | 用当前仓库重建 bundle，核对 `SHA256SUMS`/`bundle_info.txt` |

### 10.3 三条不要做的事

1. **不要在用旧文档的按键语义试新节点**：v2 是扳机离合，v3/v4 是握把离合。
2. **不要在真机上现场试方向/符号**：方向问题的定位在 `PITFALLS §9/§11` 和 sim tape，
   现场只做小幅度确认。
3. **不要覆盖 PC2 上的 `incremental_chain.py`**：那是 v2 基座；v3 用独立命名的
   `incremental_chain_v3base.py` / `incremental_chain_v3.py`，两者互不影响。

---

## 11. 已知限制（交接必须知道）

1. **v4 没有躯干/腿跟踪**：只用手柄 + 头显；固定 operator frame 只标定一次。
   engage 期间转体会给前伸方向带来同角度偏航偏差——**保持髋/脚不动**；转身后松开 grip、
   恢复朝向、**重启 sender** 重新标定。
2. **reach 球以肩为球心**：屈肘 neutral 起锚时前伸只剩约 14 cm；操作员 40–50 cm 的行程
   本来就大于臂的剩余行程。要完整行程用 `--position-scale 0.35`，或在伸出位重新起锚。
3. **前伸 z 下落**不是映射 bug：输入侧手柄压低 + reach 饱和；该记录已逐位回放一致。
   plant 关节反馈（真机 85 列 70:85）是 NaN，无法排除被控对象下垂。
4. **腕关节行程不对称**：yaw ±146°，pitch ±32°，roll 左 [-90°, +41.5°] / 右 [-41.5°, +90°]。
   大幅腕动作会先顶限位，训练/测试时先小幅度。
5. **控制器单侧静默冻结无法检测**（SDK 没有 per-controller 可用性 API）：手臂只会保持
   （delta=0），不会跳变；操作者要自己发现“手还在动但机器人不跟”。
6. **ctrl lever 取决于握持**：换人、换握法、隔天重戴都建议重标；当前默认值
   `L(0.0410, 0.0090, -0.0912)` / `R(-0.0393, 0.0115, -0.0791)` 已内置进节点，
   正式启动命令不需要带标定参数。重标后要么显式传 `--ctrl-lever-*`，要么写回常量重打包。
7. **v3 的 waist 固定、SMPL 忽略**：v4 也一样。想要腰跟随请回 v2（±0.6 rad）或等新版本。
8. **没有平衡**：v1~v4 全部是 `Develop_MC` 直驱，吊架是硬前提。

---

## 12. 验收记录与交接快照

| 时间 | 版本 | 记录 |
|---|---|---|
| 2026-10-12 | v1 | 三级真机流程全部通过：tape 回放、live 头显、收尾 `migrate Ready` |
| 2026-10-14 | v2 | 全链贯通：桥 stats `zmq_rx=391/hand_present=391`、`clamp_hits=0`；用户确认双臂扳机离合 + CLAW 开合正常 |
| 2026-10-20 ~ 10-27 | v3 | PC2 节点部署、`--self-test` 8/8、dry-run 50 Hz；真机多轮修正腕轴序/符号 |
| 2026-10-27 ~ 10-30 | v4 | 真机 3-point 多轮实测；x/y 符号定稿为 `+1/+1`（同向站位恒等）；mink 权重 1.5/(1,0.7,1) |
| 2026-10-30 | v3/v4 | PC2 已装 mink；bundle 重建；v4 tape 端到端 50 Hz、engage 窗口正常、腕 p2p ≤0.0038 rad、ikerr ≤2.1e-6 m |

**当前部署状态（写于 2026-11-03）**：

- 本机 v3 bundle 的 `bundle_info.txt`：`built_at=2026-10-30T10:29:42Z`、
  `source_commit=c6db344`、**`source_dirty=yes`**。也就是说这份 dist 包含当时未提交的改动；
  如果 PC2 部署的是它，同样带这些改动。**下次部署前必须用当前仓库重新 `build_bundle.sh`
  并核对 `SHA256SUMS`**，不要直接复用旧 dist。
- 仓库当前 `main = d116ec8`（2026-10-30 21:12），已包含 MC handoff 修复。
- 2026-11-03 本机 `eth9` 未枚举、PC2 `10.0.1.41` 不可达（上网走 eth8 热点）；
  下一次上机第一步是恢复 eth9 直连，再按 §4 检查。
- v4 真机**不是** SONIC 的 SMPL 全身链路，而是“laptop sender → PC2 v3 节点直连 HAL”；
  不要套用 SONIC 的 ritual/engage 语法（`RT+B`、`wb_engaged` 都与这条链无关）。

---

## 13. 附录

### A. 端口

| 端口 | 用途 |
|---|---|
| **5573** | v2/v3/v4 intent（laptop sender → PC2 节点） |
| **5572** | `hand_finger_cmd`（msgpack；CLAW 手桥 BIND） |
| 5563 | planner_cmd（v2 早期订阅；v3/v4 不需要） |
| 15573 | 仿真 harness 的 intent 端口（v4 sim 默认） |
| 63901 | laptop 本地 Pico PC-Service（对外） |

### B. 文件清单（v3/v4 bundle 8 个 `.py`）

```
incremental_upper_teleop_v3.py
incremental_chain_v3base.py
incremental_chain_v3.py
x2_arm_ik.py
x2_arm_fk.py
x2_arm_mink.py
x2_arm_kinematic.py      # 生成文件，随 MJCF 重建
upper_body_teleop.py     # HAL 表
```

### C. 一页命令卡（v4 真机）

```bash
# 0) 网络/吊架
ip -br addr show eth9
ping -c2 -I 10.0.1.10 10.0.1.41

# 1) PC2：状态 -> 瘫软
ssh Agibot-X2-PC2-Orin-NX
source /opt/ros/humble/setup.bash
source /home/agi/aimdk/install/setup.bash
source ~/Workspaces/czq/ros2_ws/install/setup.bash
ros2 run py_examples migrate_system_state Develop_MC

# 2) PC2：节点（先 dry）
ros2 run czq_examples incremental_upper_teleop_v3 --dry-run --record /tmp/v3_dry.npz --duration 20
ros2 run czq_examples incremental_upper_teleop_v3 --record /tmp/v3_live.npz

# 3) PC2：手桥（要夹爪）
setsid nohup /home/agi/gear-sonic/venv/bin/python3 \
  /home/agi/Workspaces/czq/gear-sonic/gear_sonic_deploy/scripts/x2_hand_zmq_to_aimdk_bridge.py \
  --wire-format msgpack --zmq-host '*' --zmq-port 5572 \
  --zmq-topic hand_finger_cmd --hand-mode claw --sides auto \
  > /tmp/hand_bridge.log 2>&1 < /dev/null &
ss -ltnp | grep 5572

# 4) laptop：sender
cd /home/nvidia/Documents/czq/GR00T-WholeBodyControl-X2
./gear_sonic/scripts/run_pico_teleop_v4_real.sh --pc2-host 10.0.1.41

# 5) 操作：grip=离合，trigger=手，X/A 长按 3s 回位
# 6) 收尾
#    Ctrl-C sender -> Ctrl-C 节点 -> migrate Ready
ros2 run py_examples migrate_system_state Ready
```

### D. 一句话交接

> **v4 = 只戴头显 + 双手柄，捏 grip 抱臂、扣 trigger 合爪；PC2 起 v3 节点和 CLAW 手桥，
> 全程吊架 + `Develop_MC`。方向不对先看 `[frame] +1/+1` 和腕 `0,2,1 / pitch -1`，
> 别现场试符号；换人/换握法先重标 lever。旧文档的“扳机离合”是 v2，不要照抄到 v4。**

---

*整理日期：2026-11-03。依据：`czq_test.md`、`czq_test_v2_incremental.md`、
`czq_quickstart_incremental_teleop.md`、`x2_pc2/v3_upper_teleop/README.md`、
`docs/x2/F03_pico_teleop.md`、`docs/x2/PICO_LIVE_TELEOP_PITFALLS.md` 与
2026-10-12 ~ 2026-10-30 的真机记录。*
