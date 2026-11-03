# SONIC 全身遥操真机上手指南（AgiBot X2 Ultra）

> **适用对象**：第一次接手 SONIC WBC 真机实验的操作者。
> **最后更新**：2026-11-03 · **当前状态**：**L4 真机全身遥操已于 2026-10-30 验收通过**。
> 本文是本实验室的“从零到全身跟随”操作手册；仓库里的 `docs/x2/`（F01~F12）仍是权威细节，
> 两者冲突时以仓库文档和实际代码为准。
>
> 相邻项目（Pico 直驱上肢、不走 SONIC 策略）见
> [`x2_upper_body_teleop_handover_CN.md`](x2_upper_body_teleop_handover_CN.md)。
> 两条链路**不要混着操作**：SONIC 是“策略 + 自平衡”，上肢遥操是“闭式解 IK + 直发关节、必须吊架”。

---

## 0. 30 秒认识这个项目

**项目意图**：把 **AgiBot X2 Ultra（31 DOF）** 接成 NVIDIA GEAR-SONIC 的第二个本体。
上游 SONIC 原本是 G1 的全身控制/运动跟踪；本分支提供 X2 的训练、微调、MuJoCo 仿真、
PC2（机器人机身内 Jetson）真机部署，以及手柄 / Quest 3 / **Pico 全身遥操**三种输入。

核心数据流（记住这一条就理解一半）：

```
Pico 头显 (SMPL 24 关节 + 双手柄)
        │  Wi-Fi / XRoboToolkit
        ▼
laptop  sender  ── :5573 pico_intent (50 Hz) ──▶  PC2
                                                    │
                              SONIC ONNX 策略 (50 Hz) │  ← 参考位姿 / SMPL token
                                                    ▼
                                          安全栈（倾角守卫 / SAFE_HOLD / E-STOP）
                                                    │
                                                    ▼
                                          HAL 关节命令 (~500 Hz)
                                                    │
                                        真机电机 / MuJoCo（仿真）
```

- **Pico 全身**走 **SMPL encoder** 路径（全身 31 DOF 跟随）。
- **手柄 / Quest**走 **kplanner / pose encoder** 路径（走路、转向、手势库）。
- 两条路径由 `x2_pose_merger.py` 仲裁，`X2_WHOLE_BODY_TELEOP=1`（默认）时可在同一场会话里切换。
- 仿真栈跑的是**真正的 C++ deploy 二进制**（docker 内），宿主 MuJoCo 只当被控对象，
  所以“仿真通过”与“真机可跑”之间距离比一般仿真小，但仍**不等于**真机放行（见 §3、§6）。

### 两条链路的分工

| | 上肢直驱遥操（另一个项目） | **SONIC WBC（本文）** |
|---|---|---|
| 控制方式 | Pico → 闭式解/IK → 直发关节位置 | Pico/手柄 → SONIC 神经网络策略 → 31 DOF |
| 平衡 | 无（`Develop_MC` 零力矩，**必须吊架**） | **策略自平衡 + 安全栈** |
| 代码 | `x2_sdk_v100` + `czq_examples` | `GR00T-WholeBodyControl-X2` + PC2 SONIC 栈 |
| 入口 | `ros2 run czq_examples ...` | **ritual 点火 + `run_pico_teleop.sh --pc2-host`** |
| 适用 | 快速手臂/腰/夹爪验证、数据采集 | 全身跟随、行走、动作库、策略评估 |

---

## 1. 角色、地址与路径

### 1.1 本实验室实际值

| 角色 | 是什么 | 地址 / 路径 |
|---|---|---|
| **laptop（本机）** | Jetson AGX Thor，跑 sender、监控、仿真 | 机器人侧网卡 `eth9`：**`10.0.1.10/24`**（另有 `192.168.254.10/24`） |
| **PC2** | 机器人机身内 Orin NX，跑 SONIC 全栈 | **`10.0.1.41`**，ssh 用户 `run`（若别名不同按实际替换） |
| **PC1** | 运动控制器（MC）主机，EM HTTP API | **`10.0.1.40:50080`**，app 名 `mc` |
| Pico 头显 | 操作者输入 | 与 laptop 同一 Wi-Fi；具体 IP 以头显 Wi-Fi 页为准 |
| 仓库（laptop） | 源码 + 仿真 + sender | `/home/nvidia/Documents/czq/GR00T-WholeBodyControl-X2` |
| PC2 安装前缀 | SONIC 栈 | `/home/agi/gear-sonic`（软链到 `~/Workspaces/czq/gear-sonic`） |

> **网络红线**：机器人网真路是 **eth9 直连**（本机 `10.0.1.10` → PC2 `10.0.1.41`，约 0.3 ms）。
> `eth9` 是 **USB 100M dongle，会掉载波**。不通时先看 `ip link show eth9` 是否 `NO-CARRIER`
> 或干脆没枚举，不要先怀疑 sshd。`eth8`（手机热点）是上网用的，不能到机器人网。
> 多网卡时建议 ssh 配置 `BindInterface eth9` + `ConnectTimeout 5`，避免切网后静默卡死。

### 1.2 关键路径速查

```
仓库(laptop):  /home/nvidia/Documents/czq/GR00T-WholeBodyControl-X2
PC2 栈:         /home/agi/gear-sonic/
  ├── ritual_start_sonic.sh / ritual_start_demo.sh   点火链
  ├── robot_env.env                                  唯一参数注入点
  ├── planner_stack/                                 kplanner + 模型 + 动作库
  ├── policies/                                      SONIC ONNX 模型
  └── log/                                           ritual_fired.log / deploy_ritual_*/
端口: 见 x2_pc2/PORT_REGISTRY.md；仿真端口 = 真机端口 +100
```

### 1.3 环境隔离（别踩）

- **sender 用项目内 `.venv_teleop`**（`xrobotoolkit-sdk` + CPU torch + mujoco）。
- **仿真/环境体检用项目内 `.venv`**（numpy 1.26.4 钉死）。
- **不要**用 `czq/xr-env`（numpy 2.x）跑本项目，也不要把两者互相装包。
- `git-lfs` 在 `/home/nvidia/miniconda3/bin`，不默认在 PATH；push 前要
  `export PATH=/home/nvidia/miniconda3/bin:$PATH`，否则 pre-push hook 直接 exit 2。

---

## 2. 版本更迭与能力对比

### 2.1 验收阶段 L1~L5（当前进度）

| 级别 | 内容 | 状态 |
|---|---|---|
| **L1 环境** | `.venv`、docker、git-lfs、模型 LFS、无 docker 运动学回放 | ✅ 2026-09-24 |
| **L2 仿真栈** | docker 内 C++ deploy + MuJoCo + ONNX kplanner | ✅ 2026-09-29（RTF 0.96~0.97） |
| **L3 Pico 遥操（tape）** | 无头显回放，验证 SMPL 路径与倾角守卫 | ✅ 2026-09-30（根倾角守卫修复后整盘 `FATAL=0`） |
| **L3 Pico 遥操（live）** | 头显实时驱动仿真里的 X2 全身 | ✅ 2026-10-08 |
| **L4 真机部署** | PC2 bring-up、colcon、ritual 点火、MC 交接 | ✅ 2026-10-10 起；2026-10-30 进 `CONTROL` |
| **L4 真机全身遥操** | Pico → PC2 deploy → 真机全身跟随 | ✅ **2026-10-30 验收通过** |
| **L5 训练/微调** | Nebius 集群训练、导出新模型 | 不在本机做（见 `docs/x2/F12_training_nebius.md`） |

> L4 的“点火成功”和“全身遥操成功”是两件事：点火只证明 deploy 接管了总线并进入 `CONTROL`；
> 全身遥操还要求头显 body 流健康、engage 被接受、`wb_engaged` 翻转。

### 2.2 模型版本

仓库自带默认集，**不需要先训练**：

| 模型线 | 文件（`gear_sonic_deploy/models/`） | 特点 | 当前用途 |
|---|---|---|---|
| **native 3-encoder 双头（默认）** | `x2_sonic_v16ft8_45000_dual.onnx`（obs 2511） + `_g1.onnx` 兄弟 | 一个 session 同时带两个 reference head；**不需要 token service** | 真机默认 `X2_RITUAL_MODEL`；仿真 `--whole-body-teleop` 默认 |
| native 3-encoder 单图 | `<name>_g1.onnx`（obs 1670） | 手柄/kplanner 主槽 | 由 `_dual` 自动找兄弟；也可单独用 |
| native token 路径（legacy） | `<name>_g1_token.onnx` + `<name>_smpl_tokenizer.onnx` | token 输入；需要 token service | 仅显式切换时用 |
| **frozen G1-core + X2 LoRA** | `x2_sonic_s1ft16000_g1.onnx` + `_g1_token.onnx` + `x2_smpl_tokenizer_v11release.onnx` | 保留公开 G1 SONIC 核心，只训 LoRA 适配 X2 | 历史/对照；不是 launcher 默认 |
| 更早的 native incumbent | HF `tinkerbuggy/sonic-x2/demo_20260909/incumbent_39k/` | 早期 native 集 | 复现/对照 |

**调参档必须与模型血统配对**（`X2_RITUAL_TUNING`）：

| 预设 | 适用 |
|---|---|
| `trained_gains_s0_inc_waistmc.yaml` | **默认**；shipped v16ft8 集，腰 pitch 按厂商 MC 刚度加强 |
| `trained_gains_s0.yaml` | vendor-plant 自训模型（unity gains） |
| `bigrun.yaml` | legacy pre-vendor-plant 模型；吊架验证过，勿手改 |

> 软化的 hip/knee 档已在硬件上证伪，**不要再试**。

模型相对 G1 stock 的能力（README_X2 的 Isaac Lab 评分，供选型参考）：

| 指标 | G1 stock | X2 trained |
|---|---|---|
| novel500 成功率 | 97.8% | 97.2% |
| hard300 成功率 | 89.0% | 73.3% |
| PHUMA OOD 成功率 | 86.8% | 66.2% |
| Pico captures（SMPL encoder） | 40.7% | **66.7%** |

### 2.3 三种操作输入

| 输入 | 路径 | 用途 |
|---|---|---|
| **手柄** | `planner_cmd :5563` → kplanner → pose encoder | 走路、转向、手势库、e-stop；**始终在线** |
| **Quest 3** | WebXR → planner + arm IK | 手臂遥操、移动（F02） |
| **Pico 全身** | `pico_intent :5573` → SMPL tokenizer → pose watchdog | 全身跟随（F03，本文重点） |
| 动作回放 | `motion_clip :5568` / Pico tape | 动作库、回归测试（F04/F05） |

### 2.4 关键修复时间线（交接必须知道）

| 日期 | 事件 / 修复 |
|---|---|
| 2026-09-29 | L2 完整跑通；L3 tape 在第 227 s 摔倒 |
| 2026-09-30 | 摔倒根因：人根朝向持续 ≳1.5 s 倾倒 >30°；命令侧 `root_tilt_guard` 默认开启（25° / 0.6 s ramp）。**memoryless 守卫会把摔倒提前 141 s**，保护必须严格 A/B |
| 2026-10-08 | L3 live 仿真通过；`trackers: 0` 也能跟随（SMPL 路径不依赖物理 tracker）；模式切换 = **按住 RT 再按 B** |
| 2026-10-10 | PC2 bring-up、colcon、MC 交接、kplanner 从 HF 下载；PC2 路径改为 `/home/agi/gear-sonic` |
| 2026-10-30 | 修复 ritual 预检被 onnxruntime ANSI 警告污染、MC handoff `stop_app` 5 s 超时 + `aima` fallback 卡 TTY；**L4 真机点火进 `CONTROL`** |
| 2026-10-30 | **L4 真机全身遥操验收通过**；首次 `ENGAGE REFUSED (identical-streak 11493)` 根因是头显体感解算冻结，重标定 + app 前台后恢复 |
| 2026-10-30 | 退出时 eth9 再次掉载波；PC2 随后重启（uptime 归零），MC 自动接管 |

### 2.5 遥操链路版本（别和上肢项目的 v3/v4 混）

| 链路 | 数据源 | 用途 | 状态 |
|---|---|---|---|
| **Pico 全身（SMPL）** | 头显 body 流 + 双手柄 | 全身 31 DOF 跟随 | **本文主线，L4 已验收** |
| Pico tape 回放 | 录制好的 intent tape | 回归/无头显测试 | 可用；自带 tape 不驱动 locomotion |
| token 路径 | `_g1_token.onnx` + SMPL tokenizer | 旧 frozen G1-core 模型 | legacy；默认双头模型不需要 token service |
| **3-point 上肢（v3/v4）** | 仅双手柄，无 body tracker | **只驱动上半身，不走 SONIC 策略** | 属另一个项目，见上肢指南 |

> 仓库 commit 里的 `v3` / `v4` 指**上肢增量为 IK 链**，不是 SONIC 全身遥操的版本号。
> 看到 `run_pico_teleop_v4_real.sh` 时不要以为它走 SONIC 策略——它是 laptop sender
> 直连 PC2 上肢节点、直发 HAL 关节命令。本文的入口始终是 `run_pico_teleop.sh --pc2-host`。

---

## 3. 安全须知（每次上机前重读）

1. **首次上真机 WBC、任何新模型/新调参档的第一跑，必须上吊架或有人护住机器人。**
   策略有自平衡，但模型不匹配时没有任何软件能替你兜底。
2. **手柄 e-stop 永远在线**（即使当前用 Pico）：
   按住 **A+X**，然后**连续扣放两个扳机（LT/RT）**。
   第一阶段 = 软停（回站立）；继续扣约 1 s = **纯阻尼（全身瘫软）**。
   机器人趴着/深蹲时直接扣到全阻尼。
3. **清除 e-stop**：松开 A+X，**等至少 5 s**，完整重复一遍手势。
   清除后机器人不会立刻动——需要有人扶正、静止约 2 s，deploy 才会走
   `SAFE_HOLD -> RECOVER` 自动站起。
4. **永远不要对站立着的机器人 `tmux kill-session` deploy 会话**——力矩瞬间掉零，机器人直接摔。
   停机只能走 §4 Step G 的正规流程。
5. **Pico 中 Ctrl-C laptop sender 是合法急停**：机器人一个 tick 内回落 kplanner/手柄。
6. **deadman 纪律**：手柄驾驶必须按住 **L2**；Pico 全身模式必须保持 engage（进入 WHOLE_BODY 自动 engage，
   退出/断流即释放）。
7. **lidar 避障守卫默认关闭**（`ENABLE_SCAN_GUARD=0`），操作者 deadman 是唯一停止手段，人不要离开。
8. **MC 重启 = 零力矩瞬间**（不是慢慢卸力）。不要把“重启一下试试”当排查手段。

---

## 4. 每次实验的完整流程

```
┌─ A 开机/网络 ─→ B laptop 自检 ─→ C PC2 状态确认 ─→ D 点火 ─→ E 验证
│                                                    │
│                              ┌─ F1 手柄驾驶（走路/手势） ─┐
│                              └─ F2 Pico 全身遥操 ────────┴─→ G 停机收尾
└────────────── 出问题随时 e-stop，先按 §7 判读 ─────────────────────────┘
```

### Step A — 开机与网络

```bash
# 1) 确认 eth9 在、有载波（USB dongle 老毛病）
ip -br addr show eth9
ip link show eth9 | grep -E 'state|NO-CARRIER'

# 2) 用机器人侧源地址 ping PC2
ping -c 3 -I 10.0.1.10 10.0.1.41

# 3) ssh 确认
ssh -o ConnectTimeout=5 run@10.0.1.41 'hostname && uptime'
```

- 机器人在厂商 MC（运动控制器）下保持**站立**，操作者在旁，手柄呼吸灯亮（已与 PC2 配对）。
- 如果 `eth9` 不存在或 `NO-CARRIER`：先查 USB 口/线/供电，必要时重插；不要改 sshd。
- 如果 ssh 卡住但 ping 通：检查 ssh 配置是否 `BindInterface eth9`，以及是否有残留的旧连接。

### Step B — laptop 侧自检

```bash
cd /home/nvidia/Documents/czq/GR00T-WholeBodyControl-X2
source .venv/bin/activate

python check_environment.py                 # LFS/模型/依赖体检，应全 PASS
pgrep -x RoboticsService                    # Pico PC-Service 应常驻（端口 63901）
# 没有就：
# bash /home/nvidia/Documents/czq/setup/start_service.sh
```

> 注意：`check_environment.py` 用 `.venv`；真正起 sender 时再切到 `.venv_teleop`。
> 不要在同一 shell 里反复手改 `PYTHONPATH`，两个环境都由脚本自己管。

### Step C — PC2 现状确认（只读）

```bash
# 一键体检（laptop 侧）
./gear_sonic_deploy/scripts/pc2_preflight.sh --pc2-host 10.0.1.41

# 当前参数注入点（PC2）
ssh run@10.0.1.41 \
  'grep -E "^: .*(X2_RITUAL_MODEL|X2_RITUAL_TUNING|X2_WHOLE_BODY_TELEOP|X2_ANCHOR_ORI_MODE)" /home/agi/gear-sonic/robot_env.env'

# 不应有残留会话；有 x2_deploy 等 = 上一场没收干净，先按 Step G 停机
ssh run@10.0.1.41 'tmux ls' || echo "no tmux sessions (正常)"
```

预期：

- `X2_RITUAL_MODEL=.../policies/x2_sonic_v16ft8_45000_dual.onnx`
- `X2_RITUAL_TUNING=.../trained_gains_s0_inc_waistmc.yaml`
- `X2_WHOLE_BODY_TELEOP=1`
- `X2_ANCHOR_ORI_MODE=body`

### Step D — 点火（二选一）

**方式 1（标准）：手柄和弦点火**

1. 机器人站立、MC 在管、手柄已连接；
2. **按住 L1+L2+R1+R2 约 3 s**（手柄震动倒计时），松开，按 **Y**；
3. 和弦触发 `ritual_start_demo.sh`：watchdog → kplanner → pose 流门禁（约 12 s）→
   deploy（姿态预检 → 停 MC → 总线交接）→ pad bridge。

**方式 2（替代）：ssh 手动点火**
（手柄不可用时；⚠️ 跳过 Y 确认门，只在校验完 Step C 且人在机器人旁边时用）

```bash
ssh run@10.0.1.41 'bash /home/agi/gear-sonic/ritual_start_sonic.sh'
```

点火链会自动：停 MC（`stop_app`，30 s 超时）→ deploy 拿总线 → soft-start 进入 `CONTROL`。
机器人在此过程中保持站立，短暂轻微下沉属正常。

### Step E — 验证点火成功

```bash
# 1) ritual 日志：等出 GATE PASSED 和 x2_deploy STARTED
ssh run@10.0.1.41 'tail -f /home/agi/gear-sonic/log/ritual_fired.log'
#    看到：GATE PASSED ... -> x2_deploy STARTED（Ctrl-C 退出 tail）

# 2) deploy 面板（只读快照，推荐；不要 Ctrl-C）
ssh run@10.0.1.41 'tmux capture-pane -p -t x2_deploy -S -60'

# 3) 会话清单
ssh run@10.0.1.41 'tmux ls'
```

**deploy 面板必须依次出现：**

```
stopping MC via PC1 EM HTTP API
[handoff] bus check OK: 1 publisher (deploy)
CONTROL
WHOLE-BODY DUAL-HEAD armed ... SMPL intent bound at tcp://0.0.0.0:5573 ... hold-on-drop ON.
Loaded ONNX: ... kind=native_dual_head
[plant] RUNTIME PLANT LOADED: vendor_20260823
```

**期望 tmux 会话：**

```
x2_pose_watchdog  pc2_kplanner  x2_deploy  pad_bridge  x2_hand_bridge  x2_motor_monitor
```

⚠️ `Loaded ONNX:` 的路径才是机器人**真正在跑**的模型（不要以推过的文件为准）。
任何一步卡住：不要按手柄、不要重试和弦，先按 §7 判读。

### Step F1 — 手柄驾驶（走路 / 手势库）

| 动作 | 按键 |
|---|---|
| 点火 | L1+L2+R1+R2 ~3 s → Y |
| 行走/转向 | **按住 L2** + 左摇杆前后；右摇杆 X = 转向 |
| 停走 | 松开 L2 |
| 速度 | 锁定档（`KPLANNER_FIXED_FWD_MPS=0.35`）；X/Y 在 Pico 模式下 ±0.1 m/s |
| 播手势/舞蹈 | L1+Y / L1+X / L1+B / L1+A 轮换 gesture bank；**L1+R1** 停止播放 |
| 蹲伏模式 | 按住 L2+R2 + 十字键下（需 G1-core planner） |
| **e-stop** | **A+X 按住 + 连扣两个扳机**（详见 §3） |

完整键位表：仓库 `docs/x2/gamepad_cheatsheet.md`。

### Step F2 — Pico 全身遥操

#### (1) 头显侧准备（每次实验）

1. Pico 与 laptop 连同一个 Wi-Fi（`ping <头显IP>` 时延 < 100 ms）。
2. 打开 **XRoboToolkit** app → 连接 laptop 当前与头显同网段的 IP → **send data ON** →
   进入 VR 场景并**保持 app 在前台**。
3. Motion Tracker app：选 **Full body** 并 **Calibrate**。
   - **物理 tracker 不是硬性前提**：SMPL 路径只需要 body 流，`trackers: 0` 也能跟随
     （2026-10-08 实测）。如果已经配对 tracker，建议腰 + 双踝共 3 个；配对前先把
     “max tracker usage” 改成 3（默认 2）。
   - 头显重启会丢标定，**每次重启后重新 Calibrate**。

> laptop 的 IP 不要写死。`run_pico_teleop.sh` 启动时会打印**候选地址列表**，按头显
> Wi-Fi 页显示的网段挑一个填进 app；`LAPTOP_HOST=<ip>` 可强制指定。

#### (2) laptop 侧连通性探针（engage 前必做）

```bash
cd /home/nvidia/Documents/czq/GR00T-WholeBodyControl-X2
.venv_teleop/bin/python -c \
"import xrobotoolkit_sdk as xrt, time
xrt.init(); time.sleep(2)
print('body:', xrt.is_body_data_available(),
      'stamp:', xrt.get_time_stamp_ns(),
      'trackers:', xrt.num_motion_data_available())"
```

期望：`body: True`，`stamp` 在增长，`trackers: 0/2/3` 都可接受。
`body: False` → 重标定 / app 是否前台 / 是否同一 Wi-Fi。

#### (3) 启动 sender（真机 = 带 `--pc2-host`）

```bash
cd /home/nvidia/Documents/czq/GR00T-WholeBodyControl-X2
source .venv_teleop/bin/activate
./gear_sonic/scripts/run_pico_teleop.sh --pc2-host 10.0.1.41

# 可选：头显里看机器人相机（需 PC2 相机桥）
# ./gear_sonic/scripts/run_pico_teleop.sh --pc2-host 10.0.1.41 --cam orbbec --cam-preview
```

- **同一目标 host 只允许一个 sender**（与 `run_pico_replay.sh` 互斥，否则 refuse 退出）。
- `--help` 会被**原样透传**给 `pico_intent_sender.py`（打印 usage 后退出，不会起 sender）；
  查参数直接看它的 argparse。
- 期望输出：

```
intent PUB -> tcp://10.0.1.41:5573 topic 'pico_intent'
[intent] 50.0 Hz  disengaged  [body OK (last new frame 0.0s ago, identical-streak 0)]
```

`body OK` 是健康信号；进入 WHOLE_BODY 后会出现 `ENGAGED [body OK ...]`。

#### (4) 进入 WHOLE_BODY 并确认 engage

- **按住右扳机（RT），再按一下 B**：进入 WHOLE_BODY，**进入瞬间自动 engage**，不需要按 A。
- 确认两处输出：
  - sender：`ENGAGED [body OK ...]`
  - deploy 面板：`WHOLE-BODY ENGAGED — smpl head`
- 退出：再按住 RT + B 回到 LOCOMOTION。
- **WHOLE_BODY 内单按 A 是 engage / disengage 开关**。
- **grip 只驱动 OmniHand 手指，不是离合**——这是最容易按错的一点。

#### (5) 首次成功判据

```bash
# PC2 上看到 sender 的连接
ssh run@10.0.1.41 'ss -tn "sport = :5573"'

# 可选：只读 x2_debug 观察 wb_engaged / safe_idle / tilt_trip
# 工具与完整用法见 ../czq/x2_sonic_runbook.html 的“x2_debug 监听脚本”
```

`x2_debug` 里 `wb_engaged` 由 0 翻 1，且机器人全身跟随，才算这一级通过。
2026-10-30 验收时 engage 后约 166 s 起多次翻转，用户确认“测试成功”。

### Step G — 停机与收尾（顺序不要乱）

```bash
# 1) 松开 grip，确认吊架/人员就位（下一步有零力矩窗口）
# 2) Ctrl-C laptop 的 sender
#    → deploy 的 SMPL 头 hold-on-drop，机器人不再跟随，但仍由 SONIC 保持站立

# 3) 正规停机：RAMP_OUT -> 交还 MC（中间是零力矩瞬间，必须有人扶/吊架承重）
cd /home/nvidia/Documents/czq/GR00T-WholeBodyControl-X2
./gear_sonic_deploy/scripts/x2_pc2_daemons.sh stop --pc2-host 10.0.1.41
#    --yes            跳过交互确认（脚本化时用）
#    --no-mc-restart  不自动拉起 MC

# 4) 确认收干净
ssh run@10.0.1.41 'tmux ls; pgrep -af x2_deploy_onnx_ref'
ssh run@10.0.1.41 'source /opt/ros/humble/setup.bash; \
source /home/agi/aimdk/install/aimdk_msgs/local_setup.bash; \
ros2 topic info /aima/hal/joint/leg/command -v 2>/dev/null | \
grep -E "Publisher count|Node name" | head -4'
```

期望：tmux 无会话、无 `x2_deploy_onnx_ref` 进程；`leg/command` 的 publisher 回到
`mc_ros2_node3100`（`Publisher count: 1`）。脚本最后打印 `safe to fire the ritual` 才算干净。

**紧急情况优先级：手柄 e-stop（全阻尼）> Ctrl-C sender > `x2_pc2_daemons.sh stop`。**

---

## 5. 模式与按键语法

### 5.1 LOCOMOTION vs WHOLE_BODY

| 维度 | LOCOMOTION | WHOLE_BODY（engage） |
|---|---|---|
| 权威来源 | kplanner（pose encoder） | SMPL token 流 |
| 走路/转向 | 摇杆 + LT deadman | LT + 操作员步伐（micro-step 门控） |
| 上肢 | 默认位形/锁存 | 全身 31 DOF 跟随 |
| 手指 | `hand_finger_cmd` 独立线，**模式无关** | 同左 |
| 退出 | — | RT+B 退出，限速回 kplanner（不是瞬跳） |
| 断流 | 手柄不受 body 断流影响 | 立即 DISENGAGED，一拍内回落 kplanner/pad |
| engage 门 | 不需要 | body 新鲜（1.2 s 内新帧 + identical-streak < 50）+ deploy 侧 <0.50 s 双重门 |

### 5.2 Pico 手柄映射（SONIC WBC）

| 动作 | 按键 |
|---|---|
| OFF ↔ LOCOMOTION | **A+B+X+Y** 同按约 0.4 s |
| LOCOMOTION ↔ **WHOLE_BODY** | **按住 RT，再按 B**；进入即自动 engage |
| WHOLE_BODY 内 engage / disengage | 单按 **A** |
| 走路/转向（LOCOMOTION） | 摇杆，**按住 LT 作 deadman** |
| 速度 ±0.1 m/s | **X / Y** |
| 灵巧手手指 | **grip**（不是离合） |
| **e-stop** | **A+X 按住 + 扣放两个扳机** |
| 录制片段（可选） | 长按 **X** ≥ 0.8 s 开始，长按 **Y** ≥ 0.8 s 结束 |

> 2026-10-08 起，进入 WHOLE_BODY 的正确手势是 **RT+B**，不是单按 B；
> 单按 B 会被 deadman 忽略。F03 旧版本写“单按 B”已过时。

---

## 6. 换模型 / 换调参档（进阶）

模型在**点火时加载**，换模型 = 改 `robot_env.env` + 重启 ritual。

```bash
# 1) laptop 上核对 obs contract（body/heading 必须与模型一致，exit 0 才能上）
python gear_sonic/scripts/onnx_obs_contract.py <你的模型>.onnx --require body

# 2) PC2 上改 robot_env.env
ssh run@10.0.1.41 'vi /home/agi/gear-sonic/robot_env.env'
#   X2_RITUAL_MODEL    = ${PC2_PREFIX}/policies/<你的模型>_dual.onnx
#   X2_WB_TOKEN_MODEL / X2_WB_TOKENIZER（仅 token 路线需要）
#   X2_RITUAL_TUNING   = 与模型血统配对的调参档

# 3) 推送模型到 PC2（md5 门禁）
./x2_pc2/push_to_pc2.sh \
  --manifest x2_pc2/push_manifest_wholebody_dual.txt \
  --pc2 run@10.0.1.41 --dry-run
./x2_pc2/push_to_pc2.sh \
  --manifest x2_pc2/push_manifest_wholebody_dual.txt \
  --pc2 run@10.0.1.41
# 期望结尾：VERIFY OK (N files)。push 不会重启任何进程，之后重新点火。
```

命名约定：`<name>_dual.onnx` + `<name>_g1.onnx` **成对**，且 `codec_fingerprint` 必须一致；
`onnx_provenance.py check` 会卡这一对。

调参档速记：

- `trained_gains_s0_inc_waistmc.yaml`：shipped 集默认（腰 pitch 按 MC 刚度加强）。
- `trained_gains_s0.yaml`：vendor-plant 自训集（unity gains）。
- `bigrun.yaml`：legacy 模型；吊架验证过，勿手改。
- **不要**用软化的 kp 档（已在硬件上证伪）。

---

## 7. 故障排查

### 7.1 判读顺序（非常关键）

**先看 sender 的状态行，再决定动哪一层。** 不要一上来重启 PC2。

1. sender 行：`rate` 是否 50 Hz、`body OK` 还是 `[NO ...]`、`identical-streak`、`last new frame`。
2. PC2 `:5573` 有没有 ESTAB（`ss -tn "sport = :5573"`）。
3. deploy 面板：`CONTROL`、`Loaded ONNX`、`WHOLE-BODY ENGAGED`、`FATAL`。
4. `x2_debug`：`wb_engaged`、`in_safe_idle`、`tilt_trip`、`pose_ref_starved`。
5. 最后才查网络（ping/eth9）。

### 7.2 症状 → 判读 → 处理

| 症状 | 判读 | 处理 |
|---|---|---|
| `ENGAGE REFUSED: body stream dead/frozen (identical-streak 大)` | 头显体感解算冻结，在反复重发同一帧缓存（**不是网络**） | 头显内 Motion Tracker → Full body → **Calibrate**，保持 app 前台。恢复后 sender 打 `body stream LIVE again`，再 **RT+B** 重进 |
| `identical-streak 0` + `last new frame` 持续变大 | 新帧根本没来（头显睡眠 / 掉线 / app 退后台） | 唤醒头显、确认 send data ON、同一 Wi-Fi |
| `body: False` 一直不变 | app 退后台 / Full-body 没开 / 标定丢失 / 不同 Wi-Fi | 重标定、app 前台、检查网络 |
| probe `trackers: 0` | **SMPL 路径不需要物理 tracker** | 忽略；若用 tracker，检查 max tracker usage=3 并重新配对 |
| `[tiltcap] ... torso on 99.99% ... root 0%` | 25° 根倾角守卫在钳制躯干（坐姿/后靠时正常） | 不是故障；站直时仍持续超限才重标定 |
| PC2 `:5573` 没有 ESTAB | sender 没起 / 网络不到 / 端口被占 | 先起 sender、查 `ss -tn`，再决定重启 |
| sender 启动即退（refuse） | 已有另一个 sender 指向同一 host | `pgrep -af pico_intent_sender`，Ctrl-C 旧的 |
| `Address already in use :5558` | 有残留进程占 watchdog 端口 | 按 Step G 停干净再点火 |
| ritual 日志没有 `GATE PASSED` | kplanner 没流出 pose | `tmux capture-pane -p -t pc2_kplanner \| tail -30` |
| deploy 面板没有 `Loaded ONNX:` | 模型路径 / 配对指纹错 | 看 x2_deploy 面板报错；核对 `robot_env.env` 和 `onnx_provenance.py check` |
| deploy 卡在 `stopping MC via PC1 EM HTTP API` | 旧版 `deploy_x2.sh` 的 5 s 超时 + `aima` fallback 查错 EM 并卡 TTY（2026-10-30 已修） | 当前版本应 30 s 超时并打印 HTTP 响应体；仍卡就**重跑同一条 ritual** 自愈（pane 含该行且无 sentinel touched 时，ritual 会清会话重启） |
| ritual pane 无日志直接消失 | 旧版预检被 onnxruntime 的 ANSI 彩色警告污染，`case` 不匹配后 FATAL 自杀（已修：`| tail -n 1`） | 确认 PC2 上是提交 `b8d6a04` 之后的版本；旧版先更新 ritual |
| `Loaded ONNX` 路径不是你以为的模型 | 推过文件 ≠ 机器人真在跑 | 以面板路径为准；必要时重新 push + 点火 |
| 机器人摔倒 | 安全栈已进 SAFE_HOLD/PURE_DAMPING | e-stop 到全阻尼 → 扶正、静止约 2 s → 自动 `SAFE_HOLD -> RECOVER` 站起；看日志确认 |
| push 卡在 `COMMITTED` 门禁 | manifest 里的文件没 commit | 先 `git commit` 再 push（门禁故意防“没存档就上机”） |
| PC2 上 `No module named gear_sonic...` | bring-up 没铺全 | 重跑 `pc2_bringup.sh`（step 7b 重铺 utils） |
| `invalid load key` 加载 pkl | laptop 侧是 LFS 指针 | `git lfs pull`，再跑 bring-up |
| 仿真 RTF 只有 0.51 | `SIM_VIEWER_MODE=container` 在容器内渲染，参考动作被两倍速灌入 | 改宿主侧 viewer，盯 RTF≈1.00 |
| sim 启动因模型不匹配拒绝 | 跑的不是真机部署的那份 | `ALLOW_MISMATCH=1` 仅用于候选模型；真机放行仍要过 MuJoCo 电池 |
| `--help` 打印 usage 后退出 | 参数被透传给内部 sender | 正常行为；查 sender 的 argparse |
| PC2 重启后 deploy/tmux 全消失 | 重启，MC 自动复活 | 检查 `/aima/hal/joint/leg/command` publisher = `mc_ros2_node3100`；从 Step A 重来 |
| eth9 掉载波 | USB 100M dongle 老毛病 | `ip link show eth9` 看 `NO-CARRIER`；重插/换口，别查 sshd |

### 7.3 已被证伪、不要再查的假设

- **“Pico 必须有 3 个 tracker”**：SMPL 路径 `trackers: 0` 也能跟随（2026-10-08）。
- **“摔倒是因为历史状态/接缝/时钟/四元数符号”**：已逐一排除；根因是 tape 里人根朝向持续倾倒。
- **“memoryless 倾角守卫只会变好”**：第一版把摔倒提前 141 s；必须严格 A/B。
- **“`--no-locomotion` 只影响走路”**：对 v2 上肢链会静默关掉整个控制器层（详见上肢指南）；
  SONIC 全身链路里它也会关掉 dual 模式，使 sender 不建手部通道。
- **“deploy 卡在 MC handoff 是网络问题”**：是 `stop_app` 超时 + `aima` fallback 查错 EM。
- **“identical-streak 大 = 网络断”**：恰恰相反，PC2 可能仍有 ~86 KB/s；冻结在头显解算。

---

## 8. 2026-10-30 真机 L4 验收记录（交接快照）

- **结果**：L4 真机全身遥操验收通过。sender = `./gear_sonic/scripts/run_pico_teleop.sh --pc2-host 10.0.1.41`
  （项目 `.venv_teleop`），用户确认机器人全身跟随、测试成功。
- **首次进入 WHOLE_BODY 被拒**：`ENGAGE REFUSED ... identical-streak 11493`。
  PC2 `:5573` 实测持续 ~86 KB/s、`x2_debug` 50 Hz，证明网络与 deploy 正常；
  根因是 Pico 体感解算从头冻结（反复喂同一帧缓存）。
  **修法**：头显内 Motion Tracker 重新标定 + 保持 XRoboToolkit 前台 → streak 归零 →
  sender 打 `body stream LIVE again` → **RT+B** 重进 WHOLE_BODY（进入即自动 engage）。
- **验收证据**：`x2_debug` 的 `wb_engaged` 0→1（engage 后约 166 s 起多次翻转）。
- **退出态**：PC2 于 21:09 重启（uptime 归零），deploy/tmux 全消失，MC 自动复活；
  正常停止命令 `x2_pc2_daemons.sh stop` 没来得及跑（eth9 在 21:06:30 掉载波）。
  结论与正常停止一致：MC 持权、无遥操进程。
- **2026-11-03 交接时网络快照**：本机 `eth9` 未枚举，PC2 `10.0.1.41` 不可达；
  上网走 `eth8`（手机热点，172.20.10.8/28），代理 7897 正常。
  **下一次上机第一步就是恢复 eth9 直连**，不要误以为 PC2 坏了。
- **代码状态**：`main` = `d116ec8`（2026-10-30 21:12），已包含 MC handoff 与 ritual 预检修复；
  比 `origin/main` 领先 50 个提交。`origin` 是上游，**不要推**；自己的 fork 是 `Zachary-CHAN`。

---

## 9. 附录

### A. 端口（真机；仿真 = +100）

| 端口 | 用途 |
|---|---|
| 5558 | pose watchdog → deploy（pose 流健康门禁） |
| 5556 | kplanner → pose merger 参考位姿 |
| 5563 | planner_cmd（手柄/VR → kplanner） |
| 5568 | motion_clip（动作片段注入） |
| **5573** | **Pico 意图流（laptop sender → PC2 deploy）** |
| 5574 | SMPL token（token 路线） |
| 5557 | x2_debug（只读状态流） |
| 63901 | laptop 本地 RoboticsService（Pico PC-Service，对外） |

### B. 点火后 PC2 的 tmux 会话

| 会话 | 内容 |
|---|---|
| x2_pose_watchdog | pose 流看门狗（必须最先起，占 :5558） |
| pc2_kplanner | ONNX kplanner（站姿锚点流 + 手柄指令） |
| x2_deploy | SONIC deploy（策略 + 安全栈，**别乱动**） |
| pad_bridge | 手柄桥（e-stop 所在） |
| x2_hand_bridge | 灵巧手桥 |
| x2_motor_monitor | 电机监控 |

### C. 实验前 60 秒检查卡（可打印）

```
□ 机器人站立，MC 在管，周围清场            □ laptop: check_environment.py PASS
□ 手柄开机且已连接（呼吸灯）               □ ssh -o ConnectTimeout=5 run@10.0.1.41 通
□ 吊架/护场就位（首次或新模型）            □ PC2: tmux ls 无残留
□ 点火（L1+L2+R1+R2 3s → Y）             □ GATE PASSED + Loaded ONNX + CONTROL 三行确认
□ Pico: app 前台 + send data ON + 已标定   □ probe body:True
□ run_pico_teleop.sh --pc2-host 10.0.1.41 □ sender rate 行 body OK
□ 进 WHOLE_BODY: RT+B（进入即 engage）     □ deploy: WHOLE-BODY ENGAGED — smpl head
□ 全程知道 e-stop 在哪: A+X + 扣扳机
```

### D. 文档地图

| 主题 | 文档 |
|---|---|
| 项目导航 / 本机部署 / 9 个坑 | `GR00T-WholeBodyControl-X2/CLAUDE.md` |
| 仿真 10 分钟 quickstart | `docs/x2/README.md` |
| Pico 全身遥操 | `docs/x2/F03_pico_teleop.md` |
| 真机 PC2、colcon、点火 | `docs/x2/F10_real_robot_colcon.md` |
| live 遥操失败特征库（必读） | `docs/x2/PICO_LIVE_TELEOP_PITFALLS.md` |
| 模式/MC 数据流 | `docs/x2/MODES_AND_MC_NOTES.md` |
| 模型变量与配对 | `MODELS.md` |
| 端口登记 | `x2_pc2/PORT_REGISTRY.md` |
| 29 条复现清单（L1~L5） | `x2_repro_checklist.html` |
| 本文件的网页版操作单 | `x2_sonic_runbook.html` |

### E. 一句话交接

> **先恢复 eth9 直连 → `check_environment.py` → PC2 preflight → ritual 点火 → 看 `CONTROL` 和
> `Loaded ONNX` → 起 `.venv_teleop` 的 sender → RT+B 进 WHOLE_BODY → 确认 `wb_engaged=1` →
> 停机走 `x2_pc2_daemons.sh stop`（中间有零力矩窗口）。**
> 出问题先看 sender 的 `identical-streak` / `body OK`，不要先重启 PC2。

---

*整理日期：2026-11-03。依据：`docs/x2/F03`、`F10`、`PICO_LIVE_TELEOP_PITFALLS`、`x2_sonic_runbook.html`
与 2026-09-24 ~ 2026-10-30 的真机记录。发现不一致时，先改仓库文档，再同步本文。*
