# GR00T WholeBodyControl X2

## VR de 3 puntos — SONIC neuronal, PICO sin trackers (simulación)

```bash
./gear_sonic/scripts/sim_onnx_planner.sh --three-point-vr
```

## VR de 3 puntos — SONIC neuronal, Quest 3 (simulación)

```bash
./gear_sonic/scripts/sim_onnx_planner.sh --three-point-vr --input-source quest3
```

## IK — DLS de HandUMI, PICO sin trackers (simulación)

```bash
./gear_sonic/scripts/sim_onnx_planner.sh --ik
```

## VR de brazos — v4 original (simulación)

```bash
./gear_sonic/scripts/run_pico_teleop_v4_sim.sh
```

## Cuerpo completo — PICO con trackers (simulación)

```bash
# Terminal 1
./gear_sonic/scripts/sim_onnx_planner.sh --whole-body-teleop

# Terminal 2
./gear_sonic/scripts/run_pico_teleop.sh
```

## Gamepad (simulación)

```bash
./gear_sonic/scripts/sim_onnx_planner.sh
```

## Quest 3 — planner e IK existentes (simulación)

```bash
./gear_sonic/scripts/sim_onnx_planner.sh --vr-only
```

## Preparar modelos neuronales sin arrancar la simulación

```bash
./gear_sonic/scripts/sim_onnx_planner.sh --three-point-vr --prepare-models
```

## Comprobar comandos sin arrancar servicios

```bash
./gear_sonic/scripts/sim_onnx_planner.sh --three-point-vr --dry-run
./gear_sonic/scripts/sim_onnx_planner.sh --ik --dry-run
```

## Detener el stack SONIC

```bash
./gear_sonic/scripts/simstack_local.sh --stop
```
