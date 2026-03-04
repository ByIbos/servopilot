<div align="center">

# 🎮 servopilot

**Anti-Windup PID Controller with Dual-Axis Support**

*Zero-dependency PID for gimbal control, servo motors, drones, and robotics.*

![Python](https://img.shields.io/badge/Python-3.7%2B-blue?logo=python&logoColor=white)
![Dependencies](https://img.shields.io/badge/Dependencies-None-brightgreen)
![License](https://img.shields.io/badge/License-MIT-yellow)
![Size](https://img.shields.io/badge/Size-~150_lines-blue)

</div>

---

## ✨ Why servopilot?

| Feature | servopilot | simple-pid | pid-py |
|---|:---:|:---:|:---:|
| Anti-windup (integral clamping) | ✅ | ✅ | ❌ |
| DualAxisPID (Yaw + Pitch) | ✅ | ❌ | ❌ |
| Debug introspection (P/I/D terms) | ✅ | ❌ | ❌ |
| Zero dependencies | ✅ | ❌ | ❌ |
| Configurable integral limit | ✅ | Partial | ❌ |
| Output clamping | ✅ | ✅ | ✅ |

> **servopilot** was built for real-time drone gimbal tracking — where you need two synchronized PID loops (yaw + pitch) running at 30+ FPS with zero overhead. No NumPy, no SciPy, no anything. Just pure Python math.

---

## 📦 Installation

```bash
pip install servopilot
```

Or install from source:
```bash
git clone https://github.com/ByIbos/servopilot.git
cd servopilot
pip install -e .
```

---

## 🚀 Quick Start

### Single-Axis PID
```python
from servopilot import PIDController

pid = PIDController(kp=0.5, ki=0.01, kd=0.3)

# In your control loop:
error = target_position - current_position
signal = pid.update(error, dt=1/30)

# Send `signal` to your motor/servo
motor.set_speed(signal)
```

### Dual-Axis Gimbal Control
```python
from servopilot import DualAxisPID

pid = DualAxisPID(kp=0.4, ki=0.008, kd=0.25)

# Every frame:
error_x = target_x - screen_center_x  # Horizontal offset
error_y = target_y - screen_center_y  # Vertical offset

yaw_signal, pitch_signal = pid.update(error_x, error_y, dt=1/30)

# Send to flight controller / gimbal
gimbal.set_yaw(yaw_signal)    # [-1.0, +1.0]
gimbal.set_pitch(pitch_signal) # [-1.0, +1.0]
```

### Debug Introspection
```python
pid = PIDController(kp=0.5, ki=0.01, kd=0.3)
signal = pid.update(error=100, dt=0.033)

# Inspect each PID component:
print(f"P: {pid.p_term:.3f}")  # Proportional contribution
print(f"I: {pid.i_term:.3f}")  # Integral contribution
print(f"D: {pid.d_term:.3f}")  # Derivative contribution
print(f"Output: {pid.output:.3f}")

# Perfect for real-time debug panels and telemetry
```

---

## 🔧 API Reference

### `PIDController(kp, ki, kd, output_limit, integral_limit)`

| Parameter | Type | Default | Description |
|---|---|---|---|
| `kp` | float | 0.5 | Proportional gain |
| `ki` | float | 0.01 | Integral gain |
| `kd` | float | 0.3 | Derivative gain |
| `output_limit` | float | 1.0 | Max absolute output [-limit, +limit] |
| `integral_limit` | float | 100.0 | Anti-windup clamp for integral |

#### Methods

| Method | Returns | Description |
|---|---|---|
| `update(error, dt)` | float | Compute control signal |
| `reset()` | None | Zero all internal state |

#### Debug Properties

| Property | Description |
|---|---|
| `p_term` | Current proportional term value |
| `i_term` | Current integral term value |
| `d_term` | Current derivative term value |
| `output` | Last computed output |
| `integral` | Current integral accumulator |

### `DualAxisPID(kp, ki, kd)`

| Method | Returns | Description |
|---|---|---|
| `update(error_x, error_y, dt)` | (float, float) | Control signals for both axes |
| `reset()` | None | Reset both axes |
| `pid_x` | PIDController | Access X-axis controller directly |
| `pid_y` | PIDController | Access Y-axis controller directly |

---

## 📐 How PID Works

```
                    ┌──────────────────────────────────────────┐
                    │              PID Controller               │
                    │                                          │
  Error ──────────→ ├──→ [Kp × error]          = P term       │
  (target - actual) │                                          │
                    ├──→ [Ki × ∫error·dt]       = I term       │──→ Output
                    │     (clamped ±100)         (anti-windup) │   (clamped ±1.0)
                    │                                          │
                    ├──→ [Kd × d(error)/dt]     = D term       │
                    │                                          │
                    └──────────────────────────────────────────┘

  • P = Reacts NOW (proportional to current error)
  • I = Remembers PAST (eliminates steady-state drift)
  • D = Predicts FUTURE (dampens oscillation)
```

---

## 🎯 Tuning Guide

| Symptom | Fix |
|---|---|
| Slow response | Increase `kp` |
| Overshooting / oscillating | Decrease `kp`, increase `kd` |
| Steady-state error (offset) | Increase `ki` |
| Integral windup (runaway) | Decrease `integral_limit` |
| Output too aggressive | Decrease `output_limit` |

**Quick recipe for drone gimbals:** Start with `kp=0.4, ki=0.008, kd=0.25` and adjust from there.

---

## 🎯 Use Cases

- **🚁 Drone Gimbal** — Keep camera centered on target
- **🤖 Line-Following Robot** — Steer based on sensor error
- **🏭 Industrial Control** — Temperature, pressure, flow regulation
- **🎮 Game AI** — Smooth NPC aiming / steering
- **🔭 Telescope Mount** — Star tracking with pan-tilt servos

---

## 📜 License

MIT License — use it anywhere, commercially or personally.

---

<div align="center">
<i>Built with ❤️ by <a href="https://github.com/ByIbos">ByIbos</a> — extracted from the <a href="https://github.com/ByIbos/Auto-ReID-Drone-Tracker">Auto-ReID Drone Tracker</a> project.</i>
</div>
