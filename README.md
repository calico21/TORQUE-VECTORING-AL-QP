# Torque Vectoring eRacing — Branch Directory Hub

Welcome to the central repository for the Torque Vectoring control logic developed for the Formula Student electric monoplaza. 

This `main` branch serves as the project lobby. The actual implementations, telemetry integration, and control algorithms are isolated in dedicated feature branches detailed below.

---

## 🌿 Branch Directory

| Branch | Description & Features |
| :--- | :--- |
| **`feat-v1-simple-effective`** | **Baseline Rule-Based Controller**<br>- Simple open-loop/closed-loop yaw moment allocation.<br>- Lightweight lookup-table-based torque distribution.<br>- Initial documentation and reference literature included. |
| **`feat-v2-intermediate`** | **State-Feedback & Dynamic Slip Control**<br>- Dynamic yaw rate reference generator.<br>- Closed-loop PI/PID control for active yaw moment compensation.<br>- Tyre slip ratio monitoring and traction limiting. |
| **`feat-v3-al-qp`** | **Constrained Allocation (Augmented Lagrangian QP)**<br>- Real-time Quadratic Programming (QP) wheel torque allocation.<br>- Augmented Lagrangian solver targeting motor torque limits & thermal constraints.<br>- Designed for sub-5ms execution on embedded targets. |
| **`feat-v4-embedded-nmpc-godmode`** | **Embedded Non-linear MPC & Differentiable Twin**<br>- Differentiable vehicle dynamics model.<br>- Non-linear Model Predictive Control (NMPC) for optimal yaw tracking and derating.<br>- STM32 micro-controller deployment code (200 Hz control loop). |

---

## 🛠️ How to Checkout a Feature Branch

To explore or run a specific version of the project, fetch all remote branches and checkout the desired feature:

```bash
# Fetch all remote branches
git fetch --all

# Switch to a specific feature branch
git checkout <branch-name>