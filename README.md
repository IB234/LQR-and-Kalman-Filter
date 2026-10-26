# Control Systems & State Estimation

This repository contains a collection of Python and MATLAB scripts dedicated to modern control theory and state estimation. The implementations focus on analyzing state-space systems, designing Linear Quadratic Regulators (LQR) for optimal control, and implementing Kalman Filters for robust state estimation under noisy conditions.

## Table of Contents
1. [Overview](#overview)
2. [Detailed File Breakdown](#detailed-file-breakdown)
    - [State-Space Analysis (MATLAB)](#state-space-analysis-matlab)
    - [Optimal Control: LQR (Python)](#optimal-control-lqr-python)
    - [State Estimation: Kalman Filter (Python)](#state-estimation-kalman-filter-python)
3. [Dependencies](#dependencies)
4. [How to Run](#how-to-run)
5. [Key Concepts Demonstrated](#key-concepts-demonstrated)

---

## Overview

The codebase is divided into three main control theory topics:
1. **System Property Analysis:** Evaluating whether dynamic systems are stable, controllable, observable, and detectable.
2. **Linear Quadratic Regulator (LQR):** Designing optimal feedback controllers for a double integrator system, exploring how tuning cost matrices ($Q$ and $R$) affects system performance and control effort.
3. **Kalman Filtering:** Estimating the true state of a linear system corrupted by process and measurement noise, evaluating filter convergence, and testing robustness against missing measurements.

---

## Detailed File Breakdown

### State-Space Analysis (MATLAB)

#### `Q1_script.m`
This MATLAB script provides a robust evaluation framework for checking the fundamental properties of discrete/continuous linear state-space systems defined by matrices $A$, $B$, and $C$. 
* **Stability:** Calculates the eigenvalues of the system matrix $A$. The system is deemed stable if all eigenvalues lie strictly inside the unit circle (for discrete systems).
* **Controllability:** Constructs the controllability matrix using `ctrb(A, B)` and checks if it has full row rank.
* **Observability:** Constructs the observability matrix using `obsv(A, C)` and checks if it has full column rank.
* **Detectability:** Applies the Popov-Belevitch-Hautus (PBH) test specifically to unstable modes (eigenvalues $\ge 1$). If an unstable mode loses rank in the PBH matrix, the system is flagged as not detectable.

### Optimal Control: LQR (Python)

#### `Q3_LQR.py` & `Ex1_Control_LQR_with_state_reference_.ipynb`
These files tackle the design and simulation of an LQR controller applied to a discrete-time double integrator system (e.g., modeling 1D kinematics of position and velocity).
* **DARE Solver:** Uses `scipy.linalg.solve_discrete_are` to solve the Discrete Algebraic Riccati Equation (DARE) and compute the optimal infinite-horizon cost matrix $P$.
* **Optimal Gain Calculation:** Computes the feedback gain matrix $K$ using $P$.
* **Trajectory Simulation:** Simulates the closed-loop system striving to reach a target reference state ($x_{ref} = [3, 0]^T$) from various initial conditions. 
* **Cost Matrix Tuning:** Demonstrates the trade-off between tracking performance and control energy by comparing two tuning strategies:
  * *Aggressive Tracking:* High state penalty ($Q$), low control penalty ($R$). Results in fast convergence but requires massive control effort.
  * *Sluggish/Energy-Saving:* Low state penalty ($Q$), high control penalty ($R$). Results in a slow, smooth approach utilizing minimal control effort.

#### `HomeWork_1.py` (LQR Section)
* Contains a custom, iterative solver for the Algebraic Riccati Equation (`solve_algebraic_riccati`) that calculates $P$ by iterating until convergence, demonstrating the underlying math behind the `scipy` solvers.

### State Estimation: Kalman Filter (Python)

#### `Q4_Kalman_filter.py`
This script builds a standard discrete-time Kalman Filter from scratch to estimate the hidden states (position and velocity) of a system based only on noisy position measurements.
* **Predict & Update:** Implements the standard two-step Kalman filter equations: state/covariance prediction (time update) and measurement correction (measurement update).
* **Convergence Analysis:** Tracks and plots the trace of the error covariance matrix ($P$) and the Kalman gains ($K_p$, $K_v$) to demonstrate how the filter's uncertainty decreases and stabilizes over time.
* **Monte Carlo Simulation:** Runs 1,000 iterations to compare the Mean Squared Error (MSE) of the Kalman Filter against a naive estimator (using raw measurements). Empirically proves the variance reduction achieved by the Kalman Filter.
* **Missing Measurements (Robustness):** Simulates a scenario where sensor data is only available every 10 time steps. The script successfully relies purely on the mathematical model (Predict step) during blackouts, updating its covariance appropriately when a measurement finally arrives.

---

## Dependencies

To run the Python scripts, you will need a standard scientific Python environment.
* **Python 3.7+**
* `numpy`: For matrix operations and random noise generation.
* `matplotlib`: For generating plots of state trajectories and filter convergence.
* `scipy`: For solving the Discrete Algebraic Riccati Equation (`solve_discrete_are`).

To run the MATLAB script:
* **MATLAB** (Control System Toolbox recommended for `ctrb` and `obsv` functions).

---

## How to Run

### Python Scripts
You can run the Python scripts directly from your terminal or command prompt. They will output console metrics and automatically open matplotlib figure windows.

```bash
# Run the LQR tuning simulation
python Q3_LQR.py

# Run the Kalman Filter and Monte Carlo simulation
python Q4_Kalman_filter.py

# Run the alternative Homework script
python HomeWork_1.py
```
*Note: If running the Jupyter Notebook (`Ex1_Control_LQR_with_state_reference_.ipynb`), open it using `jupyter notebook` or JupyterLab and run the cells sequentially.*

### MATLAB Script
1. Open `Q1_script.m` in the MATLAB IDE.
2. Click **Run** or type `Q1_script` in the Command Window.
3. The stability, controllability, observability, and detectability analyses for both sample systems will be printed directly to the MATLAB console.

---

## Key Concepts Demonstrated
* **Closed-loop State Feedback:** Driving a system to a non-zero reference point.
* **LQR Cost Function:** Balancing state error ($Q$) versus actuator effort ($R$).
* **Kalman Filter Tuning:** Handling Process Noise ($Q$) and Measurement Noise ($R$).
* **Monte Carlo Analysis:** Validating stochastic estimator performance over large sample sizes.
* **PBH Test:** A robust matrix-rank approach to checking system detectability.
