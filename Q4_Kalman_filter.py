import numpy as np
import matplotlib.pyplot as plt

def simulate_system(steps, Q, R, measurement_interval=1):
    # System matrices
    A = np.array([[1, 1], [0, 1]])
    B = np.array([[0], [1]])
    C = np.array([[1, 0]])
    u = np.array([[0]])
    
    # Initialization
    x_true = np.array([[0.0], [0.0]])
    x_pred = np.array([[0.0], [0.0]])
    P = np.eye(2)
    
    data = {'x_true': [], 'y_meas': [], 'x_pred': [], 'trace_P': [], 'Kp_pos': [], 'Kp_vel': []}
    
    for k in range(steps):
        # 1. Simulate True Environment
        w = np.random.multivariate_normal([0, 0], Q).reshape(2, 1)
        v = np.random.normal(0, np.sqrt(R))
        x_true = A @ x_true + B @ u + w
        y_meas = (C @ x_true + v).item()
        
        data['x_true'].append(x_true.flatten())
        data['y_meas'].append(y_meas)
        data['x_pred'].append(x_pred.flatten())
        data['trace_P'].append(np.trace(P))
        
        # 2. Kalman Filter Update
        measurement_available = (k % measurement_interval == 0)
        
        if measurement_available:
            Re = C @ P @ C.T + np.array([[R]])
            Kp = A @ P @ C.T @ np.linalg.inv(Re)
            innovation = y_meas - (C @ x_pred).item()
            
            # Full Predictor Update
            x_pred_next = A @ x_pred + B @ u + Kp * innovation
            P_next = A @ P @ A.T + Q - Kp @ Re @ Kp.T
        else:
            # Time Update Only (No Measurement)
            Kp = np.array([[0.0], [0.0]])
            x_pred_next = A @ x_pred + B @ u
            P_next = A @ P @ A.T + Q
            
        data['Kp_pos'].append(Kp[0,0])
        data['Kp_vel'].append(Kp[1,0])
        
        x_pred = x_pred_next
        P = P_next
        
    return data

# --- Parameters ---
Q = np.eye(2)
R = 3

# --- Part 2 & 3: Standard 50-Step Simulation ---
data_50 = simulate_system(50, Q, R)

fig, axs = plt.subplots(3, 1, figsize=(10, 12))
time = np.arange(50)

# True vs Estimates
axs[0].plot(time, np.array(data_50['x_true'])[:, 0], 'k--', label='True Position')
axs[0].plot(time, data_50['y_meas'], 'ro', markersize=4, alpha=0.5, label='Measurements')
axs[0].plot(time, np.array(data_50['x_pred'])[:, 0], 'b-', label='KF Estimate')
axs[0].set_title('Part 2: Position Tracking')
axs[0].legend()
axs[0].grid(True)

# Trace(P)
axs[1].plot(time, data_50['trace_P'], 'g-', linewidth=2)
axs[1].set_title('Part 3: Trace of Error Covariance Matrix (Convergence)')
axs[1].grid(True)

# Kp Convergence
axs[2].plot(time, data_50['Kp_pos'], label='$K_{p}$ (Position)', color='purple')
axs[2].plot(time, data_50['Kp_vel'], label='$K_{p}$ (Velocity)', color='orange')
axs[2].set_title('Part 3: Predictor Gain ($K_{p}$) Convergence')
axs[2].legend()
axs[2].grid(True)
plt.tight_layout()
plt.show()

# --- Part 4: Monte Carlo Naive vs KF ---
N_sims = 1000
mse_kf, mse_naive = [], []

for _ in range(N_sims):
    mc_data = simulate_system(50, Q, R)
    pos_true = np.array(mc_data['x_true'])[:, 0]
    pos_kf = np.array(mc_data['x_pred'])[:, 0]
    pos_naive = np.array(mc_data['y_meas'])
    
    mse_kf.append(np.mean((pos_true - pos_kf)**2))
    mse_naive.append(np.mean((pos_true - pos_naive)**2))

print(f"Monte Carlo Results ({N_sims} simulations):")
print(f"Naive Estimator MSE: {np.mean(mse_naive):.4f} (Expected ~3.0)")
print(f"Kalman Filter MSE:   {np.mean(mse_kf):.4f}")

# --- Part 5: Missing Measurements ---
data_100_missing = simulate_system(100, Q, R, measurement_interval=10)
data_100_standard = simulate_system(100, Q, R, measurement_interval=1)

plt.figure(figsize=(10, 4))
plt.plot(np.arange(100), data_100_standard['trace_P'], 'g--', label='Standard KF (All Measurements)')
plt.plot(np.arange(100), data_100_missing['trace_P'], 'r-', label='Missing Measurements (Every 10 steps)')
plt.title('Part 5: Error Covariance Trace with Missing Measurements')
plt.xlabel('Time Step k')
plt.ylabel('Trace(P)')
plt.legend()
plt.grid(True)
plt.show()