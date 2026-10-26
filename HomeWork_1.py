# to run question 3, uncomment the code below and comment out question 4.

# -------------------- Question 3: LQR Control --------------------
# import numpy as np
# import matplotlib.pyplot as plt

# # system matrices
# A = np.array([[1.0, 1.0],
#               [0.0, 1.0]])

# B = np.array([[0.0],
#               [1.0]])

# xref = np.array([[3.0],
#                  [0.0]])


# def solve_algebraic_riccati(A, B, Q, R, tol=1e-12, max_iter=10000):
#     P = Q.copy()

#     for _ in range(max_iter):

#         P_next = (
#             A.T @ P @ A
#             - A.T @ P @ B
#             @ np.linalg.inv(R + B.T @ P @ B)
#             @ B.T @ P @ A
#             + Q
#         )

#         if np.max(np.abs(P_next - P)) < tol:
#             return P_next

#         P = P_next



# # LQR gain
# Q = np.eye(2)
# R = np.eye(1)

# P = solve_algebraic_riccati(A, B, Q, R)

# K = np.linalg.inv(R + B.T @ P @ B) @ (B.T @ P @ A)

# print("LQR Gain K:")
# print(K)


# # simulation function
# def simulate(x0, steps=30):

#     x = x0.reshape(2, 1)

#     xs = [x.copy()]
#     us = []

#     for _ in range(steps):

#         # error state
#         e = x - xref

#         # LQR control
#         u = -K @ e

#         us.append(u.item())

#         # system update
#         x = A @ x + B @ u

#         xs.append(x.copy())

#     xs = np.hstack(xs)

#     return xs, np.array(us)


# # Initial conditions for the system
# initial_conditions = [
#     np.array([0.0, 0.0]),
#     np.array([0.0, -5.0]),
#     np.array([3.0, 5.0]),
# ]

# titles = [
#     r"$x_0 = [0,\ 0]^T$",
#     r"$x_0 = [0,\ -5]^T$",
#     r"$x_0 = [3,\ 5]^T$",
# ]


# # figures for each initial condition: (i), (ii), (iii)
# for x0, title in zip(initial_conditions, titles):

#     xs, us = simulate(x0)

#     t_state = np.arange(xs.shape[1])
#     t_input = np.arange(len(us))

#     plt.figure(figsize=(10, 8))

#     # position
#     plt.subplot(3,1,1)
#     plt.plot(t_state, xs[0,:], linewidth=2)
#     plt.axhline(3, linestyle='--')
#     plt.ylabel("Position")
#     plt.title(f"LQR Response for {title}")
#     plt.grid(True)

    
#     # velocity
#     plt.subplot(3,1,2)
#     plt.plot(t_state, xs[1,:], linewidth=2)
#     plt.axhline(0, linestyle='--')
#     plt.ylabel("Velocity")
#     plt.grid(True)

   
#     # control input
#     plt.subplot(3,1,3)
#     plt.plot(t_input, us, linewidth=2)
#     plt.axhline(0, linestyle='--')
#     plt.xlabel("Time step")
#     plt.ylabel("Control input")
#     plt.grid(True)
#     plt.tight_layout()
#     plt.show()


# # Tuning Q and R 

# x0_test = np.array([0.0, 0.0])
# steps = 30

# #  different tuning configurations
# configs = [
#     {"Q": np.eye(2), "R": np.eye(1), "label": "Baseline (Q=I, R=1)", "ls": "-"},    
#     {"Q": np.eye(2), "R": 100 * np.eye(1), "label": "Expensive Control (Q=I, R=100)", "ls": "--"},
#     {"Q": 100 * np.eye(2), "R": np.eye(1), "label": "High State-weight (Q=100I, R=1)", "ls": "-."},
#     {"Q": np.diag([100.0, 1.0]), "R": np.eye(1), "label": "Position-weighted (Q=diag(100,1), R=1)", "ls": ":"}
# ]

# plt.figure(figsize=(12, 10))

# for cfg in configs:
#     Q_test = cfg["Q"]
#     R_test = cfg["R"]
    
#     # calculate new gain K for this configuration
#     P_test = solve_algebraic_riccati(A, B, Q_test, R_test)
#     K_test = np.linalg.inv(R_test + B.T @ P_test @ B) @ (B.T @ P_test @ A)
    
#     # simulate
#     x = x0_test.reshape(2, 1)
#     xs = [x.copy()]
#     us = []

#     for _ in range(steps):
#         e = x - xref
#         u = -K_test @ e
#         us.append(u.item())
#         x = A @ x + B @ u
#         xs.append(x.copy())

#     xs = np.hstack(xs)
#     t_state = np.arange(xs.shape[1])
#     t_input = np.arange(len(us))

#     # position
#     plt.subplot(3, 1, 1)
#     plt.plot(t_state, xs[0, :], label=cfg["label"], linestyle=cfg["ls"], linewidth=2)
    
#     # velocity
#     plt.subplot(3, 1, 2)
#     plt.plot(t_state, xs[1, :], label=cfg["label"], linestyle=cfg["ls"], linewidth=2)
    
#     # control 
#     plt.subplot(3, 1, 3)
#     plt.plot(t_input, us, label=cfg["label"], linestyle=cfg["ls"], linewidth=2)


# # graphs
# plt.subplot(3, 1, 1)
# plt.axhline(3, color='black', linestyle='--', alpha=0.3)  
# plt.ylabel("Position")
# plt.title("LQR Response for Different Q and R Matrices")
# plt.legend()
# plt.grid(True)

# plt.subplot(3, 1, 2)
# plt.axhline(0, color='black', linestyle='--', alpha=0.3)  
# plt.ylabel("Velocity")
# plt.legend()
# plt.grid(True)

# plt.subplot(3, 1, 3)
# plt.axhline(0, color='black', linestyle='--', alpha=0.3)  
# plt.xlabel("Time step")
# plt.ylabel("Control input")
# plt.legend()
# plt.grid(True)

# plt.tight_layout()
# plt.show()





# -------------------- Question 4: Kalman Filter --------------------
import numpy as np
import matplotlib.pyplot as plt

# define system matrices
A = np.array([[1, 1], 
              [0, 1]])
B = np.array([[0], 
              [1]])
C = np.array([[1, 0]])

Q = np.eye(2)   
R = np.array([[3]]) 
N_steps = 50    # Number of time steps for simulation for part 2 as instructed in the exercise.

# Kalman filter step
def kf_predict(x_hat, P, u):
    """Predict step of the Kalman Filter."""
    x_pred = A @ x_hat + B * u
    P_pred = A @ P @ A.T + Q
    return x_pred, P_pred

def kf_update(x_pred, P_pred, y):
    """Measurement update step of the Kalman Filter."""
    S = C @ P_pred @ C.T + R
    K = P_pred @ C.T @ np.linalg.inv(S)
    x_hat = x_pred + K @ (y - C @ x_pred)
    P = (np.eye(2) - K @ C) @ P_pred
    return x_hat, P, K

# parts 2 & 3
np.random.seed(5)  # For reproducibility

x_true = np.zeros((2, N_steps))
y_meas = np.zeros(N_steps)
x_est = np.zeros((2, N_steps))
P_trace = np.zeros(N_steps)
K_hist = np.zeros((2, N_steps))

# initial conditions
x = np.array([[0], [0]])       # true initial state
x_hat = np.array([[0], [0]])   # estimated initial state
P = np.eye(2)                  # initial Error Covariance

for k in range(N_steps):
    # simulate true system
    w = np.random.multivariate_normal([0, 0], Q).reshape(2, 1)
    v = np.random.normal(0, np.sqrt(R[0,0]))
    u = 0 # no control input
    
    x = A @ x + B * u + w
    y = C @ x + v
    
    x_true[:, k] = x.flatten()
    y_meas[k] = y[0, 0]
    
    # Kalman Filter
    x_pred, P_pred = kf_predict(x_hat, P, u)
    x_hat, P, K = kf_update(x_pred, P_pred, y)
    
    # store the data
    x_est[:, k] = x_hat.flatten()
    P_trace[k] = np.trace(P)
    K_hist[:, k] = K.flatten()

# plotting part 2
plt.figure(figsize=(12, 5))

# Subplot 1: Position
plt.subplot(1, 2, 1)
plt.plot(x_true[0, :], label='True Position', color='black')
plt.plot(y_meas, label='Measurements', marker='o', linestyle='', alpha=0.5)
plt.plot(x_est[0, :], label='KF Estimate', linestyle='--')
plt.title('Position Over Time')
plt.xlabel('Time Step (k)')
plt.ylabel('Position Value (x1)')
plt.legend()
plt.grid(True)

# Subplot 2: Velocity
plt.subplot(1, 2, 2)
plt.plot(x_true[1, :], label='True Velocity', color='black')
plt.plot(x_est[1, :], label='KF Estimate', linestyle='--')
plt.title('Velocity Over Time')
plt.xlabel('Time Step (k)')
plt.ylabel('Velocity Value (x2)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()

# plotting part 3
plt.figure(figsize=(12, 5))

# Subplot 1: Trace of P
plt.subplot(1, 2, 1)
plt.plot(P_trace, linewidth=2)
plt.title('Trace of Error Covariance Matrix (P)')
plt.xlabel('Time Step (k)')
plt.ylabel('Trace(P)')
plt.grid(True)

# Subplot 2: Kalman Gain
plt.subplot(1, 2, 2)
plt.plot(K_hist[0, :], label='K_p (Position Gain)')
plt.plot(K_hist[1, :], label='K_v (Velocity Gain)')
plt.title('Kalman Gain Convergence')
plt.xlabel('Time Step (k)')
plt.ylabel('Gain Value')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()



# Naive Estimator vs Kalma Filter Variance
M_sims = 10000
err_naive = []
err_kf = []

for m in range(M_sims):
    x_t = np.zeros((2, 1))
    x_h = np.zeros((2, 1))
    P_h = np.eye(2)
    
    for k in range(50):
        w = np.random.multivariate_normal([0, 0], Q).reshape(2, 1)
        v = np.random.normal(0, np.sqrt(R[0,0]))
        x_t = A @ x_t + w
        y = C @ x_t + v
        
        x_p, P_p = kf_predict(x_h, P_h, 0)
        x_h, P_h, _ = kf_update(x_p, P_p, y)
    
    # compare at the end of the simulation
    err_naive.append((x_t[0, 0] - y[0, 0])**2) 
    err_kf.append((x_t[0, 0] - x_h[0, 0])**2)  

print(f"--- Part 4: Monte Carlo Results ({M_sims} runs) ---")
print(f"Naive Estimator Variance: {np.mean(err_naive):.2f}")
print(f"Kalman Filter Variance:   {np.mean(err_kf):.2f} (should be lower than naive)")
print("MSE = Variance + Bias^2, and both estimators are unbiased, so MSE is essentially variance here.\nA lower MSE scores indicates a more accurate estimator")


# part 5 missing measurements (T = 100)
N_steps_miss = 100
P_trace_miss = np.zeros(N_steps_miss)
P_trace_full = np.zeros(N_steps_miss)

P_m = np.eye(2)
P_f = np.eye(2)

for k in range(N_steps_miss):
    _, P_pred_f = kf_predict(np.zeros((2,1)), P_f, 0) # States don't matter for P
    S_f = C @ P_pred_f @ C.T + R
    K_f = P_pred_f @ C.T @ np.linalg.inv(S_f)
    P_f = (np.eye(2) - K_f @ C) @ P_pred_f
    P_trace_full[k] = np.trace(P_f)
    
    _, P_pred_m = kf_predict(np.zeros((2,1)), P_m, 0)
    
    if k % 10 == 0: 
        S_m = C @ P_pred_m @ C.T + R
        K_m = P_pred_m @ C.T @ np.linalg.inv(S_m)
        P_m = (np.eye(2) - K_m @ C) @ P_pred_m
    else:
        P_m = P_pred_m
        
    P_trace_miss[k] = np.trace(P_m)

plt.figure(figsize=(8, 5))
plt.plot(P_trace_miss, label='Measurement every 10 steps', color='red')
plt.plot(P_trace_full, label='Measurement every step', color='blue', linestyle='--')
plt.title('Error Covariance Trace with Missing Measurements')
plt.xlabel('Time Step')
plt.ylabel('Trace(P)')
plt.legend()
plt.grid(True)
plt.show()

