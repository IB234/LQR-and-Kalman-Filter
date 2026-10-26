import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_discrete_are

def simulate_lqr(A, B, Q, R, x0, x_ref, steps=15):
    """Solves the DARE and simulates the closed-loop double integrator."""
    # 1. Solve DARE to find P
    P = solve_discrete_are(A, B, Q, R)
    
    # 2. Calculate optimal gain K
    K = np.linalg.inv(R + B.T @ P @ B) @ (B.T @ P @ A)
    
    # 3. Simulate
    x = x0.copy()
    history_x = [x.flatten()]
    history_u = []
    
    for _ in range(steps):
        # Calculate error state
        x_tilde = x - x_ref
        
        # Calculate control input
        u = -K @ x_tilde
        history_u.append(u.flatten())
        
        # Evolve system
        x = A @ x + B @ u
        history_x.append(x.flatten())
        
    return np.array(history_x), np.array(history_u), K

# --- System Definition ---
A = np.array([[1, 1], 
              [0, 1]])
B = np.array([[0], 
              [1]])
x_ref = np.array([[3], 
                  [0]])

# Initial Conditions for Part 3
ic_list = [
    np.array([[0], [0]]),   # (i)
    np.array([[0], [-5]]),  # (ii)
    np.array([[3], [5]])    # (iii)
]

# --- Part 3: Simulation with Q=I, R=1 ---
Q_base = np.eye(2)
R_base = np.array([[1]])

fig, axs = plt.subplots(3, 1, figsize=(10, 10))

for idx, x0 in enumerate(ic_list):
    hist_x, hist_u, K_base = simulate_lqr(A, B, Q_base, R_base, x0, x_ref)
    time = np.arange(len(hist_x))
    
    axs[idx].plot(time, hist_x[:, 0], label="Position $x_t(1)$", color='blue')
    axs[idx].plot(time, hist_x[:, 1], label="Velocity $x_t(2)$", color='orange')
    axs[idx].plot(time[:-1], hist_u, label="Control Input $u_t$", color='green', linestyle='--')
    axs[idx].axhline(3, color='black', linestyle=':', label="Target Pos (3)")
    axs[idx].set_title(f"Initial Condition: Pos={x0[0,0]}, Vel={x0[1,0]}")
    axs[idx].legend()
    axs[idx].grid(True)

plt.tight_layout()
plt.show()

# --- Part 4: Tuning Q and R ---
# We will test two new scenarios starting from x0 = [0,0]^T
# 1. Aggressive position tracking (High Q11, Low R)
Q_agg = np.array([[10, 0], [0, 1]])
R_agg = np.array([[0.1]])

# 2. Sluggish / Energy-saving (Low Q, High R)
Q_slow = np.array([[1, 0], [0, 1]])
R_slow = np.array([[20]])

hist_x_agg, hist_u_agg, K_agg = simulate_lqr(A, B, Q_agg, R_agg, ic_list[0], x_ref)
hist_x_slow, hist_u_slow, K_slow = simulate_lqr(A, B, Q_slow, R_slow, ic_list[0], x_ref)

fig2, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))
time = np.arange(len(hist_x_agg))

ax1.plot(time, hist_x_agg[:, 0], label="Aggressive ($Q_{11}=10, R=0.1$)", color='red')
ax1.plot(time, hist_x_slow[:, 0], label="Sluggish ($Q_{11}=1, R=20$)", color='blue')
ax1.axhline(3, color='black', linestyle=':')
ax1.set_title("Position Trajectories vs Cost Tuning")
ax1.legend()
ax1.grid(True)

ax2.plot(time[:-1], hist_u_agg, label="Aggressive Effort", color='red')
ax2.plot(time[:-1], hist_u_slow, label="Sluggish Effort", color='blue')
ax2.set_title("Control Effort (Acceleration) vs Cost Tuning")
ax2.legend()
ax2.grid(True)

plt.show()