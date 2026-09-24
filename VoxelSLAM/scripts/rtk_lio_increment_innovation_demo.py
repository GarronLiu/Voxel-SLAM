import numpy as np
import matplotlib.pyplot as plt

np.random.seed(7)

# ============================================================
# 1. Generate synthetic data
# ============================================================
dt = 0.2
T = 40.0
t = np.arange(0, T + dt, dt)
N = len(t)

# Smooth ground-truth USV trajectory
x_true = 0.8 * t + 1.5 * np.sin(0.15 * t)
y_true = 4.0 * np.sin(0.08 * t) + 0.3 * np.sin(0.35 * t)
p_true = np.column_stack([x_true, y_true])

# Lidar-IMU trajectory:
# locally smooth and accurate, but with small long-term drift
lio_drift = np.column_stack([
    0.015 * t,
    -0.008 * t
])
lio_noise = np.random.normal(0, 0.015, size=(N, 2))
p_lio = p_true + lio_drift + lio_noise

# RTK trajectory:
# globally absolute, but has a temporary piecewise-constant shift
rtk_noise = np.random.normal(0, 0.035, size=(N, 2))
bias = np.zeros((N, 2))

mask_shift = (t >= 12.0) & (t < 27.0)
bias[mask_shift] = np.array([1.8, -1.2])

p_rtk = p_true + bias + rtk_noise

# ============================================================
# 2. Absolute RTK-LIO residual
# ============================================================
absolute_residual = p_rtk - p_lio

# ============================================================
# 3. Relative-increment innovation
#
#   nu_k = (z_k^RTK - z_{k-1}^RTK)
#          - (p_k^LIO - p_{k-1}^LIO)
#
# Under z_k^RTK = p_k + b_k + n_k:
#
#   nu_k approximately equals b_k - b_{k-1}
#
# Therefore it detects changes in RTK bias rather than
# persistent constant bias itself.
# ============================================================
delta_rtk = np.diff(p_rtk, axis=0)
delta_lio = np.diff(p_lio, axis=0)

innovation = delta_rtk - delta_lio
innovation_norm = np.linalg.norm(innovation, axis=1)

# ============================================================
# 4. Robust threshold estimated from an initial healthy period
# ============================================================
healthy_mask = t[1:] < 8.0
healthy_norm = innovation_norm[healthy_mask]

median_nu = np.median(healthy_norm)
mad_nu = np.median(np.abs(healthy_norm - median_nu))
sigma_robust = 1.4826 * mad_nu

threshold = median_nu + 6.0 * max(sigma_robust, 1e-3)

change_idx = np.where(innovation_norm > threshold)[0]
change_times = t[1:][change_idx]

print("Detected RTK bias change times [s]:",
      np.round(change_times, 2))
print("Detection threshold [m]:",
      round(float(threshold), 4))

# ============================================================
# 5. Plots
# ============================================================
plt.figure(figsize=(8, 5))
plt.plot(p_true[:, 0], p_true[:, 1], label="True")
plt.plot(p_lio[:, 0], p_lio[:, 1], label="Lidar-IMU")
plt.plot(p_rtk[:, 0], p_rtk[:, 1], ".", markersize=3, label="RTK")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Trajectory")
plt.axis("equal")
plt.grid(True, alpha=0.3)
plt.legend()
plt.show()

plt.figure(figsize=(9, 4))
plt.plot(t, absolute_residual[:, 0], label="absolute residual x")
plt.plot(t, absolute_residual[:, 1], label="absolute residual y")
plt.axvspan(12, 27, alpha=0.15, label="RTK shifted interval")
plt.xlabel("Time [s]")
plt.ylabel("RTK - LIO [m]")
plt.title("Absolute RTK-LIO residual")
plt.grid(True, alpha=0.3)
plt.legend()
plt.show()

plt.figure(figsize=(9, 4))
plt.plot(t[1:], innovation[:, 0], label="innovation x")
plt.plot(t[1:], innovation[:, 1], label="innovation y")
plt.axvline(12, linestyle="--", label="shift begins")
plt.axvline(27, linestyle="--", label="shift ends")
plt.xlabel("Time [s]")
plt.ylabel("Increment innovation [m]")
plt.title("Relative-increment innovation")
plt.grid(True, alpha=0.3)
plt.legend()
plt.show()

plt.figure(figsize=(9, 4))
plt.plot(t[1:], innovation_norm, label="innovation norm")
plt.axhline(threshold, linestyle="--", label="threshold")
plt.scatter(change_times,
            innovation_norm[change_idx],
            s=35,
            label="detected bias changes")
plt.xlabel("Time [s]")
plt.ylabel("Innovation norm [m]")
plt.title("RTK bias change-point detection")
plt.grid(True, alpha=0.3)
plt.legend()
plt.show()
