"""
Sensor Noise Plotter
Reads binary sensor data file and plots it as a line graph.
The binary file contains uint16_t values (little-endian) representing raw sensor readings.
"""

import struct
import numpy as np
import matplotlib.pyplot as plt
import os
import sys
import argparse
import time
from pathlib import Path

# Data packet format: uint16_t (little-endian)
PACKET_SIZE = 2  # 2 bytes for uint16_t
PACKET_FORMAT = "<H"  # little-endian uint16_t
PACKET_STRUCT = struct.Struct(PACKET_FORMAT)


def unwrap_modulo(x: np.ndarray, modulus: int) -> np.ndarray:
    """
    Unwrap a modulo sensor angle stream into a continuous signal.

    Assumes x is integer-like counts in [0, modulus).
    """
    if len(x) == 0:
        return x.astype(float)

    xf = x.astype(np.float64)
    half = modulus / 2.0
    unwrapped = np.empty_like(xf)
    unwrapped[0] = xf[0]

    offset = 0.0
    for i in range(1, len(xf)):
        d = xf[i] - xf[i - 1]
        if d > half:
            offset -= modulus
        elif d < -half:
            offset += modulus
        unwrapped[i] = xf[i] + offset

    return unwrapped


def finite_difference_velocity(theta: np.ndarray, dt: float) -> np.ndarray:
    if len(theta) == 0:
        return np.array([], dtype=float)
    dtheta = np.diff(theta, prepend=theta[0])
    return dtheta / dt


def alpha_beta_velocity(theta: np.ndarray, dt: float, bandwidth_hz: float) -> np.ndarray:
    """
    Alpha-Beta filter (2nd order observer) estimating velocity from position.

    Uses a continuous-time design parameterized by bandwidth, then discretized (Euler):
      alpha = 2*w*dt
      beta  = (w*dt)^2
    """
    if len(theta) == 0:
        return np.array([], dtype=float)

    w = 2.0 * np.pi * max(1e-6, float(bandwidth_hz))
    alpha = 2.0 * w * dt
    beta = (w * dt) ** 2

    alpha = float(np.clip(alpha, 0.0, 1.0))
    beta = float(np.clip(beta, 0.0, 2.0))

    x = float(theta[0])
    v = 0.0
    out_v = np.empty_like(theta, dtype=float)

    for i in range(len(theta)):
        if i > 0:
            x = x + v * dt
        e = float(theta[i]) - x
        x = x + alpha * e
        v = v + (beta / dt) * e
        out_v[i] = v

    return out_v


def steady_state_kalman_velocity(theta: np.ndarray, dt: float, sigma_theta: float, sigma_accel: float) -> np.ndarray:
    """
    2-state constant-velocity Kalman filter with steady-state gain computed once.
    State: [theta, omega], measurement: z = theta.
    """
    if len(theta) == 0:
        return np.array([], dtype=float)

    A = np.array([[1.0, dt], [0.0, 1.0]], dtype=float)
    H = np.array([[1.0, 0.0]], dtype=float)  # 1x2
    I = np.eye(2, dtype=float)

    sigma_theta = float(max(1e-12, sigma_theta))
    sigma_accel = float(max(1e-12, sigma_accel))

    q = sigma_accel**2
    Q = q * np.array(
        [[(dt**4) / 4.0, (dt**3) / 2.0], [(dt**3) / 2.0, (dt**2)]],
        dtype=float,
    )
    R = np.array([[sigma_theta**2]], dtype=float)  # 1x1

    # Iterate to steady-state covariance (offline / once).
    P = np.eye(2, dtype=float)
    K = np.zeros((2, 1), dtype=float)
    for _ in range(5000):
        P_pred = A @ P @ A.T + Q
        S = (H @ P_pred @ H.T) + R  # 1x1
        K_new = (P_pred @ H.T) / float(S[0, 0])  # 2x1
        P_new = (I - K_new @ H) @ P_pred
        if np.max(np.abs(P_new - P)) < 1e-12 and np.max(np.abs(K_new - K)) < 1e-12:
            P = P_new
            K = K_new
            break
        P = P_new
        K = K_new

    x = np.array([[float(theta[0])], [0.0]], dtype=float)
    out_v = np.empty_like(theta, dtype=float)

    for i in range(len(theta)):
        z = np.array([[float(theta[i])]], dtype=float)
        x_pred = A @ x
        e = z - (H @ x_pred)
        x = x_pred + K @ e
        out_v[i] = float(x[1, 0])

    return out_v


def _butterworth_biquad_lowpass(fs_hz: float, cutoff_hz: float, q: float = 1.0 / np.sqrt(2.0)):
    """RBJ cookbook biquad coefficients (a0 normalized to 1)."""
    fs_hz = float(max(1e-9, fs_hz))
    cutoff_hz = float(np.clip(cutoff_hz, 1e-6, 0.499 * fs_hz))
    w0 = 2.0 * np.pi * cutoff_hz / fs_hz
    cos_w0 = float(np.cos(w0))
    sin_w0 = float(np.sin(w0))
    alpha = sin_w0 / (2.0 * float(q))

    b0 = (1.0 - cos_w0) / 2.0
    b1 = 1.0 - cos_w0
    b2 = (1.0 - cos_w0) / 2.0
    a0 = 1.0 + alpha
    a1 = -2.0 * cos_w0
    a2 = 1.0 - alpha

    b = np.array([b0 / a0, b1 / a0, b2 / a0], dtype=float)
    a = np.array([1.0, a1 / a0, a2 / a0], dtype=float)
    return b, a


def biquad_df2t(x: np.ndarray, b: np.ndarray, a: np.ndarray, x0_for_init: float | None = None) -> np.ndarray:
    """
    Direct Form II Transposed IIR biquad.
    a is assumed normalized: a[0] == 1.
    """
    if len(x) == 0:
        return np.array([], dtype=float)

    b0, b1, b2 = [float(v) for v in b]
    _, a1, a2 = [float(v) for v in a]

    y = np.empty_like(x, dtype=float)

    # Initialize state to reduce startup transients for (approximately) DC inputs.
    z1 = 0.0
    z2 = 0.0
    if x0_for_init is not None:
        x0 = float(x0_for_init)
        dc_gain = (b0 + b1 + b2) / (1.0 + a1 + a2)
        y0 = dc_gain * x0
        z1 = y0 - b0 * x0
        z2 = b2 * x0 - a2 * y0

    for i in range(len(x)):
        x0 = float(x[i])
        y0 = b0 * x0 + z1
        z1 = b1 * x0 - a1 * y0 + z2
        z2 = b2 * x0 - a2 * y0
        y[i] = y0
    return y


def biquad_lpf_then_diff_then_lpf_velocity(theta: np.ndarray, dt: float, cutoff_hz: float) -> np.ndarray:
    if len(theta) == 0:
        return np.array([], dtype=float)

    fs = 1.0 / dt
    b, a = _butterworth_biquad_lowpass(fs, cutoff_hz)
    theta_lp = biquad_df2t(theta, b, a, x0_for_init=float(theta[0]))
    omega = finite_difference_velocity(theta_lp, dt)
    omega_lp = biquad_df2t(omega, b, a, x0_for_init=float(omega[0]))
    return omega_lp


def luenberger_velocity(theta: np.ndarray, dt: float, wn_hz: float, zeta: float) -> np.ndarray:
    """
    Luenberger observer for a double integrator (pure inertia model).
    Continuous-time desired observer dynamics: s^2 + 2*zeta*wn*s + wn^2.
    Discretized with Euler around a constant-velocity predictor.
    """
    if len(theta) == 0:
        return np.array([], dtype=float)

    wn = 2.0 * np.pi * max(1e-6, float(wn_hz))
    zeta = float(np.clip(zeta, 0.0, 5.0))

    k1 = 2.0 * zeta * wn * dt
    k2 = (wn**2) * dt

    x = float(theta[0])
    v = 0.0
    out_v = np.empty_like(theta, dtype=float)

    for i in range(len(theta)):
        x_pred = x + v * dt
        v_pred = v
        e = float(theta[i]) - x_pred
        x = x_pred + k1 * e
        v = v_pred + k2 * e
        out_v[i] = v

    return out_v


def savgol_velocity(theta: np.ndarray, dt: float, window: int, poly_order: int = 2) -> np.ndarray:
    """
    Savitzky-Golay 1st derivative via precomputed convolution coefficients.
    Window must be odd and >= poly_order+2.
    """
    if len(theta) == 0:
        return np.array([], dtype=float)

    window = int(window)
    if window < 3:
        raise ValueError("Savitzky-Golay window must be >= 3")
    if window % 2 == 0:
        raise ValueError("Savitzky-Golay window must be odd")
    if window < poly_order + 2:
        raise ValueError("Savitzky-Golay window too small for polynomial order")

    m = window // 2
    t = np.arange(-m, m + 1, dtype=float)  # sample index centered at 0
    A = np.vstack([t**p for p in range(poly_order + 1)]).T  # window x (poly+1)

    ATA_inv = np.linalg.inv(A.T @ A)
    pinv = ATA_inv @ A.T  # (poly+1) x window

    # Derivative at 0 is coefficient of t^1 term.
    coeff = pinv[1, :] / dt  # counts/s

    padded = np.pad(theta.astype(float), (m, m), mode="edge")
    out = np.empty_like(theta, dtype=float)
    for i in range(len(theta)):
        out[i] = float(np.dot(coeff, padded[i:i + window]))
    return out


def wls_velocity(theta: np.ndarray, dt: float, window: int) -> np.ndarray:
    """
    Windowed Least Squares differentiator (linear regression slope) over last N samples.
    """
    if len(theta) == 0:
        return np.array([], dtype=float)

    window = int(window)
    if window < 2:
        raise ValueError("WLS window must be >= 2")

    t = np.arange(window, dtype=float) * dt
    t0 = t - np.mean(t)
    denom = float(np.sum(t0**2))
    if denom <= 0.0:
        raise ValueError("WLS window produced zero denominator")

    out = np.empty_like(theta, dtype=float)
    for i in range(len(theta)):
        start = max(0, i - window + 1)
        y = theta[start:i + 1].astype(float)
        if len(y) < 2:
            out[i] = 0.0
            continue

        if len(y) != window:
            tt = np.arange(len(y), dtype=float) * dt
            tt0 = tt - np.mean(tt)
            dd = float(np.sum(tt0**2))
            y0 = y - np.mean(y)
            out[i] = float(np.sum(tt0 * y0) / dd) if dd > 0 else 0.0
        else:
            y0 = y - np.mean(y)
            out[i] = float(np.sum(t0 * y0) / denom)

    return out


def read_binary_file(filepath, debug=False):
    """
    Read binary file containing uint16_t sensor data.
    
    Args:
        filepath: Path to the binary file
        debug: If True, print first few raw bytes for debugging
        
    Returns:
        numpy array of uint16_t values
    """
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"File not found: {filepath}")
    
    # Get file info for debugging
    file_size = os.path.getsize(filepath)
    file_mtime = os.path.getmtime(filepath)
    print(f"Reading file: {filepath}")
    print(f"  File size: {file_size} bytes ({file_size // PACKET_SIZE} packets)")
    print(f"  Modified: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(file_mtime))}")
    
    values = []
    
    try:
        with open(filepath, "rb") as f:
            first_bytes_shown = False
            while True:
                packet_bytes = f.read(PACKET_SIZE)
                if len(packet_bytes) < PACKET_SIZE:
                    break  # End of file
                
                # Debug: show first few raw bytes
                if debug and not first_bytes_shown and len(values) < 5:
                    print(f"  Raw bytes {len(values)}: {packet_bytes.hex()} = {packet_bytes[0]:02x} {packet_bytes[1]:02x}")
                
                try:
                    # Unpack as little-endian uint16_t
                    value = PACKET_STRUCT.unpack(packet_bytes)[0]
                    values.append(value)
                    
                    if debug and len(values) <= 5:
                        print(f"  Unpacked value: {value} (0x{value:04x})")
                        first_bytes_shown = True
                        
                except struct.error as e:
                    print(f"Warning: Failed to unpack packet: {e}")
                    continue
        
        print(f"Read {len(values)} data points from {filepath}")
        if len(values) > 0:
            print(f"  First value: {values[0]}, Last value: {values[-1]}")
            print(f"  Value range: {min(values)} to {max(values)}")
        return np.array(values, dtype=np.uint16)
        
    except Exception as e:
        raise IOError(f"Error reading file: {e}")


def plot_sensor_data(
    values,
    title="Sensor Noise Data",
    save_path=None,
    sample_rate_hz=5000.0,
    zoom_seconds=0.1,
    modulus=16384,
    ab_bandwidth_hz=50.0,
    kalman_sigma_theta=0.5,
    kalman_sigma_accel=5000.0,
    biquad_cutoff_hz=50.0,
    luen_wn_hz=60.0,
    luen_zeta=0.7,
    sg_window=7,
    wls_window=9,
):
    """
    Plot sensor data processing outputs.

    Replaces the previous 6 plots with:
      1) Alpha-Beta filter velocity estimate
      2) Steady-State Kalman filter velocity estimate
      3) 2nd-order Butterworth biquad LPF -> differentiate -> biquad LPF
      4) Luenberger observer velocity estimate
      5) Savitzky-Golay small-window derivative
      6) Windowed least-squares (WLS) differentiator
    """
    if len(values) == 0:
        print("No data to plot")
        return

    sample_rate = float(sample_rate_hz)
    if sample_rate <= 0:
        raise ValueError("sample_rate_hz must be > 0")
    dt = 1.0 / sample_rate

    zoom_samples = min(int(float(zoom_seconds) * sample_rate), len(values))
    if zoom_samples == 0:
        print("Not enough data for analysis")
        return

    raw_zoom = values[:zoom_samples]
    time_axis = np.arange(zoom_samples, dtype=float) / sample_rate

    theta = unwrap_modulo(raw_zoom, int(modulus))
    omega_fd = finite_difference_velocity(theta, dt)

    omega_ab = alpha_beta_velocity(theta, dt, bandwidth_hz=ab_bandwidth_hz)
    omega_kf = steady_state_kalman_velocity(theta, dt, sigma_theta=kalman_sigma_theta, sigma_accel=kalman_sigma_accel)
    omega_biquad = biquad_lpf_then_diff_then_lpf_velocity(theta, dt, cutoff_hz=biquad_cutoff_hz)
    omega_luen = luenberger_velocity(theta, dt, wn_hz=luen_wn_hz, zeta=luen_zeta)
    omega_sg = savgol_velocity(theta, dt, window=sg_window, poly_order=2)
    omega_wls = wls_velocity(theta, dt, window=wls_window)

    fig, axes = plt.subplots(3, 2, figsize=(14, 15))
    ax1, ax2 = axes[0]
    ax3, ax4 = axes[1]
    ax5, ax6 = axes[2]

    def _plot_method(ax, name, omega_est):
        ax.plot(time_axis, omega_fd, linewidth=0.5, alpha=0.35, color="gray", label="Finite diff (unwrapped)")
        ax.plot(time_axis, omega_est, linewidth=0.8, alpha=0.9, label=name)
        ax.set_title(f"{title} - {name}")
        ax.set_ylabel("Velocity (counts/s)")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)

    _plot_method(ax1, f"1) Alpha-Beta (bw={ab_bandwidth_hz:.1f}Hz)", omega_ab)
    _plot_method(ax2, "2) Steady-State Kalman (2-state)", omega_kf)
    _plot_method(ax3, f"3) Biquad LPF->diff->LPF (fc={biquad_cutoff_hz:.1f}Hz)", omega_biquad)
    _plot_method(ax4, f"4) Luenberger (wn={luen_wn_hz:.1f}Hz, zeta={luen_zeta:.2f})", omega_luen)
    _plot_method(ax5, f"5) Savitzky-Golay (win={sg_window}, poly=2)", omega_sg)
    _plot_method(ax6, f"6) WLS Differentiator (win={wls_window})", omega_wls)

    for ax in (ax5, ax6):
        ax.set_xlabel("Time (s)")

    plt.tight_layout()

    print(f"\nStatistics (First {float(zoom_seconds):.3f}s):")
    print(f"  Samples analyzed: {zoom_samples}")
    print(f"  Duration: {time_axis[-1]:.3f} seconds")
    print(f"  Raw min/max: {int(np.min(raw_zoom))} / {int(np.max(raw_zoom))} counts")
    print(f"  Unwrapped theta range: {np.max(theta) - np.min(theta):.2f} counts")

    def _vstats(name, v):
        print(f"\n{name}:")
        print(f"  Mean: {np.mean(v):.3f} counts/s")
        print(f"  Std:  {np.std(v):.3f} counts/s")
        print(f"  Min:  {np.min(v):.3f} counts/s")
        print(f"  Max:  {np.max(v):.3f} counts/s")

    _vstats("Finite diff (unwrapped)", omega_fd)
    _vstats("1) Alpha-Beta", omega_ab)
    _vstats("2) Steady-State Kalman", omega_kf)
    _vstats("3) Biquad LPF->diff->LPF", omega_biquad)
    _vstats("4) Luenberger", omega_luen)
    _vstats("5) Savitzky-Golay", omega_sg)
    _vstats("6) WLS differentiator", omega_wls)

    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"\nPlot saved to: {save_path}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Plot sensor noise data from binary file")
    parser.add_argument("file", nargs="?", help="Path to binary data file (default: data/sensor_noise.bin)")
    parser.add_argument("--save", "-s", help="Save plot to file instead of displaying")
    parser.add_argument("--title", "-t", default="Sensor Noise Data", help="Plot title")
    parser.add_argument("--debug", "-d", action="store_true", help="Show debug information about raw bytes")
    parser.add_argument("--sample-rate", type=float, default=5000.0, help="Sample rate in Hz (default: 5000)")
    parser.add_argument("--zoom-seconds", type=float, default=0.1, help="Seconds to plot/analyze from start (default: 0.1)")
    parser.add_argument("--modulus", type=int, default=16384, help="Angle wrap modulus in counts (default: 16384)")

    parser.add_argument("--ab-bandwidth-hz", type=float, default=50.0, help="Alpha-Beta observer bandwidth in Hz (default: 50)")
    parser.add_argument("--kalman-sigma-theta", type=float, default=0.5, help="SSKF measurement noise std (counts) (default: 0.5)")
    parser.add_argument("--kalman-sigma-accel", type=float, default=5000.0, help="SSKF accel noise std (counts/s^2) (default: 5000)")
    parser.add_argument("--biquad-cutoff-hz", type=float, default=50.0, help="Biquad Butterworth cutoff in Hz (default: 50)")
    parser.add_argument("--luen-wn-hz", type=float, default=60.0, help="Luenberger observer natural frequency in Hz (default: 60)")
    parser.add_argument("--luen-zeta", type=float, default=0.7, help="Luenberger observer damping ratio (default: 0.7)")
    parser.add_argument("--sg-window", type=int, default=7, help="Savitzky-Golay window length (odd) (default: 7)")
    parser.add_argument("--wls-window", type=int, default=9, help="WLS window length (default: 9)")
    
    args = parser.parse_args()
    
    # Default file path
    if args.file:
        filepath = args.file
    else:
        # Look in data folder
        data_dir = Path(__file__).parent / "data"
        default_file = data_dir / "sensor_noise.bin"
        
        if default_file.exists():
            filepath = str(default_file.resolve())  # Use absolute path
            print(f"Using default file: {filepath}")
        else:
            # List available files
            if data_dir.exists():
                files = list(data_dir.glob("sensor_noise_*.bin"))
                if files:
                    # Use most recent
                    most_recent = max(files, key=os.path.getmtime)
                    filepath = str(most_recent.resolve())  # Use absolute path
                    print(f"Using most recent file: {filepath}")
                else:
                    print(f"No sensor noise files found in {data_dir}")
                    print("Please specify a file path or record some data first.")
                    sys.exit(1)
            else:
                print(f"Data directory not found: {data_dir}")
                print("Please specify a file path.")
                sys.exit(1)
    
    try:
        # Read data
        values = read_binary_file(filepath, debug=args.debug)
        
        # Plot data
        plot_sensor_data(
            values,
            title=args.title,
            save_path=args.save,
            sample_rate_hz=args.sample_rate,
            zoom_seconds=args.zoom_seconds,
            modulus=args.modulus,
            ab_bandwidth_hz=args.ab_bandwidth_hz,
            kalman_sigma_theta=args.kalman_sigma_theta,
            kalman_sigma_accel=args.kalman_sigma_accel,
            biquad_cutoff_hz=args.biquad_cutoff_hz,
            luen_wn_hz=args.luen_wn_hz,
            luen_zeta=args.luen_zeta,
            sg_window=args.sg_window,
            wls_window=args.wls_window,
        )
        
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
