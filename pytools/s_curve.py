import numpy as np
import matplotlib.pyplot as plt

class SCurve:
    def __init__(self, jerk_max, accel_max, vel_max, min_period):
        self.jmax = jerk_max
        self.amax = accel_max
        self.vmax = vel_max
        self.eps = min_period
        self.N = 1000
        self.tau = 0.0  # a quarter period
        self.J = 0.0  # chosen jerk for the current move (<= jmax)
        self.apeak = 0.0
        self.D = 0.0
        self.v0 = 0.0

    def plot_s_curve(self, D, v0=0.0):
        self.D = D
        self.v0 = v0
        self.tau = self.compute_period() / 4.0
        pos_list, vel_list = self.compute_s_curve()
        self.make_plot(pos_list, vel_list)

    def compute_s_curve(self):
        # Full motion spans [0, 4*tau]; sample the whole profile.
        ts = np.linspace(0.0, 4.0 * self.tau, self.N)
        pos_list = np.empty(self.N)
        vel_list = np.empty(self.N)
        tau = self.tau
        J = self.J
        v0 = self.v0

        for i, t in enumerate(ts):
            if t < tau:
                pos_list[i] = self.T1_position(t, v0, J)
                vel_list[i] = self.T1_velocity(t, v0, J)
            elif t < 3.0 * tau:
                pos_list[i] = self.T2n3_position(t, v0, J, tau)
                vel_list[i] = self.T2n3_velocity(t, v0, J, tau)
            else:
                pos_list[i] = self.T4_position(t, v0, J, tau)
                vel_list[i] = self.T4_velocity(t, v0, J, tau)

        return pos_list, vel_list

    def compute_period(self):
        """
        Pick the smallest feasible tau for fixed signed D and v0 such that:
            tau > eps, |J| < jmax, |a_peak| < amax (and |v| <= vmax).

        Planning is performed in the direction of D. The resulting positive
        local jerk is then given D's sign, making the profile bidirectional.
        """
        D, v0 = self.D, self.v0
        jmax, amax, vmax, eps = self.jmax, self.amax, self.vmax, self.eps
        if jmax <= 0.0 or amax <= 0.0 or vmax <= 0.0 or eps < 0.0:
            raise ValueError("jmax, amax and vmax must be positive; eps must be non-negative")
        if abs(v0) > vmax:
            raise ValueError(f"infeasible: |v0|={abs(v0)} exceeds vmax={vmax}")
        if D == 0.0:
            if v0 == 0.0:
                self.J = 0.0
                self.apeak = 0.0
                tau = max(np.nextafter(eps, np.inf), np.finfo(float).eps)
                return 4.0 * tau
            raise ValueError("infeasible: zero displacement with non-zero v0")

        direction = np.sign(D)
        distance = abs(D)
        directed_v0 = direction * v0

        # In the target direction, coasting already covers 4*directed_v0*tau.
        # Therefore positive local jerk requires tau < distance/(4*directed_v0).
        # This is a magnitude threshold on D and works identically in either
        # direction. Velocity opposite D has no such upper bound.
        tau_hi = (
            distance / (4.0 * directed_v0)
            if directed_v0 > 0.0
            else np.inf
        )
        if eps >= tau_hi:
            raise ValueError(
                f"infeasible: |D|={distance} <= "
                f"4*(sign(D)*v0)*eps={4.0 * directed_v0 * eps}; "
                "need a deceleration profile"
            )

        # |a_peak| = (distance - 4*directed_v0*tau)/(2*tau^2) < amax
        tau_a = (
            -directed_v0
            + np.sqrt(directed_v0**2 + 0.5 * amax * distance)
        ) / amax

        # |J| = (distance - 4*directed_v0*tau)/(2*tau^3) < jmax
        roots = np.roots(
            [2.0 * jmax, 0.0, 4.0 * directed_v0, -distance]
        )
        real_positive = [
            r.real for r in roots if abs(r.imag) < 1e-9 and r.real > 0.0
        ]
        if not real_positive:
            raise ValueError(
                f"No valid tau_j for D={D}, v0={v0}, jmax={jmax}"
            )
        tau_j = min(real_positive)

        # Local peak velocity = distance/(2*tau) - directed_v0 <= vmax.
        tau_v = distance / (2.0 * (vmax + directed_v0))

        # Smallest feasible tau = fastest move that still respects every limit.
        # Nudge slightly above the bound so inequalities are strict.
        tau_lo = max(eps, tau_a, tau_j, tau_v)
        if tau_lo >= tau_hi:
            raise ValueError(
                f"infeasible: tau_lo={tau_lo} >= tau_hi={tau_hi} "
                f"(D={D}, v0={v0})"
            )

        tau = min(tau_lo * (1.0 + 1e-6), 0.5 * (tau_lo + tau_hi))
        local_J = (
            distance - 4.0 * directed_v0 * tau
        ) / (2.0 * tau**3)
        J = direction * local_J

        self.J = J
        self.apeak = J * tau
        return 4.0 * tau

    @staticmethod
    def T1_velocity(t, v0, J):
        return 0.5 * J * t**2 + v0

    @staticmethod
    def T2n3_velocity(t, v0, J, tau):
        return -0.5 * J * t**2 + 2 * J * tau * t + v0 - J * tau**2

    @staticmethod
    def T4_velocity(t, v0, J, tau):
        return 0.5 * J * (t - 4 * tau)**2 + v0

    @staticmethod
    def T1_position(t, v0, J):
        return v0 * t + (1.0 / 6.0) * J * t**3

    @staticmethod
    def T2n3_position(t, v0, J, tau):
        return (
            (-1.0 / 6.0) * J * t**3
            + J * tau * t**2
            - J * tau**2 * t
            + v0 * t
            + (1.0 / 3.0) * J * tau**3
        )

    @staticmethod
    def T4_position(t, v0, J, tau):
        return (1.0 / 6.0) * J * (t - 4 * tau)**3 + v0 * t + 2 * J * tau**3

    def make_plot(self, pos_list, vel_list):
        ts = np.linspace(0.0, 4.0 * self.tau, len(pos_list))
        accel = np.gradient(vel_list, ts)
        jerk = np.gradient(accel, ts)

        fig, axes = plt.subplots(4, 1, sharex=True, figsize=(10, 9))
        series = (
            (pos_list, "position", "pos"),
            (vel_list, "velocity", "vel"),
            (accel, "acceleration", "acc"),
            (jerk, "jerk", "jerk"),
        )
        for ax, (y, ylabel, _) in zip(axes, series):
            ax.plot(ts, y)
            ax.set_ylabel(ylabel)
            ax.grid(True, alpha=0.3)
            ax.axvline(self.tau, color="gray", ls="--", lw=0.8)
            ax.axvline(3.0 * self.tau, color="gray", ls="--", lw=0.8)

        axes[-1].set_xlabel("time")
        fig.suptitle(
            f"S-curve  D={self.D:g}  v0={self.v0:g}  "
            f"τ={self.tau:.4g}  J={self.J:.4g}  apeak={self.apeak:.4g}"
        )
        fig.tight_layout()
        plt.show()


if __name__ == "__main__":

    jerk_max = 1000
    accel_max = 4
    vel_max = 10000
    eps = 0.001

    D = 0.01

    s_curve = SCurve(jerk_max, accel_max, vel_max, eps)
    print(f"jmax={s_curve.jmax}  amax={s_curve.amax}  vmax={s_curve.vmax}  eps={s_curve.eps}")
    s_curve.plot_s_curve(D)