from dataclasses import dataclass


@dataclass
class PIDController:
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0
    dt: float = 0.01
    u_min: float = -10.0
    u_max: float = 10.0
    anti_windup: bool = True

    def __post_init__(self):
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0
        self.first_update = True

    def compute(self, setpoint: float, measurement: float) -> float:
        error = setpoint - measurement

        if self.first_update:
            derivative = 0.0
            self.first_update = False
        else:
            derivative = (error - self.prev_error) / self.dt

        # candidate integral
        new_integral = self.integral + error * self.dt

        u_unsat = (
            self.kp * error
            + self.ki * new_integral
            + self.kd * derivative
        )

        u = max(self.u_min, min(self.u_max, u_unsat))

        # simple anti-windup:
        # only accept integral update if either:
        # 1) not saturated, or
        # 2) saturated but error would push output back toward unsaturated region
        if self.anti_windup:
            saturated_high = u_unsat > self.u_max
            saturated_low = u_unsat < self.u_min

            allow_integral = (
                not saturated_high and not saturated_low
                or (saturated_high and error < 0)
                or (saturated_low and error > 0)
            )

            if allow_integral:
                self.integral = new_integral
        else:
            self.integral = new_integral

        self.prev_error = error
        return u