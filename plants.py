from dataclasses import dataclass


@dataclass
class MassSpringDamperPlant:
    m: float = 1.0
    c: float = 0.5
    k: float = 2.0
    x: float = 0.0
    v: float = 0.0

    def reset(self) -> None:
        self.x = 0.0
        self.v = 0.0

    def update(self, u: float, dt: float, disturbance: float = 0.0) -> float:
        # m*x'' + c*x' + k*x = u + disturbance
        a = (u + disturbance - self.c * self.v - self.k * self.x) / self.m
        self.v += a * dt
        self.x += self.v * dt
        return self.x

    @property
    def output(self) -> float:
        return self.x


@dataclass
class FirstOrderPlant:
    tau: float = 1.0
    gain: float = 1.0
    y: float = 0.0

    def reset(self) -> None:
        self.y = 0.0

    def update(self, u: float, dt: float, disturbance: float = 0.0) -> float:
        # tau * y' + y = gain*u + disturbance
        dydt = (-self.y + self.gain * u + disturbance) / self.tau
        self.y += dydt * dt
        return self.y

    @property
    def output(self) -> float:
        return self.y