import numpy as np


def step_setpoint(t: float, magnitude: float = 1.0, step_time: float = 0.0) -> float:
    return magnitude if t >= step_time else 0.0


def disturbance_signal(t: float, enabled: bool, magnitude: float, start_time: float) -> float:
    if enabled and t >= start_time:
        return magnitude
    return 0.0


def compute_metrics(time, y, setpoint_value, tolerance=0.02):
    y = np.array(y)
    time = np.array(time)

    if len(y) == 0:
        return {
            "overshoot_pct": None,
            "steady_state_error": None,
            "settling_time": None,
            "rise_time": None,
        }

    final_value = y[-1]
    steady_state_error = setpoint_value - final_value

    peak = np.max(y)
    if setpoint_value != 0:
        overshoot_pct = max(0.0, (peak - setpoint_value) / abs(setpoint_value) * 100.0)
    else:
        overshoot_pct = 0.0

    # Rise time: first time response reaches 90% of setpoint
    rise_time = None
    if setpoint_value != 0:
        threshold = 0.9 * setpoint_value
        crossed = np.where(y >= threshold)[0]
        if len(crossed) > 0:
            rise_time = float(time[crossed[0]])

    # Settling time: first time after which response stays within tolerance band
    settling_time = None
    band = tolerance * abs(setpoint_value) if setpoint_value != 0 else tolerance
    for i in range(len(y)):
        if np.all(np.abs(y[i:] - setpoint_value) <= band):
            settling_time = float(time[i])
            break

    return {
        "overshoot_pct": overshoot_pct,
        "steady_state_error": steady_state_error,
        "settling_time": settling_time,
        "rise_time": rise_time,
    }


def run_simulation(
    controller,
    plant,
    total_time=10.0,
    dt=0.01,
    setpoint_value=1.0,
    setpoint_step_time=0.0,
    disturbance_enabled=False,
    disturbance_magnitude=0.0,
    disturbance_start_time=5.0,
):
    controller.reset()
    plant.reset()

    time = np.arange(0.0, total_time, dt)

    y_values = []
    u_values = []
    setpoints = []
    errors = []

    for t in time:
        sp = step_setpoint(t, setpoint_value, setpoint_step_time)
        d = disturbance_signal(
            t,
            enabled=disturbance_enabled,
            magnitude=disturbance_magnitude,
            start_time=disturbance_start_time,
        )

        y = plant.output
        u = controller.compute(sp, y)
        y_new = plant.update(u, dt, disturbance=d)

        y_values.append(y_new)
        u_values.append(u)
        setpoints.append(sp)
        errors.append(sp - y_new)

    metrics = compute_metrics(time, y_values, setpoint_value)

    return {
        "time": time,
        "y": np.array(y_values),
        "u": np.array(u_values),
        "setpoint": np.array(setpoints),
        "error": np.array(errors),
        "metrics": metrics,
    }