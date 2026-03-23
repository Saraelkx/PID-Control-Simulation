from pid import PIDController


def test_pid_zero_error_gives_zero_output():
    pid = PIDController(kp=2.0, ki=1.0, kd=0.5, dt=0.1)
    u = pid.compute(setpoint=1.0, measurement=1.0)
    assert abs(u) < 1e-9


def test_pid_proportional_response():
    pid = PIDController(kp=2.0, ki=0.0, kd=0.0, dt=0.1)
    u = pid.compute(setpoint=3.0, measurement=1.0)
    assert abs(u - 4.0) < 1e-9


def test_pid_respects_saturation():
    pid = PIDController(kp=100.0, ki=0.0, kd=0.0, dt=0.1, u_min=-5.0, u_max=5.0)
    u = pid.compute(setpoint=10.0, measurement=0.0)
    assert u == 5.0