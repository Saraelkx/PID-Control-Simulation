import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QSlider,
    QPushButton,
    QCheckBox,
    QComboBox,
    QGridLayout,
    QGroupBox,
)

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from pid import PIDController
from plants import MassSpringDamperPlant, FirstOrderPlant
from sim import run_simulation


class LabeledSlider(QWidget):
    def __init__(self, label_text, min_val, max_val, scale=10, default=0):
        super().__init__()
        self.scale = scale
        self.label_text = label_text

        layout = QVBoxLayout()
        self.label = QLabel()
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(min_val)
        self.slider.setMaximum(max_val)
        self.slider.setValue(default)

        self.slider.valueChanged.connect(self.update_label)
        self.update_label()

        layout.addWidget(self.label)
        layout.addWidget(self.slider)
        self.setLayout(layout)

    def update_label(self):
        self.label.setText(f"{self.label_text}: {self.value():.2f}")

    def value(self):
        return self.slider.value() / self.scale


class PIDSimulatorUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PID Controller Simulator")
        self.resize(1200, 800)

        main_layout = QHBoxLayout()
        controls_layout = QVBoxLayout()

        controls_layout.addWidget(self.build_controller_group())
        controls_layout.addWidget(self.build_plant_group())
        controls_layout.addWidget(self.build_sim_group())
        controls_layout.addWidget(self.build_metrics_group())

        self.run_button = QPushButton("Run Simulation")
        self.run_button.clicked.connect(self.update_plot)
        controls_layout.addWidget(self.run_button)

        self.auto_run_checkbox = QCheckBox("Auto-run on change")
        self.auto_run_checkbox.setChecked(True)
        controls_layout.addWidget(self.auto_run_checkbox)

        controls_layout.addStretch()

        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)

        main_layout.addLayout(controls_layout, 1)
        main_layout.addWidget(self.canvas, 2)
        self.setLayout(main_layout)

        self.connect_auto_update()
        self.update_plot()

    def build_controller_group(self):
        group = QGroupBox("PID Controller")
        layout = QVBoxLayout()

        self.kp_slider = LabeledSlider("Kp", 0, 300, scale=10, default=10)
        self.ki_slider = LabeledSlider("Ki", 0, 200, scale=10, default=0)
        self.kd_slider = LabeledSlider("Kd", 0, 100, scale=10, default=0)

        self.u_min_slider = LabeledSlider("u_min", -200, 0, scale=10, default=-100)
        self.u_max_slider = LabeledSlider("u_max", 0, 200, scale=10, default=100)

        self.anti_windup_checkbox = QCheckBox("Enable anti-windup")
        self.anti_windup_checkbox.setChecked(True)

        layout.addWidget(self.kp_slider)
        layout.addWidget(self.ki_slider)
        layout.addWidget(self.kd_slider)
        layout.addWidget(self.u_min_slider)
        layout.addWidget(self.u_max_slider)
        layout.addWidget(self.anti_windup_checkbox)

        group.setLayout(layout)
        return group

    def build_plant_group(self):
        group = QGroupBox("Plant")
        layout = QVBoxLayout()

        self.plant_selector = QComboBox()
        self.plant_selector.addItems(["Mass-Spring-Damper", "First-Order"])

        self.mass_slider = LabeledSlider("Mass m", 1, 100, scale=10, default=10)
        self.damping_slider = LabeledSlider("Damping c", 0, 100, scale=10, default=5)
        self.stiffness_slider = LabeledSlider("Stiffness k", 0, 100, scale=10, default=20)

        self.tau_slider = LabeledSlider("Tau", 1, 100, scale=10, default=10)
        self.gain_slider = LabeledSlider("Gain", 1, 100, scale=10, default=10)

        layout.addWidget(QLabel("Plant Type"))
        layout.addWidget(self.plant_selector)
        layout.addWidget(self.mass_slider)
        layout.addWidget(self.damping_slider)
        layout.addWidget(self.stiffness_slider)
        layout.addWidget(self.tau_slider)
        layout.addWidget(self.gain_slider)

        group.setLayout(layout)
        return group

    def build_sim_group(self):
        group = QGroupBox("Simulation")
        layout = QVBoxLayout()

        self.setpoint_slider = LabeledSlider("Setpoint", 0, 50, scale=10, default=10)
        self.total_time_slider = LabeledSlider("Total Time", 10, 300, scale=10, default=100)
        self.dt_slider = LabeledSlider("dt", 1, 20, scale=1000, default=10)

        self.disturbance_checkbox = QCheckBox("Enable disturbance")
        self.disturbance_checkbox.setChecked(False)

        self.disturbance_mag_slider = LabeledSlider("Disturbance", -50, 50, scale=10, default=0)
        self.disturbance_time_slider = LabeledSlider("Disturbance Start", 0, 100, scale=10, default=50)

        layout.addWidget(self.setpoint_slider)
        layout.addWidget(self.total_time_slider)
        layout.addWidget(self.dt_slider)
        layout.addWidget(self.disturbance_checkbox)
        layout.addWidget(self.disturbance_mag_slider)
        layout.addWidget(self.disturbance_time_slider)

        group.setLayout(layout)
        return group

    def build_metrics_group(self):
        group = QGroupBox("Performance Metrics")
        layout = QGridLayout()

        self.overshoot_label = QLabel("Overshoot: --")
        self.sse_label = QLabel("Steady-state error: --")
        self.settling_label = QLabel("Settling time: --")
        self.rise_label = QLabel("Rise time: --")

        layout.addWidget(self.overshoot_label, 0, 0)
        layout.addWidget(self.sse_label, 1, 0)
        layout.addWidget(self.settling_label, 2, 0)
        layout.addWidget(self.rise_label, 3, 0)

        group.setLayout(layout)
        return group

    def connect_auto_update(self):
        widgets = [
            self.kp_slider.slider,
            self.ki_slider.slider,
            self.kd_slider.slider,
            self.u_min_slider.slider,
            self.u_max_slider.slider,
            self.mass_slider.slider,
            self.damping_slider.slider,
            self.stiffness_slider.slider,
            self.tau_slider.slider,
            self.gain_slider.slider,
            self.setpoint_slider.slider,
            self.total_time_slider.slider,
            self.dt_slider.slider,
            self.disturbance_mag_slider.slider,
            self.disturbance_time_slider.slider,
        ]

        for widget in widgets:
            widget.valueChanged.connect(self.maybe_update_plot)

        self.anti_windup_checkbox.stateChanged.connect(self.maybe_update_plot)
        self.disturbance_checkbox.stateChanged.connect(self.maybe_update_plot)
        self.plant_selector.currentIndexChanged.connect(self.maybe_update_plot)

    def maybe_update_plot(self):
        if self.auto_run_checkbox.isChecked():
            self.update_plot()

    def create_plant(self):
        if self.plant_selector.currentText() == "Mass-Spring-Damper":
            return MassSpringDamperPlant(
                m=self.mass_slider.value(),
                c=self.damping_slider.value(),
                k=self.stiffness_slider.value(),
            )
        return FirstOrderPlant(
            tau=self.tau_slider.value(),
            gain=self.gain_slider.value(),
        )

    def update_metrics(self, metrics):
        overshoot = metrics["overshoot_pct"]
        sse = metrics["steady_state_error"]
        settling = metrics["settling_time"]
        rise = metrics["rise_time"]

        self.overshoot_label.setText(
            f"Overshoot: {overshoot:.2f}%" if overshoot is not None else "Overshoot: --"
        )
        self.sse_label.setText(
            f"Steady-state error: {sse:.4f}" if sse is not None else "Steady-state error: --"
        )
        self.settling_label.setText(
            f"Settling time: {settling:.3f} s" if settling is not None else "Settling time: --"
        )
        self.rise_label.setText(
            f"Rise time: {rise:.3f} s" if rise is not None else "Rise time: --"
        )

    def update_plot(self):
        dt = self.dt_slider.value()
        controller = PIDController(
            kp=self.kp_slider.value(),
            ki=self.ki_slider.value(),
            kd=self.kd_slider.value(),
            dt=dt,
            u_min=self.u_min_slider.value(),
            u_max=self.u_max_slider.value(),
            anti_windup=self.anti_windup_checkbox.isChecked(),
        )

        plant = self.create_plant()

        result = run_simulation(
            controller=controller,
            plant=plant,
            total_time=self.total_time_slider.value(),
            dt=dt,
            setpoint_value=self.setpoint_slider.value(),
            disturbance_enabled=self.disturbance_checkbox.isChecked(),
            disturbance_magnitude=self.disturbance_mag_slider.value(),
            disturbance_start_time=self.disturbance_time_slider.value(),
        )

        t = result["time"]
        y = result["y"]
        u = result["u"]
        sp = result["setpoint"]

        self.figure.clear()

        ax1 = self.figure.add_subplot(211)
        ax1.plot(t, y, label="Output y(t)")
        ax1.plot(t, sp, linestyle="--", label="Setpoint")
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel("Output")
        ax1.set_title("System Response")
        ax1.legend()
        ax1.grid(True)

        ax2 = self.figure.add_subplot(212)
        ax2.plot(t, u, label="Control Effort u(t)")
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("u(t)")
        ax2.set_title("Controller Output")
        ax2.legend()
        ax2.grid(True)

        self.figure.tight_layout()
        self.canvas.draw()

        self.update_metrics(result["metrics"])