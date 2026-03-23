import sys
import numpy as np
import matplotlib.pyplot as plt
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QSlider, QLabel, QPushButton
)
from PySide6.QtCore import Qt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# Simulation Function
def run_sim(Kp, Ki, Kd):
    m, c, k = 1.0, 0.5, 2.0
    dt = 0.01
    time = np.arange(0, 10, dt)
    setpoint = 1.0

    x, v = 0, 0
    integral = 0
    prev_error = setpoint - x

    y_values = []

    for t in time:
        error = setpoint - x
        derivative = (error - prev_error) / dt
        integral += error * dt

        u = Kp * error + Ki * integral + Kd * derivative
        
        a = (u - c*v - k*x) / m
        v += dt * a
        x += dt * v

        y_values.append(x)
        prev_error = error

    return time, y_values

# GUI Class
class PIDSimulator(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PID Simulator")
        layout = QVBoxLayout()

        self.kp_slider = self.create_slider("Kp", layout)
        self.ki_slider = self.create_slider("Ki", layout)
        self.kd_slider = self.create_slider("Kd", layout)

        self.run_button = QPushButton("Run Simulation")
        self.run_button.clicked.connect(self.update_plot)
        layout.addWidget(self.run_button)

        self.figure = Figure()

        self.canvas = FigureCanvas(self.figure)
        layout.addWidget (self.canvas)

        self.setLayout(layout)

    def create_slider(self, name, layout):
        label = QLabel(f"{name}: 1.0")
        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(0)
        slider.setMaximum(100)
        slider.setValue(10)
        slider.valueChanged.connect(lambda val, lbl=label, nm=name: lbl.setText(f"{nm}: {val/10:.1f}"))
        layout.addWidget(label)
        layout.addWidget(slider)

        return slider
    
    def update_plot(self):
        Kp = self.kp_slider.value() / 10
        Ki = self.ki_slider.value() / 10
        Kd = self.kd_slider.value() / 10

        time, x_values = run_sim(Kp, Ki, Kd)
        self.figure.clear()

        ax = self.figure.add_subplot(111)
        ax.plot(time, x_values, label="Output x(t)")
        ax.axhline(1.0, linestyle="--", label="Setpoint")

        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Position x")
        ax.set_title("PID Response (2nd Order Mass-Spring Damper)")
        ax.legend()

        self.canvas.draw()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = PIDSimulator()
    window.show()
    sys.exit(app.exec())

