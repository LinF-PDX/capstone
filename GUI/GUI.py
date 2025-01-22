import sys
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QLineEdit, QPushButton, QFrame, QSlider
)
from PyQt6.QtGui import QIcon, QPainter, QColor, QPixmap
from PyQt6.QtCore import Qt, QTimer, QElapsedTimer
import json
import os

class MainWindow(QMainWindow):
    def __init__(self): #初始化窗口和界面布局
        super().__init__()

        # Detect screen resolution and set window size to 3/4 of the screen
        screen = QApplication.primaryScreen()
        if screen is None:
            raise RuntimeError("Unable to detect the primary screen.")
        geometry = screen.geometry()
        width, height = int(geometry.width() * 0.75), int(geometry.height() * 0.75)
        self.resize(width, height)
        self.move((geometry.width() - width) // 2, (geometry.height() - height) // 2)

        self.setWindowTitle("Profilograph GUI")
        self.setWindowIcon(QIcon.fromTheme("preferences-system"))

        # Create central widget and layout
        central_widget = QWidget()
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        # Left panel for parameters
        left_panel = QFrame()
        left_panel.setFrameShape(QFrame.Shape.Box)
        left_panel_layout = QVBoxLayout()
        left_panel.setLayout(left_panel_layout)

        self.variables = {}
        variable_names = ["S_surveyDistance", "S_wheelBase", "S_heightThreashold", "actualboardwidth", "lasercolor"]
        default_values = {"S_surveyDistance": 100.0, "S_wheelBase": 1300, "S_heightThreashold": 10.0, "actualboardwidth": 13.5, "lasercolor": "green"}
        self.default_values = default_values

        #添加滑条
        
        for name in variable_names:
            label = QLabel(name)
            slider = QSlider(Qt.Orientation.Horizontal)
            input_field = QLineEdit()

            if isinstance(default_values[name], float):
                slider.setMinimum(0)
                slider.setMaximum(int(default_values[name] * 3 * 10))
                slider.setSingleStep(1)
                slider.setPageStep(10)
                input_field.setPlaceholderText("Enter a float value")
            elif isinstance(default_values[name], int):
                slider.setMinimum(0)
                slider.setMaximum(default_values[name] * 3)
                slider.setSingleStep(1)
                slider.setPageStep(10)
                input_field.setPlaceholderText("Enter an integer value")
            else:
                slider.setEnabled(False)
                input_field.setText(default_values[name])

            def sync_slider_input(slider=slider, input_field=input_field, name=name):
                if isinstance(default_values[name], float):
                    value = slider.value() / 10.0
                    input_field.setText(f"{value:.1f}")
                elif isinstance(default_values[name], int):
                    value = slider.value()
                    input_field.setText(str(value))

            def sync_input_slider(slider=slider, input_field=input_field, name=name):
                try:
                    if isinstance(default_values[name], float):
                        value = float(input_field.text())
                        slider.setValue(int(value * 10))
                    elif isinstance(default_values[name], int):
                        value = int(input_field.text())
                        slider.setValue(value)
                    elif isinstance(default_values[name], str):
                        value = str(input_field.text())
                        slider.setValue(value)
                except ValueError:
                    pass

            slider.valueChanged.connect(sync_slider_input)
            input_field.textChanged.connect(sync_input_slider)

            left_panel_layout.addWidget(label)
            left_panel_layout.addWidget(slider)
            left_panel_layout.addWidget(input_field)
            self.variables[name] = input_field

        #添加按钮
        apply_button = QPushButton("Apply Configuration")
        apply_button.clicked.connect(self.apply_configuration)
        config_button = QPushButton("Configure to Machine")
        config_button.clicked.connect(self.configure_to_machine)
        config_button.setEnabled(False)
        left_panel_layout.addWidget(apply_button)
        left_panel_layout.addWidget(config_button)

        default_button = QPushButton("Default")
        default_button.clicked.connect(self.set_default_values)
        left_panel_layout.addWidget(default_button)

        start_button = QPushButton("Start Surveying")
        stop_button = QPushButton("Stop Surveying")
        start_button.setEnabled(False)
        stop_button.setEnabled(False)
        start_button.clicked.connect(self.start_surveying)
        stop_button.clicked.connect(self.stop_surveying)
        left_panel_layout.addWidget(start_button)
        left_panel_layout.addWidget(stop_button)

        self.config_status = QLabel("")
        left_panel_layout.addWidget(self.config_status)

        # Right panel for graph and machine data
        right_panel = QFrame()
        right_panel.setFrameShape(QFrame.Shape.Box)
        right_layout = QVBoxLayout()
        right_panel.setLayout(right_layout)

        # Graph placeholder
        self.graph_label = QLabel("Graph Area")
        self.graph_label.setStyleSheet("background-color: white; border: 1px solid black;")
        self.graph_label.setMinimumHeight(int(height * 0.8))
        right_layout.addWidget(self.graph_label)

        # Machine serial input
        serial_layout = QHBoxLayout()
        self.serial_input = QLineEdit()
        self.serial_input.setPlaceholderText("Enter Machine Serial Number")
        self.serial_input.setMaximumWidth(200)
        connect_button = QPushButton("Connect")
        connect_button.clicked.connect(self.connect_to_machine)
        serial_layout.addWidget(self.serial_input)
        serial_layout.addWidget(connect_button)
        self.connection_result = QLabel("")
        serial_layout.addWidget(self.connection_result)
        right_layout.addLayout(serial_layout)

        # Connection and machine status
        status_layout = QVBoxLayout()

        connection_status_layout = QHBoxLayout()
        self.connection_status = QLabel("Connection Status: ")
        self.connection_light = QLabel()
        self.connection_light.setFixedSize(20, 20)
        connection_status_layout.addWidget(self.connection_status)
        connection_status_layout.addWidget(self.connection_light)
        status_layout.addLayout(connection_status_layout)

        machine_status_layout = QHBoxLayout()
        self.machine_status = QLabel("Machine Status: ")
        self.machine_light = QLabel()
        self.machine_light.setFixedSize(20, 20)
        machine_status_layout.addWidget(self.machine_status)
        machine_status_layout.addWidget(self.machine_light)
        status_layout.addLayout(machine_status_layout)

        right_layout.addLayout(status_layout)

        # Add panels to main layout
        main_layout.addWidget(left_panel, 1)
        main_layout.addWidget(right_panel, 3)

        # Initialize graph update
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_graph)
        self.click_timer = QElapsedTimer()

        # Connection state
        self.is_connected = False
        self.is_machine_running = False
        self.graph_data = {"C_drivenDistance": [], "C_transverseHeight": []}
        self.load_previous_configuration()
        self.update_status()

    def connect_to_machine(self): #连接到Profilograph，需在树莓派端加入AP热点逻辑，辅助树莓派连接Wifi，并发送IP回来用于通信
        serial = self.serial_input.text()
        self.connection_result.setText("")
        if serial == "12345":  # Simulated connection success
            self.is_connected = True
            self.connection_result.setText("Connected Successfully")
            self.timer.start(1000)
            self.update_status()
        else:
            self.is_connected = False
            self.connection_result.setText("Connection Failed")
        self.update_status()

    def configure_to_machine(self): #将初始配置发送到机器
        if not self.is_connected:
            self.config_status.setText("Please connect to profileograph first.")
            return

        # Simulate sending configuration
        ack = "ack"  # Simulated machine acknowledgment
        if ack == "ack":
            self.config_status.setText("Configure successfully.")
        else:
            self.config_status.setText("Error in configuration.")

    def apply_configuration(self): #用户自定义输入配置应用
        config = {name: self.variables[name].text() for name in self.variables}
        with open("temp_config.json", "w") as file:
            json.dump(config, file)
        self.config_status.setText("Done")

    def set_default_values(self): #把值设为默认值
        for name, value in self.default_values.items():
            self.variables[name].setText(str(value))

    def load_previous_configuration(self): #把用户上次apply的配置加载进来
        if os.path.exists("temp_config.json"):
            with open("temp_config.json", "r") as file:
                config = json.load(file)
                for name, value in config.items():
                    if name in self.variables:
                        self.variables[name].setText(value)

    def start_surveying(self): #给机器发送开始指令
        if self.click_timer.isValid() and self.click_timer.elapsed() < 2000:
            return
        self.click_timer.restart()
        self.is_machine_running = True
        self.update_status()

    def stop_surveying(self): #给机器发送结束指令
        if self.click_timer.isValid() and self.click_timer.elapsed() < 2000:
            return
        self.click_timer.restart()
        self.is_machine_running = False
        self.update_status()

    def update_status(self): #实时更新机器的运行状态和连接状态
        if self.is_connected:
            self.connection_status.setText("Connection Status: Connected")
            self.connection_light.setStyleSheet("background-color: green; border-radius: 10px;")
            if self.is_machine_running:
                self.machine_status.setText("Machine Status: Running")
                self.machine_light.setStyleSheet("background-color: green; border-radius: 10px;")
            else:
                self.machine_status.setText("Machine Status: Stopped")
                self.machine_light.setStyleSheet("background-color: red; border-radius: 10px;")
        else:
            self.connection_status.setText("Connection Status: Disconnected")
            self.connection_light.setStyleSheet("background-color: red; border-radius: 10px;")
            self.machine_status.setText("Machine Status: Stopped")
            self.machine_light.setStyleSheet("background-color: gray; border-radius: 10px;")

        #self.centralWidget().findChild(QPushButton, "StartSurveyingButton").setEnabled(self.is_connected and not self.is_machine_running)
        #self.centralWidget().findChild(QPushButton, "StopSurveyingButton").setEnabled(self.is_connected and self.is_machine_running)

    def update_graph(self): #画图
        if not self.is_machine_running or not self.graph_data["C_drivenDistance"]:
            return

        pixmap = QPixmap(self.graph_label.size())
        pixmap.fill(Qt.GlobalColor.white)

        painter = QPainter(pixmap)
        pen = painter.pen()

        for i in range(1, len(self.graph_data["C_drivenDistance"])):
            x1 = self.graph_data["C_drivenDistance"][i - 1]
            y1 = self.graph_data["C_transverseHeight"][i - 1]
            x2 = self.graph_data["C_drivenDistance"][i]
            y2 = self.graph_data["C_transverseHeight"][i]

            if y1 <= float(self.variables["S_heightThreashold"].text()):
                pen.setColor(Qt.GlobalColor.black)
            else:
                pen.setColor(Qt.GlobalColor.red)

            painter.setPen(pen)
            painter.drawLine(x1, y1, x2, y2)

        painter.end()
        self.graph_label.setPixmap(pixmap)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec())