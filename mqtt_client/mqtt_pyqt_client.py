import json
import os
import socket
import sys
import time

try:
    from PyQt6.QtCore import QObject, QProcess, QThread, QTimer, pyqtSignal, pyqtSlot
    from PyQt6.QtWidgets import (
        QApplication,
        QCheckBox,
        QFormLayout,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QSpinBox,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )

    QT_API = "PyQt6"
except ImportError:
    from PyQt5.QtCore import QObject, QProcess, QThread, QTimer, pyqtSignal, pyqtSlot
    from PyQt5.QtWidgets import (
        QApplication,
        QCheckBox,
        QFormLayout,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QSpinBox,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )

    QT_API = "PyQt5"

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None


GRIPPER_CMD_TOPIC = "robot/gripper/cmd"
STATUS_TOPIC = "robot/status"
APP_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(APP_DIR)
MOSQUITTO_CANDIDATES = (
    r"C:\Program Files\mosquitto\mosquitto.exe",
    r"C:\Program Files (x86)\mosquitto\mosquitto.exe",
)
DIRECT_ETHERNET_HOST = "192.168.137.1"
LOCALHOST = "127.0.0.1"
DEFAULT_MQTT_PORT = 1883


def find_default_mosquitto():
    for path in MOSQUITTO_CANDIDATES:
        if os.path.exists(path):
            return path
    return "mosquitto"


def can_connect(host, port, timeout=0.25):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def find_default_broker_host():
    return DIRECT_ETHERNET_HOST


class MqttWorker(QObject):
    log = pyqtSignal(str)
    connected_changed = pyqtSignal(bool)
    status_received = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.client = None
        self.connected = False

    @pyqtSlot(str, int)
    def connect_to_broker(self, host, port):
        if mqtt is None:
            self.log.emit("paho-mqtt is not installed. Run: pip install paho-mqtt")
            self.connected_changed.emit(False)
            return

        if self.client is not None:
            self.disconnect_from_broker()

        try:
            try:
                self.client = mqtt.Client(
                    mqtt.CallbackAPIVersion.VERSION2,
                    client_id=f"pyqt_gripper_{int(time.time())}",
                )
            except AttributeError:
                self.client = mqtt.Client(client_id=f"pyqt_gripper_{int(time.time())}")

            self.client.on_connect = self._on_connect
            self.client.on_disconnect = self._on_disconnect
            self.client.on_message = self._on_message
            self.client.connect(host, port, keepalive=60)
            self.client.loop_start()
            self.log.emit(f"Connecting to {host}:{port} ...")
        except Exception as exc:
            self.log.emit(f"Connect failed: {exc}")
            self.log.emit(
                "Hint: if a local Mosquitto service is already running, try "
                "127.0.0.1; for STM32 direct Ethernet, make sure Mosquitto "
                "is listening on 192.168.137.1:1883."
            )
            self.connected_changed.emit(False)

    @pyqtSlot()
    def disconnect_from_broker(self):
        if self.client is None:
            return

        try:
            self.client.loop_stop()
            self.client.disconnect()
        except Exception as exc:
            self.log.emit(f"Disconnect failed: {exc}")
        finally:
            self.client = None
            self.connected = False
            self.connected_changed.emit(False)

    @pyqtSlot(str, str)
    def publish(self, topic, payload):
        if self.client is None or not self.connected:
            self.log.emit("Publish skipped: MQTT is not connected")
            return

        result = self.client.publish(topic, payload, qos=0, retain=False)
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            self.log.emit(f"Publish failed topic={topic} rc={result.rc}")
            return

        self.log.emit(f"TX {topic}: {payload}")

    @staticmethod
    def _reason_code_is_success(reason_code):
        if reason_code == 0:
            return True

        value = getattr(reason_code, "value", None)
        if value == 0:
            return True

        try:
            return int(reason_code) == 0
        except (TypeError, ValueError):
            return str(reason_code).lower() == "success"

    def _on_connect(self, client, userdata, flags, reason_code, *args):
        ok = self._reason_code_is_success(reason_code)
        self.connected = ok
        self.connected_changed.emit(ok)
        if ok:
            self.log.emit("MQTT connected")
            client.subscribe(STATUS_TOPIC, qos=0)
            self.log.emit(f"Subscribed {STATUS_TOPIC}")
        else:
            self.log.emit(f"MQTT connect rejected rc={reason_code}")

    def _on_disconnect(self, client, userdata, reason_code=None, *args):
        self.connected = False
        self.connected_changed.emit(False)
        self.log.emit(f"MQTT disconnected rc={reason_code}")

    def _on_message(self, client, userdata, msg):
        text = msg.payload.decode("utf-8", errors="replace")
        self.log.emit(f"RX {msg.topic}: {text}")
        if msg.topic != STATUS_TOPIC:
            return

        try:
            self.status_received.emit(json.loads(text))
        except json.JSONDecodeError as exc:
            self.log.emit(f"Status JSON parse failed: {exc}")


class MainWindow(QMainWindow):
    connect_requested = pyqtSignal(str, int)
    disconnect_requested = pyqtSignal()
    publish_requested = pyqtSignal(str, str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("STM32 MQTT Server And Gripper Publisher")
        self.setMinimumSize(920, 620)

        self.worker_thread = QThread(self)
        self.worker = MqttWorker()
        self.worker.moveToThread(self.worker_thread)

        self.connect_requested.connect(self.worker.connect_to_broker)
        self.disconnect_requested.connect(self.worker.disconnect_from_broker)
        self.publish_requested.connect(self.worker.publish)
        self.worker.log.connect(self.append_log)
        self.worker.connected_changed.connect(self.set_connected)
        self.worker.status_received.connect(self.update_status)
        self.worker_thread.start()

        self.loop_timer = QTimer(self)
        self.loop_timer.timeout.connect(self.send_next_loop_command)
        self.loop_state = 0
        self.broker_process = QProcess(self)
        self.broker_process.readyReadStandardOutput.connect(self.read_broker_stdout)
        self.broker_process.readyReadStandardError.connect(self.read_broker_stderr)
        self.broker_process.finished.connect(self.broker_finished)

        self._build_ui()
        self.set_connected(False)

    def closeEvent(self, event):
        self.loop_timer.stop()
        self.stop_broker()
        self.disconnect_requested.emit()
        self.worker_thread.quit()
        self.worker_thread.wait(2000)
        super().closeEvent(event)

    def _build_ui(self):
        root = QWidget()
        main = QVBoxLayout(root)

        broker_box = QGroupBox("Local MQTT Broker")
        broker_layout = QGridLayout(broker_box)
        self.mosquitto_edit = QLineEdit(find_default_mosquitto())
        self.config_edit = QLineEdit(os.path.join(PROJECT_DIR, "mosquitto_windows.conf"))
        self.start_broker_btn = QPushButton("Start Broker")
        self.stop_broker_btn = QPushButton("Stop Broker")
        self.broker_label = QLabel("Stopped")
        broker_layout.addWidget(QLabel("Mosquitto"), 0, 0)
        broker_layout.addWidget(self.mosquitto_edit, 0, 1)
        broker_layout.addWidget(QLabel("Config"), 1, 0)
        broker_layout.addWidget(self.config_edit, 1, 1)
        broker_layout.addWidget(self.start_broker_btn, 0, 2)
        broker_layout.addWidget(self.stop_broker_btn, 1, 2)
        broker_layout.addWidget(self.broker_label, 0, 3, 2, 1)

        self.start_broker_btn.clicked.connect(self.start_broker)
        self.stop_broker_btn.clicked.connect(self.stop_broker)

        connection_box = QGroupBox("MQTT Connection")
        connection_layout = QHBoxLayout(connection_box)
        self.host_edit = QLineEdit(find_default_broker_host())
        self.port_spin = QSpinBox()
        self.port_spin.setRange(1, 65535)
        self.port_spin.setValue(DEFAULT_MQTT_PORT)
        self.connect_btn = QPushButton("Connect")
        self.disconnect_btn = QPushButton("Disconnect")
        self.connection_label = QLabel("Disconnected")
        connection_layout.addWidget(QLabel("Broker"))
        connection_layout.addWidget(self.host_edit, 1)
        connection_layout.addWidget(QLabel("Port"))
        connection_layout.addWidget(self.port_spin)
        connection_layout.addWidget(self.connect_btn)
        connection_layout.addWidget(self.disconnect_btn)
        connection_layout.addWidget(self.connection_label)

        self.connect_btn.clicked.connect(self.connect_clicked)
        self.disconnect_btn.clicked.connect(self.disconnect_requested.emit)

        control_box = QGroupBox("Gripper Command")
        control_layout = QGridLayout(control_box)
        self.left_enabled = QCheckBox("Left")
        self.left_enabled.setChecked(True)
        self.right_enabled = QCheckBox("Right")
        self.right_enabled.setChecked(True)
        self.position_spin = self._make_spinbox(0, 100, 50)
        self.speed_spin = self._make_spinbox(0, 1000, 200)
        self.torque_spin = self._make_spinbox(0, 1000, 100)
        self.send_btn = QPushButton("Send")
        self.open_btn = QPushButton("Send 100")
        self.close_btn = QPushButton("Send 0")
        self.loop_btn = QPushButton("Start Loop")
        self.interval_spin = self._make_spinbox(100, 10000, 500)
        self.interval_spin.setSuffix(" ms")

        control_layout.addWidget(self.left_enabled, 0, 0)
        control_layout.addWidget(self.right_enabled, 0, 1)
        control_layout.addWidget(QLabel("Position %"), 1, 0)
        control_layout.addWidget(self.position_spin, 1, 1)
        control_layout.addWidget(QLabel("Speed rpm"), 2, 0)
        control_layout.addWidget(self.speed_spin, 2, 1)
        control_layout.addWidget(QLabel("Torque 0.01A"), 3, 0)
        control_layout.addWidget(self.torque_spin, 3, 1)
        control_layout.addWidget(self.send_btn, 4, 0)
        control_layout.addWidget(self.close_btn, 4, 1)
        control_layout.addWidget(self.open_btn, 4, 2)
        control_layout.addWidget(QLabel("Loop interval"), 5, 0)
        control_layout.addWidget(self.interval_spin, 5, 1)
        control_layout.addWidget(self.loop_btn, 5, 2)

        self.send_btn.clicked.connect(self.send_manual_command)
        self.close_btn.clicked.connect(lambda: self.send_position(0))
        self.open_btn.clicked.connect(lambda: self.send_position(100))
        self.loop_btn.clicked.connect(self.toggle_loop)

        status_box = QGroupBox("robot/status")
        status_layout = QFormLayout(status_box)
        self.left_status = QLabel("-")
        self.right_status = QLabel("-")
        self.kinco_status = QLabel("-")
        self.zeroerr_status = QLabel("-")
        self.gpio_status = QLabel("-")
        status_layout.addRow("Left gripper", self.left_status)
        status_layout.addRow("Right gripper", self.right_status)
        status_layout.addRow("Kinco", self.kinco_status)
        status_layout.addRow("ZeroErr", self.zeroerr_status)
        status_layout.addRow("GPIO input", self.gpio_status)

        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)

        main.addWidget(broker_box)
        main.addWidget(connection_box)
        main.addWidget(control_box)
        main.addWidget(status_box)
        main.addWidget(self.log_view, 1)
        self.setCentralWidget(root)

    def _make_spinbox(self, minimum, maximum, value):
        spin = QSpinBox()
        spin.setRange(minimum, maximum)
        spin.setValue(value)
        return spin

    def connect_clicked(self):
        self.connect_requested.emit(self.host_edit.text().strip(), self.port_spin.value())

    def start_broker(self):
        if self.broker_process.state() != self._process_state_not_running():
            self.append_log("Broker is already running")
            return

        program = self.mosquitto_edit.text().strip()
        config = self.config_edit.text().strip()
        args = ["-c", config, "-v"]
        self.broker_process.start(program, args)
        if not self.broker_process.waitForStarted(3000):
            self.append_log(
                "MQTT服务器启动失败：没有找到 mosquitto.exe，或 Mosquitto 没有加入 PATH。"
            )
            self.append_log(
                r"处理方法：安装 Mosquitto for Windows，或在 Mosquitto 输入框填写完整路径，"
                r"例如 C:\Program Files\mosquitto\mosquitto.exe"
            )
            self.broker_label.setText("Stopped")
            return

        self.broker_label.setText("Running")
        self.host_edit.setText(DIRECT_ETHERNET_HOST)
        self.append_log(f"Broker started: {program} {' '.join(args)}")
        self.append_log("Broker should listen on 192.168.137.1:1883 for direct Ethernet.")

    def stop_broker(self):
        if self.broker_process.state() == self._process_state_not_running():
            self.broker_label.setText("Stopped")
            return

        self.broker_process.terminate()
        if not self.broker_process.waitForFinished(2000):
            self.broker_process.kill()
            self.broker_process.waitForFinished(2000)
        self.broker_label.setText("Stopped")
        self.append_log("Broker stopped")

    def broker_finished(self, exit_code, exit_status):
        self.broker_label.setText("Stopped")
        status_name = getattr(exit_status, "name", str(exit_status))
        self.append_log(f"Broker exited code={exit_code} status={status_name}")

    def _process_state_not_running(self):
        return getattr(getattr(QProcess, "ProcessState", QProcess), "NotRunning")

    def read_broker_stdout(self):
        text = bytes(self.broker_process.readAllStandardOutput()).decode(
            "utf-8", errors="replace"
        )
        for line in text.splitlines():
            self.append_log(f"BROKER {line}")

    def read_broker_stderr(self):
        text = bytes(self.broker_process.readAllStandardError()).decode(
            "utf-8", errors="replace"
        )
        for line in text.splitlines():
            self.append_log(f"BROKER {line}")

    def set_connected(self, connected):
        self.connection_label.setText("Connected" if connected else "Disconnected")
        self.connect_btn.setEnabled(not connected)
        self.disconnect_btn.setEnabled(connected)

    def make_gripper_payload(self, position):
        percent = max(0, min(100, int(position)))
        command = {"mode": 0}
        item = {
            "position": percent,
            "speed": self.speed_spin.value(),
            "torque": self.torque_spin.value(),
        }
        if self.left_enabled.isChecked():
            command["left"] = item
        if self.right_enabled.isChecked():
            command["right"] = item
        return json.dumps(command, separators=(",", ":"))

    def send_position(self, position):
        if not self.left_enabled.isChecked() and not self.right_enabled.isChecked():
            QMessageBox.warning(self, "No side selected", "Select left, right, or both grippers.")
            return
        payload = self.make_gripper_payload(position)
        self.publish_requested.emit(GRIPPER_CMD_TOPIC, payload)

    def send_manual_command(self):
        self.send_position(self.position_spin.value())

    def toggle_loop(self):
        if self.loop_timer.isActive():
            self.loop_timer.stop()
            self.loop_btn.setText("Start Loop")
            self.append_log("Loop stopped")
            return

        self.loop_state = 0
        self.loop_timer.start(self.interval_spin.value())
        self.loop_btn.setText("Stop Loop")
        self.append_log("Loop started")
        self.send_next_loop_command()

    def send_next_loop_command(self):
        self.send_position(0 if self.loop_state == 0 else 100)
        self.loop_state = 1 - self.loop_state

    def update_status(self, status):
        self.left_status.setText(self._format_dict(status.get("left_gripper")))
        self.right_status.setText(self._format_dict(status.get("right_gripper")))
        self.kinco_status.setText(self._format_dict(status.get("sys_kinco")))
        self.zeroerr_status.setText(self._format_dict(status.get("sys_zeroerr")))
        gpio_in = status.get("gpio_in")
        self.gpio_status.setText(str(gpio_in) if gpio_in is not None else "-")

    def _format_dict(self, value):
        if not isinstance(value, dict):
            return "-"
        return "  ".join(f"{key}={val}" for key, val in value.items())

    def append_log(self, text):
        self.log_view.append(text)


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.append_log(f"Qt binding: {QT_API}")
    window.show()
    if hasattr(app, "exec"):
        sys.exit(app.exec())
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
