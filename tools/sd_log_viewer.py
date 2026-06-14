#!/usr/bin/env python3
import sys
from pathlib import Path

try:
    from PyQt5 import QtCore, QtWidgets
except ImportError:
    try:
        from PyQt6 import QtCore, QtWidgets
    except ImportError as exc:
        raise SystemExit("Install PyQt5 or PyQt6 first: pip install PyQt5") from exc

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

from parse_sd_log import (
    BLOCK_SIZE,
    TYPE_IMU,
    TYPE_JOINT,
    TYPE_TACTILE,
    IMU_FLOAT_COUNT,
    JOINT_FLOAT_COUNT,
    TACTILE_U16_COUNT,
    parse_one_block,
    type_name,
)


REG_SD_LOG_STATUS = 0x0082
REG_SD_ERROR_CODE = 0x0083
REG_SD_CURRENT_FILE_SIZE = 0x008C
REG_SD_CURRENT_WRITE_CNT = 0x0090
REG_SD_DISK_LAST_RESULT = 0x0094
REG_SD_DISK_HAL_STATUS = 0x0095
REG_SD_DISK_HAL_ERROR = 0x0096

SD_LOG_STATUS_IDLE = 0x0000
SD_LOG_STATUS_RECORDING = 0x0001
REG_CMD = 0x0020
CMD_LOG_START = 0x0094
CMD_LOG_STOP = 0x0096
CMD_SD_RESET = 0x0098
FRESULT_NAMES = {
    0x00: "FR_OK",
    0x01: "FR_DISK_ERR",
    0x02: "FR_INT_ERR",
    0x03: "FR_NOT_READY",
    0x04: "FR_NO_FILE",
    0x05: "FR_NO_PATH",
    0x06: "FR_INVALID_NAME",
    0x07: "FR_DENIED",
    0x08: "FR_EXIST",
    0x09: "FR_INVALID_OBJECT",
    0x0A: "FR_WRITE_PROTECTED",
    0x0B: "FR_INVALID_DRIVE",
    0x0C: "FR_NOT_ENABLED",
    0x0D: "FR_NO_FILESYSTEM",
    0x0E: "FR_MKFS_ABORTED",
    0x0F: "FR_TIMEOUT",
    0x10: "FR_LOCKED",
    0x11: "FR_NOT_ENOUGH_CORE",
    0x12: "FR_TOO_MANY_OPEN_FILES",
    0x13: "FR_INVALID_PARAMETER",
}
BAUDRATES = [
    "4000000",
    "3500000",
    "3000000",
    "2500000",
    "2000000",
    "1500000",
    "1152000",
    "1000000",
    "921600",
    "576000",
    "500000",
    "460800",
    "230400",
    "115200",
    "74800",
    "57600",
    "38400",
    "19200",
    "9600",
    "4800",
    "2400",
    "1200",
]

APP_STYLE = """
QWidget {
    background: #f4f6f8;
    color: #1f2933;
    font-family: "Microsoft YaHei", "Segoe UI", Arial;
    font-size: 13px;
}
QGroupBox {
    background: #ffffff;
    border: 1px solid #d7dde5;
    border-radius: 8px;
    margin-top: 14px;
    padding: 12px;
    font-weight: 600;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 14px;
    padding: 0 6px;
    color: #243b53;
}
QLabel {
    background: transparent;
}
QComboBox, QSpinBox, QLineEdit {
    background: #ffffff;
    border: 1px solid #c6d0dc;
    border-radius: 5px;
    min-height: 30px;
    padding: 2px 8px;
}
QComboBox:focus, QSpinBox:focus, QLineEdit:focus {
    border-color: #1f7ae0;
}
QPushButton {
    background: #eef4fb;
    border: 1px solid #b8c7d8;
    border-radius: 5px;
    padding: 6px 12px;
    color: #183b56;
    font-weight: 600;
}
QPushButton:hover {
    background: #dcecff;
    border-color: #8ab8ed;
}
QPushButton:pressed {
    background: #c8ddf6;
}
QPushButton:disabled {
    background: #edf0f3;
    color: #9aa5b1;
    border-color: #d8dee6;
}
QPlainTextEdit {
    background: #101820;
    color: #dce9f5;
    border: 1px solid #263746;
    border-radius: 6px;
    padding: 8px;
    font-family: Consolas, "Courier New";
    font-size: 12px;
}
QTableWidget {
    background: #ffffff;
    alternate-background-color: #f7f9fb;
    gridline-color: #dbe3ec;
    border: 1px solid #d7dde5;
    border-radius: 6px;
}
QHeaderView::section {
    background: #edf2f7;
    color: #243b53;
    border: 0;
    border-right: 1px solid #d7dde5;
    border-bottom: 1px solid #d7dde5;
    padding: 6px;
    font-weight: 600;
}
QFrame#metricCard {
    background: #f8fafc;
    border: 1px solid #dbe3ec;
    border-radius: 7px;
}
QLabel#metricTitle {
    color: #64748b;
    font-size: 12px;
}
QLabel#metricValue {
    color: #0f172a;
    font-size: 18px;
    font-weight: 700;
}
"""


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc


def append_crc(frame: bytes) -> bytes:
    crc = crc16_modbus(frame)
    return frame + bytes((crc & 0xFF, crc >> 8))


def read_holding(slave: int, start: int, count: int) -> bytes:
    frame = bytes((slave, 0x03, start >> 8, start & 0xFF, count >> 8, count & 0xFF))
    return append_crc(frame)


def write_one_register_with_fc16(slave: int, reg: int, value: int) -> bytes:
    frame = bytes(
        (
            slave,
            0x10,
            reg >> 8,
            reg & 0xFF,
            0x00,
            0x01,
            0x02,
            value >> 8,
            value & 0xFF,
        )
    )
    return append_crc(frame)


def parse_read_u16_response(response: bytes) -> int:
    if len(response) != 7 or response[1] != 0x03 or response[2] != 2:
        raise ValueError(f"bad u16 response: {response.hex(' ').upper()}")
    check_crc(response)
    return (response[3] << 8) | response[4]


def parse_read_words_response(response: bytes, count: int) -> list[int]:
    expected_len = 5 + count * 2
    if len(response) != expected_len or response[1] != 0x03 or response[2] != count * 2:
        raise ValueError(f"bad words response: {response.hex(' ').upper()}")
    check_crc(response)
    words = []
    for index in range(count):
        offset = 3 + index * 2
        words.append((response[offset] << 8) | response[offset + 1])
    return words


def words_to_u32(words: list[int]) -> int:
    return words[0] | (words[1] << 16)


def words_to_u64(words: list[int]) -> int:
    return words[0] | (words[1] << 16) | (words[2] << 32) | (words[3] << 48)


def check_crc(frame: bytes) -> None:
    if len(frame) < 4:
        raise ValueError("response too short")
    stored = frame[-2] | (frame[-1] << 8)
    calc = crc16_modbus(frame[:-2])
    if stored != calc:
        raise ValueError(f"crc error: got 0x{stored:04X}, expected 0x{calc:04X}")


def parse_bin_rows(path: Path) -> tuple[list[dict], int, int]:
    data = path.read_bytes()
    block_count = len(data) // BLOCK_SIZE
    trailing = len(data) % BLOCK_SIZE
    rows = []
    bad_blocks = 0

    for block_index in range(block_count):
        block = data[block_index * BLOCK_SIZE : (block_index + 1) * BLOCK_SIZE]
        try:
            rows.extend(parse_one_block(block, block_index))
        except ValueError:
            bad_blocks += 1

    return rows, bad_blocks, trailing


class SerialPanel(QtWidgets.QGroupBox):
    def __init__(self):
        super().__init__("SD 记录控制")
        self.serial_port = None

        self.port_combo = QtWidgets.QComboBox()
        self.baud_combo = QtWidgets.QComboBox()
        self.baud_combo.addItems(BAUDRATES)
        self.baud_combo.setCurrentText("3000000")
        self.slave_spin = QtWidgets.QSpinBox()
        self.slave_spin.setRange(1, 247)
        self.slave_spin.setValue(1)

        self.refresh_btn = QtWidgets.QPushButton("刷新串口")
        self.open_btn = QtWidgets.QPushButton("打开串口")
        self.query_addr_btn = QtWidgets.QPushButton("查询地址")
        self.start_btn = QtWidgets.QPushButton("开始记录")
        self.stop_btn = QtWidgets.QPushButton("停止记录")
        self.reset_sd_btn = QtWidgets.QPushButton("SD驱动复位")
        self.status_btn = QtWidgets.QPushButton("查询状态")

        self.state_label = QtWidgets.QLabel("状态: -")
        self.size_label = QtWidgets.QLabel("长度: -")
        self.count_label = QtWidgets.QLabel("块数: -")
        self.error_label = QtWidgets.QLabel("错误: -")
        self.disk_label = QtWidgets.QLabel("底层: -")
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(200)
        self.log.setPlaceholderText("通信日志")

        self.port_combo.setMinimumWidth(130)
        self.baud_combo.setMinimumWidth(120)
        self.slave_spin.setMinimumWidth(72)
        for button in (
            self.refresh_btn,
            self.open_btn,
            self.query_addr_btn,
            self.start_btn,
            self.stop_btn,
            self.reset_sd_btn,
            self.status_btn,
        ):
            button.setMinimumHeight(34)

        top = QtWidgets.QGridLayout()
        top.setHorizontalSpacing(12)
        top.setVerticalSpacing(8)
        top.addWidget(QtWidgets.QLabel("串口"), 0, 0)
        top.addWidget(self.port_combo, 0, 1)
        top.addWidget(QtWidgets.QLabel("波特率"), 0, 2)
        top.addWidget(self.baud_combo, 0, 3)
        top.addWidget(QtWidgets.QLabel("从机地址"), 0, 4)
        top.addWidget(self.slave_spin, 0, 5)
        top.addWidget(self.refresh_btn, 0, 6)
        top.addWidget(self.open_btn, 0, 7)
        top.addWidget(self.query_addr_btn, 0, 8)
        top.addWidget(self.start_btn, 1, 0, 1, 3)
        top.addWidget(self.stop_btn, 1, 3, 1, 3)
        top.addWidget(self.reset_sd_btn, 1, 6, 1, 2)
        top.addWidget(self.status_btn, 1, 8, 1, 1)

        status = QtWidgets.QGridLayout()
        status.setHorizontalSpacing(10)
        status.addWidget(self.make_metric("记录状态", self.state_label), 0, 0)
        status.addWidget(self.make_metric("文件长度", self.size_label), 0, 1)
        status.addWidget(self.make_metric("写入块数", self.count_label), 0, 2)
        status.addWidget(self.make_metric("错误码", self.error_label), 0, 3)
        status.addWidget(self.make_metric("底层诊断", self.disk_label), 0, 4)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addLayout(top)
        layout.addLayout(status)
        layout.addWidget(self.log)

        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.open_btn.clicked.connect(self.toggle_open)
        self.query_addr_btn.clicked.connect(self.query_slave_address)
        self.start_btn.clicked.connect(self.start_record)
        self.stop_btn.clicked.connect(self.stop_record)
        self.reset_sd_btn.clicked.connect(self.reset_sd_driver)
        self.status_btn.clicked.connect(self.query_status)
        self.refresh_ports()
        self.update_buttons()

    def make_metric(self, title: str, value_label: QtWidgets.QLabel) -> QtWidgets.QFrame:
        frame = QtWidgets.QFrame()
        frame.setObjectName("metricCard")
        title_label = QtWidgets.QLabel(title)
        title_label.setObjectName("metricTitle")
        value_label.setObjectName("metricValue")
        layout = QtWidgets.QVBoxLayout(frame)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.addWidget(title_label)
        layout.addWidget(value_label)
        return frame

    def append_log(self, text: str) -> None:
        self.log.appendPlainText(text)

    def refresh_ports(self) -> None:
        current = self.port_combo.currentText()
        self.port_combo.clear()
        if list_ports is not None:
            for port in list_ports.comports():
                self.port_combo.addItem(port.device)
        if current:
            index = self.port_combo.findText(current)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)

    def update_buttons(self) -> None:
        opened = self.serial_port is not None and self.serial_port.is_open
        self.open_btn.setText("关闭串口" if opened else "打开串口")
        self.query_addr_btn.setEnabled(opened)
        self.start_btn.setEnabled(opened)
        self.stop_btn.setEnabled(opened)
        self.reset_sd_btn.setEnabled(opened)
        self.status_btn.setEnabled(opened)

    def toggle_open(self) -> None:
        if serial is None:
            QtWidgets.QMessageBox.warning(self, "Missing dependency", "Install pyserial first: pip install pyserial")
            return

        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()
            self.serial_port = None
            self.append_log("串口已关闭")
            self.update_buttons()
            return

        port = self.port_combo.currentText()
        if not port:
            QtWidgets.QMessageBox.warning(self, "没有串口", "请先选择串口。")
            return

        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=int(self.baud_combo.currentText()),
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=0.4,
                write_timeout=0.4,
            )
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "打开失败", str(exc))
            self.serial_port = None
        else:
            self.append_log(f"串口已打开: {port}")
        self.update_buttons()

    def transaction(self, request: bytes, expected_len: int) -> bytes:
        if self.serial_port is None or not self.serial_port.is_open:
            raise RuntimeError("serial is not open")
        self.serial_port.reset_input_buffer()
        self.serial_port.write(request)
        self.serial_port.flush()
        response = self.serial_port.read(expected_len)
        self.append_log(f"TX: {request.hex(' ').upper()}")
        self.append_log(f"RX: {response.hex(' ').upper()}")
        if len(response) != expected_len:
            raise TimeoutError(f"expected {expected_len} bytes, got {len(response)}")
        check_crc(response)
        return response

    def start_record(self) -> None:
        self.write_command(CMD_LOG_START)

    def stop_record(self) -> None:
        self.write_command(CMD_LOG_STOP)

    def reset_sd_driver(self) -> None:
        self.write_command(CMD_SD_RESET)

    def write_command(self, command: int) -> None:
        request = write_one_register_with_fc16(self.slave_spin.value(), REG_CMD, command)
        try:
            self.append_log(f"CMD: 0x{command:04X}")
            self.transaction(request, 8)
            self.query_status()
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "命令失败", str(exc))

    def query_slave_address(self) -> None:
        request = read_holding(0x00, 0x0000, 1)
        try:
            response = self.transaction(request, 7)
            address = response[0]
            value = parse_read_u16_response(response)
            if address != value:
                self.append_log(f"地址查询提示: 回包地址={address}, 寄存器值={value}")
            if 1 <= address <= 247:
                self.slave_spin.setValue(address)
                self.append_log(f"当前从机地址: {address}")
            else:
                raise ValueError(f"invalid slave address: {address}")
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "地址查询失败", str(exc))

    def query_u16(self, reg: int) -> int:
        request = read_holding(self.slave_spin.value(), reg, 1)
        response = self.transaction(request, 7)
        return parse_read_u16_response(response)

    def query_words(self, reg: int, count: int) -> list[int]:
        request = read_holding(self.slave_spin.value(), reg, count)
        response = self.transaction(request, 5 + count * 2)
        return parse_read_words_response(response, count)

    def query_status(self) -> None:
        try:
            status = self.query_u16(REG_SD_LOG_STATUS)
            error = self.query_u16(REG_SD_ERROR_CODE)
            size = words_to_u64(self.query_words(REG_SD_CURRENT_FILE_SIZE, 4))
            count = words_to_u32(self.query_words(REG_SD_CURRENT_WRITE_CNT, 2))
            disk_result = self.query_u16(REG_SD_DISK_LAST_RESULT)
            hal_status = self.query_u16(REG_SD_DISK_HAL_STATUS)
            hal_error = words_to_u32(self.query_words(REG_SD_DISK_HAL_ERROR, 2))
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "查询失败", str(exc))
            return

        state = "REC" if status == SD_LOG_STATUS_RECORDING else "IDLE" if status == SD_LOG_STATUS_IDLE else f"0x{status:04X}"
        self.state_label.setText(state)
        self.error_label.setText(f"0x{error:04X} {FRESULT_NAMES.get(error, '')}".strip())
        self.size_label.setText(f"{size / 1024:.1f} KB")
        self.count_label.setText(str(count))
        self.disk_label.setText(f"D:{disk_result} H:{hal_status} E:0x{hal_error:08X}")


class LogTablePanel(QtWidgets.QGroupBox):
    def __init__(self):
        super().__init__("BIN 表格查看")
        self.rows = []

        self.path_edit = QtWidgets.QLineEdit()
        self.browse_btn = QtWidgets.QPushButton("打开 BIN")
        self.type_combo = QtWidgets.QComboBox()
        self.type_combo.addItem("imu", TYPE_IMU)
        self.type_combo.addItem("joint", TYPE_JOINT)
        self.type_combo.addItem("tactile", TYPE_TACTILE)
        self.summary_label = QtWidgets.QLabel("未加载文件")
        self.table = QtWidgets.QTableWidget()
        self.table.setAlternatingRowColors(True)
        self.table.setSortingEnabled(True)

        top = QtWidgets.QHBoxLayout()
        top.addWidget(self.path_edit, 1)
        top.addWidget(self.browse_btn)
        top.addWidget(QtWidgets.QLabel("视图"))
        top.addWidget(self.type_combo)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addLayout(top)
        layout.addWidget(self.summary_label)
        layout.addWidget(self.table, 1)

        self.browse_btn.clicked.connect(self.open_file)
        self.type_combo.currentIndexChanged.connect(self.populate_table)

    def open_file(self) -> None:
        path_text, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open LOG BIN", "", "BIN files (*.bin *.BIN);;All files (*)")
        if not path_text:
            return
        path = Path(path_text)
        try:
            self.rows, bad_blocks, trailing = parse_bin_rows(path)
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "解析失败", str(exc))
            return

        self.path_edit.setText(str(path))
        self.summary_label.setText(
            f"帧数: {len(self.rows)} | 坏块: {bad_blocks} | 尾部字节: {trailing}"
        )
        self.populate_table()

    def populate_table(self) -> None:
        data_type = self.type_combo.currentData()
        rows = [row for row in self.rows if row["data_type"] == data_type]
        value_count = {
            TYPE_IMU: IMU_FLOAT_COUNT,
            TYPE_JOINT: JOINT_FLOAT_COUNT,
            TYPE_TACTILE: TACTILE_U16_COUNT,
        }[data_type]
        value_prefix = type_name(data_type)
        headers = ["block", "frame", "hand", "data_id", "timestamp_us", "crc_ok"]
        headers.extend(f"{value_prefix}_{index}" for index in range(value_count))

        self.table.clear()
        self.table.setColumnCount(len(headers))
        self.table.setHorizontalHeaderLabels(headers)
        self.table.setRowCount(len(rows))

        for row_index, row in enumerate(rows):
            values = [
                row["block"],
                row["frame"],
                row["hand"],
                f"0x{row['data_id']:02X}",
                row["timestamp_us"],
                int(row["crc_ok"]),
                *row["values"],
            ]
            for col_index, value in enumerate(values):
                if isinstance(value, float):
                    text = f"{value:.6g}"
                else:
                    text = str(value)
                self.table.setItem(row_index, col_index, QtWidgets.QTableWidgetItem(text))

        self.table.resizeColumnsToContents()


class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("STM32 SD 记录测试工具")
        self.resize(1360, 820)
        self.setStyleSheet(APP_STYLE)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical if hasattr(QtCore.Qt, "Orientation") else QtCore.Qt.Vertical)
        splitter.addWidget(SerialPanel())
        splitter.addWidget(LogTablePanel())
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(splitter)


def main() -> None:
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec() if hasattr(app, "exec") else app.exec_())


if __name__ == "__main__":
    main()
