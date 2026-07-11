# -*- coding: utf-8 -*-
"""
HandPC_autoscale_panel.py
--------------------------------
演示：让“手指控制”这块区域的控件尺寸与字体大小，随该区域尺寸自动自适应。
你可以直接运行本文件查看效果；也可以把 `FingerControlPanel` 类拷贝到你的项目，
用它替换你原来“手指控制”区域的容器小部件。

要点：
- 通过重载 resizeEvent，在区域尺寸变化时计算缩放因子 scale；
- 递归设置字体大小（pointSizeF）；
- 动态生成 QSlider 的样式（groove 粗细、handle 大小等）以匹配缩放因子；
- 合理设置 SizePolicy + 伸缩因子，保证可用空间优先给到这块区域；
- 极端小尺寸时仍可配合 QScrollArea 以避免拥挤（本示例默认不包滚动，你可在你的主界面外面包一层 QScrollArea）。
"""

import sys
from PySide6.QtCore import Qt, QSettings, QByteArray
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QApplication, QWidget, QMainWindow, QLabel, QSlider, QHBoxLayout, QVBoxLayout,
    QGridLayout, QSizePolicy, QGroupBox, QSpacerItem, QMessageBox
)

# ======================= 可调参数（按你的设计基准） =======================
BASE_WIDTH  = 720      # 该区域“设计时”的参考宽度
BASE_HEIGHT = 840      # 该区域“设计时”的参考高度
BASE_PT     = 12.0     # 参考字体大小（pt）
MIN_PT      = 9.0      # 字体下限（pt）
MAX_PT      = 22.0     # 字体上限（pt）

BASE_GROOVE = 6        # 参考滑槽粗细（px）
BASE_HANDLE = 16       # 参考手柄尺寸（px）
BASE_SLIDER_H = 28     # 参考滑动条整体高度（px）

CLAMP_MIN_SCALE = 0.6  # 缩放因子最小值（避免太小不可读）
CLAMP_MAX_SCALE = 2.0  # 缩放因子最大值（避免过大）


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def set_font_recursive(widget, point_size):
    """递归设置小部件及其孩子的字体 pointSizeF。"""
    f = widget.font()
    f.setPointSizeF(point_size)
    widget.setFont(f)
    for ch in widget.findChildren(QWidget):
        f2 = ch.font()
        f2.setPointSizeF(point_size)
        ch.setFont(f2)


def slider_stylesheet(groove_px: int, handle_px: int):
    """根据像素尺寸生成水平 QSlider 的样式表。"""
    # 注意：未指定颜色，让其跟随系统/样式；如需自定义颜色可自行扩展。
    return f"""
    QSlider::groove:horizontal {{
        height: {groove_px}px;
        border-radius: {groove_px // 2}px;
        margin: 0px;
    }}
    QSlider::handle:horizontal {{
        width: {handle_px}px;
        height: {handle_px}px;
        border-radius: {handle_px // 2}px;
        margin: -{max(0, (handle_px - groove_px)//2)}px 0px;
    }}
    """


class FingerRow(QWidget):
    """一行：标签 + 3 个水平滑动条（示意拇/食/中/无/小每指的 3 个关节）"""
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.title = title
        self.label = QLabel(title)
        self.s1 = QSlider(Qt.Horizontal)
        self.s2 = QSlider(Qt.Horizontal)
        self.s3 = QSlider(Qt.Horizontal)

        for s in (self.s1, self.s2, self.s3):
            s.setRange(-90, 90)
            s.setValue(0)
            s.setSingleStep(1)
            s.setPageStep(5)
            s.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        # 行布局
        row = QHBoxLayout(self)
        row.setContentsMargins(8, 4, 8, 4)
        row.setSpacing(8)

        self.label.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        row.addWidget(self.label, 0)   # 标签占少量宽度
        row.addWidget(self.s1, 1)
        row.addWidget(self.s2, 1)
        row.addWidget(self.s3, 1)

        # 初始样式（会在外层统一缩放时覆盖）
        self.update_slider_visuals(scale=1.0)

    def update_slider_visuals(self, scale: float):
        """根据缩放因子调整滑条粗细/手柄大小/整体高度。"""
        groove = max(2, int(BASE_GROOVE * scale))
        handle = max(10, int(BASE_HANDLE * scale))
        height = max(18, int(BASE_SLIDER_H * scale))

        ss = slider_stylesheet(groove, handle)
        for s in (self.s1, self.s2, self.s3):
            s.setStyleSheet(ss)
            s.setFixedHeight(height)


class FingerControlPanel(QWidget):
    """
    右侧“手指控制”区域容器：
    - 根据自身尺寸计算缩放因子 scale
    - 递归设置字体
    - 调整所有 FingerRow 的滑动条视觉参数
    """
    def __init__(self, parent=None):
        super().__init__(parent)

        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)

        # 顶层布局（可替换为你的原有布局结构）
        outer = QVBoxLayout(self)
        outer.setContentsMargins(10, 10, 10, 10)
        outer.setSpacing(10)

        # 标题
        self.title = QLabel("✋ 手指控制")
        self.title.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.title.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)

        # 分组区（你可换成原来的 QGroupBox/其他容器）
        self.group = QGroupBox("关节调节")
        g_layout = QVBoxLayout(self.group)
        g_layout.setContentsMargins(10, 10, 10, 10)
        g_layout.setSpacing(6)

        # 5 根手指（示例：每指 3 关节）
        self.rows = [
            FingerRow("拇指"),
            FingerRow("食指"),
            FingerRow("中指"),
            FingerRow("无名指"),
            FingerRow("小指"),
        ]
        for r in self.rows:
            g_layout.addWidget(r)

        # 占位弹簧，撑满剩余空间（让行之间不至于过挤）
        g_layout.addItem(QSpacerItem(0, 0, QSizePolicy.Minimum, QSizePolicy.Expanding))

        outer.addWidget(self.title, 0)
        outer.addWidget(self.group, 1)  # 分组占据主要高度

        # 初始应用一次缩放
        self.apply_scale()

    # === 关键：随尺寸变化进行自适应 ===
    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        self.apply_scale()

    def compute_scale(self) -> float:
        w = max(1, self.width())
        h = max(1, self.height())
        # 取宽、高两个方向的“最保守缩放”
        scale_w = w / float(BASE_WIDTH)
        scale_h = h / float(BASE_HEIGHT)
        scale = min(scale_w, scale_h)
        scale = clamp(scale, CLAMP_MIN_SCALE, CLAMP_MAX_SCALE)
        return scale

    def apply_scale(self):
        scale = self.compute_scale()

        # 字体：基于 BASE_PT 做缩放，并做上下限夹制
        pt = clamp(BASE_PT * scale, MIN_PT, MAX_PT)
        set_font_recursive(self, pt)

        # 子控件：滑动条视觉参数（粗细/手柄/高度）
        for row in self.rows:
            row.update_slider_visuals(scale)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("HandPC — 手指控制自适应演示")
        self._init_ui()
        self._restore_window()

    def _init_ui(self):
        # 主容器
        cw = QWidget()
        self.setCentralWidget(cw)
        h = QHBoxLayout(cw)
        h.setContentsMargins(10, 10, 10, 10)
        h.setSpacing(10)

        # 左侧占位（你的 3D/状态/其他区域）
        left = QGroupBox("占位：你的 3D 或其它区域")
        left.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        left_layout = QVBoxLayout(left)
        left_label = QLabel("这里放你的 3D 视图/状态面板等")
        left_label.setAlignment(Qt.AlignCenter)
        left_layout.addWidget(left_label)

        # 右侧：我们的“手指控制”自适应区域
        self.finger_panel = FingerControlPanel()
        self.finger_panel.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)

        # 左右布局：左边更宽，右边固定最小宽度但可伸缩
        h.addWidget(left, 3)
        h.addWidget(self.finger_panel, 2)

        # 初始大小（24寸 1920×1080）
        self.resize(1920, 1080)

    # ===== 持久化窗口大小（避免 saveState/restoreState 报错） =====
    def _restore_window(self):
        s = QSettings("ZhanhaoZhou", "HandPC_AutoscaleDemo")
        geo = s.value("MainWindow/geometry", None)
        if isinstance(geo, QByteArray):
            self.restoreGeometry(geo)

    def closeEvent(self, ev):
        # 保存几何尺寸 + 最大化状态
        s = QSettings("ZhanhaoZhou", "HandPC_AutoscaleDemo")
        s.setValue("MainWindow/geometry", self.saveGeometry())
        super().closeEvent(ev)


def main():
    app = QApplication(sys.argv)
    # 高 DPI 支持（PySide6 通常默认开启；按需保留）
    # QApplication.setHighDpiScaleFactorRoundingPolicy(Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)

    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
