# HandPC.py — 集成 URDF 3D 模型并用左侧 6 个位置滑动条驱动关节
import sys
import struct
import time
import math
import re
from PySide6.QtCore import QSettings, QCoreApplication, QByteArray

from PySide6.QtWidgets import (
    QApplication, QWidget, QFrame, QHBoxLayout, QVBoxLayout, QLabel, QPushButton,
    QComboBox, QFormLayout,
    QSpinBox, QDoubleSpinBox, QTextEdit, QGroupBox, QMessageBox, QSlider, QGridLayout,
    QScrollArea, QDialog, QAbstractSpinBox, QTableWidget, QTableWidgetItem, QCheckBox,
    QSizePolicy, QFileDialog, QInputDialog, QHeaderView,
    QAbstractItemView,
    QProgressBar,
    QSplitter,
    QGraphicsDropShadowEffect,
)
from PySide6.QtCore import (
    Qt, QTimer, Signal, QEvent,
    QSize, QRectF, Property, QPropertyAnimation, QEasingCurve
)
from PySide6.QtCore import QPropertyAnimation, QEasingCurve, QRect
from PySide6.QtSerialPort import QSerialPort, QSerialPortInfo
import pyqtgraph as pg

from PySide6.QtGui import QStandardItemModel, QStandardItem, QIcon, QPixmap, QPainter, QColor, QBrush, QFont, \
    QFontMetrics
from pathlib import Path

# ---- PyInstaller 通用资源路径 ----
from pathlib import Path as _PathForRes
from Crypto.Cipher import AES, PKCS1_OAEP
from Crypto.PublicKey import RSA

from PySide6.QtWidgets import QGraphicsDropShadowEffect
from PySide6.QtGui import QColor


def resource_path(rel: str) -> str:
    """
    支持源码运行与 PyInstaller 单文件运行的资源路径获取：
    resource_path("assets/app.ico") -> 绝对路径
    """
    import sys
    if getattr(sys, 'frozen', False) and hasattr(sys, '_MEIPASS'):
        base = _PathForRes(sys._MEIPASS)
    else:
        base = _PathForRes(__file__).resolve().parent
    return str((base / rel).resolve())


# ====== 集成 urdf_test（FingerPoseViewer）======
URDF_AVAILABLE = True
# ---- 以下为内嵌版 urdf_test.py ----
# -*- coding: utf-8 -*-
"""
finger_joint_viewer_independent_rules_pose.py
PySide6 + VTK + urdfpy

特性：
- 关节独立控制（不级联）
- 规则：四指（食/中/无/小）joint1 锁定；拇指 3 关节全反向；食指 2/4 反向；中/无/小 指 4 反向
- 顶部“忽略URDF限位”默认开启
- ✅ 全局位姿接口：
  * 旋转：set_model_orientation_deg(...) / set_model_orientation_quat(...)
  * 平移：set_model_translation_mm(...) / set_model_translation_m(...)
  * 组合：set_model_pose_deg_mm(...)
  * 启动默认：INIT_WORLD_RPY_DEG / INIT_WORLD_XYZ_MM
- ✅ UI 控件：
  * “模型朝向（R/P/Y）”三条滑动条（度）
  * “模型平移（X/Y/Z）”三条滑动条（毫米）

说明：
- 旋转使用 R = Rz * Ry * Rx（Yaw-Pitch-Roll 右乘）约定。
- 平移滑条单位是 mm（内部换算成 m 应用于场景）。
"""

import os, sys, re, math, traceback
import numpy as np
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional, Set

# 某些库旧写法兼容
import numpy as _np

if not hasattr(_np, "float"): _np.float = float
if not hasattr(_np, "int"): _np.int = int
if not hasattr(_np, "bool"): _np.bool = bool

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication, QWidget, QFrame, QHBoxLayout, QVBoxLayout, QLabel, QPushButton,
    QComboBox, QFormLayout,
    QSpinBox, QDoubleSpinBox, QTextEdit, QGroupBox, QMessageBox, QSlider, QGridLayout,
    QScrollArea, QDialog, QAbstractSpinBox, QTableWidget, QTableWidgetItem, QCheckBox,
    QSizePolicy, QFileDialog, QInputDialog, QHeaderView,
    QAbstractItemView,
    QProgressBar
)

# VTK
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from vtkmodules.vtkRenderingCore import (
    vtkRenderer, vtkRenderWindow, vtkActor, vtkPolyDataMapper, vtkLight, vtkProperty
)
from vtkmodules.vtkCommonMath import vtkMatrix4x4
from vtkmodules.vtkInteractionStyle import vtkInteractorStyleTrackballCamera
from vtkmodules.vtkRenderingAnnotation import vtkAxesActor
from vtkmodules.vtkInteractionWidgets import vtkOrientationMarkerWidget
from vtkmodules.vtkIOGeometry import vtkSTLReader, vtkOBJReader
from vtkmodules.vtkIOPLY import vtkPLYReader
from vtkmodules.vtkFiltersCore import vtkPolyDataNormals
from vtkmodules.vtkFiltersSources import vtkCubeSource
import vtkmodules.vtkRenderingOpenGL2  # noqa
import vtkmodules.vtkInteractionStyle  # noqa

# urdfpy
URDF_OK = False
try:
    from urdfpy import URDF

    URDF_OK = True
except Exception as e:
    print("[ERROR] urdfpy 导入失败：", f"{e.__class__.__name__}: {e}")
    traceback.print_exc()

# ===== 按你的工程修改 =====
URDF_PATH = resource_path("youshou_urdf/urdf/youshou_urdf.urdf")
PACKAGE_ROOTS = {"youshou_urdf": resource_path("youshou_urdf")}

# ===== 启动默认全局位姿 =====
# 启动默认全局位姿
INIT_WORLD_RPY_DEG = (-91.0, -33.0, 3.0)  # Roll, Pitch, Yaw（度）
INIT_WORLD_XYZ_MM = (0.0, -50.0, 0.0)  # X, Y, Z（毫米）

# —— 自动识别“手指”的关键词（不区分大小写）——
HINT_INDEX = ["index"]
HINT_MIDDLE = ["middle"]
HINT_RING = ["ring"]
HINT_LITTLE = ["little", "pinky"]
HINT_THUMB = ["thumb"]
FINGER_NAME_HINTS = ["finger", "digit"] + HINT_INDEX + HINT_MIDDLE + HINT_RING + HINT_LITTLE + HINT_THUMB

# 若自动识别失败，可在此手动填每根手指“根关节名”（优先使用）
MANUAL_ROOTS: List[str] = [
    # "index_mcp", "middle_mcp", "ring_mcp", "little_mcp", "thumb_cmc"
]

# UI 滑动条角度范围（度）
UI_DEG_MIN, UI_DEG_MAX = -90, 90

# —— 规则：锁定/反向（索引从 0 开始；0=joint1, 1=joint2, 3=joint4）——
LOCK_JOINTS: Dict[str, Set[int]] = {
    "index": {},
    "middle": {},
    "ring": {},
    "little": {},
    "thumb": set(),
    "other": set(),
}
INVERT_JOINTS: Dict[str, Set[int]] = {
    "thumb": {0, 1, 2},  # 拇指 3 关节全反向
    "index": {1, 3},  # 食指 2/4 反向
    "middle": {3},  # 中指 4 反向
    "ring": {3},  # 无名指 4 反向
    "little": {3},  # 小指 4 反向
    "other": set(),
}

# —— 限位“度写的阈值”（>~2π 认为是度）——
_DEG_LIMIT_THRESHOLD = 2 * math.pi + 1e-3


def resolve_package_uris(urdf_path: str, package_roots: Dict[str, str]) -> str:
    with open(urdf_path, "r", encoding="utf-8") as f:
        xml = f.read()

    def _repl(m):
        full = m.group(1);
        pkg = m.group(2);
        rel = m.group(3) or ""
        root = package_roots.get(pkg)
        if not root:
            print(f"[WARN] 未提供包根：{pkg}，保留 package://{full}")
            return f'filename="package://{full}"'
        abs_path = os.path.abspath(os.path.join(root, rel)).replace("\\", "/")
        return f'filename="{abs_path}"'

    pat = r'filename\s*=\s*"(?:package://)(([^"/]+)/(.+?))"'
    xml2 = re.sub(pat, _repl, xml, flags=re.IGNORECASE)
    out_path = os.path.join(os.path.dirname(urdf_path),
                            os.path.splitext(os.path.basename(urdf_path))[0] + "_resolved.urdf")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(xml2)
    return out_path


@dataclass
class VisualItem:
    actor: vtkActor
    link_name: str
    T_Lv: np.ndarray


class FingerPoseViewer(QWidget):
    BG = (0.10, 0.12, 0.16)
    DEFAULT_RGB = (0.75, 0.85, 1.0)
    FEATURE_ANGLE_DEG = 35.0

    def __init__(self):
        super().__init__()

        # 顶部工具栏
        top = QHBoxLayout();
        top.setContentsMargins(0, 0, 0, 0)
        self.lab_path = QLabel(f"URDF: {URDF_PATH}")
        self.lab_path.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.cb_ignore_limits = QCheckBox("忽略URDF限位");
        self.cb_ignore_limits.setChecked(True)
        self.btn_fit = QPushButton("适配显示");
        self.btn_reset_view = QPushButton("重置视图")
        top.addWidget(self.lab_path, 1)
        top.addWidget(self.cb_ignore_limits)
        top.addWidget(self.btn_fit);
        top.addWidget(self.btn_reset_view)

        # 中间：QSplitter 垂直分隔
        splitter = QSplitter(Qt.Vertical);
        splitter.setChildrenCollapsible(False)

        # VTK 视图
        self.vtk = QVTKRenderWindowInteractor();
        self.vtk.setFocusPolicy(Qt.NoFocus)
        self.vtk.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.ren = vtkRenderer();
        self.ren.SetBackground(*self.BG);
        self.ren.TwoSidedLightingOn()
        rw: vtkRenderWindow = self.vtk.GetRenderWindow();
        rw.AddRenderer(self.ren)
        self.iren = rw.GetInteractor();
        self.iren.SetInteractorStyle(vtkInteractorStyleTrackballCamera())
        axes = vtkAxesActor();
        self.om = vtkOrientationMarkerWidget()
        self.om.SetOrientationMarker(axes);
        self.om.SetInteractor(self.iren)
        self.om.SetViewport(0.80, 0.02, 0.98, 0.20);
        self.om.SetEnabled(1);
        self.om.InteractiveOff()

        vtk_frame = QFrame();
        vbox = QVBoxLayout(vtk_frame);
        vbox.setContentsMargins(0, 0, 0, 0);
        vbox.addWidget(self.vtk)
        splitter.addWidget(vtk_frame)

        # 控制面板（滚动）
        self.ctrl_area = QScrollArea();
        self.ctrl_area.setWidgetResizable(True)
        self.ctrl_container = QWidget()
        self.ctrl_vbox = QVBoxLayout(self.ctrl_container);
        self.ctrl_vbox.setContentsMargins(8, 8, 8, 8);
        self.ctrl_vbox.setSpacing(8)
        self.ctrl_area.setWidget(self.ctrl_container)
        self.ctrl_area.setMinimumHeight(360)
        self.ctrl_area.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        splitter.addWidget(self.ctrl_area)
        splitter.setSizes([900, 380])

        # 灯光
        l1 = vtkLight();
        l1.SetLightTypeToHeadlight();
        l1.SetIntensity(0.9);
        self.ren.AddLight(l1)
        l2 = vtkLight();
        l2.SetLightTypeToSceneLight();
        l2.SetPosition(300, 300, 300);
        l2.SetFocalPoint(0, 0, 0);
        l2.SetIntensity(0.8);
        self.ren.AddLight(l2)
        l3 = vtkLight();
        l3.SetLightTypeToSceneLight();
        l3.SetPosition(-300, -200, -250);
        l3.SetFocalPoint(0, 0, 0);
        l3.SetIntensity(0.35);
        self.ren.AddLight(l3)

        # 外层布局
        root = QVBoxLayout(self);
        root.setContentsMargins(10, 10, 10, 10);
        root.setSpacing(6)
        root.addLayout(top)
        self.lab_status = QLabel("状态: 就绪");
        root.addWidget(self.lab_status)
        root.addWidget(splitter, 1)

        # 状态量
        self.robot: Optional[URDF] = None
        self.visuals: List[VisualItem] = []
        self.link_of_joint: Dict[str, str] = {}
        self.parent_link_of_joint: Dict[str, str] = {}
        self.children_joints: Dict[str, List[str]] = {}
        self.joint_limits_raw: Dict[str, Tuple[Optional[float], Optional[float]]] = {}
        self.joint_limits: Dict[str, Tuple[Optional[float], Optional[float]]] = {}
        self.joint_types: Dict[str, str] = {}

        self.chains: List[List[str]] = []  # 每根手指的 joint 名称序列（近端->远端）
        self.chain_kind: List[str] = []  # index/middle/ring/little/thumb/other
        self.chain_sliders: List[List[QSlider]] = []
        self.chain_val_labels: List[List[QLabel]] = []
        self.joint2widgets: Dict[str, Tuple[QSlider, QLabel]] = {}
        self.joint2chainpos: Dict[str, Tuple[int, int]] = {}

        self.current_cfg: Dict[str, float] = {}

        # ===== 全局位姿（旋转+平移） =====
        self.world_rpy_deg = list(INIT_WORLD_RPY_DEG)  # [roll, pitch, yaw]（度）
        self.world_xyz_mm = list(INIT_WORLD_XYZ_MM)  # [x, y, z]（毫米）
        self.T_world = np.eye(4)  # 4x4 齐次矩阵

        self.cam_init = None

        # 绑定
        self.btn_fit.clicked.connect(self.fit_camera)
        self.btn_reset_view.clicked.connect(self.reset_camera)
        self.cb_ignore_limits.toggled.connect(self._on_ignore_limits_toggled)

        self.iren.Initialize();
        rw.Render()
        QTimer.singleShot(0, self._load_and_build)

    # ===== 加载/构建 =====
    def _load_and_build(self):
        if not URDF_OK:
            self._err("urdfpy 导入失败");
            self._placeholder();
            return
        if not os.path.exists(URDF_PATH):
            self._err(f"URDF 不存在：{URDF_PATH}");
            self._placeholder();
            return
        try:
            resolved = resolve_package_uris(URDF_PATH, PACKAGE_ROOTS)
            self.robot = URDF.load(resolved)
        except Exception as e:
            self._err(f"解析 URDF 失败：{e}");
            traceback.print_exc();
            self._placeholder();
            return

        self._build_joint_maps()
        self._build_finger_chains()
        # 初始化全局位姿矩阵
        self._refresh_world_matrix()
        self._build_visuals_and_show()
        self._build_controls()

        self.fit_camera();
        self.cam_init = self._snapshot_cam()
        self._ok("加载完成（不级联；四指joint1锁定；拇指&若干关节反向；支持全局旋转与平移）。")
        self.vtk.GetRenderWindow().Render()

    def _build_joint_maps(self):
        self.link_of_joint.clear();
        self.parent_link_of_joint.clear()
        self.children_joints.clear();
        self.joint_limits.clear();
        self.joint_types.clear()
        self.joint_limits_raw.clear()

        for j in self.robot.joints:
            self.joint_types[j.name] = j.joint_type
            self.link_of_joint[j.name] = j.child
            self.parent_link_of_joint[j.name] = j.parent
            low, up = None, None
            if j.limit is not None and hasattr(j.limit, "lower") and hasattr(j.limit, "upper"):
                if j.joint_type in ("revolute", "prismatic"):
                    low = float(j.limit.lower) if j.limit.lower is not None else None
                    up = float(j.limit.upper) if j.limit.upper is not None else None
            self.joint_limits_raw[j.name] = (low, up)

        for jn, (lo, up) in self.joint_limits_raw.items():
            lo2, up2 = lo, up
            if lo2 is not None and abs(lo2) > _DEG_LIMIT_THRESHOLD: lo2 = math.radians(lo2)
            if up2 is not None and abs(up2) > _DEG_LIMIT_THRESHOLD: up2 = math.radians(up2)
            self.joint_limits[jn] = (lo2, up2)

        for ja in self.robot.joints:
            self.children_joints.setdefault(ja.name, [])
        for ja in self.robot.joints:
            for jb in self.robot.joints:
                if ja is jb: continue
                if self.parent_link_of_joint[jb.name] == self.link_of_joint[ja.name]:
                    self.children_joints[ja.name].append(jb.name)

    def _classify_by_root(self, root_name: str) -> str:
        low = root_name.lower()
        if any(k in low for k in HINT_THUMB):  return "thumb"
        if any(k in low for k in HINT_INDEX):  return "index"
        if any(k in low for k in HINT_MIDDLE): return "middle"
        if any(k in low for k in HINT_RING):   return "ring"
        if any(k in low for k in HINT_LITTLE): return "little"
        return "other"

    def _is_finger_like(self, jn: str) -> bool:
        low = jn.lower()
        return any(k in low for k in FINGER_NAME_HINTS)

    def _is_rotary(self, jn: str) -> bool:
        return self.joint_types.get(jn, "") in ("revolute", "continuous")

    def _build_finger_chains(self):
        all_joint_names = [j.name for j in self.robot.joints]
        roots = [r for r in MANUAL_ROOTS if r in all_joint_names]
        if not roots:
            cand = [jn for jn in all_joint_names if self._is_finger_like(jn) and self._is_rotary(jn)]
            pred = {jn: 0 for jn in cand}
            for a in cand:
                pa = self.parent_link_of_joint.get(a)
                if pa is None: continue
                for b in cand:
                    if a == b: continue
                    if self.link_of_joint.get(b) == pa:
                        pred[a] += 1
            roots = [jn for jn, c in pred.items() if c == 0]

        chains: List[List[str]] = []
        kinds: List[str] = []
        for r in roots:
            if not self._is_rotary(r): continue
            chain = [r];
            cur = r;
            seen = {r}
            while True:
                ch = [c for c in self.children_joints.get(cur, []) if self._is_finger_like(c) and self._is_rotary(c)]
                if len(ch) != 1: break
                n = ch[0]
                if n in seen: break
                chain.append(n);
                seen.add(n);
                cur = n
            if chain:
                chains.append(chain);
                kinds.append(self._classify_by_root(r))

        self.chains = chains
        self.chain_kind = kinds

        if not chains:
            self._warn("未自动识别到手指链；可在 MANUAL_ROOTS 填写根关节名。")
        else:
            for i, ch in enumerate(chains):
                print(f"[CHAIN {i}][{kinds[i]}] " + " -> ".join(ch))

    def reset_pose_to_default(self):
        """将全局位姿（旋转+平移）恢复到默认 INIT_* 数值。"""
        self.set_model_pose_deg_mm(
            INIT_WORLD_RPY_DEG[0], INIT_WORLD_RPY_DEG[1], INIT_WORLD_RPY_DEG[2],
            INIT_WORLD_XYZ_MM[0], INIT_WORLD_XYZ_MM[1], INIT_WORLD_XYZ_MM[2]
        )

    def _build_visuals_and_show(self):
        self._clear_scene()
        self.current_cfg = {}
        link_fk = self.robot.link_fk(cfg=self.current_cfg)
        for link, T_wL in link_fk.items():
            visuals = getattr(link, "visuals", None)
            if not visuals: continue
            for vis in visuals:
                mesh_path, scale = self._extract_mesh(vis)
                if not mesh_path: continue
                color = self._extract_color(vis) or self.DEFAULT_RGB
                T_Lv = self._pose_to_T(getattr(vis, "origin", None))
                T_wv = T_wL @ T_Lv
                actor = self._actor_from_mesh(mesh_path, color)
                if actor is None: continue
                # 注意：最终乘以 self.T_world（含旋转+平移）
                self._apply_T(actor, self.T_world @ T_wv)
                if scale is not None:
                    if np.isscalar(scale):
                        s = float(scale);
                        actor.SetScale(s, s, s)
                    else:
                        sx, sy, sz = map(float, scale);
                        actor.SetScale(sx, sy, sz)
                self.ren.AddActor(actor)
                self.visuals.append(VisualItem(actor=actor, link_name=link.name, T_Lv=T_Lv))

    def _build_controls(self):
        # 清空旧控件
        while self.ctrl_vbox.count():
            it = self.ctrl_vbox.takeAt(0)
            w = it.widget()
            if w: w.deleteLater()
        self.chain_sliders.clear();
        self.chain_val_labels.clear()
        self.joint2widgets.clear();
        self.joint2chainpos.clear()

        # ===== 全局旋转（R/P/Y）控件 =====
        gb_rot = QGroupBox("模型朝向（全局旋转：Roll / Pitch / Yaw，单位：度）")
        grid_r = QGridLayout(gb_rot)
        rpy_names = ["Roll (X)", "Pitch (Y)", "Yaw (Z)"]
        self.world_sliders_rot: List[QSlider] = []
        self.world_value_labels_rot: List[QLabel] = []
        for i, name in enumerate(rpy_names):
            lab_name = QLabel(name + ":")
            s = QSlider(Qt.Horizontal);
            s.setRange(-180, 180);
            s.setSingleStep(1);
            s.setPageStep(10)
            s.setValue(int(round(self.world_rpy_deg[i])))
            lab_val = QLabel(f"{int(round(self.world_rpy_deg[i]))}°");
            lab_val.setFixedWidth(56)

            def on_world_rot_changed(v, idx=i, lv=lab_val):
                lv.setText(f"{int(v)}°")
                self.world_rpy_deg[idx] = float(v)
                self._refresh_world_matrix()
                self._update_fk_and_actors()

            s.valueChanged.connect(on_world_rot_changed)
            self.world_sliders_rot.append(s);
            self.world_value_labels_rot.append(lab_val)
            grid_r.addWidget(lab_name, i, 0);
            grid_r.addWidget(s, i, 1);
            grid_r.addWidget(lab_val, i, 2)
        btn_rot_zero = QPushButton("全局旋转恢复默认")
        btn_rot_zero.clicked.connect(lambda: self.set_model_orientation_deg(
            INIT_WORLD_RPY_DEG[0], INIT_WORLD_RPY_DEG[1], INIT_WORLD_RPY_DEG[2]
        ))
        grid_r.addWidget(btn_rot_zero, 3, 1)
        self.ctrl_vbox.addWidget(gb_rot)

        # ===== 全局平移（X/Y/Z）控件（单位：mm） =====
        gb_trn = QGroupBox("模型平移（全局：X / Y / Z，单位：毫米，向右/向前/向上为正）")
        grid_t = QGridLayout(gb_trn)
        xyz_names = ["X", "Y", "Z"]
        self.world_sliders_trn: List[QSlider] = []
        self.world_value_labels_trn: List[QLabel] = []
        TRN_MIN_MM, TRN_MAX_MM = -500, 500  # ±500 mm 可改
        for i, name in enumerate(xyz_names):
            lab_name = QLabel(name + ":")
            s = QSlider(Qt.Horizontal);
            s.setRange(TRN_MIN_MM, TRN_MAX_MM);
            s.setSingleStep(1);
            s.setPageStep(20)
            s.setValue(int(round(self.world_xyz_mm[i])))
            lab_val = QLabel(f"{int(round(self.world_xyz_mm[i]))} mm");
            lab_val.setFixedWidth(72)

            def on_world_trn_changed(v, idx=i, lv=lab_val):
                lv.setText(f"{int(v)} mm")
                self.world_xyz_mm[idx] = float(v)
                self._refresh_world_matrix()
                self._update_fk_and_actors()

            s.valueChanged.connect(on_world_trn_changed)
            self.world_sliders_trn.append(s);
            self.world_value_labels_trn.append(lab_val)
            grid_t.addWidget(lab_name, i, 0);
            grid_t.addWidget(s, i, 1);
            grid_t.addWidget(lab_val, i, 2)
        btn_trn_zero = QPushButton("全局平移恢复默认")
        btn_trn_zero.clicked.connect(lambda: self.set_model_translation_mm(
            INIT_WORLD_XYZ_MM[0], INIT_WORLD_XYZ_MM[1], INIT_WORLD_XYZ_MM[2]
        ))
        grid_t.addWidget(btn_trn_zero, 3, 1)
        self.ctrl_vbox.addWidget(gb_trn)

        # ===== 手指关节控件 =====
        if not self.chains:
            tip = QLabel("未检测到手指链（可在 MANUAL_ROOTS/FINGER_NAME_HINTS 调整）。")
            self.ctrl_vbox.addWidget(tip);
            self.ctrl_vbox.addStretch(1);
            return

        for ci, chain in enumerate(self.chains):
            kind = self.chain_kind[ci]
            box = QGroupBox(f"手指 {ci:02d}  [{kind}]  — 根关节 {chain[0]}  (关节数: {len(chain)})")
            form = QFormLayout(box);
            form.setLabelAlignment(Qt.AlignRight)

            sliders_row: List[QSlider] = []
            labels_row: List[QLabel] = []

            for ji, jname in enumerate(chain):
                tags = []
                if ji in LOCK_JOINTS.get(kind, set()):  tags.append("LOCK")
                if ji in INVERT_JOINTS.get(kind, set()): tags.append("INV")
                tag_str = ("  [" + ",".join(tags) + "]") if tags else ""

                row = QHBoxLayout()
                s = QSlider(Qt.Horizontal);
                s.setRange(UI_DEG_MIN, UI_DEG_MAX);
                s.setSingleStep(1);
                s.setPageStep(10);
                s.setValue(0)
                s.setFocusPolicy(Qt.StrongFocus)
                lab = QLabel("0°");
                lab.setFixedWidth(80)
                row.addWidget(s, 1);
                row.addWidget(lab)
                form.addRow(f"{ji + 1:02d}. {jname}{tag_str}", row)

                # 映射记录
                self.joint2widgets[jname] = (s, lab)
                self.joint2chainpos[jname] = (ci, ji)

                if ji in LOCK_JOINTS.get(kind, set()):
                    s.setEnabled(False)
                else:
                    def on_changed(v, jn=jname):
                        self._apply_single_joint_angle(jn, math.radians(float(v)))

                    s.valueChanged.connect(on_changed)

                sliders_row.append(s);
                labels_row.append(lab)

            self.chain_sliders.append(sliders_row)
            self.chain_val_labels.append(labels_row)
            self.ctrl_vbox.addWidget(box)

        # 重置按钮
        btn_reset = QPushButton("重置所有关节");
        btn_reset.clicked.connect(self._reset_all)
        btn_pose_zero = QPushButton("全局位姿恢复默认（旋转+平移）")
        btn_pose_zero.clicked.connect(self.reset_pose_to_default)
        h = QHBoxLayout();
        h.addWidget(btn_reset);
        h.addWidget(btn_pose_zero);
        self.ctrl_vbox.addLayout(h)
        self.ctrl_vbox.addStretch(1)

    # ===== 单关节应用 =====
    def _apply_single_joint_angle(self, joint_name: str, rads_input: float):
        ci, ji = self.joint2chainpos.get(joint_name, (-1, -1))
        if ci < 0: return
        kind = self.chain_kind[ci]
        if ji in LOCK_JOINTS.get(kind, set()):  # 锁定直接返回
            return
        val = -rads_input if ji in INVERT_JOINTS.get(kind, set()) else rads_input
        applied = self._clamp_by_limit(joint_name, val)
        self.current_cfg[joint_name] = applied

        s, lab = self.joint2widgets[joint_name]
        deg_in = int(round(math.degrees(rads_input)))
        deg_ap = int(round(math.degrees(applied)))
        if not self.cb_ignore_limits.isChecked() and abs(
                deg_ap - (-deg_in if ji in INVERT_JOINTS.get(kind, set()) else deg_in)) >= 1:
            lab.setText(f"{deg_in}° → {deg_ap}°")
        else:
            lab.setText(f"{deg_in}°")

        self._update_fk_and_actors()

    # ===== 全局位姿：矩阵刷新 =====
    def _refresh_world_matrix(self):
        # 旋转
        roll, pitch, yaw = [math.radians(float(d)) for d in self.world_rpy_deg]
        cx, sx = math.cos(roll), math.sin(roll)
        cy, sy = math.cos(pitch), math.sin(pitch)
        cz, sz = math.cos(yaw), math.sin(yaw)
        # R = Rz * Ry * Rx
        R = np.array([
            [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
            [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
            [-sy, cy * sx, cy * cx]
        ], dtype=float)
        # 平移（mm -> m）
        tx, ty, tz = [float(x) / 1000.0 for x in self.world_xyz_mm]
        T = np.eye(4, dtype=float)
        T[:3, :3] = R
        T[:3, 3] = [tx, ty, tz]
        self.T_world = T

    # ===== 公共接口：旋转（度） =====
    def set_model_orientation_deg(self, roll: Optional[float] = None, pitch: Optional[float] = None,
                                  yaw: Optional[float] = None):
        if roll is not None: self.world_rpy_deg[0] = float(roll)
        if pitch is not None: self.world_rpy_deg[1] = float(pitch)
        if yaw is not None: self.world_rpy_deg[2] = float(yaw)
        # 同步 UI
        for i, s in enumerate(self.world_sliders_rot):
            s.blockSignals(True);
            s.setValue(int(round(self.world_rpy_deg[i])));
            s.blockSignals(False)
            self.world_value_labels_rot[i].setText(f"{int(round(self.world_rpy_deg[i]))}°")
        self._refresh_world_matrix()
        self._update_fk_and_actors()

    # ===== 公共接口：旋转（四元数） =====
    def set_model_orientation_quat(self, qx: float, qy: float, qz: float, qw: float):
        n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
        qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
        xx, yy, zz = qx * qx, qy * qy, qz * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        wx, wy, wz = qw * qx, qw * qy, qw * qz
        R = np.array([
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)]
        ], dtype=float)
        # 反解 R -> rpy（与 _refresh_world_matrix 的 Rz*Ry*Rx 对齐）
        pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
        self.world_rpy_deg = [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]
        # 同步 UI
        for i, s in enumerate(self.world_sliders_rot):
            s.blockSignals(True);
            s.setValue(int(round(self.world_rpy_deg[i])));
            s.blockSignals(False)
            self.world_value_labels_rot[i].setText(f"{int(round(self.world_rpy_deg[i]))}°")
        # 写矩阵（平移不变）
        T = np.eye(4, dtype=float);
        T[:3, :3] = R;
        T[:3, 3] = [self.world_xyz_mm[0] / 1000.0,
                    self.world_xyz_mm[1] / 1000.0,
                    self.world_xyz_mm[2] / 1000.0]
        self.T_world = T
        self._update_fk_and_actors()

    # ===== 公共接口：平移（毫米） =====
    def set_model_translation_mm(self, x: Optional[float] = None, y: Optional[float] = None, z: Optional[float] = None):
        if x is not None: self.world_xyz_mm[0] = float(x)
        if y is not None: self.world_xyz_mm[1] = float(y)
        if z is not None: self.world_xyz_mm[2] = float(z)
        # 同步 UI
        for i, s in enumerate(self.world_sliders_trn):
            s.blockSignals(True);
            s.setValue(int(round(self.world_xyz_mm[i])));
            s.blockSignals(False)
            self.world_value_labels_trn[i].setText(f"{int(round(self.world_xyz_mm[i]))} mm")
        self._refresh_world_matrix()
        self._update_fk_and_actors()

    # ===== 公共接口：平移（米） =====
    def set_model_translation_m(self, x: Optional[float] = None, y: Optional[float] = None, z: Optional[float] = None):
        mm = list(self.world_xyz_mm)
        if x is not None: mm[0] = float(x) * 1000.0
        if y is not None: mm[1] = float(y) * 1000.0
        if z is not None: mm[2] = float(z) * 1000.0
        self.set_model_translation_mm(mm[0], mm[1], mm[2])

    # ===== 便捷接口：同时设置旋转(度) + 平移(mm) =====
    def set_model_pose_deg_mm(self, roll: float, pitch: float, yaw: float, x_mm: float, y_mm: float, z_mm: float):
        self.world_rpy_deg = [float(roll), float(pitch), float(yaw)]
        self.world_xyz_mm = [float(x_mm), float(y_mm), float(z_mm)]
        # 同步 UI
        for i, s in enumerate(self.world_sliders_rot):
            s.blockSignals(True);
            s.setValue(int(round(self.world_rpy_deg[i])));
            s.blockSignals(False)
            self.world_value_labels_rot[i].setText(f"{int(round(self.world_rpy_deg[i]))}°")
        for i, s in enumerate(self.world_sliders_trn):
            s.blockSignals(True);
            s.setValue(int(round(self.world_xyz_mm[i])));
            s.blockSignals(False)
            self.world_value_labels_trn[i].setText(f"{int(round(self.world_xyz_mm[i]))} mm")
        self._refresh_world_matrix()
        self._update_fk_and_actors()

    def _reset_all(self):
        for chain in self.chains:
            for jn in chain:
                self.current_cfg[jn] = 0.0
                if jn in self.joint2widgets:
                    s, lab = self.joint2widgets[jn]
                    s.blockSignals(True);
                    s.setValue(0);
                    s.blockSignals(False)
                    lab.setText("0°")
        self._update_fk_and_actors()

    # ===== 限位/忽略 =====
    def _clamp_by_limit(self, joint_name: str, val: float) -> float:
        if self.cb_ignore_limits.isChecked():
            return val
        jtype = self.joint_types.get(joint_name, "")
        if jtype == "continuous":
            return val
        lo, up = self.joint_limits.get(joint_name, (None, None))
        if lo is not None and val < lo: val = lo
        if up is not None and val > up: val = up
        return val

    def _on_ignore_limits_toggled(self, checked: bool):
        print(f"[CFG] 忽略URDF限位 = {checked}")
        for ci, chain in enumerate(self.chains):
            kind = self.chain_kind[ci]
            for ji, jn in enumerate(chain):
                if ji in LOCK_JOINTS.get(kind, set()):
                    self.current_cfg[jn] = 0.0
                    continue
                s, lab = self.joint2widgets[jn]
                inp = math.radians(float(s.value()))
                val = -inp if ji in INVERT_JOINTS.get(kind, set()) else inp
                applied = self._clamp_by_limit(jn, val)
                self.current_cfg[jn] = applied
                deg_in = int(round(math.degrees(inp)))
                deg_ap = int(round(math.degrees(applied)))
                if not checked and abs(deg_ap - (-deg_in if ji in INVERT_JOINTS.get(kind, set()) else deg_in)) >= 1:
                    lab.setText(f"{deg_in}° → {deg_ap}°")
                else:
                    lab.setText(f"{deg_in}°")
        self._update_fk_and_actors()

    # ===== FK 刷新 =====
    def _update_fk_and_actors(self):
        link_fk = self.robot.link_fk(cfg=self.current_cfg)
        name2T = {lnk.name: T for lnk, T in link_fk.items()}
        for vi in self.visuals:
            T_wL = name2T.get(vi.link_name, np.eye(4))
            T = self.T_world @ (T_wL @ vi.T_Lv)  # 关键：全局位姿乘上去
            self._apply_T(vi.actor, T)
        self.vtk.GetRenderWindow().Render()

    # ===== 解析工具 =====
    def _extract_mesh(self, visual):
        g = getattr(visual, "geometry", None)
        if g is None or getattr(g, "mesh", None) is None: return None, None
        m = g.mesh;
        uri = getattr(m, "filename", None);
        scale = getattr(m, "scale", None)
        if uri is None: return None, None
        p = uri.replace("\\", "/")
        if not os.path.isabs(p):
            p = os.path.abspath(os.path.join(os.path.dirname(URDF_PATH), p)).replace("\\", "/")
        if not os.path.exists(p):
            self._warn(f"找不到网格: {uri} -> {p}")
            return None, None
        return p, scale

    def _extract_color(self, visual):
        mat = getattr(visual, "material", None)
        if mat is None: return None
        rgba = getattr(mat, "rgba", None)
        if rgba is None: return None
        r, g, b, a = rgba
        return (float(r), float(g), float(b))

    def _pose_to_T(self, origin):
        if origin is None: return np.eye(4)
        rpy = getattr(origin, "rpy", None) or getattr(origin, "rotation", (0.0, 0.0, 0.0))
        xyz = getattr(origin, "xyz", None) or getattr(origin, "translation", (0.0, 0.0, 0.0))
        roll, pitch, yaw = map(float, rpy)
        cx, sx = math.cos(roll), math.sin(roll)
        cy, sy = math.cos(pitch), math.sin(pitch)
        cz, sz = math.cos(yaw), math.sin(yaw)
        R = np.array([
            [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
            [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
            [-sy, cy * sx, cy * cx]
        ], float)
        T = np.eye(4);
        T[:3, :3] = R;
        T[:3, 3] = [float(xyz[0]), float(xyz[1]), float(xyz[2])]
        return T

    # ===== VTK 工具 =====
    def _actor_from_mesh(self, mesh_path: str, rgb=(0.8, 0.9, 1.0)):
        ext = os.path.splitext(mesh_path)[1].lower()
        if ext == ".stl":
            reader = vtkSTLReader();
            reader.SetFileName(mesh_path);
            reader.Update()
            normals = vtkPolyDataNormals();
            normals.SetInputConnection(reader.GetOutputPort())
            normals.ConsistencyOn();
            normals.SplittingOn();
            normals.SetFeatureAngle(float(self.FEATURE_ANGLE_DEG));
            normals.Update()
            mapper = vtkPolyDataMapper();
            mapper.SetInputConnection(normals.GetOutputPort())
        elif ext == ".ply":
            reader = vtkPLYReader();
            reader.SetFileName(mesh_path);
            reader.Update()
            mapper = vtkPolyDataMapper();
            mapper.SetInputConnection(reader.GetOutputPort())
        elif ext == ".obj":
            reader = vtkOBJReader();
            reader.SetFileName(mesh_path);
            reader.Update()
            mapper = vtkPolyDataMapper();
            mapper.SetInputConnection(reader.GetOutputPort())
        else:
            self._warn(f"暂不支持的网格格式: {ext} ({mesh_path})");
            return None
        actor = vtkActor();
        actor.SetMapper(mapper)
        prop: vtkProperty = actor.GetProperty()
        prop.SetColor(*rgb);
        prop.SetAmbient(0.35);
        prop.SetDiffuse(0.70);
        prop.SetSpecular(0.25);
        prop.SetSpecularPower(22.0)
        return actor

    def _apply_T(self, actor: vtkActor, T: np.ndarray):
        m = vtkMatrix4x4()
        for r in range(4):
            for c in range(4):
                m.SetElement(r, c, float(T[r, c]))
        actor.SetUserMatrix(m)

    # ===== 相机 =====
    def _bounds(self):
        ac = self.ren.GetActors();
        ac.InitTraversal();
        first = True
        xmin = xmax = ymin = ymax = zmin = zmax = 0.0
        while True:
            a = ac.GetNextActor()
            if not a: break
            b = a.GetBounds()
            if not b: continue
            if first:
                xmin, xmax, ymin, ymax, zmin, zmax = b;
                first = False
            else:
                xmin = min(xmin, b[0]);
                xmax = max(xmax, b[1])
                ymin = min(ymin, b[2]);
                ymax = max(ymax, b[3])
                zmin = min(zmin, b[4]);
                zmax = max(zmax, b[5])
        return None if first else (xmin, xmax, ymin, ymax, zmin, zmax)

    def fit_camera(self):
        b = self._bounds()
        if b is None: self.ren.ResetCamera(); return
        xmin, xmax, ymin, ymax, zmin, zmax = b
        cx = 0.5 * (xmin + xmax);
        cy = 0.5 * (ymin + ymax);
        cz = 0.5 * (zmin + zmax)
        dx = xmax - xmin;
        dy = ymax - ymin;
        dz = zmax - zmin
        diag = max(1e-6, math.sqrt(dx * dx + dy * dy + dz * dz))
        cam = self.ren.GetActiveCamera()
        cam.SetFocalPoint(cx, cy, cz)
        va = cam.GetViewAngle() or 30.0
        radius = 0.5 * diag;
        dist = radius / math.tan(math.radians(va) / 2.0) * 1.2
        cam.SetPosition(cx, cy, cz + dist);
        cam.SetViewUp(0, 1, 0)
        near = max(0.1, dist - 2.5 * radius);
        far = dist + 2.5 * radius
        cam.SetClippingRange(near, far);
        self.ren.ResetCameraClippingRange()

    def _snapshot_cam(self):
        cam = self.ren.GetActiveCamera()
        return dict(pos=cam.GetPosition(), focal=cam.GetFocalPoint(),
                    up=cam.GetViewUp(), ang=cam.GetViewAngle(), clip=cam.GetClippingRange())

    def _restore_cam(self, snap):
        cam = self.ren.GetActiveCamera()
        cam.SetPosition(*snap["pos"]);
        cam.SetFocalPoint(*snap["focal"])
        cam.SetViewUp(*snap["up"]);
        cam.SetViewAngle(snap["ang"]);
        cam.SetClippingRange(*snap["clip"])
        self.ren.ResetCameraClippingRange()

    def reset_camera(self):
        if self.cam_init:
            self._restore_cam(self.cam_init)
        else:
            self.fit_camera();
            self.cam_init = self._snapshot_cam()

    # ===== 其他 =====
    def _clear_scene(self):
        ac = self.ren.GetActors();
        ac.InitTraversal();
        rm = []
        while True:
            a = ac.GetNextActor()
            if not a: break
            rm.append(a)
        for a in rm: self.ren.RemoveActor(a)
        self.visuals.clear();
        self.cam_init = None

    def _placeholder(self):
        self._clear_scene()
        cube = vtkCubeSource();
        cube.SetXLength(50);
        cube.SetYLength(50);
        cube.SetZLength(50)
        mapper = vtkPolyDataMapper();
        mapper.SetInputConnection(cube.GetOutputPort())
        actor = vtkActor();
        actor.SetMapper(mapper);
        actor.GetProperty().SetColor(0.7, 0.9, 1.0)
        self.ren.AddActor(actor);
        self.fit_camera();
        self.cam_init = self._snapshot_cam()

    def _ok(self, msg):
        self.lab_status.setText("状态: " + msg);
        print("[OK]", msg)

    def _warn(self, msg):
        self.lab_status.setText("状态: " + msg);
        print("[WARN]", msg)

    def _err(self, msg):
        self.lab_status.setText("状态: " + msg);
        print("[ERROR]", msg)


def _app_icon_path() -> str:
    return resource_path("assets/wall-e.png")


# =============== 通用工具 ===============
def modbus_crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for ch in data:
        crc ^= ch
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return struct.pack('<H', crc)


def crc32_accumulate(data: bytes, pre_crc: int = 0xFFFFFFFF) -> int:
    crc = pre_crc
    for b in data:
        crc ^= (b << 24) & 0xFFFFFFFF
        for _ in range(8):
            if crc & 0x80000000:
                crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF
    return crc & 0xFFFFFFFF


def float_to_half_be(f: float) -> bytes:
    le = struct.pack("<e", float(f))
    return le[::-1]


def half_be_to_float(b1: int, b2: int) -> float:
    le = bytes((b2, b1))
    return struct.unpack("<e", le)[0]


# =============== 步长规划弹出框 ===============
class StepPlanDialog(QDialog):
    FINGER_NAMES = [
        "拇指-弯曲",
        "拇指-侧摆",
        "食指",
        "中指",
        "无名指",
        "小指",
    ]

    def __init__(self, parent=None, main=None):
        super().__init__(parent)
        self.setWindowTitle("步长规划")
        self.resize(900, 540)
        self.main = main
        if main:
            self.setStyleSheet(main.build_qss())

        root = QHBoxLayout(self)
        root.setSpacing(10)
        root.setContentsMargins(10, 10, 10, 10)

        # 左侧
        left = QVBoxLayout()
        left.setSpacing(10)

        # 表格
        self.table = QTableWidget(len(self.FINGER_NAMES), 4, self)
        self.table.setHorizontalHeaderLabels([
            "手指",
            "起始角度(°)",
            "步长(°)",
            "速度(°/s)"
        ])
        self.table.verticalHeader().setVisible(False)
        self.table.setColumnWidth(0, 130)
        self.table.setColumnWidth(1, 120)
        self.table.setColumnWidth(2, 120)
        self.table.setColumnWidth(3, 120)

        self.start_spins = []
        self.step_spins = []
        self.speed_spins = []

        for row, name in enumerate(self.FINGER_NAMES):
            item = QTableWidgetItem(name)
            item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
            self.table.setItem(row, 0, item)

            start_spin = QDoubleSpinBox()
            start_spin.setDecimals(2)
            start_spin.setRange(0.0, 180.0)
            start_spin.setValue(0.0)
            start_spin.setObjectName("InputSpin")
            start_spin.setButtonSymbols(QAbstractSpinBox.NoButtons)
            start_spin.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            start_spin.valueChanged.connect(self.update_preview)
            self.table.setCellWidget(row, 1, start_spin)
            self.start_spins.append(start_spin)

            step_spin = QDoubleSpinBox()
            step_spin.setDecimals(2)
            step_spin.setRange(-180.0, 180.0)
            step_spin.setValue(0.0)
            step_spin.setObjectName("InputSpin")
            step_spin.setButtonSymbols(QAbstractSpinBox.NoButtons)
            step_spin.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            step_spin.valueChanged.connect(self.update_preview)
            self.table.setCellWidget(row, 2, step_spin)
            self.step_spins.append(step_spin)

            speed_spin = QDoubleSpinBox()
            speed_spin.setDecimals(1)
            speed_spin.setRange(0.0, 500.0)
            speed_spin.setValue(50.0)
            speed_spin.setObjectName("InputSpin")
            speed_spin.setButtonSymbols(QAbstractSpinBox.NoButtons)
            speed_spin.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            speed_spin.valueChanged.connect(self.update_preview)
            self.table.setCellWidget(row, 3, speed_spin)
            self.speed_spins.append(speed_spin)

        left.addWidget(self.table, 1)

        # 参数区
        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignRight)
        form.setFormAlignment(Qt.AlignLeft | Qt.AlignTop)
        form.setHorizontalSpacing(12)
        form.setVerticalSpacing(6)

        self.spin_frame_count = QSpinBox()
        self.spin_frame_count.setRange(1, 2000)
        self.spin_frame_count.setValue(100)
        self.spin_frame_count.setObjectName("InputSpin")
        self.spin_frame_count.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.spin_frame_count.valueChanged.connect(self.update_preview)
        form.addRow("帧数:", self.spin_frame_count)

        self.spin_frame_delay = QSpinBox()
        self.spin_frame_delay.setRange(1, 10000)
        self.spin_frame_delay.setValue(10)
        self.spin_frame_delay.setObjectName("InputSpin")
        self.spin_frame_delay.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.spin_frame_delay.valueChanged.connect(self.update_preview)
        form.addRow("每帧延时(ms):", self.spin_frame_delay)

        self.spin_first_delay = QSpinBox()
        self.spin_first_delay.setRange(0, 60000)
        self.spin_first_delay.setValue(2000)
        self.spin_first_delay.setObjectName("InputSpin")
        self.spin_first_delay.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.spin_first_delay.valueChanged.connect(self.update_preview)
        form.addRow("首帧延时(ms):", self.spin_first_delay)

        self.spin_last_delay = QSpinBox()
        self.spin_last_delay.setRange(0, 60000)
        self.spin_last_delay.setValue(2000)
        self.spin_last_delay.setObjectName("InputSpin")
        self.spin_last_delay.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.spin_last_delay.valueChanged.connect(self.update_preview)
        form.addRow("末帧延时(ms):", self.spin_last_delay)

        self.spin_loop_count = QSpinBox()
        self.spin_loop_count.setRange(-1, 10000)
        self.spin_loop_count.setValue(-1)
        self.spin_loop_count.setObjectName("InputSpin")
        self.spin_loop_count.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.spin_loop_count.valueChanged.connect(self.update_preview)
        form.addRow("循环次数(-1无限):", self.spin_loop_count)

        self.chk_return = QCheckBox("")
        self.chk_return.setStyleSheet("""
            QCheckBox {
                background: rgba(255,255,255,0.015);
                border: 1px solid rgba(120,176,255,0.15);
                border-radius: 6px;
                padding: 3px;
            }
            QCheckBox::indicator {
                width: 15px;
                height: 15px;
                border-radius: 4px;
                border: 1px solid rgba(220,230,255,0.25);
                background: rgba(8,10,14,0.35);
            }
            QCheckBox::indicator:checked {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                                            stop:0 #9cc4ff,
                                            stop:1 #4f8cff);
                border: 1px solid rgba(200,230,255,0.7);
            }
        """)
        self.chk_return.stateChanged.connect(self.update_preview)
        form.addRow("是否返程:", self.chk_return)

        left.addLayout(form)

        # 按钮
        btn_row = QHBoxLayout()
        btn_row.addStretch(1)
        self.btn_gen = QPushButton("生成")
        self.btn_gen.setObjectName("PrimaryButton")
        self.btn_gen.clicked.connect(self.on_generate)
        self.btn_close = QPushButton("关闭")
        self.btn_close.clicked.connect(self.reject)
        btn_row.addWidget(self.btn_gen)
        btn_row.addWidget(self.btn_close)
        left.addLayout(btn_row)

        root.addLayout(left, 1)

        # 右侧预览
        self.preview_edit = QTextEdit()
        self.preview_edit.setReadOnly(True)
        self.preview_edit.setMinimumWidth(320)
        self.preview_edit.setObjectName("GlobalLog")
        root.addWidget(self.preview_edit, 0)

        self.update_preview()

    def load_from_plan(self, plan: dict):
        starts = plan.get("starts", [])
        steps = plan.get("steps", [])
        speeds = plan.get("speeds", [])

        for i, sp in enumerate(self.start_spins):
            if i < len(starts):
                sp.blockSignals(True)
                sp.setValue(starts[i])
                sp.blockSignals(False)

        for i, sp in enumerate(self.step_spins):
            if i < len(steps):
                sp.blockSignals(True)
                sp.setValue(steps[i])
                sp.blockSignals(False)

        for i, sp in enumerate(self.speed_spins):
            if i < len(speeds):
                sp.blockSignals(True)
                sp.setValue(speeds[i])
                sp.blockSignals(False)

        frame_count = plan.get("frame_count", None)
        if frame_count is None:
            frames = plan.get("frames", [])
            frame_count = len(frames) if frames else self.spin_frame_count.value()
        self.spin_frame_count.blockSignals(True)
        self.spin_frame_count.setValue(frame_count)
        self.spin_frame_count.blockSignals(False)

        self.spin_frame_delay.blockSignals(True)
        self.spin_frame_delay.setValue(plan.get("per_delay", 10))
        self.spin_frame_delay.blockSignals(False)

        self.spin_first_delay.blockSignals(True)
        self.spin_first_delay.setValue(plan.get("first_delay", 2000))
        self.spin_first_delay.blockSignals(False)

        self.spin_last_delay.blockSignals(True)
        self.spin_last_delay.setValue(plan.get("last_delay", 2000))
        self.spin_last_delay.blockSignals(False)

        self.spin_loop_count.blockSignals(True)
        self.spin_loop_count.setValue(plan.get("loop_count", -1))
        self.spin_loop_count.blockSignals(False)

        self.chk_return.blockSignals(True)
        self.chk_return.setChecked(plan.get("with_return", False))
        self.chk_return.blockSignals(False)

        self.update_preview()

    def build_plan_data(self):
        starts = [sp.value() for sp in self.start_spins]
        steps = [sp.value() for sp in self.step_spins]
        speeds = [sp.value() for sp in self.speed_spins]

        frame_count = self.spin_frame_count.value()
        per_delay = self.spin_frame_delay.value()
        first_delay = self.spin_first_delay.value()
        last_delay = self.spin_last_delay.value()
        loop_count = self.spin_loop_count.value()
        with_return = self.chk_return.isChecked()

        frames = []
        for i in range(frame_count):
            angles = [starts[j] + steps[j] * i for j in range(6)]
            frames.append({
                "angles": angles,
                "speeds": speeds,
                "delay_ms": per_delay
            })

        if with_return and frame_count > 1:
            for i in range(frame_count - 2, -1, -1):
                angles = [starts[j] + steps[j] * i for j in range(6)]
                frames.append({
                    "angles": angles,
                    "speeds": speeds,
                    "delay_ms": per_delay
                })

        plan = {
            "frames": frames,
            "frame_count": frame_count,
            "per_delay": per_delay,
            "first_delay": first_delay,
            "last_delay": last_delay,
            "loop_count": loop_count,
            "loop_infinite": (loop_count == -1),
            "with_return": with_return,
            "starts": starts,
            "steps": steps,
            "speeds": speeds,
        }
        return plan

    def update_preview(self):
        plan = self.build_plan_data()
        frames = plan["frames"]

        lines = []
        max_show = 200

        if self.main is not None:
            slave_addr = self.main.current_slave_addr
        else:
            slave_addr = 0xC8

        for i, fr in enumerate(frames[:max_show]):
            full_frame = self.build_modbus10_frame(slave_addr, fr["angles"], fr["speeds"])
            hex_str = " ".join(f"{b:02X}" for b in full_frame)
            lines.append(f"#{i:03d}  {hex_str}")

        if len(frames) > max_show:
            lines.append(f"... 共 {len(frames)} 帧")

        loops = "∞" if plan["loop_infinite"] else str(plan["loop_count"])
        head = f"[预览] 帧数:{len(frames)}, 循环:{loops}, 返程:{'是' if plan['with_return'] else '否'}\n"
        self.preview_edit.setPlainText(head + "\n".join(lines))

    def build_modbus10_frame(self, slave_addr: int, angles, speeds) -> bytes:
        start_reg = 0x0002
        reg_count = 0x000C
        byte_count = 0x18
        payload = b""
        for v in angles:
            payload += float_to_half_be(v)
        for v in speeds:
            payload += float_to_half_be(v)
        frame_head = struct.pack(">B B H H B", slave_addr & 0xFF, 0x10, start_reg, reg_count, byte_count)
        frame = frame_head + payload
        crc = modbus_crc16(frame)
        full = frame + crc
        return full

    def on_generate(self):
        plan = self.build_plan_data()
        if self.main:
            self.main.receive_step_plan(plan)
        # 不关对话框


class FistTestDialog(QDialog):
    FINGER_INFOS = [
        ("拇指-弯曲", 43.5),
        ("拇指-侧摆", 86.0),
        ("食指", 80.0),
        ("中指", 80.0),
        ("无名指", 80.0),
        ("小指", 80.0),
    ]

    def __init__(self, parent=None, main=None):
        super().__init__(parent)
        self.setWindowTitle("握拳测试")
        self.resize(920, 560)
        self.main = main
        if main:
            self.setStyleSheet(main.build_qss())

        root = QVBoxLayout(self)
        root.setSpacing(10)
        root.setContentsMargins(10, 10, 10, 10)

        self.table = QTableWidget(len(self.FINGER_INFOS), 5, self)
        self.table.setHorizontalHeaderLabels([
            "手指",
            "初始张开角度(°)",
            "目标握拳角度(°)",
            "闭合速度(°/s)",
            "张开速度(°/s)",
        ])
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionMode(QAbstractItemView.NoSelection)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setColumnWidth(0, 120)
        self.table.setColumnWidth(1, 150)
        self.table.setColumnWidth(2, 150)
        self.table.setColumnWidth(3, 140)
        self.table.setColumnWidth(4, 140)
        try:
            self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
            self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        except Exception:
            pass

        self.open_angle_boxes = []
        self.close_angle_boxes = []
        self.close_speed_boxes = []
        self.open_speed_boxes = []

        for row, (name, ang_max) in enumerate(self.FINGER_INFOS):
            item = QTableWidgetItem(name)
            item.setFlags(Qt.ItemIsEnabled)
            self.table.setItem(row, 0, item)

            open_spin = QDoubleSpinBox()
            open_spin.setDecimals(2)
            open_spin.setRange(0.0, ang_max)
            open_spin.setValue(0.0)
            open_spin.setObjectName("InputSpin")
            open_spin.setButtonSymbols(QAbstractSpinBox.NoButtons)
            open_spin.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self.table.setCellWidget(row, 1, open_spin)
            self.open_angle_boxes.append(open_spin)

            close_spin = QDoubleSpinBox()
            close_spin.setDecimals(2)
            close_spin.setRange(0.0, ang_max)
            close_spin.setValue(0.0)
            close_spin.setObjectName("InputSpin")
            close_spin.setButtonSymbols(QAbstractSpinBox.NoButtons)
            close_spin.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self.table.setCellWidget(row, 2, close_spin)
            self.close_angle_boxes.append(close_spin)

            close_speed_spin = QDoubleSpinBox()
            close_speed_spin.setDecimals(1)
            close_speed_spin.setRange(0.0, 90.0)
            close_speed_spin.setValue(50.0)
            close_speed_spin.setObjectName("InputSpin")
            close_speed_spin.setButtonSymbols(QAbstractSpinBox.NoButtons)
            close_speed_spin.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self.table.setCellWidget(row, 3, close_speed_spin)
            self.close_speed_boxes.append(close_speed_spin)

            open_speed_spin = QDoubleSpinBox()
            open_speed_spin.setDecimals(1)
            open_speed_spin.setRange(0.0, 90.0)
            open_speed_spin.setValue(50.0)
            open_speed_spin.setObjectName("InputSpin")
            open_speed_spin.setButtonSymbols(QAbstractSpinBox.NoButtons)
            open_speed_spin.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self.table.setCellWidget(row, 4, open_speed_spin)
            self.open_speed_boxes.append(open_speed_spin)

        root.addWidget(self.table, 1)

        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignRight)
        form.setHorizontalSpacing(12)
        form.setVerticalSpacing(6)

        self.spin_close_hold = QSpinBox()
        self.spin_close_hold.setRange(0, 600000)
        self.spin_close_hold.setValue(2000)
        self.spin_close_hold.setObjectName("InputSpin")
        self.spin_close_hold.setButtonSymbols(QAbstractSpinBox.NoButtons)
        form.addRow("闭合保持(ms):", self.spin_close_hold)

        self.spin_open_hold = QSpinBox()
        self.spin_open_hold.setRange(0, 600000)
        self.spin_open_hold.setValue(2000)
        self.spin_open_hold.setObjectName("InputSpin")
        self.spin_open_hold.setButtonSymbols(QAbstractSpinBox.NoButtons)
        form.addRow("张开保持(ms):", self.spin_open_hold)

        self.spin_loop_count = QSpinBox()
        self.spin_loop_count.setRange(1, 100000)
        self.spin_loop_count.setValue(1)
        self.spin_loop_count.setObjectName("InputSpin")
        self.spin_loop_count.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.spin_loop_count.valueChanged.connect(self._on_loop_count_changed)
        form.addRow("循环次数(抓握次数):", self.spin_loop_count)

        self.lab_loop_progress = QLabel("当前循环次数：0 / 1")
        self.lab_loop_progress.setObjectName("FormLabel")
        form.addRow("当前循环次数:", self.lab_loop_progress)

        root.addLayout(form)

        self.lab_status = QLabel("状态：待命")
        self.lab_status.setObjectName("FormLabel")
        root.addWidget(self.lab_status)

        btn_row = QHBoxLayout()
        btn_row.addStretch(1)
        self.btn_apply = QPushButton("应用初始角度")
        self.btn_apply.setObjectName("SecondaryButton")
        self.btn_apply.clicked.connect(self.on_apply_initial)
        self.btn_start = QPushButton("开始测试")
        self.btn_start.setObjectName("PrimaryButton")
        self.btn_start.clicked.connect(self.on_start)
        self.btn_stop = QPushButton("停止测试")
        self.btn_stop.setObjectName("SecondaryButton")
        self.btn_stop.setEnabled(False)
        self.btn_stop.clicked.connect(self.on_stop)
        self.btn_close = QPushButton("关闭")
        self.btn_close.clicked.connect(self.reject)
        btn_row.addWidget(self.btn_apply)
        btn_row.addWidget(self.btn_start)
        btn_row.addWidget(self.btn_stop)
        btn_row.addWidget(self.btn_close)
        root.addLayout(btn_row)

    def _on_loop_count_changed(self, value: int):
        self.set_loop_progress(0, max(1, int(value)))

    def build_plan_data(self):
        return {
            "open_angles": [sp.value() for sp in self.open_angle_boxes],
            "close_angles": [sp.value() for sp in self.close_angle_boxes],
            "close_speeds": [sp.value() for sp in self.close_speed_boxes],
            "open_speeds": [sp.value() for sp in self.open_speed_boxes],
            "close_hold_ms": int(self.spin_close_hold.value()),
            "open_hold_ms": int(self.spin_open_hold.value()),
            "loop_count": int(self.spin_loop_count.value()),
        }

    def load_from_plan(self, plan: dict):
        open_angles = plan.get("open_angles", [])
        close_angles = plan.get("close_angles", [])
        close_speeds = plan.get("close_speeds", [])
        open_speeds = plan.get("open_speeds", [])

        for i, sp in enumerate(self.open_angle_boxes):
            if i < len(open_angles):
                sp.setValue(float(open_angles[i]))
        for i, sp in enumerate(self.close_angle_boxes):
            if i < len(close_angles):
                sp.setValue(float(close_angles[i]))
        for i, sp in enumerate(self.close_speed_boxes):
            if i < len(close_speeds):
                sp.setValue(float(close_speeds[i]))
        for i, sp in enumerate(self.open_speed_boxes):
            if i < len(open_speeds):
                sp.setValue(float(open_speeds[i]))

        self.spin_close_hold.setValue(int(plan.get("close_hold_ms", 2000)))
        self.spin_open_hold.setValue(int(plan.get("open_hold_ms", 2000)))
        self.spin_loop_count.setValue(max(1, int(plan.get("loop_count", 1))))
        self.set_loop_progress(0, max(1, int(plan.get("loop_count", 1))))

    def on_apply_initial(self):
        if self.main is not None:
            self.main.apply_fist_test_initial_to_ui(self.build_plan_data())
            self.lab_status.setText("状态：已应用初始角度到左侧控制区")

    def on_start(self):
        if self.main is not None:
            self.main.start_fist_test(self.build_plan_data(), dialog=self)

    def on_stop(self):
        if self.main is not None:
            self.main.stop_fist_test(from_user=True)

    def set_loop_progress(self, current_loop: int, total_loop: int):
        total_loop = max(1, int(total_loop))
        current_loop = max(0, min(int(current_loop), total_loop))
        self.lab_loop_progress.setText(f"{current_loop} / {total_loop}")

    def set_running(self, running: bool, status: str = ""):
        self.btn_start.setEnabled(not running)
        self.btn_stop.setEnabled(running)
        if status:
            self.lab_status.setText(status)
        else:
            self.lab_status.setText("状态：运行中" if running else "状态：待命")


# =============== 嵌入式 URDF 视图封装 ===============
# =============== 动作序列对话框（记录/导入/保存/播放） ===============
class ActionSequenceDialog(QDialog):
    """
    动作序列窗口：
    - 显示和编辑每一条“位置×6、速度×6、延时ms”的记录
    - 导入/保存文本格式（人可读）
      文本格式定义（逐行）:
        第1行: "LHP-ACTSEQ v1"
        后续每一行一条记录:
            idx;pos=p1,p2,p3,p4,p5,p6;spd=s1,s2,s3,s4,s5,s6;delay=ms
        允许行首/行内空白；允许以 "#" 开头的注释行；数值为十进制浮点/整数。
    - 播放/停止：调用主窗口的开始/停止接口，并在播放时高亮当前条目
    """
    headers = ["编号"] + [f"pos{i + 1}(°)" for i in range(6)] + [f"spd{i + 1}(°/s)" for i in range(6)] + ["delay(ms)"]

    def __init__(self, parent=None, main=None):
        super().__init__(parent)
        self.main = main
        self.setWindowTitle("动作序列")
        self.resize(980, 520)
        if main:
            try:
                self.setStyleSheet(main.build_qss())
            except Exception:
                pass

        root = QVBoxLayout(self)
        topbar = QHBoxLayout()
        self.btn_import = QPushButton("导入动作数据")
        self.btn_save = QPushButton("保存动作数据")
        self.btn_clear = QPushButton("清空")
        self.btn_del = QPushButton("删除选中")
        self.btn_up = QPushButton("上移")
        self.btn_down = QPushButton("下移")
        topbar.addWidget(self.btn_import)
        topbar.addWidget(self.btn_save)
        topbar.addSpacing(12)
        topbar.addWidget(self.btn_clear)
        topbar.addWidget(self.btn_del)
        topbar.addWidget(self.btn_up)
        topbar.addWidget(self.btn_down)
        topbar.addStretch(1)
        self.btn_loop = QPushButton("循环：关")
        self.btn_loop.setCheckable(True)
        self.btn_loop.setObjectName("SecondaryButton")
        self.btn_loop.toggled.connect(self.update_loop_style)
        topbar.addWidget(self.btn_loop)
        self.btn_play = QPushButton("播放动作")
        self.btn_play.setObjectName("PrimaryButton")
        self.btn_stop = QPushButton("停止播放")
        self.btn_stop.setObjectName("PrimaryButton")
        topbar.addWidget(self.btn_play)
        topbar.addWidget(self.btn_stop)
        root.addLayout(topbar)

        self.tip = QLabel("当前没有记录的动作序列")
        self.tip.setObjectName("FormLabel")
        self.tip.setAlignment(Qt.AlignCenter)

        self.table = QTableWidget(0, len(self.headers), self)
        self.table.setHorizontalHeaderLabels(self.headers)
        self.table.verticalHeader().setVisible(False)
        # 选中行为：整行 + 单选
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)

        # 点击选中时高亮编号
        self.table.itemSelectionChanged.connect(self.on_user_select_row)

        # 颜色与状态
        self._color_pick = QColor(255, 214, 102, 180)  # 选中编号的底色（琥珀色）
        self._color_play = QColor(124, 180, 255, 120)  # 播放高亮行（你已有接近的颜色）
        self._last_select = None  # 上一次用户选中的行

        try:
            self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        except Exception:
            pass
        try:
            self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        except Exception:
            pass

        root.addWidget(self.table, 1)
        root.addWidget(self.tip)

        # Signals
        self.btn_import.clicked.connect(self.on_import)
        self.btn_save.clicked.connect(self.on_save)
        self.btn_clear.clicked.connect(self.on_clear)
        self.btn_del.clicked.connect(self.on_delete)
        self.btn_up.clicked.connect(lambda: self.on_move(-1))
        self.btn_down.clicked.connect(lambda: self.on_move(+1))
        self.btn_play.clicked.connect(self.on_play)
        self.btn_stop.clicked.connect(self.on_toggle_play_stop)

        self._last_hl = None
        self.refresh_empty_tip()

    def on_user_select_row(self):
        """用户点击选择某行：高亮该行的‘编号’单元格，并加粗字体。"""
        # 清除上一个选择的编号高亮（若不是当前正在播放的行）
        if self._last_select is not None and 0 <= self._last_select < self.table.rowCount():
            if self._last_select != getattr(self, "_last_hl", None):  # 避免覆盖播放高亮
                it_prev = self.table.item(self._last_select, 0)
                if it_prev:
                    it_prev.setBackground(QBrush())
                    f = it_prev.font()
                    f.setBold(False)
                    it_prev.setFont(f)

        # 新的选择
        sel = self.table.selectionModel().selectedRows()
        if not sel:
            self._last_select = None
            return
        r = sel[0].row()
        self._last_select = r

        it = self.table.item(r, 0)
        if it:
            it.setBackground(QBrush(self._color_pick))
            f = it.font()
            f.setBold(True)
            it.setFont(f)

    def update_loop_style(self, checked):
        try:
            self.btn_loop.setText("循环：开" if checked else "循环：关")
            self.btn_loop.setObjectName("PrimaryButton" if checked else "SecondaryButton")
            self.btn_loop.style().unpolish(self.btn_loop)
            self.btn_loop.style().polish(self.btn_loop)
        except Exception:
            pass
        self.refresh_row_numbers()

    def refresh_row_numbers(self):
        rows = self.table.rowCount()
        for r in range(rows):
            it = self.table.item(r, 0)
            if it is None:
                it = QTableWidgetItem(str(r))
                self.table.setItem(r, 0, it)
            it.setText(str(r))
            it.setTextAlignment(Qt.AlignCenter)
            it.setFlags(it.flags() & ~Qt.ItemIsEditable)

    def refresh_empty_tip(self):
        empty = (self.table.rowCount() == 0)
        self.tip.setVisible(empty)

    def set_frames(self, frames):
        self.table.setRowCount(0)
        for fr in frames:
            self.append_frame(fr, select=False)
        self.refresh_empty_tip()
        self.refresh_row_numbers()

    def get_frames(self):
        frames = []
        rows = self.table.rowCount()
        for r in range(rows):
            vals = []
            for c in range(1, 13):
                it = self.table.item(r, c)
                try:
                    vals.append(float(it.text().strip()) if it else 0.0)
                except Exception:
                    vals.append(0.0)
            it_delay = self.table.item(r, 13)
            try:
                dms = int(float(it_delay.text().strip())) if it_delay else 0
            except Exception:
                dms = 0
            frames.append({"pos": vals[:6], "spd": vals[6:12], "delay_ms": max(1, dms)})
        return frames

    def append_frame(self, frame, select=True):
        r = self.table.rowCount()
        self.table.insertRow(r)

        pos = frame.get("pos", [0.0] * 6)
        spd = frame.get("spd", [0.0] * 6)
        dms = int(frame.get("delay_ms", 10))

        # —— 编号列（第0列） ——
        idx_item = QTableWidgetItem(str(r))
        idx_item.setTextAlignment(Qt.AlignCenter)
        idx_item.setFlags(idx_item.flags() & ~Qt.ItemIsEditable)
        self.table.setItem(r, 0, idx_item)

        # —— 位置/速度（从第1列开始写入，共12列） ——
        data = [*pos, *spd]
        for c in range(12):
            it = QTableWidgetItem(f"{float(data[c]):.3f}")
            it.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self.table.setItem(r, c + 1, it)  # ← 关键修复：c+1

        # —— 延时（放在第13列） ——
        it = QTableWidgetItem(str(dms))
        it.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.table.setItem(r, 13, it)

        if select:
            self.table.selectRow(r)
        self.refresh_empty_tip()
        self.refresh_row_numbers()

    def highlight_row(self, idx):
        # 清除旧高亮
        if self._last_hl is not None and 0 <= self._last_hl < self.table.rowCount():
            for c in range(self.table.columnCount()):
                it = self.table.item(self._last_hl, c)
                if it:
                    it.setBackground(QBrush())  # reset
        # 设置新高亮
        if 0 <= idx < self.table.rowCount():
            for c in range(self.table.columnCount()):
                it = self.table.item(idx, c)
                if it:
                    it.setBackground(QBrush(QColor(124, 180, 255, 80)))
            self.table.selectRow(idx)
            self._last_hl = idx
        else:
            self._last_hl = None

    def clear_highlight(self):
        if self._last_hl is not None and 0 <= self._last_hl < self.table.rowCount():
            for c in range(self.table.columnCount()):
                it = self.table.item(self._last_hl, c)
                if it:
                    it.setBackground(QBrush())
        self._last_hl = None

    def on_import(self):
        fn, _ = QFileDialog.getOpenFileName(self, "导入动作数据", "", "Action Text (*.txt);;All Files (*)")
        if not fn:
            return
        try:
            with open(fn, "r", encoding="utf-8") as f:
                txt = f.read()

            # 兼容 Windows/Mac/Linux 的换行 以及 文件里写成字面量的 "\n"
            import re as _re
            raw_lines = _re.split(r'(?:\r\n|\r|\n|\\n)', txt)

            # 去掉空行，并处理 BOM
            lines = []
            for s in raw_lines:
                if not s:
                    continue
                s = s.strip()
                if not s:
                    continue
                if s and s[0] == "\ufeff":  # BOM
                    s = s.lstrip("\ufeff")
                lines.append(s)

            if not lines:
                QMessageBox.warning(self, "格式错误", "文件为空")
                return

            # 头行校验
            head = lines[0]
            if head != "LHP-ACTSEQ v1":
                QMessageBox.warning(self, "格式错误", "文件格式错误")
                return

            # 逐行解析
            frames = []
            for s in lines[1:]:
                if not s or s.startswith("#"):
                    continue
                parts = [p.strip() for p in s.split(";")]
                if len(parts) != 4:
                    continue
                pos_str = parts[1].split("=", 1)[-1]
                spd_str = parts[2].split("=", 1)[-1]
                dly_str = parts[3].split("=", 1)[-1]

                try:
                    pos = [float(x.strip()) for x in pos_str.split(",")]
                    spd = [float(x.strip()) for x in spd_str.split(",")]
                    dms = int(float(dly_str.strip()))
                except Exception:
                    continue

                if len(pos) != 6 or len(spd) != 6:
                    continue

                frames.append({"pos": pos, "spd": spd, "delay_ms": max(1, dms)})

            # 刷新到表格与主窗口缓存
            self.set_frames(frames)
            if self.main is not None:
                self.main.action_seq = frames[:]

        except Exception as e:
            QMessageBox.critical(self, "导入失败", f"{e}")

    def on_save(self):
        fn, _ = QFileDialog.getSaveFileName(self, "保存动作数据", "action_seq.txt",
                                            "Action Text (*.txt);;All Files (*)")
        if not fn:
            return
        try:
            frames = self.get_frames()
            with open(fn, "w", encoding="utf-8") as f:
                f.write("LHP-ACTSEQ v1\\n")
                for i, fr in enumerate(frames):
                    pos = ",".join(f"{v:.3f}" for v in fr["pos"])
                    spd = ",".join(f"{v:.3f}" for v in fr["spd"])
                    dms = int(fr["delay_ms"])
                    f.write(f"{i};pos={pos};spd={spd};delay={dms}\\n")
            QMessageBox.information(self, "已保存", f"共 {len(frames)} 条记录")
        except Exception as e:
            QMessageBox.critical(self, "保存失败", f"{e}")

    def on_clear(self):
        self.table.setRowCount(0)
        self.refresh_empty_tip()
        self.refresh_row_numbers()
        if self.main is not None:
            self.main.action_seq = []

    def on_delete(self):
        rows = sorted(set(i.row() for i in self.table.selectedIndexes()))
        for r in reversed(rows):
            self.table.removeRow(r)
        self.refresh_empty_tip()
        self.refresh_row_numbers()

    def on_move(self, delta):
        rows = sorted(set(i.row() for i in self.table.selectedIndexes()))
        if len(rows) != 1:
            return
        r = rows[0]
        nr = r + delta
        if nr < 0 or nr >= self.table.rowCount():
            return
        for c in range(self.table.columnCount()):
            it_r = self.table.takeItem(r, c)
            it_nr = self.table.takeItem(nr, c)
            self.table.setItem(r, c, it_nr)
            self.table.setItem(nr, c, it_r)
        self.table.selectRow(nr)
        self.refresh_row_numbers()

    def on_play(self):
        frames = self.get_frames()  # 以当前表格为准
        if self.main is not None:
            self.main.start_action_play(frames, dialog=self)  # 永远从头
            self.update_toggle_label()  # 播放后按钮显示“停止播放”

    def on_stop(self):
        if self.main is not None:
            self.main.stop_action_play()

    def on_toggle_play_stop(self):
        """当下按钮名为‘停止播放’时 -> 暂停；为‘继续播放’时 -> 继续。"""
        if self.main is None:
            return
        if getattr(self.main, "action_playing", False):
            # 正在播 -> 暂停，并切到“继续播放”
            self.main.pause_action_play()
        elif getattr(self.main, "action_paused", False):
            # 已暂停 -> 继续，并切回“停止播放”
            self.main.resume_action_play(dialog=self)
        # 其它状态不动（例如自然播完）
        self.update_toggle_label()

    def update_toggle_label(self):
        """根据主窗口状态刷新停止按钮文案。"""
        playing = bool(getattr(self.main, "action_playing", False))
        paused = bool(getattr(self.main, "action_paused", False))
        if playing:
            self.btn_stop.setText("停止播放")
            self.btn_stop.setEnabled(True)
        elif paused:
            self.btn_stop.setText("继续播放")  # 仅手动停止（暂停）后才出现
            self.btn_stop.setEnabled(True)
        else:
            # 自然结束或未开始：回到“停止播放”字样，但可按需置灰
            self.btn_stop.setText("停止播放")
            # 你可以选择置灰：self.btn_stop.setEnabled(False)


class EmbeddedURDFView(QWidget):
    """
    用 urdf_test.FingerPoseViewer 作为子控件嵌入：
    - 隐藏 FingerPoseViewer 自己的控制面板与顶栏
    - 暴露 set_six_angles_deg(...) 接口来按 6 个 UI 值驱动模型
      (thumb joint1, thumb joint2, index j2, middle j2, ring j2, little j2)
    - 暴露 reset_camera()
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        if URDF_AVAILABLE:
            self.viewer = FingerPoseViewer()
            # 隐藏内部控制区/状态/顶部控件
            for attr in ("ctrl_area", "lab_status", "lab_path", "cb_ignore_limits", "btn_fit", "btn_reset_view"):
                w = getattr(self.viewer, attr, None)
                if w:
                    try:
                        w.setVisible(False)
                    except Exception:
                        pass
            layout.addWidget(self.viewer, 1)
            self.initial_pan = -0.01  # 向右平移的强度：相机距离的 10%，可改大/小
            QTimer.singleShot(280, self._apply_initial_pan)

            # === 初始缩放（>1 放大，<1 缩小，1 不变）===
            self.initial_zoom = 1.35  # 想更大就调更大，例如 1.8

            # 初始化完成后应用（等 urdf/渲染器就绪）
            QTimer.singleShot(250, self._apply_initial_zoom)

            # === 基准角（单位：度）===
            # 拇指 0/1/2 = 关节1/2/3；其余手指 1/2/3 = 第2/3/4关节
            self.base_pose_deg = {
                "thumb": {0: -17.0, 1: 11.0, 2: -8.0},
                "index": {0: 7.0, 1: -3.0, 2: -1.0, 3: 15.0},
                "middle": {0: -2.0, 1: -4.0, 2: -1.0, 3: 2.0},
                "ring": {0: -5.0, 1: 3.0, 2: 9.0, 3: -6.0},
                "little": {0: -16.0, 1: -1.0, 2: 2.0, 3: 10.0},
            }

            self._base_applied = False
            QTimer.singleShot(150, self._apply_base_pose)
        else:
            # 降级占位
            ph = QFrame()
            v = QVBoxLayout(ph)
            lab = QLabel("未检测到 urdf_test 或依赖，显示占位")
            lab.setAlignment(Qt.AlignCenter)
            v.addWidget(lab)
            layout.addWidget(ph, 1)
            self.viewer = None

        self._last_vals = None

    def _apply_initial_pan(self):
        if self.viewer is None:
            return
        ren = getattr(self.viewer, "ren", None) or getattr(self.viewer, "renderer", None)
        rw = getattr(self.viewer, "vtk", None)
        if ren is None:
            QTimer.singleShot(120, self._apply_initial_pan)
            return
        cam = ren.GetActiveCamera()

        # —— 计算屏幕“向右”的世界方向 = viewDir × viewUp ——（纯标准库向量运算）
        def v_add(a, b):
            return (a[0] + b[0], a[1] + b[1], a[2] + b[2])

        def v_mul(a, s):
            return (a[0] * s, a[1] * s, a[2] * s)

        def v_cross(a, b):
            return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])

        def v_norm(a):
            import math
            l = math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
            return (0.0, 0.0, 0.0) if l == 0 else (a[0] / l, a[1] / l, a[2] / l)

        pos = cam.GetPosition()
        focal = cam.GetFocalPoint()
        vup = cam.GetViewUp()
        vdir = cam.GetDirectionOfProjection()  # 从相机指向焦点的方向
        right = v_norm(v_cross(vdir, vup))
        k = float(self.initial_pan) * cam.GetDistance()
        shift = v_mul(right, k)

        cam.SetPosition(*v_add(pos, shift))
        cam.SetFocalPoint(*v_add(focal, shift))
        ren.ResetCameraClippingRange()
        if rw is not None:
            win = rw.GetRenderWindow() if hasattr(rw, "GetRenderWindow") else None
            if win is not None:
                win.Render()

    def _apply_initial_zoom(self):
        if self.viewer is None:
            return
        # 等待渲染器就绪
        ren = getattr(self.viewer, "ren", None) or getattr(self.viewer, "renderer", None)
        rw = getattr(self.viewer, "vtk", None)
        if ren is None:
            QTimer.singleShot(120, self._apply_initial_zoom)
            return
        cam = ren.GetActiveCamera()
        try:
            cam.Zoom(float(self.initial_zoom))  # 关键：设置初始可视尺寸
            ren.ResetCameraClippingRange()
            if rw is not None:
                win = rw.GetRenderWindow() if hasattr(rw, "GetRenderWindow") else None
                if win is not None:
                    win.Render()
        except Exception as e:
            print("[3D] 初始化缩放失败：", e)

    def _apply_base_pose(self):
        # 等待 urdf_test.FingerPoseViewer 完成加载
        if self.viewer is None or getattr(self.viewer, "robot", None) is None or not getattr(self.viewer, "chains", []):
            QTimer.singleShot(120, self._apply_base_pose)
            return
        try:
            for ci, chain in enumerate(self.viewer.chains):
                kind = self.viewer.chain_kind[ci] if ci < len(self.viewer.chain_kind) else "other"
                pose_map = self.base_pose_deg.get(kind, None)
                if not pose_map:
                    continue
                for ji, jn in enumerate(chain):
                    if ji not in pose_map:
                        continue
                    # 锁定的关节跳过
                    if ji in LOCK_JOINTS.get(kind, set()):
                        continue
                    deg = float(pose_map[ji])
                    rads_in = math.radians(deg)
                    val = -rads_in if ji in INVERT_JOINTS.get(kind, set()) else rads_in
                    applied = self.viewer._clamp_by_limit(jn, val)
                    self.viewer.current_cfg[jn] = applied
            self.viewer._update_fk_and_actors()
            # 立即刷新 VTK
            rw = getattr(self.viewer, "vtk", None)
            if rw is not None:
                win = rw.GetRenderWindow() if hasattr(rw, "GetRenderWindow") else None
                if win is not None:
                    win.Render()
            self._base_applied = True
        except Exception as e:
            print("[3D] 应用基准角失败：", e)

    def reset_camera(self):
        if self.viewer is not None:
            try:
                self.viewer.reset_camera()
                self._apply_initial_pan()
                # 重置后继续套用初始缩放
                ren = getattr(self.viewer, "ren", None) or getattr(self.viewer, "renderer", None)
                if ren is not None:
                    cam = ren.GetActiveCamera()
                    cam.Zoom(float(self.initial_zoom))
                    ren.ResetCameraClippingRange()
                rw = getattr(self.viewer, "vtk", None)
                if rw is not None:
                    win = rw.GetRenderWindow() if hasattr(rw, "GetRenderWindow") else None
                    if win is not None:
                        win.Render()

                # 关键：主动刷新 VTK
                rw = getattr(self.viewer, "vtk", None)
                if rw is not None:
                    win = rw.GetRenderWindow() if hasattr(rw, "GetRenderWindow") else None
                    if win is not None:
                        win.Render()
            except Exception as e:
                print("[3D] 重置视图渲染失败：", e)

    def set_six_angles_deg(self, thumb1, thumb2, idx2, mid2, ring2, lit2):
        """由外部 6 个 UI 角度（度，视为Δ增量）在“基准角”之上驱动 3D。
        新增：所有手指的“第三关节”= 基准第三关节 +（第二关节的Δ增量）
        """
        if self.viewer is None:
            return
        if getattr(self.viewer, "robot", None) is None or not getattr(self.viewer, "chains", []):
            self._last_vals = (thumb1, thumb2, idx2, mid2, ring2, lit2)
            QTimer.singleShot(100, lambda: self._retry_apply())
            return

        # 现在：第1个滑条控制拇指“关节2”，第2个滑条控制拇指“关节1”
        delta_map = {
            "thumb": {0: float(thumb2), 1: float(thumb1)},  # ← 互换 thumb1/thumb2
            "index": {1: float(idx2)},
            "middle": {1: float(mid2)},
            "ring": {1: float(ring2)},
            "little": {1: float(lit2)},
        }

        try:
            for ci, chain in enumerate(self.viewer.chains):
                kind = self.viewer.chain_kind[ci] if ci < len(self.viewer.chain_kind) else "other"
                if kind not in delta_map:
                    continue
                base = self.base_pose_deg.get(kind, {})
                inv_set = INVERT_JOINTS.get(kind, set())
                lock_set = LOCK_JOINTS.get(kind, set())

                # 供“第三关节跟随”使用的 Δ(second)
                delta_second = delta_map[kind].get(1, None)  # 非拇指：链索引1；拇指同样是索引1

                for ji, jn in enumerate(chain):
                    if ji in lock_set:
                        continue

                    apply_deg = None
                    if ji in delta_map[kind]:
                        # 受控关节：基准 + 自己的Δ
                        apply_deg = float(base.get(ji, 0.0)) + float(delta_map[kind][ji])
                    elif ji == 2 and delta_second is not None:
                        # 第三关节：基准 +（第二关节的Δ作为偏移量）
                        apply_deg = float(base.get(ji, 0.0)) + float(delta_second)
                    else:
                        continue  # 其它关节保持基准角（不改）

                    rads_in = math.radians(apply_deg)
                    val = -rads_in if ji in inv_set else rads_in
                    applied = self.viewer._clamp_by_limit(jn, val)
                    self.viewer.current_cfg[jn] = applied

            self.viewer._update_fk_and_actors()
        except Exception as e:
            print("[3D] 应用 UI Δ角失败：", e)

    def _retry_apply(self):
        if self._last_vals is None:
            return
        t1, t2, i2, m2, r2, l2 = self._last_vals
        self._last_vals = None
        self.set_six_angles_deg(t1, t2, i2, m2, r2, l2)


# =============== 通讯配置对话框 ===============
class CommConfigDialog(QDialog):
    def __init__(self, parent=None, main=None):
        super().__init__(parent)
        self.setWindowTitle("通讯设置")
        self.setModal(True)
        self.setMinimumWidth(420)
        self.main = main
        self.setStyleSheet(main.build_qss())

        layout = QVBoxLayout(self)
        layout.setSpacing(12)

        group_serial = QGroupBox("485 串口连接")
        group_serial.setObjectName("CardGroup")
        f1 = QFormLayout(group_serial)
        f1.setContentsMargins(14, 14, 14, 14)

        self.cmb_port = QComboBox()
        self.cmb_port.setObjectName("InputCombo")
        self.refresh_ports()
        if main and main.last_port_name:
            idx = self.cmb_port.findText(main.last_port_name)
            if idx >= 0:
                self.cmb_port.setCurrentIndex(idx)

        self.btn_refresh_port = QPushButton("刷新")
        self.btn_refresh_port.clicked.connect(self.refresh_ports)

        port_row = QHBoxLayout()
        port_row.addWidget(self.cmb_port, 1)
        port_row.addWidget(self.btn_refresh_port)
        port_row_w = QWidget()
        port_row_w.setLayout(port_row)

        self.cmb_baud = QComboBox()
        self.cmb_baud.setObjectName("InputCombo")
        for b in [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]:
            self.cmb_baud.addItem(str(b), b)
        if main and main.last_pc_baud:
            self.cmb_baud.setCurrentText(str(main.last_pc_baud))
        else:
            self.cmb_baud.setCurrentText("921600")

        # 当前连接使用的从机地址，可编辑
        self.spin_conn_addr = QSpinBox()
        self.spin_conn_addr.setRange(1, 247)
        self.spin_conn_addr.setObjectName("InputSpin")
        self.spin_conn_addr.setButtonSymbols(QAbstractSpinBox.NoButtons)
        if main is not None:
            self.spin_conn_addr.setValue(main.current_slave_addr)
        else:
            self.spin_conn_addr.setValue(0xC8)
        # 修改为可编辑后，数值变化时立刻更新当前从机地址
        self.spin_conn_addr.valueChanged.connect(self._on_conn_addr_changed)

        self.btn_open = QPushButton("连接" if not (main and main.serial.isOpen()) else "断开")
        self.btn_open.clicked.connect(self.toggle_serial)

        f1.addRow("串口号:", port_row_w)
        # 这里改名
        f1.addRow("波特率:", self.cmb_baud)
        f1.addRow("从机地址:", self.spin_conn_addr)
        f1.addRow("", self.btn_open)

        # 如果当前主串口已经处于连接状态（可能是通过快速连接等方式），
        # 则在 485 串口连接区域中禁用端口、波特率和从机地址的修改，
        # 以保证“无论从哪串口连接上的时候”配置都不可更改。
        if main is not None and main.serial.isOpen():
            self.cmb_port.setEnabled(False)
            self.btn_refresh_port.setEnabled(False)
            self.cmb_baud.setEnabled(False)
            if hasattr(self, "spin_conn_addr"):
                self.spin_conn_addr.setEnabled(False)
            # 按钮文案与状态保持一致
            self.btn_open.setText("断开")

        layout.addWidget(group_serial)
        group_dev = QGroupBox("从机 Modbus 设置")
        group_dev.setObjectName("CardGroup")
        f2 = QFormLayout(group_dev)
        f2.setContentsMargins(14, 14, 14, 14)

        self.spin_addr = QSpinBox()
        self.spin_addr.setRange(1, 247)
        if main:
            self.spin_addr.setValue(main.current_slave_addr)
        self.spin_addr.setObjectName("InputSpin")
        self.spin_addr.setButtonSymbols(QAbstractSpinBox.NoButtons)

        # 地址输入 1-247，并在括号中显示对应的十六进制
        self.lab_addr_hex = QLabel()
        self._update_addr_hex_label(self.spin_addr.value())
        self.spin_addr.valueChanged.connect(self._on_spin_addr_changed)

        self.cmb_slave_baud = QComboBox()
        self.cmb_slave_baud.setObjectName("InputCombo")
        baud_map = [
            ("9600", 1), ("19200", 2), ("38400", 3), ("57600", 4),
            ("115200", 5), ("230400", 6), ("460800", 7), ("921600", 8),
        ]
        for text, code in baud_map:
            self.cmb_slave_baud.addItem(text, code)
        if main:
            for i in range(self.cmb_slave_baud.count()):
                if self.cmb_slave_baud.itemData(i) == main.current_baud_code:
                    self.cmb_slave_baud.setCurrentIndex(i)
                    break

        btn_row = QHBoxLayout()
        self.btn_write_addr = QPushButton("写地址")
        self.btn_write_addr.clicked.connect(self.write_addr)
        self.btn_write_baud = QPushButton("写波特率")
        self.btn_write_baud.clicked.connect(self.write_baud)
        btn_row.addWidget(self.btn_write_addr)
        btn_row.addWidget(self.btn_write_baud)

        self.btn_broadcast = QPushButton("广播读地址+波特率")
        self.btn_broadcast.setObjectName("SecondaryButton")
        self.btn_broadcast.clicked.connect(self.broadcast_read)

        # 广播发现的从机地址和波特率显示
        self.lab_broadcast_addr = QLabel("—")
        self.lab_broadcast_baud = QLabel("—")

        addr_row = QHBoxLayout()
        addr_row.addWidget(self.spin_addr)
        addr_row.addWidget(self.lab_addr_hex)
        addr_row.addStretch(1)
        addr_row_w = QWidget()
        addr_row_w.setLayout(addr_row)

        f2.addRow("新地址:", addr_row_w)
        f2.addRow("新波特率:", self.cmb_slave_baud)
        f2.addRow(btn_row)
        f2.addRow(self.btn_broadcast)
        f2.addRow("从机地址:", self.lab_broadcast_addr)
        f2.addRow("从机波特率:", self.lab_broadcast_baud)
        layout.addWidget(group_dev)

        btn_close_row = QHBoxLayout()
        btn_close_row.addStretch(1)
        self.btn_close_dialog = QPushButton("关闭")
        self.btn_close_dialog.clicked.connect(self.close)
        btn_close_row.addWidget(self.btn_close_dialog)
        layout.addLayout(btn_close_row)
        layout.addStretch(1)

    def refresh_ports(self):
        cur = self.cmb_port.currentText()
        self.cmb_port.clear()
        for p in QSerialPortInfo.availablePorts():
            self.cmb_port.addItem(p.portName())
        if cur:
            idx = self.cmb_port.findText(cur)
            if idx >= 0:
                self.cmb_port.setCurrentIndex(idx)

    def toggle_serial(self):
        if not self.main:
            return
        serial = self.main.serial
        if serial.isOpen():
            # 断开连接
            serial.close()
            self.main.set_conn_status(False)
            self.main.update_top_info()
            self.btn_open.setText("连接")
            # 允许修改串口号、波特率和地址
            self.cmb_port.setEnabled(True)
            self.btn_refresh_port.setEnabled(True)
            self.cmb_baud.setEnabled(True)
            if hasattr(self, "spin_conn_addr"):
                self.spin_conn_addr.setEnabled(True)
        else:
            # 建立连接
            port_name = self.cmb_port.currentText()
            if not port_name:
                QMessageBox.warning(self, "提示", "没有可用串口")
                return
            baud = int(self.cmb_baud.currentText())
            serial.setPortName(port_name)
            serial.setBaudRate(baud)
            if serial.open(QSerialPort.ReadWrite):
                self.main.last_port_name = port_name
                self.main.last_pc_baud = baud
                self.main.set_conn_status(True)
                self.main.update_top_info()
                self.btn_open.setText("断开")
                # 连接成功后，不允许再修改串口号、波特率和地址
                self.cmb_port.setEnabled(False)
                self.btn_refresh_port.setEnabled(False)
                self.cmb_baud.setEnabled(False)
                if hasattr(self, "spin_conn_addr"):
                    self.spin_conn_addr.setEnabled(False)
            else:
                QMessageBox.critical(self, "错误", "串口打开失败")

    def build_write06_ex(self, slave_addr: int, reg_addr: int, value_low: int) -> bytes:
        frame = bytearray()
        frame.append(slave_addr & 0xFF)
        frame.append(0x06)
        frame += struct.pack(">H", reg_addr)
        frame += struct.pack(">H", 0x0001)
        frame.append(0x02)
        frame.append(0x00)
        frame.append(value_low & 0xFF)
        crc = modbus_crc16(bytes(frame))
        frame += crc
        return bytes(frame)

    def write_addr(self):
        if not self.main:
            return
        new_addr = self.spin_addr.value()
        frame = self.build_write06_ex(self.main.current_slave_addr, 0x0000, new_addr)
        self.main.enqueue_frame(frame, desc=f"write addr=0x{new_addr:02X}", expect="cfg", priority=1)
        self.main.current_slave_addr = new_addr
        self.main.update_top_info()
        self.spin_addr.setValue(new_addr)
        # 同步更新 485 串口连接区域中的从机地址显示
        self._update_conn_addr_label(new_addr)

    def write_baud(self):
        if not self.main:
            return
        code = self.cmb_slave_baud.currentData()
        frame = self.build_write06_ex(self.main.current_slave_addr, 0x0001, code)
        self.main.enqueue_frame(frame, desc=f"write baud_code={code}", expect="cfg", priority=1)
        self.main.current_baud_code = code
        self.main.update_top_info()

    def _update_addr_hex_label(self, val: int):
        if hasattr(self, "lab_addr_hex") and self.lab_addr_hex is not None:
            self.lab_addr_hex.setText(f"(0x{val:02X})")

    def _update_conn_addr_label(self, addr: int):
        """更新 485 串口连接区域中显示的从机地址输入框"""
        if hasattr(self, "spin_conn_addr") and self.spin_conn_addr is not None:
            block = self.spin_conn_addr.blockSignals(True)
            try:
                self.spin_conn_addr.setValue(addr)
            finally:
                self.spin_conn_addr.blockSignals(block)

    def _on_conn_addr_changed(self, val: int):
        """当 485 串口连接区域中的从机地址被手动修改时的回调"""
        if self.main:
            self.main.current_slave_addr = val
            # 主界面顶部的地址显示也要同步
            self.main.update_top_info()

    def _on_spin_addr_changed(self, val: int):
        self._update_addr_hex_label(val)

    def show_broadcast_result(self, slave_addr: int, baud_code: int):
        """在对话框中显示广播返回的从机地址和波特率"""
        # 地址显示为十进制 + 十六进制
        self.lab_broadcast_addr.setText(f"{slave_addr} (0x{slave_addr:02X})")
        # 同步更新 485 串口连接区域中的从机地址显示
        self._update_conn_addr_label(slave_addr)
        # 波特率根据编码转换
        baud_map = {
            1: 9600, 2: 19200, 3: 38400, 4: 57600,
            5: 115200, 6: 230400, 7: 460800, 8: 921600,
        }
        real_baud = baud_map.get(baud_code)
        if real_baud is not None:
            self.lab_broadcast_baud.setText(f"{real_baud} (code={baud_code})")
        else:
            self.lab_broadcast_baud.setText(f"未知 (code=0x{baud_code:02X})")

    def broadcast_read(self):
        if not self.main:
            return
        frame = struct.pack(">B B H H", 0x00, 0x03, 0x0000, 0x0002)
        crc = modbus_crc16(frame)
        self.main.enqueue_frame(frame + crc, desc="broadcast read addr+baud", expect="broadcast", priority=1)


class OTAUpgradeDialog(QDialog):
    """
    OTA 升级弹窗：
    - 选择加密固件文件
    - 一键升级（发送升级请求 -> 发送长度帧 -> 发送内容帧(128B对齐) -> 请求校验）
    - 日志输出
    协议要点：
      * 升级请求：Modbus 0x10 到起始地址 0x0100，寄存器数量 0x0004，字节数 0x04，数据 12 34 56 78。
        必须先收到 C8 10 01 00 00 04 04 12 34 56 78 57 F2，再收到 C8 10 01 00 00 04 04 87 65 34 21 C2 B5，才继续。
      * OTA 长度帧：AA F4 04 [len(4B,BE)] [CRC16(LE)] 55（len = 128字节对齐后的补齐长度），正确回复为“原帧返回”。
      * OTA 内容帧：AA F5 86 [frame_no(2B,BE)] [128B数据] [CRC32(4B,BE)] [CRC16(LE)] 55。
        - 数据不足 128 字节时末帧补 0x00 到 128 字节；
        - CRC32 对 128B 数据区按给定算法逐字节累计；
        - CRC16 从帧头 AA 到 CRC32 末字节（含）计算，追加时使用 Modbus 小端两字节。
      * 请求校验帧：AA F6 04 FF FF FF FF [CRC16(LE)] 55；正确回复固定：AA F6 04 12 34 56 78 .. .. 55
      * 错误帧：EA F9 04 A5 A5 A5 [err] [CRC16] 55
        - err ∈ [0x00..0x0B]：帧错误 -> 重发当前内容帧
        - err ∈ [0x10..]   ：Flash 错误 -> 重新开始 OTA（长度帧->内容帧->校验）
    """

    def __init__(self, parent=None, main=None):
        super().__init__(parent)
        self.setWindowTitle("OTA 升级")
        self.setModal(True)
        self.setMinimumWidth(640)
        self.main = main
        if main:
            self.setStyleSheet(main.build_qss())

        lay = QVBoxLayout(self)

        # === 外层分组：一个框把所有内容框起来 ===
        sec_all = QGroupBox()  # ★ 外层大框标题可改
        all_lay = QVBoxLayout(sec_all)

        # 文件选择（分区）
        sec_file = QGroupBox("固件文件")
        # 如果不想内层再有边框，可取消下一行的注释：
        # sec_file.setFlat(True)                    # ★ 让内层分区“无边框”
        row = QHBoxLayout()
        self.lab_file = QLabel("未选择固件")
        try:
            self.lab_file.setTextInteractionFlags(Qt.TextSelectableByMouse)
        except Exception:
            pass
        self.btn_pick = QPushButton("选择文件…")
        self.btn_pick.clicked.connect(self.on_pick)
        row.addWidget(self.lab_file, 1)
        row.addWidget(self.btn_pick)
        sec_file.setLayout(row)
        all_lay.addWidget(sec_file)  # ★ 加到外层框里

        # 分隔线
        line1 = QFrame()
        line1.setFrameShape(QFrame.HLine)
        line1.setFrameShadow(QFrame.Sunken)
        all_lay.addWidget(line1)  # ★ 加到外层框里

        # 升级控制（分区）
        sec_ctrl = QGroupBox("固件升级")
        # sec_ctrl.setFlat(True)                   # ★ 如果不想要内层边框就解开
        ctrl = QHBoxLayout()
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)

        self.btn_start = QPushButton("一键升级")
        self.btn_start.setObjectName("PrimaryButton")
        self.btn_start.clicked.connect(self.on_start)

        self.btn_stop = QPushButton("停止升级")
        self.btn_stop.setObjectName("SecondaryButton")
        self.btn_stop.setEnabled(False)
        self.btn_stop.clicked.connect(self.on_stop)

        self.btn_close = QPushButton("关闭")
        self.btn_close.clicked.connect(self.reject)

        ctrl.addWidget(self.progress, 1)
        ctrl.addWidget(self.btn_start)
        ctrl.addWidget(self.btn_stop)
        ctrl.addWidget(self.btn_close)

        sec_ctrl.setLayout(ctrl)
        all_lay.addWidget(sec_ctrl)  # ★ 加到外层框里

        # 分隔线
        line2 = QFrame()
        line2.setFrameShape(QFrame.HLine)
        line2.setFrameShadow(QFrame.Sunken)
        all_lay.addWidget(line2)  # ★ 加到外层框里

        # 日志（分区）
        sec_log = QGroupBox("日志输出")
        # sec_log.setFlat(True)                    # ★ 如果不想要内层边框就解开
        v = QVBoxLayout()
        self.log_edit = QTextEdit()
        self.log_edit.setReadOnly(True)
        self.log_edit.setObjectName("GlobalLog")
        self.log_edit.setMinimumHeight(220)
        v.addWidget(self.log_edit)
        sec_log.setLayout(v)
        all_lay.addWidget(sec_log, 1)  # ★ 加到外层框里

        # 最后把“外层大框”放进对话框主布局
        lay.addWidget(sec_all, 1)

        # 底部一行：当前固件版本（左下角小字）
        bottom = QHBoxLayout()
        self.lab_current_fw = QLabel()
        self.lab_current_fw.setObjectName("OtaVersionLabel")
        bottom.addWidget(self.lab_current_fw)
        bottom.addStretch(1)
        lay.addLayout(bottom)

        # 根据主窗口当前版本初始化显示
        self.sync_current_version_from_main()

        # 状态
        self.active = False
        self.expect = None
        self.upgrade_seq_pos = 0
        self.file_path = None
        self.decrypted_data = None
        self.padded_data = None
        self.data_frames = []
        self.cur_idx = 0  # 0-based index into frames
        self.cur_frame_send_retries = 0
        self.max_frame_retries = 5
        self.flash_restart_count = 0
        self.max_flash_restarts = 2
        self.upgrade_ack_bytes = []
        self.upgrade_seq_pos = 0  # 0:等待首包, 1:已收首包, 2:完成  # 收到的两帧相同应答
        self.len_frame_bytes = None

        self._ota_len_frame_tx: bytes | None = None  # 最近一次发送的“长度帧”（整帧字节）
        self._ota_rx_buf = bytearray()  # OTA 接收缓冲（用于匹配完整回显）

        # 固定资源路径（项目 assets 内）
        self.path_priv = resource_path("assets/second.txt")
        self.path_keypkt = resource_path("assets/direct.txt")

        # 仅设置对象名（用来精确选择，不影响布局/逻辑）
        self.setObjectName("OtaDlg")  # 作用域限定到这个对话框
        sec_log.setObjectName("OtaLog")

        # 只改“边框颜色”（不设置背景），外层与内层边框区分色
        self.setStyleSheet(self.styleSheet() + """
        QDialog#OtaDlg QGroupBox#OtaLog  { border: 1px solid #2B4D86; border-radius: 8px; padding-top: 8px; }        """)

    def sync_current_version_from_main(self):
        """从主窗口同步当前固件版本到左下角标签"""
        text = "当前固件版本：---"
        if self.main is not None and getattr(self.main, "current_fw_version", None):
            text = f"当前固件版本：{self.main.current_fw_version}"
        if hasattr(self, "lab_current_fw"):
            self.lab_current_fw.setText(text)

    # ---------- UI/日志 ----------
    def log(self, s: str):
        now = time.strftime("%H:%M:%S")
        self.log_edit.append(f"[{now}] {s}")

    def on_pick(self):
        fn, _ = QFileDialog.getOpenFileName(self, "选择固件", "",
                                            "Firmware Files (*.bin *.hex *.enc *.fw);;All Files (*)")
        if not fn:
            return
        self.file_path = fn
        self.lab_file.setText(fn)

    def on_start(self):
        if not self.main or not self.main.serial.isOpen():
            self.log("❌ 串口未连接，无法升级。")
            return
        if not self.file_path:
            self.log("❌ 请先选择固件文件。")
            return
        # 初始化状态
        self.active = True
        self.expect = "upgrade"
        self.upgrade_ack_bytes = []
        self.upgrade_seq_pos = 0  # 0:等待首包, 1:已收首包, 2:完成
        self.decrypted_data = None
        self.padded_data = None
        self.data_frames = []
        self.cur_idx = 0
        self.cur_frame_send_retries = 0
        self.flash_restart_count = 0
        self.progress.setValue(0)

        self.btn_start.setEnabled(False)
        self.btn_pick.setEnabled(False)
        self.btn_close.setEnabled(False)
        self.btn_stop.setEnabled(True)  # ← NEW: 开始后允许“停止升级”

        self.log("▶ 开始 OTA：发送升级请求 (Modbus 0x10 @0x0100)...")
        frame = self.build_upgrade_request_frame()
        self.main.enqueue_frame(frame, desc="[OTA] upgrade-req", expect="ota-upgrade", priority=1)

    # NEW: 用户主动停止升级
    def on_stop(self):
        if not self.active:
            self.log("ℹ 当前不在升级流程。")
            return
        # 停止本地状态机
        self.active = False
        self.expect = None
        self.upgrade_seq_pos = 0
        self.cur_idx = 0
        self.cur_frame_send_retries = 0
        self.flash_restart_count = 0

        # 恢复按钮
        self.btn_start.setEnabled(True)
        self.btn_pick.setEnabled(True)
        self.btn_close.setEnabled(True)
        self.btn_stop.setEnabled(False)

        # 清空主窗口中与 OTA 相关的待发项/占用
        if self.main:
            try:
                self.main.cancel_ota_pipeline()
                self.main.log("[OTA] 用户已停止升级，已清空 OTA 待发队列。")
            except Exception as e:
                self.log(f"⚠ 清理队列异常: {e}")

        self.log("⏹ 已停止升级（后续收到的 OTA 响应将被忽略）。")

    # ---------- 帧构造 ----------
    def build_upgrade_request_frame(self) -> bytes:
        addr = self.main.current_slave_addr if self.main else 0x01
        func = 0x10
        start_reg = 0x0100
        reg_cnt = 0x0002
        byte_cnt = 0x04
        payload = bytes([0x12, 0x34, 0x56, 0x78])
        head = struct.pack(">B B H H B", addr & 0xFF, func, start_reg, reg_cnt, byte_cnt)
        body = head + payload
        return body + modbus_crc16(body)

    def build_len_frame(self, total_len: int) -> bytes:
        # AA F4 04 [len32(BE)] [CRC16(LE)] 55
        b = bytearray()
        b.extend(b'\xAA\xF4')
        b.append(0x04)
        b.extend(struct.pack(">I", total_len))
        b.extend(modbus_crc16(bytes(b)))  # CRC16 over header..len
        b.append(0x55)
        return bytes(b)

    def build_data_frame(self, frame_no: int, chunk128: bytes, pre_crc: int = 0xFFFFFFFF) -> bytes:
        # AA F5 86 [no(2B,BE)] [128B] [CRC32(4B,BE)] [CRC16(LE)] 55
        buf = bytearray()
        buf.extend(b'\xAA\xF5')
        buf.append(0x86)
        buf.extend(struct.pack(">H", frame_no & 0xFFFF))
        if len(chunk128) != 128:
            raise ValueError("chunk 长度不是 128")
        c32 = crc32_accumulate(chunk128, pre_crc) & 0xFFFFFFFF
        buf.extend(chunk128)
        buf.extend(struct.pack(">I", c32))
        buf.extend(modbus_crc16(bytes(buf)))
        buf.append(0x55)
        return bytes(buf)

    def build_verify_frame(self) -> bytes:
        # AA F6 04 FF FF FF FF [CRC16] 55
        b = bytearray()
        b.extend(b'\xAA\xF6')
        b.append(0x04)
        b.extend(b'\xFF\xFF\xFF\xFF')
        b.extend(modbus_crc16(bytes(b)))
        b.append(0x55)
        return bytes(b)

    # ---------- 解密 ----------
    def decrypt_firmware(self, enc_path: str) -> bytes:
        # 1) 读私钥 & 解密 AES 密钥（assets/encrypted_key.bin, assets/rsa_private.pem）
        if not os.path.exists(self.path_priv) or not os.path.exists(self.path_keypkt):
            raise FileNotFoundError("缺少文件")
        with open(self.path_priv, "rb") as f:
            priv = RSA.import_key(f.read())
        with open(self.path_keypkt, "rb") as f:
            rsa_blob = f.read()
        aes_key = PKCS1_OAEP.new(priv).decrypt(rsa_blob)

        # 2) 读加密固件：nonce(16) + ciphertext + tag(16)
        with open(enc_path, "rb") as f:
            enc = f.read()
        if len(enc) < 32:
            raise ValueError("固件文件长度不足（<32B）")
        nonce = enc[:16]
        ciphertext = enc[16:-16]
        tag = enc[-16:]
        cipher = AES.new(aes_key, AES.MODE_GCM, nonce=nonce)
        plain = cipher.decrypt_and_verify(ciphertext, tag)
        return plain

    def _prepare_frames(self, data: bytes):
        total_len = len(data)
        pad_len = ((total_len + 127) // 128) * 128
        if pad_len != total_len:
            padded = data + b"\x00" * (pad_len - total_len)
        else:
            padded = data
        self.padded_data = padded
        self.data_frames = []
        seed = 0xFFFFFFFF
        for i in range(pad_len // 128):
            chunk = padded[i * 128:(i + 1) * 128]
            self.data_frames.append(self.build_data_frame(i + 1, chunk, seed))
            seed = crc32_accumulate(chunk, seed) & 0xFFFFFFFF
        self.len_frame_bytes = self.build_len_frame(pad_len)
        self.cur_idx = 0
        self.cur_frame_send_retries = 0
        self.progress.setValue(0)

    # ---------- 发送流程 ----------
    def _send_len(self):
        self.expect = "len"
        self.log("→ 发送文件长度帧 ...")
        self.main.enqueue_frame(self.len_frame_bytes, desc="[OTA] len", expect="ota-len", priority=1)

    def _send_next_data(self):
        if not self.active:
            return
        if self.cur_idx >= len(self.data_frames):
            # 全部发送完毕 -> 请求校验
            self._send_verify()
            return

        self.expect = "data"  # 过程中随时可能收到错误帧
        frame = self.data_frames[self.cur_idx]
        # 无确认的内容帧：使用 ota-noack，主通道自动在极短延时后释放
        self.main.enqueue_frame(frame, desc=f"[OTA] data {self.cur_idx + 1}/{len(self.data_frames)}",
                                expect="ota-data", priority=1)

    def _after_data_sent(self):
        # 改为请求-应答模式：不再在这里推进索引/发送下一帧。
        return

    def _send_verify(self):
        self.expect = "verify"
        self.log("→ 发送请求校验帧 ...")
        vf = self.build_verify_frame()
        self.main.enqueue_frame(vf, desc="[OTA] verify", expect="ota-verify", priority=1)

    # ---------- RX & 超时 ----------
    def handle_serial_data(self, raw: bytes) -> bool:
        """返回 True 表示已消费（由 OTA 处理），不再让主逻辑解析。"""
        if not self.active:
            return False

        # 错误帧（优先判断） EA F9 04 A5 A5 A5 [err] [crc16] 55
        if len(raw) >= 9 and raw[0] == 0xEA and raw[1] == 0xF9 and raw[2] == 0x04 and raw[3:6] == b"\xA5\xA5\xA5":
            if len(raw) >= 10:
                err = raw[6]
                self.log(f"⚠ 错误帧：0x{err:02X}")
                if err <= 0x0B:
                    # 帧错误：重发当前内容帧
                    self._resend_current_frame()
                else:
                    # Flash 错误：重启流程
                    self._restart_ota()
            return True

        # 升级请求：应答序列必须严格为两帧固定内容，但地址和 CRC 跟随当前从机地址变化：
        #  1) addr 10 01 00 00 02 04 12 34 56 78 crc
        #  2) addr 10 01 00 00 02 04 87 65 43 21 crc
        if self.expect == "upgrade":
            if self.upgrade_seq_pos == 0 and self._match_upgrade_reply_payload(raw, b"\x12\x34\x56\x78"):
                self.upgrade_seq_pos = 1
                self.log("✔ 收到升级应答 1/2")
                return True
            if self.upgrade_seq_pos == 1 and self._match_upgrade_reply_payload(raw, b"\x87\x65\x43\x21"):
                self.upgrade_seq_pos = 2
                self.log("✔ 收到升级应答 2/2")
                try:
                    file_lower = (self.file_path or "").lower()
                    if file_lower.endswith(".enc"):
                        # 加密固件：按原来的流程先解密再烧录
                        self.log("正在处理固件 ...")
                        data = self.decrypt_firmware(self.file_path)
                        self.log(f"✔ 处理成功，固件大小 {len(data)} 字节")
                    elif file_lower.endswith(".bin"):
                        # 未加密固件：直接读取
                        self.log("正在处理固件 ...")
                        with open(self.file_path, "rb") as f:
                            data = f.read()
                        self.log(f"✔ 处理成功，固件大小 {len(data)} 字节")
                    else:
                        # 只允许 .bin / .enc，其他后缀直接报错
                        self.log("❌ 不支持的固件，请重新选择文件。")
                        self._done(False)
                        return True
                except Exception as e:
                    self.log(f"❌ 读取失败：{e}")
                    self._done(False)
                    return True
                self._prepare_frames(data)
                self._send_len()
                return True
            # 其它 0x10 回复一律忽略，直到命中上述两帧

        # 长度帧回显（原帧返回）
        if self.expect == "len":
            if raw == self.len_frame_bytes:
                self.log("✔ 长度帧回显一致，开始发送数据 ...")
                # 开始流水发送
                self._send_next_data()
                return True

        # 内容帧应答：AA F5 04 A5 A5 [no(2B,BE)] [CRC16(LE)] 55
        if self.expect == "data":
            if len(raw) >= 10 and raw[0] == 0xAA and raw[1] == 0xF5 and raw[2] == 0x04 and raw[3:5] == b"\xA5\xA5" and \
                    raw[-1] == 0x55:
                no = (raw[5] << 8) | raw[6]
                crc_recv = raw[7:9]
                crc_calc = modbus_crc16(raw[:7])
                exp_no = (self.cur_idx + 1) & 0xFFFF
                if crc_recv == crc_calc and no == exp_no:
                    self.log(f"✔ 内容帧应答 OK：#{no}")
                    pct = int((self.cur_idx + 1) * 100 / max(1, len(self.data_frames)))
                    self.progress.setValue(min(99, pct))
                    self.cur_idx += 1
                    self._send_next_data()
                    return True

        # 校验固定回复：AA F6 04 12 34 56 78 .. .. 55
        if self.expect == "verify":
            if len(raw) >= 9 and raw[0] == 0xAA and raw[1] == 0xF6 and raw[2] == 0x04 and raw[
                3:7] == b"\x12\x34\x56\x78":
                self.log("✔ 从机校验通过，OTA 完成！")
                self._done(True)
                return True

        # 其它情况允许主逻辑继续解析
        return False

    def _match_upgrade_reply_payload(self, raw: bytes, payload: bytes) -> bool:
        """匹配升级请求的扩展回包，地址/CRC 按当前从机动态计算。"""
        if len(raw) < 13 or len(payload) != 4:
            return False
        addr = self.main.current_slave_addr & 0xFF if self.main else raw[0]
        frame = raw[:13]
        expected_head = bytes([addr, 0x10, 0x01, 0x00, 0x00, 0x02, 0x04])
        if frame[:7] != expected_head or frame[7:11] != payload:
            return False
        return modbus_crc16(frame[:11]) == frame[11:13]

    def _parse_upgrade_reply(self, raw: bytes):
        """尝试解析 0x10 写多寄存器的两种回复：
           - 标准：8B  => addr 10 start(2) qty(2) CRC(2)
           - 扩展：13B => addr 10 start(2) qty(2) 04 data(4) CRC(2)
           返回一个签名 (addr, 0x10, 0x0100, 0x0004)；失败返回 None。
        """
        if len(raw) < 8:
            return None
        addr = self.main.current_slave_addr & 0xFF if self.main else raw[0]
        if raw[0] != addr or raw[1] != 0x10:
            return None
        # 头部匹配：起始地址 0x0100，数量 0x0004
        if not (raw[2] == 0x01 and raw[3] == 0x00 and raw[4] == 0x00 and raw[5] == 0x04):
            return None

        # 扩展回包：至少 13 字节且第 7 字节是 0x04；CRC 覆盖 raw[:-2]
        if len(raw) >= 13 and raw[6] == 0x04:
            if modbus_crc16(raw[:-2]) == raw[-2:]:
                return (addr, 0x10, 0x0100, 0x0004)

        # 标准回包：正好 8 字节；CRC 覆盖前 6 字节
        if len(raw) == 8 and modbus_crc16(raw[:6]) == raw[6:8]:
            return (addr, 0x10, 0x0100, 0x0004)

        return None

    def on_tx_timeout(self, kind: str):
        if not self.active:
            return
        if kind == "ota-upgrade":
            self.log("⏳ 升级请求等待超时，重发 ...")
            self.main.enqueue_frame(self.build_upgrade_request_frame(), desc="[OTA] upgrade-req(retry)",
                                    expect="ota-upgrade", priority=1)
        elif kind == "ota-len":
            self.log("⏳ 长度帧等待超时，重发 ...")
            self.main.enqueue_frame(self.len_frame_bytes, desc="[OTA] len(retry)", expect="ota-len", priority=1)
        elif kind == "ota-verify":
            self.log("⏳ 校验等待超时，重发 ...")
            self.main.enqueue_frame(self.build_verify_frame(), desc="[OTA] verify(retry)", expect="ota-verify",
                                    priority=1)
        elif kind == "ota-data":
            self.log("⏳ 内容帧等待应答超时，重发当前帧 ...")
            self._resend_current_frame()
        # ota-noack 不会触发这里

    def _resend_current_frame(self):
        if not self.active or self.expect not in ("data", "len"):
            return
        if self.expect == "len":
            self.cur_frame_send_retries += 1
            if self.cur_frame_send_retries > self.max_frame_retries:
                self.log("❌ 长度帧重试次数过多，放弃。")
                self._done(False)
                return
            self.log(f"↻ 重发长度帧 (retry {self.cur_frame_send_retries})")
            self.main.enqueue_frame(self.len_frame_bytes, desc="[OTA] len(retry)", expect="ota-len", priority=1)
            return
        # 内容帧
        self.cur_frame_send_retries += 1
        if self.cur_frame_send_retries > self.max_frame_retries:
            self.log("❌ 当前内容帧重试次数过多，放弃。")
            self._done(False)
            return
        idx = self.cur_idx
        if idx >= len(self.data_frames):
            idx = len(self.data_frames) - 1
        self.log(f"↻ 重发内容帧 #{idx + 1} (retry {self.cur_frame_send_retries})")
        self.main.enqueue_frame(self.data_frames[idx], desc=f"[OTA] data RESEND {idx + 1}", expect="ota-data",
                                priority=1)

    def _restart_ota(self):
        self.flash_restart_count += 1
        if self.flash_restart_count > self.max_flash_restarts:
            self.log("❌ Flash 错误重启次数过多，放弃。")
            self._done(False)
            return
        self.log("↻ FLASH 错误，重新开始 OTA 流程（从长度帧）...")
        # 回到长度帧（不需要再次升级请求）
        self.cur_idx = 0
        self.cur_frame_send_retries = 0
        self.progress.setValue(0)
        self._send_len()

    def _done(self, ok: bool):
        self.active = False
        self.expect = None
        self.upgrade_seq_pos = 0
        self.btn_start.setEnabled(True)
        self.btn_pick.setEnabled(True)
        self.btn_close.setEnabled(True)
        self.btn_stop.setEnabled(False)  # ← NEW: 结束时禁用停止
        self.progress.setValue(100 if ok else 0)
        if ok:
            self.log("OTA 升级成功。")
        else:
            self.log("❌ OTA 升级失败。")


class CircleMultiComboBox(QComboBox):
    """
    一个带“圆圈选中/未选中”图标的多选下拉框。
    第0项固定为“全部”，点击它会全选/全不选；其余为具体通道。
    - 文本前使用固定大小图标，避免选中时文字左右抖动
    - 文本颜色可按曲线颜色设置，保证和图例一致
    - 发出 selectionChanged() 信号
    """
    selectionChanged = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("InputCombo")
        self.setModel(QStandardItemModel(self))
        self.view().installEventFilter(self)
        # 让点击列表项不立即关闭弹窗
        self.view().viewport().installEventFilter(self)

        # 选中/未选中图标（固定 14x14，避免抖动）
        self._icon_on = self._make_icon(filled=True)
        self._icon_off = self._make_icon(filled=False)

        self._names = []  # ["全部", "拇指-弯曲", ...]
        self._colors = []  # [None, QColor(...), ...]
        self._checked = []  # 仅保存通道的选中状态（不含“全部”）

    def _make_icon(self, filled: bool) -> QIcon:
        pix = QPixmap(14, 14)
        pix.fill(Qt.transparent)
        p = QPainter(pix)
        p.setRenderHint(QPainter.Antialiasing)
        pen = QColor(156, 174, 255)  # 边框色（淡蓝）
        fill = QColor(124, 180, 255)  # 填充色（更亮一点）
        p.setPen(pen)
        if filled:
            p.setBrush(fill)
        p.drawEllipse(2, 2, 10, 10)
        p.end()
        return QIcon(pix)

    # names: ["全部", ...6个通道名]
    # colors: [None, QColor/str*6] —— 与曲线颜色一一对应（以图例为准）
    def set_items(self, names, colors):
        self._names = list(names)
        self._colors = []
        self.model().clear()
        for i, nm in enumerate(self._names):
            it = QStandardItem(nm)
            it.setEditable(False)
            it.setIcon(self._icon_on if i == 0 else self._icon_off)  # 初始：全部“开”，通道稍后统一设置
            # 颜色（从第1项起按曲线颜色上色）
            col = None
            if i < len(colors):
                c = colors[i]
                if isinstance(c, QColor):
                    col = c
                elif isinstance(c, str):
                    col = QColor(c)
            self._colors.append(col)
            if col is not None and i > 0:
                it.setData(QBrush(col), Qt.ForegroundRole)
            self.model().appendRow(it)

        # 默认全选所有通道
        n_ch = max(0, len(self._names) - 1)
        self._checked = [True] * n_ch
        self._refresh_icons()
        self._refresh_all_icon()

    def eventFilter(self, obj, ev):
        is_view = (obj is self.view()) or (obj is self.view().viewport())
        if is_view:
            # —— 鼠标：按下/释放/双击 都吞掉，避免 QComboBox 收起弹窗
            if ev.type() in (QEvent.MouseButtonPress, QEvent.MouseButtonRelease, QEvent.MouseButtonDblClick):
                if ev.type() == QEvent.MouseButtonPress:
                    # Qt6: 用 position().toPoint()，避免 DeprecationWarning
                    pt = ev.position().toPoint() if hasattr(ev, "position") else ev.pos()
                    idx = self.view().indexAt(pt)
                    if idx.isValid():
                        row = idx.row()
                        if row == 0:
                            all_on = all(self._checked) if self._checked else False
                            self.set_all(not all_on, emit_signal=True)
                        else:
                            self.set_item_checked(row - 1, not self._checked[row - 1], emit_signal=True)
                    ev.accept()
                # 无论 press/release/dblclick 都返回 True，阻止默认选择->收起 的行为
                return True

            # —— 键盘：空格/回车切换选中，但不收起
            if ev.type() == QEvent.KeyPress and hasattr(ev, "key"):
                if ev.key() in (Qt.Key_Space, Qt.Key_Return, Qt.Key_Enter):
                    idx = self.view().currentIndex()
                    if idx.isValid():
                        row = idx.row()
                        if row == 0:
                            all_on = all(self._checked) if self._checked else False
                            self.set_all(not all_on, emit_signal=True)
                        else:
                            self.set_item_checked(row - 1, not self._checked[row - 1], emit_signal=True)
                    ev.accept()
                    return True
        return super().eventFilter(obj, ev)

    def set_all(self, checked: bool, emit_signal: bool = True):
        for i in range(len(self._checked)):
            self._checked[i] = bool(checked)
        self._refresh_icons()
        self._refresh_all_icon()
        if emit_signal:
            self.selectionChanged.emit()

    def is_item_checked(self, idx: int) -> bool:
        if 0 <= idx < len(self._checked):
            return self._checked[idx]
        return False

    def set_item_checked(self, idx: int, checked: bool, *, update_all: bool = True, emit_signal: bool = True):
        if 0 <= idx < len(self._checked):
            self._checked[idx] = bool(checked)
            self._set_row_icon(idx + 1, self._icon_on if checked else self._icon_off)  # +1 跳过“全部”
            if update_all:
                self._refresh_all_icon()
            if emit_signal:
                self.selectionChanged.emit()

    def _set_row_icon(self, row: int, icon: QIcon):
        it = self.model().item(row)
        if it is not None:
            it.setIcon(icon)

    def _refresh_icons(self):
        for i, ok in enumerate(self._checked):
            self._set_row_icon(i + 1, self._icon_on if ok else self._icon_off)

    def _refresh_all_icon(self):
        all_on = all(self._checked) if self._checked else False
        self._set_row_icon(0, self._icon_on if all_on else self._icon_off)


class ModelDriveToggle(QWidget):
    """
    胶囊形状的两端切换控件：
    左侧：滑动条（False）
    右侧：灵巧手（True）
    modeChanged(bool) 信号对外通知
    """
    modeChanged = Signal(bool)  # True = 灵巧手（接收数据），False = 滑动条

    def __init__(self, parent=None):
        super().__init__(parent)
        self._value = 0.0  # 0 = 左侧，1 = 右侧，用于动画插值
        self._use_recv = False

        self.setFixedHeight(34)
        self.setMinimumWidth(190)

        # 阴影（整块胶囊浮起来）
        shadow = QGraphicsDropShadowEffect(self)
        shadow.setBlurRadius(20)
        shadow.setOffset(0, 3)
        shadow.setColor(QColor(0, 0, 0, 150))
        self.setGraphicsEffect(shadow)

        # 动画
        self._anim = QPropertyAnimation(self, b"animValue", self)
        self._anim.setDuration(180)
        self._anim.setEasingCurve(QEasingCurve.OutCubic)

    def sizeHint(self):
        return QSize(210, 34)

    # ========== 供 QPropertyAnimation 使用的属性 ==========
    def getAnimValue(self):
        return self._value

    def setAnimValue(self, v: float):
        self._value = v
        self.update()

    animValue = Property(float, getAnimValue, setAnimValue)

    # ========== 对外接口 ==========
    def setMode(self, use_recv: bool, animate: bool = True):
        """True = 灵巧手；False = 滑动条"""
        target = 1.0 if use_recv else 0.0
        self._use_recv = use_recv

        if animate:
            self._anim.stop()
            self._anim.setStartValue(self._value)
            self._anim.setEndValue(target)
            self._anim.start()
        else:
            self._value = target
            self.update()

        self.modeChanged.emit(self._use_recv)

    def mode(self) -> bool:
        return self._use_recv

    # ========== 交互 ==========
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            # Qt6 推荐用 position()，避免弃用警告
            x = event.position().x()
            left_half = x < self.width() / 2
            self.setMode(not left_half, animate=True)
        super().mousePressEvent(event)

    # ========== 绘制 ==========
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        rect = self.rect()

        # 外层胶囊
        radius = rect.height() / 2
        bg = QColor(255, 255, 255, 35)  # 外层淡白
        border = QColor(0, 0, 0, 110)
        painter.setPen(border)
        painter.setBrush(bg)
        painter.drawRoundedRect(rect.adjusted(0, 0, -1, -1), radius, radius)

        # 内部滑块（绿色胶囊）
        pad = 3
        slot_width = (rect.width() - pad * 2) / 2.0
        slot_height = rect.height() - pad * 2
        slider_radius = slot_height / 2

        x = pad + self._value * slot_width
        slider_rect = QRectF(x, pad, slot_width, slot_height)

        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(79, 217, 200))  # #4fd9c8 左右
        painter.drawRoundedRect(slider_rect, slider_radius, slider_radius)

        # 文本
        painter.setFont(self.font())
        fm = QFontMetrics(self.font())
        left_text = "滑动条"
        right_text = "灵巧手"

        left_rect = rect.adjusted(0, 0, -rect.width() // 2, 0)
        right_rect = rect.adjusted(rect.width() // 2, 0, 0, 0)

        # 根据滑块位置决定哪个是“激活”一侧（动画中间也跟着变化）
        active_left = self._value < 0.5

        # 左文字
        color_left = QColor(11, 16, 23) if active_left else QColor(230, 235, 255, 210)
        painter.setPen(color_left)
        painter.drawText(left_rect, Qt.AlignCenter, left_text)

        # 右文字
        color_right = QColor(11, 16, 23) if not active_left else QColor(230, 235, 255, 210)
        painter.setPen(color_right)
        painter.drawText(right_rect, Qt.AlignCenter, right_text)


# =============== 主窗口 ===============
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.ota_dialog = None
        self.comm_config_dialog = None
        self.setWindowTitle("灵巧手上位机 v1.1.0")
        self.setMinimumSize(1400, 690)
        self.setStyleSheet(self.build_qss())

        # 当前设备信息
        self.current_slave_addr = 0xC8
        self.current_baud_code = 0x08
        self.last_port_name = ""
        self.last_pc_baud = 115200
        self.current_fw_version = None

        # 3D 模型驱动来源：False=UI 滑动条/角度框，True=串口接收的 6 个角度
        self.use_recv_angles_for_model = False

        # 规划相关
        self.last_step_plan = None
        self.step_plan_dialog = None
        self.plan_exec_timer = QTimer(self)
        self.plan_exec_timer.setSingleShot(True)
        self.plan_exec_timer.timeout.connect(self.plan_exec_next)
        self.plan_executing = False
        self.plan_exec_index = 0
        self.plan_exec_loop_done = 0

        self.last_fist_test_plan = {
            "open_angles": [0.0] * 6,
            "close_angles": [0.0] * 6,
            "close_speeds": [50.0] * 6,
            "open_speeds": [50.0] * 6,
            "close_hold_ms": 2000,
            "open_hold_ms": 2000,
            "loop_count": 1,
        }
        self.fist_test_dialog = None
        self.fist_test_timer = QTimer(self)
        self.fist_test_timer.setSingleShot(True)
        self.fist_test_timer.timeout.connect(self.fist_test_next)
        self.fist_test_running = False
        self.fist_test_frames = []
        self.fist_test_index = 0
        self.fist_test_current_loop = 0
        self.fist_test_total_loop = int(self.last_fist_test_plan.get("loop_count", 1))
        # —— 动作序列相关 ——
        self.action_seq = []
        self.action_seq_dialog = None
        self.action_play_timer = QTimer(self)
        self.action_play_timer.setSingleShot(True)
        self.action_play_timer.timeout.connect(self.action_play_next)
        self.action_playing = False
        self.action_play_index = 0
        self.action_loop = False
        self.action_paused = False

        # 串口
        self.serial = QSerialPort(self)
        self.serial.errorOccurred.connect(self.on_serial_error)
        self.serial.readyRead.connect(self.on_serial_data)

        # 轮询
        self.monitor_running = False
        self.monitor_timer = QTimer(self)
        self.monitor_timer.setSingleShot(True)
        self.monitor_timer.timeout.connect(self.monitor_tick)
        self._poll_expect = None

        # 通讯测试期望
        self._comm_expect = None
        self._comm_active_btn = None

        self.modbus_error_map = {
            0x01: "非法功能码",
            0x02: "非法寄存器地址",
            0x03: "非法寄存器数量",
            0x04: "从机设备故障",
            0xE1: "帧过短/长度不合法",
            0xE2: "CRC 校验失败",
            0xE3: "从站地址不匹配",
        }

        # === 发送管道 ===
        self.tx_queue = []  # [{'frame':..., 'desc':..., 'expect':..., 'priority':...}, ...]
        self.tx_busy = False
        self.tx_expect = None
        self._current_tx_item = None
        self.tx_timeout_timer = QTimer(self)
        self.tx_timeout_timer.setSingleShot(True)
        self.tx_timeout_timer.timeout.connect(self.on_tx_timeout)

        # ===== UI 布局 =====
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # 顶部
        top = QHBoxLayout()
        top.setContentsMargins(16, 12, 16, 4)
        self.lab_addr = QLabel()
        self.lab_baud = QLabel()
        self.lab_conn = QLabel()
        self.lab_addr.setObjectName("TopTag")
        self.lab_baud.setObjectName("TopTag")
        self.lab_conn.setObjectName("ConnIndicatorDisconnected")
        self.update_top_info()

        self.btn_quick_connect = QPushButton("连接")
        self.btn_quick_connect.setFixedWidth(70)
        self.btn_quick_connect.clicked.connect(self.quick_connect)
        self.btn_quick_disconnect = QPushButton("断开")
        self.btn_quick_disconnect.setFixedWidth(70)
        self.btn_quick_disconnect.clicked.connect(self.quick_disconnect)

        btn_cfg = QPushButton("⚙ 通讯配置…")
        btn_cfg.clicked.connect(self.open_config)

        # 软复位按钮：放在 OTA 左边
        self.btn_soft_reset = QPushButton("程序复位")
        self.btn_soft_reset.setObjectName("SecondaryButton")
        self.btn_soft_reset.setFixedWidth(100)
        self.btn_soft_reset.clicked.connect(self.soft_reset_slave)

        # 程序复位冷却状态
        self._soft_reset_cooldown = False

        self.btn_ota = QPushButton("⬆ OTA升级…")
        self.btn_ota.setObjectName("SecondaryButton")
        self.btn_ota.clicked.connect(self.open_ota_dialog)

        # 固件版本按钮：视觉上当标签用
        self.btn_fw_version = QPushButton("固件版本：---")
        self.btn_fw_version.setObjectName("FwVersionTag")
        self.btn_fw_version.setFlat(True)  # 去掉立体效果
        self.btn_fw_version.setCursor(Qt.PointingHandCursor)
        self.btn_fw_version.clicked.connect(self.on_click_fw_version)

        top.addWidget(self.lab_addr)
        top.addWidget(self.lab_baud)
        top.addWidget(self.lab_conn)
        top.addSpacing(8)
        top.addWidget(self.btn_quick_connect)
        top.addWidget(self.btn_quick_disconnect)

        # 左侧内容之后先留一段弹性空间
        top.addStretch(1)

        # 中间的固件版本按钮（视觉上是标签）
        top.addWidget(self.btn_fw_version)

        top.addStretch(1)
        top.addWidget(self.btn_soft_reset)
        top.addWidget(self.btn_ota)
        top.addWidget(btn_cfg)
        main_layout.addLayout(top)

        # 中间三列
        center = QHBoxLayout()
        center.setContentsMargins(16, 4, 16, 4)
        center.setSpacing(10)

        finger_group = self.build_finger_panel_group()
        monitor_group = self.build_monitor_panel_group()
        model_group = self.build_right_panel_group()

        center.addWidget(finger_group, 1)
        center.addWidget(monitor_group, 1)
        center.addWidget(model_group, 1)

        main_layout.addLayout(center, 1)

        # 底部
        bottom = QHBoxLayout()
        bottom.setContentsMargins(16, 4, 16, 10)
        bottom.setSpacing(10)

        log_group = self.build_log_group()
        comm_group = self.build_comm_test_group()

        bottom.addWidget(log_group, 3)
        bottom.addWidget(comm_group, 1)

        main_layout.addLayout(bottom)
        self._restore_window()

    def _restore_window(self):
        """启动时恢复上次窗口几何和状态；首次运行则给出合理默认。"""
        s = QSettings()
        geo = s.value("MainWindow/geometry", type=QByteArray)
        state = s.value("MainWindow/windowState", type=QByteArray)
        maxim = s.value("MainWindow/isMaximized", False, bool)

        # 合理的保底：即便没有记录，也别小到挤出滚动条
        self.setMinimumSize(1400, 690)

        if geo is not None and len(geo) > 0:
            self.restoreGeometry(geo)
        if state is not None and len(state) > 0:
            self.restoreState(state)

        if maxim:
            self.showMaximized()
        else:
            # 首次运行没有记录？按你的偏好默认最大化
            if geo is None:
                self.showMaximized()

    def _save_window(self):
        """退出前保存窗口几何和状态。"""
        s = QSettings()
        s.setValue("MainWindow/isMaximized", self.isMaximized())
        # 只有在非最大化/非全屏时才更新几何，避免把“铺满屏”的矩形当成常态
        if not self.isMaximized() and not self.isFullScreen():
            s.setValue("MainWindow/geometry", self.saveGeometry())

    def closeEvent(self, e):
        """窗口关闭时触发保存。"""
        try:
            self._save_window()
        finally:
            super().closeEvent(e)

    def _curve_key(self, s: str) -> str:
        """把名字标准化（去空格、连字符，转小写），避免'拇指-弯曲' vs '拇指弯曲'不一致。"""
        return re.sub(r'[\s\-\–\—_]', '', str(s)).lower()

    def _legend_label_text(self, label) -> str:
        """尽量从 pyqtgraph 的 LabelItem 取到实际文字。"""
        # 常见：label.item 是 QGraphicsTextItem
        try:
            if hasattr(label, 'item') and hasattr(label.item, 'toPlainText'):
                return label.item.toPlainText()
        except Exception:
            pass
        # 兜底：有些版本提供 text 或 textItem
        try:
            if hasattr(label, 'text') and callable(label.text):
                return label.text()
        except Exception:
            pass
        try:
            if hasattr(label, 'textItem') and hasattr(label.textItem, 'toPlainText'):
                return label.textItem.toPlainText()
        except Exception:
            pass
        return ''

    # ========== 发送管道 ==========
    def enqueue_frame(self, frame: bytes, desc: str = "", expect: str = None, priority: int = 0):
        """
        所有串口发送都走这里
        priority: 1=高(规划/配置)，0=低(监控)
        """
        item = {
            "frame": frame,
            "desc": desc,
            "expect": expect,
            "priority": priority,
        }
        if priority > 0:
            # 插在第一个高优先级段末尾
            idx = 0
            while idx < len(self.tx_queue) and self.tx_queue[idx]["priority"] > 0:
                idx += 1
            self.tx_queue.insert(idx, item)
        else:
            self.tx_queue.append(item)
        self.try_send_next()

    def try_send_next(self):
        if self.tx_busy:
            return
        if not self.tx_queue:
            return
        item = self.tx_queue.pop(0)
        self._current_tx_item = item
        frame = item["frame"]
        desc = item["desc"]
        expect = item["expect"]

        hex_str = " ".join(f"{b:02X}" for b in frame)
        # OTA 升级过程中屏蔽发送帧在主日志中的显示，只保留接收帧
        mute_tx = isinstance(expect, str) and expect.startswith("ota-")
        if not mute_tx:
            if desc:
                self.log(f"--> {hex_str}    {desc}")
            else:
                self.log(f"--> {hex_str}")

        if self.serial.isOpen():
            self.serial.write(frame)
        else:
            self.log("[WARN] 串口未连接，已生成帧但未发送")

        self.tx_busy = True
        self.tx_expect = expect
        # 根据不同类型给不同超时
        timeout_ms = 50
        if expect and expect.startswith("poll"):
            timeout_ms = 60
        elif expect and expect.startswith("plan"):
            timeout_ms = 80
        elif expect and expect == "broadcast":
            timeout_ms = 80
        self.tx_timeout_timer.start(timeout_ms)

    def on_tx_timeout(self):
        # OTA 专用：不把无应答当错误，并让对话框决定是否重发
        if self.tx_expect and isinstance(self.tx_expect, str) and self.tx_expect.startswith('ota'):
            if self.ota_dialog is not None:
                try:
                    self.ota_dialog.on_tx_timeout(self.tx_expect)
                except Exception as e:
                    self.log_error(f"[OTA] 超时回调异常: {e}")
            # 不记录 TX 错误，直接释放
            self.tx_busy = False
            self.tx_expect = None
            self._current_tx_item = None
            self.try_send_next()
            return

        self.log_error("[TX] 等待从机应答超时")
        self.tx_busy = False
        self.tx_expect = None
        self._current_tx_item = None
        self.try_send_next()

    def release_bus_and_send_next(self):
        self.tx_timeout_timer.stop()
        self.tx_busy = False
        self.tx_expect = None
        self._current_tx_item = None
        self.try_send_next()

    def cancel_ota_pipeline(self):
        """停止 OTA 时清掉 OTA 待发项，并释放当前 OTA 占用。"""
        self.tx_queue = [
            item for item in self.tx_queue
            if not (
                (isinstance(item.get("expect"), str) and item["expect"].startswith("ota-"))
                or str(item.get("desc", "")).startswith("[OTA]")
            )
        ]
        current_expect = self.tx_expect
        current_desc = ""
        if self._current_tx_item:
            current_desc = str(self._current_tx_item.get("desc", ""))
        current_is_ota = (
            (isinstance(current_expect, str) and current_expect.startswith("ota-"))
            or current_desc.startswith("[OTA]")
        )
        if current_is_ota:
            self.tx_timeout_timer.stop()
            self.tx_busy = False
            self.tx_expect = None
            self._current_tx_item = None
        self.try_send_next()

    def has_high_priority_pending(self) -> bool:
        return any(item["priority"] > 0 for item in self.tx_queue)

    # ========== 接收步长规划 ==========
    def receive_step_plan(self, plan: dict):
        self.last_step_plan = plan
        total_frames = len(plan["frames"])
        loop_desc = "∞" if plan["loop_infinite"] else str(plan["loop_count"])
        self.log(
            f"[PLAN] 帧数:{total_frames}, 每帧:{plan['per_delay']}ms, 循环:{loop_desc}, 返程:{'是' if plan['with_return'] else '否'}")
        self.btn_exec_plan.setEnabled(True)

    # ========== 顶部 ==========
    def update_top_info(self):
        """更新主界面顶部的地址和波特率显示为“当前连接”的实际值"""
        if self.serial.isOpen():
            # 地址使用当前从机地址（Modbus 地址）
            self.lab_addr.setText(f"地址: 0x{self.current_slave_addr:02X}")
            # 波特率直接使用当前串口实际波特率
            try:
                pc_baud = int(self.serial.baudRate())
            except Exception:
                pc_baud = self.last_pc_baud
            self.lab_baud.setText(f"波特率: {pc_baud}")
            self.lab_conn.setText("● 已连接")
            self.lab_conn.setObjectName("ConnIndicatorConnected")
        else:
            # 未连接时不显示历史值，而是置为占位
            self.lab_addr.setText("地址: --")
            self.lab_baud.setText("波特率: --")
            self.lab_conn.setText("● 未连接")
            self.lab_conn.setObjectName("ConnIndicatorDisconnected")
        self.lab_conn.style().unpolish(self.lab_conn)
        self.lab_conn.style().polish(self.lab_conn)

    def set_conn_status(self, ok: bool):
        if ok:
            self.lab_conn.setText("● 已连接")
            self.lab_conn.setObjectName("ConnIndicatorConnected")
        else:
            self.lab_conn.setText("● 未连接")
            self.lab_conn.setObjectName("ConnIndicatorDisconnected")
            # 断开连接时清空固件版本显示
            self.current_fw_version = None
            if hasattr(self, "btn_fw_version"):
                self.btn_fw_version.setText("固件版本：---")
        self.lab_conn.style().unpolish(self.lab_conn)
        self.lab_conn.style().polish(self.lab_conn)

    def quick_connect(self):
        if self.serial.isOpen():
            self.log("[INFO] 已连接")
            return
        if not self.last_port_name:
            self.log_error("[ERR] 没有记住的串口，请先打开通讯配置连接一次")
            return
        self.serial.setPortName(self.last_port_name)
        self.serial.setBaudRate(self.last_pc_baud)
        if self.serial.open(QSerialPort.ReadWrite):
            self.set_conn_status(True)
            self.update_top_info()
            self.log(f"[INFO] 快速连接: {self.last_port_name} @ {self.last_pc_baud}")
            # 若通讯配置窗口存在，则同步其按钮和控件状态：
            # 无论从哪里建立的连接，485 串口连接区域中的端口/波特率/地址都不可修改
            if hasattr(self, "comm_config_dialog") and self.comm_config_dialog is not None:
                dlg = self.comm_config_dialog
                if hasattr(dlg, "btn_open"):
                    dlg.btn_open.setText("断开")
                if hasattr(dlg, "cmb_port"):
                    dlg.cmb_port.setEnabled(False)
                if hasattr(dlg, "btn_refresh_port"):
                    dlg.btn_refresh_port.setEnabled(False)
                if hasattr(dlg, "cmb_baud"):
                    dlg.cmb_baud.setEnabled(False)
                if hasattr(dlg, "spin_conn_addr"):
                    dlg.spin_conn_addr.setEnabled(False)
        else:
            self.log_error("[ERR] 快速连接失败")

    def quick_disconnect(self):
        if self.serial.isOpen():
            self.serial.close()
            self.set_conn_status(False)
            self.update_top_info()
            self.log("[INFO] 串口已断开")
            # 若通讯配置窗口存在，同步按钮和控件状态（恢复为可编辑）
            if hasattr(self, "comm_config_dialog") and self.comm_config_dialog is not None:
                dlg = self.comm_config_dialog
                if hasattr(dlg, "btn_open"):
                    dlg.btn_open.setText("连接")
                if hasattr(dlg, "cmb_port"):
                    dlg.cmb_port.setEnabled(True)
                if hasattr(dlg, "btn_refresh_port"):
                    dlg.btn_refresh_port.setEnabled(True)
                if hasattr(dlg, "cmb_baud"):
                    dlg.cmb_baud.setEnabled(True)
                if hasattr(dlg, "spin_conn_addr"):
                    dlg.spin_conn_addr.setEnabled(True)

    def on_click_fw_version(self):
        """点击“固件版本：---”按钮，请求下位机固件版本"""
        if not self.serial.isOpen():
            self.log_error("[FW] 请先连接串口，再获取固件版本")
            return

        # 使用已有的 0x03 打包函数，起始寄存器=0x0B00，数量=0x0005
        frame = self.build_read_holding_frame(0x0B00, 0x0005)
        # 高优先级发送，期望类型标记为 fw-version
        self.enqueue_frame(frame,
                           desc="[FW] 读取固件版本",
                           expect="fw-version",
                           priority=1)

    def open_config(self):
        dlg = CommConfigDialog(self, main=self)
        self.comm_config_dialog = dlg
        try:
            dlg.exec()
        finally:
            self.comm_config_dialog = None

    # ========== 1. 手指控制 ==========

    def open_ota_dialog(self):
        if self.ota_dialog is None:
            self.ota_dialog = OTAUpgradeDialog(self, main=self)
        # 每次打开前同步一下当前固件版本
        try:
            self.ota_dialog.sync_current_version_from_main()
        except Exception:
            pass
        self.ota_dialog.show()
        self.ota_dialog.raise_()
        self.ota_dialog.activateWindow()

    def build_finger_panel_group(self) -> QGroupBox:
        group = QGroupBox("✋ 手指控制")
        group.setObjectName("SectionGroup")
        lay = QVBoxLayout(group)
        lay.setContentsMargins(10, 12, 10, 2)
        lay.setSpacing(8)

        top_bar = QHBoxLayout()
        self.btn_open_fist_test = QPushButton("握拳测试")
        self.btn_open_fist_test.setObjectName("SecondaryButton")
        self.btn_open_fist_test.setFixedWidth(96)
        self.btn_open_fist_test.clicked.connect(self.open_fist_test_dialog)
        top_bar.addWidget(self.btn_open_fist_test)
        top_bar.addStretch(1)
        self.btn_open_action = QPushButton("动作序列")
        self.btn_open_action.setObjectName("SecondaryButton")
        self.btn_open_action.setFixedWidth(96)
        self.btn_open_action.clicked.connect(self.open_action_seq_dialog)

        self.btn_open_plan = QPushButton("运动规划")
        self.btn_open_plan.setObjectName("SecondaryButton")
        self.btn_open_plan.setFixedWidth(90)
        self.btn_open_plan.clicked.connect(self.open_step_plan_dialog)

        self.btn_exec_plan = QPushButton("执行规划")
        self.btn_exec_plan.setObjectName("PrimaryButton")
        self.btn_exec_plan.setFixedWidth(90)
        self.btn_exec_plan.setEnabled(True)  # 允许点击；无规划时在处理函数里提示
        self.btn_exec_plan.clicked.connect(self.start_execute_plan)

        self.btn_stop_plan = QPushButton("中止规划")
        self.btn_stop_plan.setObjectName("PrimaryButton")
        self.btn_stop_plan.setFixedWidth(90)
        self.btn_stop_plan.setEnabled(False)
        self.btn_stop_plan.clicked.connect(self.stop_execute_plan)

        top_bar.addWidget(self.btn_open_action)
        top_bar.addWidget(self.btn_open_plan)
        top_bar.addWidget(self.btn_exec_plan)
        top_bar.addWidget(self.btn_stop_plan)
        lay.addLayout(top_bar)

        # —— 中间的 6 行手指控件，放到一个可滚动区域里 ——
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)  # 只保留外层 group 的边框就行
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        inner = QWidget()
        v = QVBoxLayout(inner)
        v.setContentsMargins(6, 6, 6, 6)
        v.setSpacing(0)

        self.finger_angle_boxes = []
        self.finger_speed_boxes = []

        fingers = [
            ("拇指-弯曲", 43.5),
            ("拇指-侧摆", 86.0),
            ("食指", 80.0),
            ("中指", 80.0),
            ("无名指", 80.0),
            ("小指", 80.0),
        ]
        for idx, (name, ang_max) in enumerate(fingers):
            row = self.create_finger_row(idx, name, ang_max)
            # 这里可以改成 Preferred/Minimum，表示可以稍微压一点，但别太狠
            row.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Minimum)
            v.addWidget(row)

        # 把原来的 inner 塞进 scroll 里
        scroll.setWidget(inner)
        lay.addWidget(scroll, 1)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(6)
        self.btn_send_fingers = QPushButton("下发控制指令")
        self.btn_send_fingers.setObjectName("PrimaryButton")
        self.btn_send_fingers.clicked.connect(self.send_finger_motion)

        self.btn_record_action = QPushButton("记录动作")
        self.btn_record_action.setObjectName("SecondaryButton")
        self.btn_record_action.clicked.connect(self.record_action)

        # 重置：仅修改 UI，不下发指令
        self.btn_reset_ui = QPushButton("重置滑条")
        self.btn_reset_ui.setObjectName("SecondaryButton")
        self.btn_reset_ui.clicked.connect(self.reset_finger_ui)

        # 手指复位：不修改 UI，只下发角度=0、速度=50 的控制指令
        self.btn_reset_fingers = QPushButton("手指复位")
        self.btn_reset_fingers.setObjectName("SecondaryButton")
        self.btn_reset_fingers.clicked.connect(self.reset_finger_command)

        btn_row.addWidget(self.btn_record_action, 1)
        btn_row.addWidget(self.btn_send_fingers, 2)
        btn_row.addWidget(self.btn_reset_ui, 1)
        btn_row.addWidget(self.btn_reset_fingers, 1)
        lay.addLayout(btn_row)

        # —— 关键：将 6 个角度值变化同步到 3D 模型 ——
        for sp in self.finger_angle_boxes:
            sp.valueChanged.connect(self.sync_ui_to_model)

        return group

    def open_step_plan_dialog(self):
        if self.step_plan_dialog is None:
            self.step_plan_dialog = StepPlanDialog(self, main=self)

        # 每次打开前，把当前的规划参数同步过去
        if self.last_step_plan is not None:
            try:
                self.step_plan_dialog.load_from_plan(self.last_step_plan)
            except Exception:
                pass

        # 同步一下按钮状态（下面第 4 步会加 _update_plan_buttons）
        try:
            self._update_plan_buttons()
        except Exception:
            pass

        # 使用非模态方式打开，这样还能点击主窗口
        self.step_plan_dialog.show()
        self.step_plan_dialog.raise_()
        self.step_plan_dialog.activateWindow()

    def open_fist_test_dialog(self):
        if self.fist_test_dialog is None:
            self.fist_test_dialog = FistTestDialog(self, main=self)

        if self.last_fist_test_plan is not None:
            try:
                self.fist_test_dialog.load_from_plan(self.last_fist_test_plan)
            except Exception:
                pass

        self.fist_test_dialog.set_running(self.fist_test_running,
                                          "状态：运行中" if self.fist_test_running else "状态：待命")
        total_loop = max(1, int(self.last_fist_test_plan.get("loop_count", 1)))
        current_loop = self.fist_test_current_loop if self.fist_test_running else 0
        self.fist_test_dialog.set_loop_progress(current_loop, total_loop)
        self.fist_test_dialog.show()
        self.fist_test_dialog.raise_()
        self.fist_test_dialog.activateWindow()

    def apply_fist_test_initial_to_ui(self, plan: dict):
        self.last_fist_test_plan = dict(plan)
        if len(self.finger_angle_boxes) < 6 or len(self.finger_speed_boxes) < 6:
            return

        open_angles = list(plan.get("open_angles", [0.0] * 6))[:6]
        close_speeds = list(plan.get("close_speeds", [50.0] * 6))[:6]

        for i, sp in enumerate(self.finger_angle_boxes[:6]):
            sp.setValue(float(open_angles[i]))
        for i, sp in enumerate(self.finger_speed_boxes[:6]):
            sp.setValue(float(close_speeds[i]))

        self.log("[FIST] 已将握拳测试初始角度应用到左侧控制区")

    def build_fist_test_frames(self, plan: dict):
        open_angles = list(plan.get("open_angles", [0.0] * 6))[:6]
        close_angles = list(plan.get("close_angles", [0.0] * 6))[:6]
        close_speeds = list(plan.get("close_speeds", [50.0] * 6))[:6]
        open_speeds = list(plan.get("open_speeds", [50.0] * 6))[:6]
        close_hold_ms = max(0, int(plan.get("close_hold_ms", 2000)))
        open_hold_ms = max(0, int(plan.get("open_hold_ms", 2000)))
        loop_count = max(1, int(plan.get("loop_count", 1)))

        while len(open_angles) < 6:
            open_angles.append(0.0)
        while len(close_angles) < 6:
            close_angles.append(0.0)
        while len(close_speeds) < 6:
            close_speeds.append(50.0)
        while len(open_speeds) < 6:
            open_speeds.append(50.0)

        final_open = open_angles[:6]
        final_close = close_angles[:6]

        frames = [{
            "angles": final_open[:],
            "speeds": open_speeds[:],
            "delay_ms": open_hold_ms,
            "phase": "open",
            "label": "初始张开",
            "loop_no": 0,
        }]

        for idx in range(loop_count):
            loop_no = idx + 1
            frames.append({
                "angles": final_close[:],
                "speeds": close_speeds[:],
                "delay_ms": close_hold_ms,
                "phase": "close",
                "label": f"第{loop_no}次闭合",
                "loop_no": loop_no,
            })
            frames.append({
                "angles": final_open[:],
                "speeds": open_speeds[:],
                "delay_ms": open_hold_ms,
                "phase": "open",
                "label": f"第{loop_no}次张开",
                "loop_no": loop_no,
            })
        return frames

    def start_fist_test(self, plan: dict, dialog=None):
        if not self.serial.isOpen():
            QMessageBox.information(self, "提示", "请先连接串口，再开始握拳测试。")
            self.log("[FIST] 串口未连接，无法开始握拳测试")
            return
        if self.plan_executing:
            QMessageBox.information(self, "提示", "当前正在执行运动规划，请先中止规划。")
            return
        if self.action_playing or self.action_paused:
            QMessageBox.information(self, "提示", "当前正在播放动作序列，请先停止动作播放。")
            return

        self.last_fist_test_plan = dict(plan)
        self.fist_test_dialog = dialog or self.fist_test_dialog
        self.fist_test_frames = self.build_fist_test_frames(plan)
        self.fist_test_index = 0
        self.fist_test_running = True
        self.fist_test_current_loop = 0
        self.fist_test_total_loop = max(1, int(plan.get("loop_count", 1)))

        if self.fist_test_dialog is not None:
            self.fist_test_dialog.set_running(True, "状态：握拳测试运行中")
            self.fist_test_dialog.set_loop_progress(0, self.fist_test_total_loop)

        self.log(f"[FIST] 开始握拳测试，循环次数={self.fist_test_total_loop}")
        self.fist_test_next()

    def fist_test_next(self):
        if not self.fist_test_running:
            return
        if self.fist_test_index >= len(self.fist_test_frames):
            self.stop_fist_test(from_user=False)
            return

        frame = self.fist_test_frames[self.fist_test_index]
        self.send_joint_command(frame.get("angles", [0.0] * 6),
                                frame.get("speeds", [50.0] * 6),
                                expect="fist",
                                priority=1)

        delay_ms = max(1, int(frame.get("delay_ms", 10)))
        label = frame.get("label", f"步骤{self.fist_test_index + 1}")
        loop_no = max(0, int(frame.get("loop_no", 0)))
        self.fist_test_current_loop = loop_no
        self.log(f"[FIST] {label}，保持 {delay_ms} ms")

        if self.fist_test_dialog is not None:
            self.fist_test_dialog.set_running(True,
                                              f"状态：{label}（{self.fist_test_index + 1}/{len(self.fist_test_frames)}）")
            self.fist_test_dialog.set_loop_progress(loop_no, self.fist_test_total_loop)

        self.fist_test_index += 1
        self.fist_test_timer.start(delay_ms)

    def stop_fist_test(self, from_user: bool = True):
        self.fist_test_timer.stop()
        was_running = self.fist_test_running
        self.fist_test_running = False
        self.fist_test_index = 0
        self.fist_test_frames = []
        final_loop = self.fist_test_total_loop if (not from_user and was_running) else self.fist_test_current_loop
        self.fist_test_current_loop = 0

        if self.fist_test_dialog is not None:
            status = "状态：握拳测试已停止" if from_user and was_running else "状态：握拳测试已完成"
            self.fist_test_dialog.set_running(False, status)
            self.fist_test_dialog.set_loop_progress(final_loop, max(1, self.fist_test_total_loop))

        if was_running:
            self.log("[FIST] 握拳测试已停止" if from_user else "[FIST] 握拳测试已完成")

    def create_finger_row(self, idx: int, name: str, ang_max: float) -> QWidget:
        """
        单行指位控制：
        - 标签与数字框内文字按手指数颜色区分
        - 小指行：更紧凑、更小字号，避免在笔记本上被遮挡
        - 其他行为保持不变
        """
        finger_colors = {
            "拇指-弯曲": "#5DA3FF",
            "拇指-侧摆": "#8BE9FD",
            "拇指": "#5DA3FF",
            "食指": "#50FA7B",
            "中指": "#F1FA8C",
            "无名指": "#FFB86C",
            "小指": "#FF79C6",
        }
        is_little = (name == "小指")
        col = finger_colors.get(name, None)

        row = QFrame()
        row.setObjectName("FingerRow")
        h = QVBoxLayout(row)
        if is_little:
            h.setContentsMargins(8, 2, 8, 2)
            h.setSpacing(2)
        else:
            h.setContentsMargins(10, 2, 10, 2)
            h.setSpacing(2)

        # 手指名称（按颜色区分）
        lab_name = QLabel(name)
        lab_name.setObjectName("FormLabel")
        if col:
            lab_name.setStyleSheet(f"color: {col};")
        if is_little:
            f = lab_name.font()
            try:
                f.setPointSize(max(10, f.pointSize() - 2))
            except Exception:
                pass
            lab_name.setFont(f)
        h.addWidget(lab_name)

        # —— 角度 ——
        angle_line = QHBoxLayout()
        lab_angle = QLabel("角度(°):")
        lab_angle.setFixedWidth(62)
        if is_little:
            f2 = lab_angle.font()
            try:
                f2.setPointSize(max(9, f2.pointSize() - 2))
            except Exception:
                pass
            lab_angle.setFont(f2)

        angle_spin = QDoubleSpinBox()
        angle_spin.setDecimals(2)
        angle_spin.setRange(0.0, ang_max)
        angle_spin.setSingleStep(0.5)
        angle_spin.setFixedWidth(86 if is_little else 90)
        angle_spin.setFixedHeight(22 if is_little else 24)
        angle_spin.setObjectName("InputSpin")
        angle_spin.setButtonSymbols(QAbstractSpinBox.NoButtons)
        angle_spin.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        # 数字框内文字颜色 & 内边距，避免裁切
        if col:
            angle_spin.setStyleSheet("QDoubleSpinBox QLineEdit { padding: 0px 2px; }")
        if is_little:
            f3 = angle_spin.font()
            try:
                f3.setPointSize(max(9, f3.pointSize() - 2))
            except Exception:
                pass
            angle_spin.setFont(f3)

        angle_slider = QSlider(Qt.Horizontal)
        angle_slider.setObjectName("AngleSlider")
        angle_slider.setMinimum(0)
        angle_slider.setMaximum(int(ang_max * 10))
        angle_slider.setFixedHeight(12 if is_little else 14)

        def on_spin(v, s=angle_slider):
            s.blockSignals(True)
            s.setValue(int(v * 10))
            s.blockSignals(False)

        def on_slider(v, sp=angle_spin, owner=self):
            sp.blockSignals(True)
            sp.setValue(v / 10.0)
            sp.blockSignals(False)
            owner.sync_ui_to_model()

        angle_spin.valueChanged.connect(on_spin)
        angle_slider.valueChanged.connect(on_slider)

        angle_line.addWidget(lab_angle)
        angle_line.addWidget(angle_spin)
        angle_line.addWidget(angle_slider, 1)
        h.addLayout(angle_line)

        # —— 速度 ——
        speed_line = QHBoxLayout()
        lab_speed = QLabel("速度(°/s):")
        lab_speed.setFixedWidth(62)
        if is_little:
            f4 = lab_speed.font()
            try:
                f4.setPointSize(max(9, f4.pointSize() - 2))
            except Exception:
                pass
            lab_speed.setFont(f4)

        speed_spin = QDoubleSpinBox()
        speed_spin.setDecimals(1)
        speed_spin.setRange(0.0, 90.0)
        speed_spin.setSingleStep(1.0)
        speed_spin.setValue(50.0)
        speed_spin.setFixedWidth(86 if is_little else 90)
        speed_spin.setFixedHeight(22 if is_little else 24)
        speed_spin.setObjectName("InputSpin")
        speed_spin.setButtonSymbols(QAbstractSpinBox.NoButtons)
        speed_spin.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        if col:
            speed_spin.setStyleSheet("QDoubleSpinBox QLineEdit { padding: 0px 2px; }")
        if is_little:
            f5 = speed_spin.font()
            try:
                f5.setPointSize(max(9, f5.pointSize() - 2))
            except Exception:
                pass
            speed_spin.setFont(f5)

        speed_slider = QSlider(Qt.Horizontal)
        speed_slider.setObjectName("SpeedSlider")
        speed_slider.setMinimum(0)
        speed_slider.setMaximum(90)
        speed_slider.setValue(50)
        speed_slider.setFixedHeight(12 if is_little else 14)

        def on_speed_spin(v, s=speed_slider):
            s.blockSignals(True)
            s.setValue(int(v))
            s.blockSignals(False)

        def on_speed_slider(v, sp=speed_spin):
            sp.blockSignals(True)
            sp.setValue(float(v))
            sp.blockSignals(False)

        speed_spin.valueChanged.connect(on_speed_spin)
        speed_slider.valueChanged.connect(on_speed_slider)

        speed_line.addWidget(lab_speed)
        speed_line.addWidget(speed_spin)
        speed_line.addWidget(speed_slider, 1)
        h.addLayout(speed_line)

        self.finger_angle_boxes.append(angle_spin)
        self.finger_speed_boxes.append(speed_spin)
        return row

    def sync_ui_to_model(self):
        """把左侧 6 个角度值同步到 3D 模型（thumb1, thumb2, index2, middle2, ring2, little2）"""
        # 如果当前选择“接收角度驱动”，则不再用 UI 数值去覆盖模型
        if getattr(self, "use_recv_angles_for_model", False):
            return

        if not hasattr(self, "model_view") or self.model_view is None:
            return
        if len(self.finger_angle_boxes) < 6:
            return
        vals = [sp.value() for sp in self.finger_angle_boxes[:6]]
        # 依次：拇1、拇2、食2、中2、无2、小2
        self.model_view.set_six_angles_deg(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5])

    def on_model_drive_mode_changed(self, checked: bool):
        """模型显示驱动来源切换：False=滑动条, True=接收角度"""
        self.use_recv_angles_for_model = bool(checked)

        # 从“接收角度”切回“滑动条”时，立刻用当前 UI 数值刷新一次模型
        if not checked:
            try:
                self.sync_ui_to_model()
            except Exception:
                pass

    # ========= 规划执行控制 =========
    def start_execute_plan(self):
        if self.last_step_plan is None:
            self.log("[WARN] 没有规划数据")
            QMessageBox.information(self, "提示", "当前没有可执行的规划，请先生成或加载规划。")
            return
        if not self.serial.isOpen():
            self.log("[WARN] 串口未连接，不能执行规划")
            return
        self.plan_executing = True
        self.plan_exec_index = 0
        self.plan_exec_loop_done = 0
        self.btn_exec_plan.setEnabled(True)  # 允许点击；无规划时在处理函数里提示
        self.btn_stop_plan.setEnabled(True)
        self.log("[PLAN] 开始执行规划")
        self.plan_exec_next()

    def stop_execute_plan(self):
        self.plan_executing = False
        self.plan_exec_timer.stop()
        self.btn_exec_plan.setEnabled(True if self.last_step_plan else False)
        self.btn_stop_plan.setEnabled(False)
        self.log("[PLAN] 已中止规划")

    def plan_exec_next(self):
        if not self.plan_executing:
            return
        plan = self.last_step_plan
        if plan is None:
            self.stop_execute_plan()
            return

        frames = plan["frames"]
        if not frames:
            self.stop_execute_plan()
            return

        total_frames = len(frames)
        if self.plan_exec_index >= total_frames:
            if plan["loop_infinite"]:
                self.plan_exec_index = 0
            else:
                self.plan_exec_loop_done += 1
                if self.plan_exec_loop_done >= plan["loop_count"]:
                    self.stop_execute_plan()
                    return
                self.plan_exec_index = 0

        idx = self.plan_exec_index
        fr = frames[idx]
        self.send_plan_frame(fr)

        if idx == 0:
            delay = plan["first_delay"]
        elif idx == total_frames - 1:
            delay = plan["last_delay"]
        else:
            delay = plan["per_delay"]
        if delay < 1:
            delay = 1

        self.plan_exec_index += 1
        self.plan_exec_timer.start(delay)

    def send_plan_frame(self, frame: dict):
        angles = frame["angles"]
        speeds = frame["speeds"]
        start_reg = 0x0002
        reg_count = 0x000C
        byte_count = 0x18
        payload = b""
        for v in angles:
            payload += float_to_half_be(v)
        for v in speeds:
            payload += float_to_half_be(v)
        frame_head = struct.pack(">B B H H B", self.current_slave_addr, 0x10, start_reg, reg_count, byte_count)
        mod_frame = frame_head + payload
        crc = modbus_crc16(mod_frame)
        full = mod_frame + crc
        # 规划帧优先级高
        self.enqueue_frame(full, desc="", expect="plan", priority=1)

    # ========= 动作序列：入口、记录、播放 =========
    def open_action_seq_dialog(self):
        if self.action_seq_dialog is None:
            self.action_seq_dialog = ActionSequenceDialog(self, main=self)
        self.action_seq_dialog.set_frames(self.action_seq)
        self.action_seq_dialog.show()
        self.action_seq_dialog.raise_()
        self.action_seq_dialog.activateWindow()

    def record_action(self):
        if len(self.finger_angle_boxes) < 6 or len(self.finger_speed_boxes) < 6:
            QMessageBox.warning(self, "提示", "控件未就绪")
            return
        vals_pos = [sp.value() for sp in self.finger_angle_boxes[:6]]
        vals_spd = [sp.value() for sp in self.finger_speed_boxes[:6]]
        dms, ok = QInputDialog.getInt(self, "记录动作", "帧间延时 (ms):", 1000, 1, 600000, 1)
        if not ok:
            return
        frame = {"pos": vals_pos, "spd": vals_spd, "delay_ms": int(dms)}
        self.action_seq.append(frame)
        self.log(f"[ACT] 记录 位置={['%.2f' % v for v in vals_pos]} 速度={['%.1f' % v for v in vals_spd]} 延时={dms}ms")
        if self.action_seq_dialog is not None:
            try:
                self.action_seq_dialog.append_frame(frame, select=True)
            except Exception:
                pass

    def stop_action_play(self):
        self.action_play_timer.stop()
        self.action_playing = False
        self.action_paused = False
        self.action_play_index = 0
        if self.action_seq_dialog is not None:
            try:
                self.action_seq_dialog.clear_highlight()
                self.action_seq_dialog.update_toggle_label()
            except Exception:
                pass
        self.log("[ACT] 已停止播放")

    def start_action_play(self, frames=None, dialog=None):
        """
        始终从头开始播放：
        - frames 为 None：使用当前缓存 self.action_seq
        - frames 非 None：用传入帧覆盖缓存后从头播
        """
        # 标准化并更新序列（如有传入）
        if frames is not None:
            self.action_seq = [
                dict(
                    pos=list(map(float, fr.get("pos", [0.0] * 6)))[:6],
                    spd=list(map(float, fr.get("spd", [0.0] * 6)))[:6],
                    delay_ms=int(fr.get("delay_ms", 10)),
                )
                for fr in frames
            ]

        if not self.action_seq:
            QMessageBox.information(self, "提示", "当前没有记录的动作序列。")
            return

        self.action_seq_dialog = dialog or self.action_seq_dialog

        # 读取循环状态（来自对话框按钮/复选框）
        dlg = self.action_seq_dialog
        self.action_loop = bool(
            (getattr(dlg, "btn_loop", None) and dlg.btn_loop.isChecked()) or
            (getattr(dlg, "chk_loop", None) and dlg.chk_loop.isChecked())
        )

        # —— 关键：从头开始
        self.action_play_index = 0
        self.action_paused = False
        self.action_playing = True
        self.log("[ACT] 开始播放动作序列（从头）")

        self.action_play_next()

    def resume_action_play(self, dialog=None):
        """从暂停处继续播放（不重置索引）。"""
        if not self.action_seq or not self.action_paused:
            return
        self.action_seq_dialog = dialog or self.action_seq_dialog
        self.action_playing = True
        self.action_paused = False
        self.log("[ACT] 继续播放")
        # 立刻从当前索引继续
        self.action_play_next()

    def pause_action_play(self):
        """手动停止=暂停：保留索引，可继续。"""
        self.action_play_timer.stop()
        self.action_playing = False
        self.action_paused = True
        self.log("[ACT] 已暂停")
        # 同步按钮文案
        if self.action_seq_dialog:
            try:
                self.action_seq_dialog.update_toggle_label()
            except Exception:
                pass

    def finish_action_play(self):
        """自然播放结束：不可继续（不显示‘继续播放’）。"""
        self.action_play_timer.stop()
        self.action_playing = False
        self.action_paused = False  # 重要：不要置为可继续
        self.log("[ACT] 播放结束")
        if self.action_seq_dialog:
            try:
                self.action_seq_dialog.update_toggle_label()
            except Exception:
                pass

    def action_play_next(self):
        if not self.action_playing:
            return
        if not self.action_seq:
            self.stop_action_play()
            return
        if self.action_play_index >= len(self.action_seq):
            if self.action_loop:
                self.action_play_index = 0
            else:
                self.finish_action_play()  # ← 不再可继续
                return
        fr = self.action_seq[self.action_play_index]
        if self.action_seq_dialog is not None:
            try:
                self.action_seq_dialog.highlight_row(self.action_play_index)
            except Exception:
                pass
        self.send_action_frame(fr)
        delay = int(fr.get("delay_ms", 10))
        if delay < 1:
            delay = 1
        self.action_play_index += 1
        self.action_play_timer.start(delay)

    def send_action_frame(self, frame: dict):
        addr = self.current_slave_addr
        start_reg = 0x0002
        reg_count = 0x000C
        byte_count = 0x18
        pos_vals = list(frame.get("pos", [0.0] * 6))[:6]
        spd_vals = list(frame.get("spd", [0.0] * 6))[:6]
        payload = b""
        for v in pos_vals:
            payload += float_to_half_be(v)
        for v in spd_vals:
            payload += float_to_half_be(v)
        frame_head = struct.pack(">B B H H B", addr, 0x10, start_reg, reg_count, byte_count)
        mod_frame = frame_head + payload
        crc = modbus_crc16(mod_frame)
        full = mod_frame + crc
        self.enqueue_frame(full, desc="", expect="act", priority=1)

    def _apply_channel_visibility(self):
        """按多选框当前勾选，统一更新曲线显隐，并立即刷新。"""
        if not hasattr(self, "monitor_plot_curves") or not hasattr(self, "cmb_channel_sel"):
            return
        for i, c in enumerate(self.monitor_plot_curves):
            vis = self.cmb_channel_sel.is_item_checked(i)
            c.setVisible(vis)

        # 立刻刷新，确保不需要再点图表
        self.monitor_plot.scene().update()
        QApplication.processEvents()

    def _on_channels_selection_changed(self):
        """多选框变动 -> 更新曲线显隐 + 图例外观 + 立即刷新"""
        self._apply_channel_visibility()

    def _init_legend_interactions(self):
        """图例可点击：按'标签文字'定位到正确曲线；点击文字或色块都联动下拉框（不改灰度）。"""
        legend = getattr(self, "monitor_legend", None)
        if legend is None:
            return

        # 确保有 “名称 -> 曲线索引” 的映射
        if not hasattr(self, "_name_to_curve_idx"):
            self._name_to_curve_idx = {self._curve_key(nm): i for i, nm in enumerate(self.monitor_plot_titles)}

        def _bind_one(sample, label):
            # 注意：点击时“现取 label 文字”，保证匹配的是当前图例文字
            def _handler(ev=None, *, label_ref=label):
                txt = self._legend_label_text(label_ref)
                key = self._curve_key(txt)
                idx = self._name_to_curve_idx.get(key, None)
                if idx is None or idx < 0 or idx >= len(self.monitor_plot_curves):
                    # 名称不匹配就不处理（可以在此打印调试）
                    return
                curv = self.monitor_plot_curves[idx]
                new_vis = not curv.isVisible()
                curv.setVisible(new_vis)
                # 回写到多选下拉框（不触发二次刷新）
                self.cmb_channel_sel.set_item_checked(idx, new_vis, update_all=True, emit_signal=False)

                # 立即刷新：图表 + 下拉框
                self.monitor_plot.scene().update()
                self.cmb_channel_sel.update()
                try:
                    self.cmb_channel_sel.view().viewport().update()
                except Exception:
                    pass
                QApplication.processEvents()

            # 同时绑定“文字”和“色块”的点击事件
            try:
                label.mousePressEvent = _handler
            except Exception:
                pass
            try:
                sample.mousePressEvent = _handler
            except Exception:
                pass

        # 遍历 legend 每一行
        for (sample, label) in getattr(legend, "items", []):
            _bind_one(sample, label)

    def _set_legend_row_active(self, idx: int, active: bool):
        """图例第 idx 行：active=True 不透明；False 半透明"""
        if not hasattr(self, "_legend_rows"):
            return
        if 0 <= idx < len(self._legend_rows):
            sample, label = self._legend_rows[idx]
            opacity = 1.0 if active else 0.25
            # 两个图元都可以设透明度
            sample.setOpacity(opacity)
            label.setOpacity(opacity)

        # ========== 2. 数据监视 ==========

    def build_monitor_panel_group(self) -> QGroupBox:
        group = QGroupBox("📈 数据监视")
        group.setObjectName("SectionGroup")
        v = QVBoxLayout(group)
        v.setContentsMargins(10, 10, 10, 10)
        v.setSpacing(8)

        # 顶部控制条：数据类型 + 频率 + 开始/停止 + 状态
        ctrl = QHBoxLayout()
        ctrl.addWidget(QLabel("数据类型:"))
        self.cmb_data_type = QComboBox()
        self.cmb_data_type.setObjectName("InputCombo")
        self.cmb_data_type.addItems(["位置（°）", "速度（°/s）", "电流（A）"])
        self.cmb_data_type.currentIndexChanged.connect(self.on_monitor_type_changed)
        ctrl.addWidget(self.cmb_data_type)

        ctrl.addSpacing(10)
        ctrl.addWidget(QLabel("频率(Hz):"))
        self.monitor_freq_spin = QSpinBox()
        self.monitor_freq_spin.setRange(1, 100)
        self.monitor_freq_spin.setValue(20)
        self.monitor_freq_spin.setObjectName("InputSpin")
        self.monitor_freq_spin.setButtonSymbols(QAbstractSpinBox.NoButtons)
        ctrl.addWidget(self.monitor_freq_spin)

        self.btn_monitor_start = QPushButton("开始监控")
        self.btn_monitor_start.setObjectName("PrimaryButton")
        self.btn_monitor_start.clicked.connect(self.start_monitoring)
        self.btn_monitor_stop = QPushButton("停止")
        self.btn_monitor_stop.clicked.connect(self.stop_monitoring)
        self.lab_monitor_status = QLabel("停止")
        self.lab_monitor_status.setObjectName("FormLabel")

        ctrl.addSpacing(8)
        ctrl.addWidget(self.btn_monitor_start)
        ctrl.addWidget(self.btn_monitor_stop)
        ctrl.addSpacing(8)
        ctrl.addWidget(self.lab_monitor_status)
        ctrl.addStretch(1)
        v.addLayout(ctrl)

        # 选择显示：圈圈多选（含“全部”）
        sel = QHBoxLayout()
        sel.addWidget(QLabel("显示:"))
        self.monitor_plot_titles = ["拇指弯曲", "拇指侧摆", "食指", "中指", "无名指", "小指"]

        self.cmb_channel_sel = CircleMultiComboBox()
        sel.addWidget(self.cmb_channel_sel)
        sel.addStretch(1)
        v.addLayout(sel)

        # 单张图 + 六条曲线
        self.monitor_plot = pg.PlotWidget()
        self.monitor_plot.setBackground("#0d1117")
        self.monitor_plot.showGrid(x=True, y=True, alpha=0.25)
        self.monitor_legend = self.monitor_plot.addLegend(offset=(10, 10))
        # 锁定 X 轴左边界不小于 0（防止鼠标滚轮缩小时把 <0 的时间缩进视图）
        try:
            _pi = self.monitor_plot.getPlotItem() if hasattr(self.monitor_plot, "getPlotItem") else getattr(
                self.monitor_plot, "plotItem", None)
            _vb = getattr(_pi, "vb", None)
            if _vb is None and _pi is not None and hasattr(_pi, "getViewBox"):
                _vb = _pi.getViewBox()
            if _vb is not None:
                _vb.setLimits(xMin=0)
        except Exception:
            pass
        # ← 保存返回的 LegendItem

        pens = [
            pg.mkPen("#5DA3FF", width=2),
            pg.mkPen("#8BE9FD", width=2),
            pg.mkPen("#50FA7B", width=2),
            pg.mkPen("#F1FA8C", width=2),
            pg.mkPen("#FFB86C", width=2),
            pg.mkPen("#FF79C6", width=2),
        ]
        self.monitor_plot_curves = []
        for i, name in enumerate(self.monitor_plot_titles):
            curve = self.monitor_plot.plot(pen=pens[i], name=name)  # 图例项用同名
            self.monitor_plot_curves.append(curve)

        v.addWidget(self.monitor_plot, 1)

        # 6条曲线创建完成后：
        self._name_to_curve_idx = {self._curve_key(nm): i for i, nm in enumerate(self.monitor_plot_titles)}

        # —— 颜色与图例一致：把颜色传给下拉框（第0项为“全部”，颜色 None）
        combo_colors = [None] + [pens[i].color() for i in range(6)]
        self.cmb_channel_sel.set_items(["全部"] + self.monitor_plot_titles,
                                       [None] + [pens[i].color() for i in range(6)])
        self.cmb_channel_sel.selectionChanged.connect(self._on_channels_selection_changed)

        # —— 绑定图例点击：点击图例文字即可显隐，并回写到下拉框
        self._init_legend_interactions()

        # 数据缓冲（维持原有结构）
        self.monitor_data_pos = [[0.0] * 50 for _ in range(6)]
        self.monitor_data_vel = [[0.0] * 50 for _ in range(6)]
        self.monitor_data_cur = [[0.0] * 50 for _ in range(6)]

        self.update_plot_titles()
        self.refresh_monitor_plots()
        return group

    def on_monitor_type_changed(self):
        self.update_plot_titles()
        self.refresh_monitor_plots()

    def update_plot_titles(self):
        # 单张图标题：数据类型 +（若选择单关节，在 legend 已显示名字，这里仅显示类型即可）
        dname = self.cmb_data_type.currentText() if hasattr(self, "cmb_data_type") else ""
        if hasattr(self, "monitor_plot") and self.monitor_plot is not None:
            self.monitor_plot.setTitle(f"{dname}")

    def refresh_monitor_plots(self):
        # 选择数据源
        dname = self.cmb_data_type.currentText() if hasattr(self, "cmb_data_type") else "位置（°）"
        if dname == "位置（°）":
            src = self.monitor_data_pos
        elif dname == "速度（°/s）":
            src = self.monitor_data_vel
        else:
            src = self.monitor_data_cur

        # 更新数据到曲线
        for i in range(6):
            self.monitor_plot_curves[i].setData(src[i])

        def refresh_monitor_plots(self):
            # 选择数据源
            dname = self.cmb_data_type.currentText() if hasattr(self, "cmb_data_type") else "位置（°）"
            if dname == "位置（°）":
                src = self.monitor_data_pos
            elif dname == "速度（°/s）":
                src = self.monitor_data_vel
            else:
                src = self.monitor_data_cur

            # 更新数据到曲线
            for i in range(6):
                self.monitor_plot_curves[i].setData(src[i])

            # 根据多选框的勾选来控制显隐
            self._apply_channel_visibility()

    def start_monitoring(self):
        if not self.serial.isOpen():
            self.log("[WARN] 串口未连接，不能开始监控")
            return
        self.monitor_running = True
        self.lab_monitor_status.setText("监控中...")
        self.monitor_tick()

    def stop_monitoring(self):
        self.monitor_running = False
        self.lab_monitor_status.setText("停止")
        self.monitor_timer.stop()
        self._poll_expect = None

    def build_read_holding_frame(self, start_addr: int, qty: int) -> bytes:
        addr = self.current_slave_addr
        frame = struct.pack(">B B H H", addr, 0x03, start_addr, qty)
        crc = modbus_crc16(frame)
        return frame + crc

    def monitor_tick(self):
        if not self.monitor_running:
            return
        if not self.serial.isOpen():
            self.stop_monitoring()
            return

        t0 = time.monotonic()
        dname = self.cmb_data_type.currentText()
        if dname == "位置（°）":
            start_addr = 0x000E
            poll_kind = "poll-pos"
        elif dname == "速度（°/s）":
            start_addr = 0x0014
            poll_kind = "poll-vel"
        else:
            start_addr = 0x001A
            poll_kind = "poll-cur"

        # 若总线忙或有高优先级排队，就跳过本次
        if self.tx_busy or self.has_high_priority_pending():
            self.log("[MON] 总线忙，本次监控跳过")
        else:
            frame = self.build_read_holding_frame(start_addr, 0x0006)
            # 记录我们期望的类型，方便解析时知道要写哪一块缓存
            self._poll_expect = poll_kind
            self.enqueue_frame(frame, desc="", expect=poll_kind, priority=0)

        freq = self.monitor_freq_spin.value()
        base_ms = max(5, int(1000 / freq))
        elapsed_ms = int((time.monotonic() - t0) * 1000)
        next_ms = base_ms - elapsed_ms
        if next_ms < 2:
            next_ms = 2
        self.monitor_timer.start(next_ms)

    def update_monitor_buffer(self, buf_list, values):
        for i in range(min(6, len(values))):
            buf_list[i].append(values[i])
            if len(buf_list[i]) > 120:
                buf_list[i].pop(0)

        # ========== 3. 模型显示 ==========

    def build_right_panel_group(self) -> QGroupBox:
        group = QGroupBox("🤖 模型显示")
        group.setObjectName("SectionGroup")
        v = QVBoxLayout(group)
        v.setContentsMargins(10, 10, 10, 10)
        v.setSpacing(8)

        # 中间：URDF 视图
        self.model_view = EmbeddedURDFView(group)
        v.addWidget(self.model_view, 1)

        # 底部一行：数据驱动源 + 胶囊切换 + 重置视图
        bottom_row = QHBoxLayout()
        bottom_row.setContentsMargins(0, 0, 0, 0)
        bottom_row.setSpacing(10)

        lbl_source = QLabel("数据驱动源：")
        lbl_source.setMinimumWidth(90)
        bottom_row.addWidget(lbl_source)

        # 胶囊控件
        self.model_drive_toggle = ModelDriveToggle(group)
        # 初始状态：False = 滑动条
        self.model_drive_toggle.setMode(False, animate=False)
        self.model_drive_toggle.modeChanged.connect(self.on_model_drive_mode_changed)
        bottom_row.addWidget(self.model_drive_toggle)

        bottom_row.addStretch(1)

        # 重置视图按钮（同一行）
        self.btn_reset_view = QPushButton("重置视图")
        self.btn_reset_view.clicked.connect(lambda: self.model_view.reset_camera())
        bottom_row.addWidget(self.btn_reset_view)

        v.addLayout(bottom_row)

        # 初始把当前 UI 值推给 3D
        QTimer.singleShot(200, self.sync_ui_to_model)

        return group

    def set_model_drive_source(self, use_recv: bool):
        """
        更新“数据驱动源”按钮状态并切换模式：
        use_recv=False -> 滑动条
        use_recv=True  -> 灵巧手(接收角度)
        """
        # 先把按钮选中状态处理好（保证两端互斥）
        if hasattr(self, "btn_drive_slider"):
            self.btn_drive_slider.setChecked(not use_recv)
        if hasattr(self, "btn_drive_hand"):
            self.btn_drive_hand.setChecked(use_recv)

        # 调用你之前写的逻辑
        self.on_model_drive_mode_changed(use_recv)

        # 移动胶囊里的高亮块（带动画）
        self._update_model_drive_indicator(True)

    def _update_model_drive_indicator(self, animate: bool = True):
        """根据当前模式把绿色高亮块移动到对应按钮下面（可带动画）"""
        if not hasattr(self, "model_drive_indicator"):
            return

        # 当前目标按钮：False=滑动条, True=灵巧手
        target_btn = self.btn_drive_hand if getattr(
            self, "use_recv_angles_for_model", False
        ) else self.btn_drive_slider

        if target_btn is None:
            return

        rect = target_btn.geometry()
        if not rect.isValid():
            return

        margin = 2
        new_rect = QRect(
            rect.left() - margin,
            rect.top() - margin,
            rect.width() + margin * 2,
            rect.height() + margin * 2,
        )

        cur = self.model_drive_indicator.geometry()
        # 第一次或者不需要动画：直接跳过去
        if not cur.isValid() or cur.width() == 0 or not animate:
            self.model_drive_indicator.setGeometry(new_rect)
            return

        # 做一个几百毫秒的几何动画
        if not hasattr(self, "_model_drive_anim"):
            self._model_drive_anim = QPropertyAnimation(
                self.model_drive_indicator, b"geometry", self
            )
            self._model_drive_anim.setDuration(180)
            self._model_drive_anim.setEasingCurve(QEasingCurve.OutCubic)

        anim = self._model_drive_anim
        anim.stop()
        anim.setStartValue(cur)
        anim.setEndValue(new_rect)
        anim.start()

        # ========== 4. 日志 ==========

    def build_log_group(self) -> QGroupBox:
        log_group = QGroupBox("📝 Modbus 收发日志")
        log_group.setObjectName("SectionGroup")
        log_layout = QVBoxLayout(log_group)
        log_layout.setContentsMargins(10, 10, 10, 10)

        log_top = QHBoxLayout()
        log_top.addWidget(QLabel("日志"))
        log_top.addStretch(1)
        self.btn_clear_log = QPushButton("清除")
        self.btn_clear_log.setObjectName("SecondaryButton")
        self.btn_clear_log.setFixedWidth(70)
        self.btn_clear_log.clicked.connect(self.clear_log)
        log_top.addWidget(self.btn_clear_log)
        log_layout.addLayout(log_top)

        self.text_log = QTextEdit()
        self.text_log.setReadOnly(True)
        self.text_log.setObjectName("GlobalLog")
        self.text_log.setMinimumHeight(180)
        log_layout.addWidget(self.text_log)
        return log_group

        # ========== 5. 通讯测试 ==========

    def build_comm_test_group(self) -> QGroupBox:
        comm_group = QGroupBox("🔧 通讯测试")
        comm_group.setObjectName("SectionGroup")
        comm_layout = QVBoxLayout(comm_group)
        comm_layout.setContentsMargins(10, 10, 10, 10)

        btnrow = QHBoxLayout()
        self.btn_comm_pos = QPushButton("读位置")
        self.btn_comm_pos.clicked.connect(self.comm_read_pos)
        self.btn_comm_vel = QPushButton("读速度")
        self.btn_comm_vel.clicked.connect(self.comm_read_vel)
        self.btn_comm_cur = QPushButton("读电流")
        self.btn_comm_cur.clicked.connect(self.comm_read_cur)
        btnrow.addWidget(self.btn_comm_pos)
        btnrow.addWidget(self.btn_comm_vel)
        btnrow.addWidget(self.btn_comm_cur)
        comm_layout.addLayout(btnrow)

        grid = QGridLayout()
        grid.setSpacing(4)
        self.comm_labels = []
        names = ["拇指-弯曲", "拇指-侧摆", "食指", "中指", "无名指", "小指"]
        for i, name in enumerate(names):
            ln = QLabel(name + ":")
            ln.setObjectName("FormLabel")
            lv = QLabel("--")
            lv.setObjectName("DataLabel")
            r = i // 2
            c = (i % 2) * 2
            grid.addWidget(ln, r, c)
            grid.addWidget(lv, r, c + 1)
            self.comm_labels.append(lv)
        comm_layout.addLayout(grid)
        return comm_group

    def set_comm_active_btn(self, btn: QPushButton):
        base = "background: rgba(89, 129, 255, 0.3); border: 1px solid rgba(89,129,255,0.7); border-radius:10px; padding:6px 13px; color:#fff;"
        active = "background: rgba(120,176,255,0.95); border:1px solid rgba(120,176,255,1); color:#0d1117; border-radius:10px; padding:6px 13px; font-weight:600;"
        self.btn_comm_pos.setStyleSheet(base)
        self.btn_comm_vel.setStyleSheet(base)
        self.btn_comm_cur.setStyleSheet(base)
        if btn is not None:
            btn.setStyleSheet(active)
            self._comm_active_btn = btn

    def comm_read_pos(self):
        if not self.serial.isOpen():
            self.log("[WARN] 串口未连接，不能读位置")
            return
        frame = self.build_read_holding_frame(0x000E, 0x0006)
        self._comm_expect = "pos"
        self.set_comm_active_btn(self.btn_comm_pos)
        self.enqueue_frame(frame, desc="", expect="comm-pos", priority=1)

    def comm_read_vel(self):
        if not self.serial.isOpen():
            self.log("[WARN] 串口未连接，不能读速度")
            return
        frame = self.build_read_holding_frame(0x0014, 0x0006)
        self._comm_expect = "vel"
        self.set_comm_active_btn(self.btn_comm_vel)
        self.enqueue_frame(frame, desc="", expect="comm-vel", priority=1)

    def comm_read_cur(self):
        if not self.serial.isOpen():
            self.log("[WARN] 串口未连接，不能读电流")
            return
        frame = self.build_read_holding_frame(0x001A, 0x0006)
        self._comm_expect = "cur"
        self.set_comm_active_btn(self.btn_comm_cur)
        self.enqueue_frame(frame, desc="", expect="comm-cur", priority=1)

        # ========== 串口接收 ==========

    def on_serial_error(self, err):
        # 记录错误
        self.log(f"[SERIAL ERROR] {err}")
        # 只要不是 NoError，就认为串口处于异常状态，需要主动断开
        try:
            from PySide6.QtSerialPort import QSerialPort
        except Exception:
            QSerialPort = None

        need_close = False
        if QSerialPort is None:
            # 无法获取枚举类型，保守起见，除 None 以外都断开
            if err is not None:
                need_close = True
        else:
            # 只在 NoError 时不处理，其余错误全部视为连接异常（包括 PermissionError、ResourceError 等）
            if err != QSerialPort.NoError:
                need_close = True

        if need_close:
            try:
                if self.serial.isOpen():
                    self.log_error("[ERR] 串口异常，已自动断开连接")
                    self.serial.close()
                # 更新主界面连接状态
                self.set_conn_status(False)
                self.update_top_info()
                # 若通讯配置窗口存在，同步按钮和控件状态
                if hasattr(self, "comm_config_dialog") and self.comm_config_dialog is not None:
                    dlg = self.comm_config_dialog
                    if hasattr(dlg, "btn_open"):
                        dlg.btn_open.setText("连接")
                    if hasattr(dlg, "cmb_port"):
                        dlg.cmb_port.setEnabled(True)
                    if hasattr(dlg, "btn_refresh_port"):
                        dlg.btn_refresh_port.setEnabled(True)
                    if hasattr(dlg, "cmb_baud"):
                        dlg.cmb_baud.setEnabled(True)
                    if hasattr(dlg, "spin_conn_addr"):
                        dlg.spin_conn_addr.setEnabled(True)
            except Exception:
                # 避免在错误处理流程中再次抛异常
                pass

    def on_serial_data(self):
        data = self.serial.readAll()
        raw = bytes(data)
        hex_str = " ".join(f"{b:02X}" for b in raw)
        self.log(f"<-- {hex_str}")

        # OTA：让升级对话框优先消费自定义协议帧
        if self.ota_dialog is not None and self.ota_dialog.isVisible():
            try:
                if self.ota_dialog.handle_serial_data(raw):
                    self.release_bus_and_send_next()
                    return
            except Exception as e:
                self.log_error(f"[OTA] RX 处理异常: {e}")

        if len(raw) < 3:
            self.release_bus_and_send_next()
            return
        addr = raw[0]
        func = raw[1]

        # 异常帧
        if func & 0x80:
            if len(raw) >= 3:
                err_code = raw[2]
                human = self.modbus_error_map.get(err_code, f"未知错误0x{err_code:02X}")
                self.log_error(
                    f"[MODBUS EXCEPTION] from 0x{addr:02X}, func=0x{func:02X}, err=0x{err_code:02X} ({human})")
            self.release_bus_and_send_next()
            return

        # 广播回复
        if addr == 0x00 and func == 0x03:
            # 00 03 00 00 00 02 04 00 0a 00 08 CRC
            if len(raw) >= 11 and raw[2] == 0x00 and raw[3] == 0x00 and raw[4] == 0x00 and raw[5] == 0x02:
                if raw[6] == 0x04:
                    slave_addr = (raw[7] << 8) | raw[8]
                    baud_code = (raw[9] << 8) | raw[10]
                    self.apply_broadcast_values(slave_addr & 0xFF, baud_code & 0xFF)
            self.release_bus_and_send_next()
            return

        # 固件版本读取
        if (addr == self.current_slave_addr and func == 0x03
                and self.tx_expect == "fw-version"):
            ver = self.parse_fw_version(raw)
            if ver is not None:
                self.current_fw_version = ver
                if hasattr(self, "btn_fw_version"):
                    self.btn_fw_version.setText(f"固件版本：{ver}")
                # 如果 OTA 窗口已打开，同步一下显示（第七步会加函数）
                if self.ota_dialog is not None:
                    try:
                        self.ota_dialog.sync_current_version_from_main()
                    except Exception as e:
                        self.log_error(f"[OTA] 同步固件版本失败: {e}")
            else:
                # 解析失败就清空
                self.current_fw_version = None
                if hasattr(self, "btn_fw_version"):
                    self.btn_fw_version.setText("固件版本：---")

            self.release_bus_and_send_next()
            return

        # 程序复位应答：addr 匹配当前从站，功能码 0x06，期望值为 "soft-reset"
        if addr == self.current_slave_addr and func == 0x06 and self.tx_expect == "soft-reset":
            # 如果你想更严一点，可以在这里再校验寄存器地址和数据，比如：
            # if len(raw) >= 8:
            #     reg = (raw[2] << 8) | raw[3]
            #     val = (raw[4] << 8) | raw[5]
            #     if not (reg == 0x0A00 and val == 0xFFFF):
            #         self.log_error("[RESET] 程序复位应答内容异常")
            #         self.release_bus_and_send_next()
            #         return

            self._start_soft_reset_cooldown()
            self.release_bus_and_send_next()
            return

        # 通讯测试
        # 通讯测试
        if addr == self.current_slave_addr and func == 0x03 and self.tx_expect and self.tx_expect.startswith("comm"):
            values = self.parse_extended_03(raw)
            if values is not None:
                # 更新右侧通讯测试的数值
                for i in range(6):
                    if i < len(values):
                        self.comm_labels[i].setText(f"{values[i]:.3f}")
                    else:
                        self.comm_labels[i].setText("--")

                # ★ 使用通讯读位置的数据驱动 3D 模型
                if (
                        getattr(self, "use_recv_angles_for_model", False)  # 当前是“灵巧手”模式
                        and self.tx_expect == "comm-pos"  # 这次是读位置
                        and hasattr(self, "model_view")
                        and self.model_view is not None
                        and len(values) >= 6
                ):
                    self.model_view.set_six_angles_deg(
                        values[0], values[1], values[2],
                        values[3], values[4], values[5]
                    )

            self._comm_expect = None
            self.release_bus_and_send_next()
            return

        # 轮询
        if addr == self.current_slave_addr and func == 0x03 and self.tx_expect and self.tx_expect.startswith("poll"):
            values = self.parse_extended_03(raw)
            if values is not None:
                if self.tx_expect == "poll-pos":
                    # 更新监控缓存
                    self.update_monitor_buffer(self.monitor_data_pos, values)

                    # ★ 使用轮询位置数据驱动 3D 模型
                    if (
                            getattr(self, "use_recv_angles_for_model", False)
                            and hasattr(self, "model_view")
                            and self.model_view is not None
                            and len(values) >= 6
                    ):
                        self.model_view.set_six_angles_deg(
                            values[0], values[1], values[2],
                            values[3], values[4], values[5]
                        )

                elif self.tx_expect == "poll-vel":
                    self.update_monitor_buffer(self.monitor_data_vel, values)
                elif self.tx_expect == "poll-cur":
                    self.update_monitor_buffer(self.monitor_data_cur, values)

                self.refresh_monitor_plots()
            self._poll_expect = None
            self.release_bus_and_send_next()
            return

        # 配置 or 规划 or 其他 0x03
        self.release_bus_and_send_next()

    def parse_extended_03(self, raw: bytes):
        if len(raw) < 7:
            return None
        start_reg = (raw[2] << 8) | raw[3]
        qty = (raw[4] << 8) | raw[5]
        byte_count = raw[6]
        expected_len = 7 + byte_count + 2
        if len(raw) == expected_len + 2 and raw[-2:] == b"\x0D\x0A":
            frame_no_crlf = raw[:expected_len]
        elif len(raw) >= expected_len:
            frame_no_crlf = raw[:expected_len]
        else:
            return None

        data_start = 7
        data_end = 7 + byte_count
        payload = frame_no_crlf[data_start:data_end]
        crc_recv = frame_no_crlf[data_end:data_end + 2]
        calc_crc = modbus_crc16(frame_no_crlf[:data_end])
        if crc_recv != calc_crc:
            self.log_error(f"[MODBUS] 扩展03 CRC错误: start=0x{start_reg:04X}, qty={qty}")
            return None

        values = []
        for i in range(0, byte_count, 2):
            v = half_be_to_float(payload[i], payload[i + 1])
            values.append(v)
        return values

    def parse_fw_version(self, raw: bytes):
        """
        解析固件版本返回帧：
        addr, 0x03, 0x0B, 0x00, 0x00, 0x05, 0x0A, payload(10B), CRC(2B) [可选 0D 0A]
        payload 格式：
          [0..1]  主版本号 (uint16, BE)
          [2..3]  次版本号 (uint16, BE)
          [4..5]  修订版本号 (uint16, BE)
          [6..9]  修订日期 (uint32, BE, 形如 20251119)
        """
        if len(raw) < 7:
            return None

        start_reg = (raw[2] << 8) | raw[3]
        qty = (raw[4] << 8) | raw[5]
        byte_count = raw[6]

        # 校验寄存器地址、数量和字节数是否符合约定
        if start_reg != 0x0B00 or qty != 0x0005 or byte_count != 0x0A:
            return None

        expected_len = 7 + byte_count + 2  # 不含可选 CRLF
        if len(raw) == expected_len + 2 and raw[-2:] == b"\x0D\x0A":
            frame_no_crlf = raw[:expected_len]
        elif len(raw) >= expected_len:
            frame_no_crlf = raw[:expected_len]
        else:
            return None

        data_start = 7
        data_end = data_start + byte_count
        payload = frame_no_crlf[data_start:data_end]
        crc_recv = frame_no_crlf[data_end:data_end + 2]
        calc_crc = modbus_crc16(frame_no_crlf[:data_end])
        if crc_recv != calc_crc:
            self.log_error("[FW] 固件版本帧 CRC 错误")
            return None

        # 解析 payload
        major = (payload[0] << 8) | payload[1]
        minor = (payload[2] << 8) | payload[3]
        patch = (payload[4] << 8) | payload[5]
        date_val = ((payload[6] << 24) |
                    (payload[7] << 16) |
                    (payload[8] << 8) |
                    payload[9])

        # 约定 date_val 本身就是 yyyymmdd 数字
        date_str = f"{date_val:08d}"
        return f"V{major}.{minor}.{patch}_{date_str}"

    def apply_broadcast_values(self, slave_addr: int, baud_code: int):
        self.log(f"[BCAST] addr=0x{slave_addr:02X}, baud_code=0x{baud_code:02X}")
        self.current_slave_addr = slave_addr
        self.current_baud_code = baud_code
        self.update_top_info()
        # 地址变化后，之前的固件版本不再可靠，清空
        self.current_fw_version = None
        if hasattr(self, "btn_fw_version"):
            self.btn_fw_version.setText("固件版本：---")

        # 将广播的结果同步到通讯配置窗口（如果已打开）
        if self.comm_config_dialog is not None:
            try:
                self.comm_config_dialog.show_broadcast_result(slave_addr, baud_code)
            except Exception as e:
                self.log_error(f"[BCAST] 更新通讯配置窗口结果失败: {e}")
                # 地址变化后，之前的固件版本不再可靠，清空
                self.current_fw_version = None
                if hasattr(self, "btn_fw_version"):
                    self.btn_fw_version.setText("固件版本：---")

        # ========== 手动下发 0x10 ==========

    def send_joint_command(self, angles, speeds, expect="plan", priority=1, desc=""):
        addr = self.current_slave_addr
        start_reg = 0x0002
        reg_count = 0x000C
        byte_count = 0x18

        pos_vals = list(angles)[:6]
        spd_vals = list(speeds)[:6]
        while len(pos_vals) < 6:
            pos_vals.append(0.0)
        while len(spd_vals) < 6:
            spd_vals.append(50.0)

        payload = b""
        for v in pos_vals:
            payload += float_to_half_be(float(v))
        for v in spd_vals:
            payload += float_to_half_be(float(v))
        frame_head = struct.pack(">B B H H B", addr, 0x10, start_reg, reg_count, byte_count)
        frame = frame_head + payload
        crc = modbus_crc16(frame)
        full = frame + crc
        self.enqueue_frame(full, desc=desc, expect=expect, priority=priority)

    def send_finger_motion(self):
        pos_vals = [spin.value() for spin in self.finger_angle_boxes]
        spd_vals = [spin.value() for spin in self.finger_speed_boxes]
        self.send_joint_command(pos_vals, spd_vals, expect="plan", priority=1, desc="")

    def reset_finger_command(self):
        """手指复位：不修改 UI，只下发 6 个关节角度为 0、速度为 50 的运动控制指令"""
        try:
            self.send_joint_command([0.0] * 6, [50.0] * 6, expect="plan", priority=1, desc="")
        except Exception as e:
            self.log_error(f"[ERR] 手指复位指令发送失败: {e}")

    def reset_finger_ui(self):
        """重置：6 个角度清零，速度统一设置为 50，仅更新界面和 3D 模型，不下发指令"""
        try:
            if len(self.finger_angle_boxes) >= 6 and len(self.finger_speed_boxes) >= 6:
                # 角度清零（不屏蔽信号，让滑动条和 3D 模型自动联动）
                for sp in self.finger_angle_boxes[:6]:
                    sp.setValue(0.0)
                # 速度设置为 50（同样不屏蔽信号，让滑动条联动）
                for sp in self.finger_speed_boxes[:6]:
                    sp.setValue(50.0)
                # 上面 setValue 已经触发 sync_ui_to_model，这里无需再额外调用
        except Exception:
            # 出现异常时，不影响后续操作
            pass

    def soft_reset_slave(self):
        """复位：给下位机发送软复位指令（0x06 写 0x0A00，数据 0xFFFF）"""

        # ① 冷却期内直接返回，不再发送
        if self._soft_reset_cooldown:
            self.log("[RESET] 程序复位冷却中，2 秒内不能重复发送")
            return

        addr = self.current_slave_addr & 0xFF
        func = 0x06
        start_reg = 0x0A00
        reg_qty = 0x0001
        byte_count = 0x02
        data_val = 0xFFFF

        # 按照你现在的打包方式
        frame = struct.pack(">B B H H B H", addr, func, start_reg, reg_qty, byte_count, data_val)
        crc = modbus_crc16(frame)
        full = frame + crc

        # ② 这里改成专门的 expect，用于识别复位应答
        self.enqueue_frame(full, desc="[RESET] 程序复位", expect="soft-reset", priority=1)

    def _start_soft_reset_cooldown(self):
        """收到从机复位应答后，开始 2 秒冷却并禁用按钮"""
        if self._soft_reset_cooldown:
            return
        self._soft_reset_cooldown = True

        try:
            self.btn_soft_reset.setEnabled(False)
        except Exception:
            pass

        # 2 秒后自动结束冷却
        QTimer.singleShot(2000, self._end_soft_reset_cooldown)

    def _end_soft_reset_cooldown(self):
        """冷却结束，恢复按钮可用"""
        self._soft_reset_cooldown = False
        try:
            self.btn_soft_reset.setEnabled(True)
        except Exception:
            pass

        # ========== 日志 ==========

    def log(self, text: str):
        now = time.strftime("%H:%M:%S")
        self.text_log.append(f"[{now}] {text}")

    def log_error(self, text: str):
        now = time.strftime("%H:%M:%S")
        self.text_log.append(f"<span style='color:#ff7777'>[{now}] {text}</span>")

    def clear_log(self):
        self.text_log.clear()

        # ========== 样式 ==========

    def build_qss(self) -> str:
        return """
            QWidget {
                background: #0b0f16;
                color: #f2f4ff;
                font-family: "Microsoft YaHei", "Segoe UI", "Helvetica Neue";
                font-size: 14px;
            }
            QGroupBox#SectionGroup {
                background: rgba(255,255,255,0.015);
                border: 1px solid rgba(120,160,255,0.25);
                border-radius: 14px;
                margin-top: 4px;
            }
            QGroupBox#SectionGroup::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding: 0 6px;
                margin-left: 10px;
                color: #dce6ff;
                font-weight: 600;
            }
            QGroupBox#CardGroup {
                background: rgba(255,255,255,0.018);
                border: 1px solid rgba(255,255,255,0.04);
                border-radius: 14px;
                margin-top: 4px;
            }
            QPushButton {
                background: rgba(89, 129, 255, 0.3);
                border: 1px solid rgba(89, 129, 255, 0.7);
                border-radius: 10px;
                padding: 6px 13px;
                color: #ffffff;
            }
            QPushButton:hover {
                background: rgba(89, 129, 255, 0.7);
            }
            #PrimaryButton {
                background: rgba(120, 176, 255, 0.9);
                border: 1px solid rgba(120, 176, 255, 1);
                color: #0d1117;
                font-weight: 600;
            }
            #PrimaryButton:hover {
                background: rgba(150, 196, 255, 1);
            }
            #SecondaryButton {
                background: rgba(255, 196, 77, 0.35);
                border: 1px solid rgba(255, 196, 77, 0.55);
                color: #ffe0a5;
                font-weight: 600;
            }
            #SecondaryButton:hover {
                background: rgba(255, 196, 77, 0.55);
            }
            #TopTag {
                background: rgba(255,255,255,0.03);
                border: 1px solid rgba(255,255,255,0.02);
                border-radius: 8px;
                padding: 3px 9px;
            }
            QPushButton#FwVersionTag {
                background: transparent;
                border: none;
                padding: 3px 9px;
                color: #dce6ff;
                font-weight: 500;
            }
            QPushButton#FwVersionTag:hover {
                color: #ffffff;
                text-decoration: underline;
            }
            QLabel#OtaVersionLabel {
                color: rgba(220, 230, 255, 0.7);
                font-size: 12px;
            }

            #ConnIndicatorConnected {
                color: #36e17c;
                font-weight: 600;
            }
            #ConnIndicatorDisconnected {
                color: #ff6b6b;
                font-weight: 600;
            }
            QDoubleSpinBox#InputSpin, QSpinBox#InputSpin, QComboBox#InputCombo {
                background: rgba(15, 23, 35, 0.85);
                border: 1px solid rgba(156, 174, 255, 0.35);
                border-radius: 6px;
                padding: 2px 4px;
            }
            QDoubleSpinBox#InputSpin:disabled,
            QSpinBox#InputSpin:disabled,
            QComboBox#InputCombo:disabled {
                background: rgba(8, 12, 20, 0.95);
                border: 1px dashed rgba(140, 148, 182, 0.9);
                color: rgba(180, 188, 210, 0.85);
            }
            QTextEdit#GlobalLog {
                background: rgba(8, 10, 14, 0.45);
                border: 1px solid rgba(255,255,255,0.02);
                border-radius: 6px;
            }
            QFrame#FingerRow {
                background: rgba(255,255,255,0.012);
                border: 1px solid rgba(255,255,255,0.012);
                border-radius: 10px;
            }
            QLabel#DataLabel {
                background: rgba(12,16,22,0.6);
                border: 1px solid rgba(255,255,255,0.03);
                border-radius: 4px;
                padding: 2px 4px;
            }
            QSlider::groove:horizontal {
                background: rgba(255,255,255,0.05);
                height: 6px;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #7cb4ff;
                width: 14px;
                height: 14px;
                margin: -4px 0;
                border-radius: 7px;
            }
            QSlider::sub-page:horizontal {
                background: rgba(124, 180, 255, 0.6);
                border-radius: 3px;
            }

             /* 模型驱动源：胶囊切换按钮 */
            QFrame#ModelDriveSegment {
                background: rgba(255, 255, 255, 0.15);      /* 外层胶囊底色 */
                border-radius: 999px;
                border: 0px solid rgba(0, 0, 0, 0.45);
            }
            QFrame#ModelDriveIndicator {
                background: #4fd9c8;                         /* 绿色滑块 */
                border-radius: 999px;
            }
            QPushButton#ModelDriveButton {
                border: none;
                padding: 6px 24px;
                min-width: 80px;
                background: transparent;                     /* 文字在上层，不再自己上色 */
                color: rgba(230, 235, 255, 0.80);
                font-size: 13px;
                border-radius: 999px;
            }
            QPushButton#ModelDriveButton:hover {
                color: #ffffff;
            }
            QPushButton#ModelDriveButton:checked {
                color: #0b1016;                              /* 选中一侧文字变深 */
                font-weight: 600;
            }

            #ModelPlaceholder {
                background: rgba(255,255,255,0.015);
                border: 1px dashed rgba(255,255,255,0.06);
                border-radius: 12px;
            }

            """


def main():
    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("OpenLoong.LHandPC")
        except Exception:
            pass

    app = QApplication(sys.argv)
    app.setWindowIcon(QIcon(_app_icon_path()))
    # 供 QSettings 使用的组织名/应用名（可改成你喜欢的）
    QCoreApplication.setOrganizationName("OpenLoong")
    QCoreApplication.setApplicationName("LHand-PC")

    win = MainWindow()
    # 不要这里强行 showMaximized；交给“恢复函数”决定
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
