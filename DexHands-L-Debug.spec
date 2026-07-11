# -*- mode: python ; coding: utf-8 -*-
from PyInstaller.utils.hooks import collect_submodules

hiddenimports = ['PySide6.QtSerialPort']
hiddenimports += collect_submodules('vtkmodules.vtkCommonCore')
hiddenimports += collect_submodules('vtkmodules.vtkCommonDataModel')
hiddenimports += collect_submodules('vtkmodules.vtkFiltersCore')
hiddenimports += collect_submodules('vtkmodules.vtkIOGeometry')
hiddenimports += collect_submodules('vtkmodules.vtkRenderingCore')
hiddenimports += collect_submodules('vtkmodules.vtkRenderingOpenGL2')
hiddenimports += collect_submodules('vtkmodules.vtkInteractionStyle')


a = Analysis(
    ['HandPC-Debug.py'],
    pathex=[],
    binaries=[],
    datas=[('assets', 'assets'), ('youshou_urdf', 'youshou_urdf')],
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='DexHands-L-Debug',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=['assets\\wall-e.png'],
)
