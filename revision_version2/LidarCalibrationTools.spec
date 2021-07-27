# -*- mode: python ; coding: utf-8 -*-

block_cipher = None


a = Analysis(['D:\\git_ws\\calibration_guitool\\revision_version2\\LidarCalibrationTools.py'],
             pathex=['D:\\git_ws\\calibration_guitool\\revision_version2'],
             binaries=[],
             datas=[],
             hiddenimports=['sklearn.neighbors._typedefs','sklearn.utils._cython_blas', 'zmq.backend.cython', 'vtkmodules','vtkmodules.all','vtkmodules.qt.QVTKRenderWindowInteractor','vtkmodules.util','vtkmodules.util.numpy_support'],
             hookspath=[],
             runtime_hooks=[],
             excludes=[],
             win_no_prefer_redirects=False,
             win_private_assemblies=False,
             cipher=block_cipher,
             noarchive=False)
pyz = PYZ(a.pure, a.zipped_data,
             cipher=block_cipher)
exe = EXE(pyz,
          a.scripts,
          [],
          exclude_binaries=True,
          name='LidarCalibrationTools',
          debug=True,
          bootloader_ignore_signals=False,
          strip=False,
          upx=True,
          console=True , icon='exe_icon.ico')
coll = COLLECT(exe,
               a.binaries,
               a.zipfiles,
               a.datas,
               strip=False,
               upx=True,
               upx_exclude=[],
               name='LidarCalibrationTools')
