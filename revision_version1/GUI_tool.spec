# -*- mode: python ; coding: utf-8 -*-

block_cipher = None


a = Analysis(['C:\\git\\calibration_guitool\\revision_version1\\GUI_tool.py'],
             pathex=['C:\\git\\calibration_guitool\\revision_version1'],
             binaries=[],
             datas=[],
             hiddenimports=['sklearn.neighbors.typedefs','sklearn.utils._cython_blas'],
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
          name='Calibration GUI Tool',
          debug=False,
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
               name='Calibration GUI Tool')
