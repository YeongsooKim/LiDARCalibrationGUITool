# -*- mode: python ; coding: utf-8 -*-

block_cipher = None


a = Analysis(['D:\\git_ws\\calibrationtool\\revision_version1\\GUI_tool.py'],
             pathex=['D:\\git_ws\\calibrationtool\\revision_version1'],
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
          name='GUI_tool',
          debug=False,
          bootloader_ignore_signals=False,
          strip=False,
          upx=True,
          console=True )
coll = COLLECT(exe,
               a.binaries,
               a.zipfiles,
               a.datas,
               strip=False,
               upx=True,
               upx_exclude=[],
               name='GUI_tool')
