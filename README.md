calibrationtool

환경구성방법
conda 가상환경 사용
1. conda create -n <환경이름>
2. conda activate <환경이름>
3. requirement.txt 파일이 있는 디렉토리로 이동(cd C:\\~\\calibration_guitool\\revision_version1)
4. requirement_first.txt 파일 내부의 내용 스크립트로 복사해 패키지들 설치(ex. conda install -c conda-forge -y pip ...)
5. requirement_second.txt 파일 pip통해 설치 (pip install -r requirment_second.txt)
5.1 설치중 에러발생시 5번 다시 실행
   
exe파일 만드는 방법
1. 'pyinstaller GUI_tool.py'
2. 실행파일 제작후 생성되는 dist 폴더와 build 폴더 삭제
3. reversion1 폴더에 생성된 GUI_tool.spec 파일 아래 내용과 같이 수정
4. 'pyinstaller GUI_tool.spec' 실행

-------------GUI_tool.spec--------------------------------------------------------
```
# -*- mode: python ; coding: utf-8 -*-

block_cipher = None


a = Analysis(['C:\\git\\calibration_guitool\\revision_version1\\GUI_tool.py'],
             pathex=['C:\\git\\calibration_guitool\\revision_version1'],
             binaries=[],
             datas=[],
             hiddenimports=['sklearn.neighbors._typedefs','sklearn.utils._cython_blas','zmq.backend.cython'],
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
```

--------------------------------------------------------------------------------------