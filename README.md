calibrationtool

환경구성방법
conda 가상환경 사용

0. 터미널 실행
1. conda create -n <환경이름>
2. conda activate <환경이름>
3. requirement_*.txt 파일이 있는 디렉토리로 이동(cd C:\\~\\calibration_guitool\\revision_version1)
4. requirement_first.txt 파일 내부의 내용 스크립트로 복사해 패키지들 설치(ex. conda install -c conda-forge -y pip ...)
5. requirement_second.txt 파일 pip통해 설치 (pip install -r requirment_second.txt)
6. 설치중 에러발생시 5번 다시 실행
7. scipy.optimize 함수 수정
8. <환경이름>/Lib/site-package/scipy/optimize/_minimize.py 파일 열기
9. def minimize 함수 선언부에 thread=None을 아래와 같이 삽입
```
def minimize(fun, x0, args=(), thread=None, method=None, jac=None, hess=None,
             hessp=None, bounds=None, constraints=(), tol=None,
             callback=None, options=None):
```
10. minimize 함수 내부에 
    `return _minimize_powell(fun, x0, args, callback, bounds, **options)`
    부분을 
    `return _minimize_powell(fun, x0, args, thread, callback, bounds, **options)`
    으로 변경
  
11. <환경이름>/Lib/site-package/scipy/optimize/optimize.py 파일 열기  
12. optimize.py 함수 내부의 `def _minimize_powell`함수에 `thead=None`을 아래와 같이 삽입
```
def _minimize_powell(func, x0, args=(), thread=None, callback=None, bounds=None,
                     xtol=1e-4, ftol=1e-4, maxiter=None, maxfev=None,
                     disp=False, direc=None, return_all=False,
                     **unknown_options):
```

13. `_minimize_powell` 함수 내부의 while문 안에 
    `if not thread._status:
         break` 를 아레와 같이 추가한다.
```
 while True:
        if not thread._status:
            break
        fx = fval
        bigind = 0
        delta = 0.0
        ...
```


.exe파일 만드는 방법
1. `pyinstaller GUI_tool.py`
2. 실행파일 제작후 생성되는 dist 폴더와 build 폴더 삭제
3. reversion1 폴더에 생성된 GUI_tool.spec 파일 아래 내용과 같이 수정
4. `pyinstaller GUI_tool.spec` 실행
5. common을 dist내부의 프로그렘 폴더에 복사

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