calibrationtool

The method for virtual environment setting
Use conda virtual environment

0. Run Anaconda prompt
1. conda create -n <env_name>
2. conda activate <env_name>
3. Move to directory with requirement_*.txt (cd C:\\~\\calibration_guitool\\revision_version1)
4. Copy all text in requirement_first.txt file and paste in terminal to install libraries(ex. conda install -c conda-forge -y pip ...)
5. scipy.optimize 함수 수정
6. Open the file '<env_name>/Lib/site-package/scipy/optimize/_minimize.py'
7. in area of 'def minimize' insert 'thread=None' like below example
```
def minimize(fun, x0, args=(), thread=None, method=None, jac=None, hess=None,
             hessp=None, bounds=None, constraints=(), tol=None,
             callback=None, options=None):
```
8. In the minimize function 
    `return _minimize_powell(fun, x0, args, callback, bounds, **options)`
    part change to
    `return _minimize_powell(fun, x0, args, thread, callback, bounds, **options)`
  
9. Open <env_name>/Lib/site-package/scipy/optimize/optimize.py  
10. In definition part `def _minimize_powell`, insert `thead=None`like below example
```
def _minimize_powell(func, x0, args=(), thread=None, callback=None, bounds=None,
                     xtol=1e-4, ftol=1e-4, maxiter=None, maxfev=None,
                     disp=False, direc=None, return_all=False,
                     **unknown_options):
```

11. In fucntion `_minimize_powell`, in the while loop insert 
    `if not thread._status:
         break` as like below
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