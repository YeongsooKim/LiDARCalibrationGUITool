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
'pyinstaller GUI_tool.spec'
