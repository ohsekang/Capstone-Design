%%
% https://kr.mathworks.com/help/matlab/ref/ftp.html?s_tid=doc_ta#d123e380327

%% FTP 형태로 server와 통신하기 위해 객체 생성

ftpobj = ftp("capstone5.dothome.co.kr/","capstone5","scoutmini5!");

%% 리눅스 change directory와 같음 (html 폴더로 이동)
cd(ftpobj, "html");

%% 현재 디렉토리에 있는 폴더 검색 (리눅스 ls -al과 비슷)
dir(ftpobj)

%% data 가져와서 형태 변환
% id, x, y 값은 아래 주소에 table 형태로 저장할 예정이기 때문에 php 파일을 이용해서 값 추가
% http://capstone5.dothome.co.kr/myadmin/index.php?route=/sql&server=1&db=capstone5&table=test&pos=0
% 테이블에서 x,y,id 순서대로 열 단위로 데이터를 가져옴

getData = webread("http://capstone5.dothome.co.kr/getData.php")
getData = replace(getData, '"', '')

getData = split(getData(2:end-1), ',')

getData = reshape(getData, height(getData)/3, [])

%% 테이블의 모든 데이터를 삭제
webread("http://capstone5.dothome.co.kr/deleteAll.php")

%% table에서 id가 0인 행을 삭제
webread("http://capstone5.dothome.co.kr/deleteId.php?id=0")

%% x,y,id를 테이블에 추가
webread("http://capstone5.dothome.co.kr/insertData.php?x=7&y=12&id=99")

%% 해당 id를 기준으로 x,y값을 변경
webread("http://capstone5.dothome.co.kr/changeData.php?x=5&y=5&id=10")

%% 현재 디렉토리에 우측에 오는 파일 추가
mput(ftpobj, 'Map.fig')
mput(ftpobj, 'arch.tif')

%% 현재 디렉토리에 있는 폴더 검색 (리눅스 ls -al과 비슷)
dir(ftpobj)

%% pc의 현재 폴더(좌측에 위치한 파일이 있는 위치)에 우측 파일을 저장
mget(ftpobj, 'arch.tif')

%% 현재 폴더 위치에 저장된 arch 파일을 실행
imshow(arch)

%% 현재 디렉토리에서 Map.fig 파일을 삭제 후 디렉토리에 있는 폴더 검색
delete(ftpobj, 'Map.fig')
dir(ftpobj)