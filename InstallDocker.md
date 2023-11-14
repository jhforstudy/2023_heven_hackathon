## Docker 환경 구성

#### 1. docker 설치

* 다음의 링크를 참고하여 Docker Desktop [설치](https://docs.docker.com/desktop/install/windows-install/)

![image](https://github.com/jhforstudy/2023_heven_hackathon/assets/48710703/e4c9d8f2-27d7-4b7f-85bb-3e1bf78b59d7)

해당 파일 다운로드 후, 실행하여 설치 (체크박스 모두 클릭)

#### 2. Docker Desktop 실행하여 해커톤 시뮬레이터 다운로드

* 윈도우 검색창에서 "Windows Powershell"을 검색하여 실행
* 터미널 창에 다음을 입력
```
docker run -p 6080:80 -e RESOLUTION=1920x1080 --shm-size=512m a2jinhee/2023_hackathon:latest
```
![image](https://github.com/jhforstudy/2023_heven_hackathon/assets/48710703/8213df98-5c29-44f7-b1f1-bbfe24420404)

자동으로 Dockerhub에서 image를 받아 와서 실행함.

![image](https://github.com/jhforstudy/2023_heven_hackathon/assets/48710703/403f20fe-3ae0-4d2c-9f85-82fe18f53e75)

모든 과정이 완료되는 지 확인 (약 10분 가량 소요됨)

* 윈도우 검색창에서 "Docker Desktop"을 검색하여 실행

![image](https://github.com/jhforstudy/2023_heven_hackathon/assets/48710703/460facd1-cb13-4ee6-8eeb-2079b6d5d1d9)

해커톤 시뮬레이터가 제대로 다운로드 됐는지 확인 후, 우측의 재생 버튼 클릭, 이후 Run 클릭 (Optional Setting 설정 필요 없음)

#### 3. 가상 환경 접속

* 웹 브라우저 (크롬) 을 열고, 다음의 주소로 접속
```
http://127.0.0.1:6080/
```

개발 환경이 설치된 Ubuntu OS를 웹 브라우저에서 이용 가능
![캡처2](https://user-images.githubusercontent.com/48710703/199906904-f54b5a5a-8a8c-4a25-b977-b8a2e6381994.PNG)

#### 4. Visual Studio Code 설치 및 Extension 깔기

* 구글 검색창에 "vscode" 검색하여, 설치

![image](https://github.com/jhforstudy/2023_heven_hackathon/assets/48710703/b07828c5-816b-4dd2-8be2-2802366a6ee4)

![image](https://github.com/jhforstudy/2023_heven_hackathon/assets/48710703/d0836f1d-2ecf-467d-9e29-91d311bc1abe)

![image](https://github.com/jhforstudy/2023_heven_hackathon/assets/48710703/52ed4955-b68f-488b-9b4c-3d66ca7b763d)

![image](https://github.com/jhforstudy/2023_heven_hackathon/assets/48710703/65ec9621-b050-4ff9-b6af-77eeeabba2c0)

![image](https://github.com/jhforstudy/2023_heven_hackathon/assets/48710703/d1babbae-4c75-4765-bc83-b7a0b0c2b826)

* Extension 설치 후, 프로그램 재실행

![image](https://github.com/jhforstudy/2023_heven_hackathon/assets/48710703/754ce6b2-baac-41cc-b4a3-668b17b2e974)

좌측 중앙의 "Extensions" 아이콘 클릭 후, "Remote Development" 검색하여 설치

![image](https://github.com/jhforstudy/2023_heven_hackathon/assets/48710703/d2c9d432-efe6-48b8-b332-94ed4a76c4d2)

설치 완료 시 위와 같이 뜸. 프로그램 재실행

* 자율주행 알고리즘 코드 접근
1. 왼쪽 옆에 컴퓨터 아이콘(Remote Explorer)을 누름.

    ![image](https://github.com/jhforstudy/2023_heven_hackathon/assets/48710703/4fef99fc-c53a-4939-9765-e5576aec4b08)

2. Dev Containers에서 dorowu/ubuntu-desktop-lxde-vnc:focal 옆 화살표를 누름.

    ![image](https://github.com/jhforstudy/2023_heven_hackathon/assets/48710703/a0ec6cce-16c5-4872-bf01-bc1f1addd8e3)

3. root/ 폴더로 들어가면 해커톤 시뮬레이터 디스크 내에 있는 모든 폴더에 접근할 수 있음

    그 중에서, `catkin_ws/src/2023_heven_hackathon/scripts/brain.py` 를 수정하면 됨.
  
    ![image](https://github.com/jhforstudy/2023_heven_hackathon/assets/48710703/20b213cd-0454-46e3-af81-6ae45a182a60)

#### 5. 특이사항  
- 인터넷은 chrome 대신 firefox 사용할 것
