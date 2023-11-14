## Docker 환경 구성

#### 1. docker 설치

* 다음의 링크를 참고하여 Docker 및 WSL 2 [설치](https://myjamong.tistory.com/296)

#### 2. Docker image pull 및 실행

* Windows Powershell을 관리자 권한으로 실행
* 터미널 창에 다음을 입력
```
docker run -p 6080:80 -e RESOLUTION=1920x1080 --shm-size=512m jhforstudy/heven_hack:latest
```
자동으로 Dockerhub에서 image를 받아 와서 실행 함.

위의 명령어가 성공적으로 실행된 후,

**Docker Desktop**을 실행하여 현재 작동되고 있는 Container를 확인 가능
![캡처](https://user-images.githubusercontent.com/48710703/199906569-ff047cd3-61af-49cf-8d66-f69add64935c.PNG)

#### 3. 가상 환경 접속

* 웹 브라우저 (크롬) 을 열고, 다음의 주소로 접속
```
http://127.0.0.1:6080/
```

개발 환경이 설치된 Ubuntu OS를 웹 브라우저에서 이용 가능
![캡처2](https://user-images.githubusercontent.com/48710703/199906904-f54b5a5a-8a8c-4a25-b977-b8a2e6381994.PNG)

--- 

#### 1. docker 설치 

#### 2. Docker image pull 및 실행 
```
docker run -p 6080:80 -e RESOLUTION=1920x1080 -v /dev/shm:/dev/shm dorowu/ubuntu-desktop-lxde-vnc:focal
```

> 특이사항  
- 인터넷은 chrome 대신 firefox 사용할 것- chrome이 이상하게 안됨. 
- lubuntu image를 처음 pull할 때 sudo apt-get update 시 발생하는 에러 
    ```
    E: The repository 'http://dl.google.com/linux/chrom/deb stable InRelase' is not signed. 
    ```
    => 참고: [apt-get update 에러 해결](https://yunikism.tistory.com/6)
- lubuntu image에 git 이미 깔려 있음. 

#### 3. Visual Studio Code Extension 깔기 
- Remote Development
- Dev Containers

    1. 왼쪽 옆에 컴퓨터 아이콘(Remote Explorer)을 누른다.
    2. Dev Containers에서 dorowu/ubuntu-desktop-lxde-vnc:focal 옆 화살표를 누른다.
    - Dev Containers에서 컨테이너가 보이지 않을 경우, 터미널에 `docker start [컨테이너 이름]` 쓴다
    3. root/ 폴더로 들어가면 lubuntu 이미지에 있는 모든 폴더를 접근할 수 있다.  