# Calibration Room 0.3.0 Version

> Park Seongjun [hamishcode@gmail.com]

<br/>

---
<br/>

- calibration-room 0.3.0 version 설명
  - 개별 센서에 대한 알고리즘 테스트용 버전
  -  소스 코드 구조
     - 개발 패키지(MAIN)
       - depth_pose_estimator : RGB-D Camera 위치 및 자세 추정
       - camera_pose_estimator : RGB Camera 위치 및 자세 추정
       - lidar_pose_estimator : 2D LiDAR 위치 및 자세 추정
     - 개발 패키지(SUB)
       - depth_chessboard_pose_estimator : Chessboard 용 RGB-D Camera 위치 및 자세 추정
       - depth_center_finder : Color Image 및 IR Image 간 변환 관계 추정
     - 라이브러리
       - libuvc
       - ls01b_ros
       - 1sm10_ros
       - ros_astra_camera
       - usb_cam

- calibration-room 초기 설정 방법
  - // libuvc
    - cd ~/calibration-room/auto_pose_estimation/src/libuvc
    - mkdir build && cd build
    - cmake .. && make -j4
    - sudo make install
    - sudo ldconfig
  - // build all packages in auto_pose_estimation
    - cd ~/calibration-room/auto_pose_estimation
    - catkin_make
  - // ros_astra_camera
    - source ~/calibration-room/auto_pose_estimation/devel/setup.bash
    - roscd astra_camera
    - ./scripts/create_udev_rules
  - // penga camera rules file
    - roscd depth_(chessboard_)pose_estimator
    - ./scripts/create_camera_rules
  - // install every node_modules for pose_estimation_gui server & client
    - cd ~/calibration-room/pose_estiamtion_gui/server
    - npm install
    - cd ~/calibration-room/pose_estimation_gui/client
    - npm install

<br/>

- calibration-room 사용방법
  - // 권한 부여
    - sudo chmod 777 /dev/ttyUSB*
    - sudo chmod 777 /dev/video*
  - // 설정파일 로드
    - source ~/calibration-room/auto_pose_estimation/devel/setup.bash
  - // 웹 서버 작동
    - cd ~/calibration-room/pose_estimation_gui/server
    - npm run dev
  - // 웹 클라이언트 작동
    - cd ~/calibration-room/pose_estimation_gui/client
    - npm run dev
  - // 이미지가 뜨지 않을 경우, 새로고침을 권장하며, 해당 파트는 차후 개선될 예정

<br/>

- 명령어로 calibration-room을 사용하는 방법
  - DEPTH
    - roslaunch depth_pose_estimator depth_pose_estimator_launch.launch
    - roslaunch depth_chessboard_pose_estimator depth_chessboard_pose_estimator_launch.launch
    - [YET] roslaunch depth_pose_estimator all_pose_estimator_launch.launch
    - [YET] roslaunch depth_chessboard_pose_estimator all_pose_estimator_launch.launch
  - LIDAR
    - [Terminal 1]
      - roslaunch lidar_pose_estimator lidar_pose_estimator_launch.launch
    - [Terminal 2]
      - rostopic pub /lidar_reference std_msgs/Bool "data: true"
    - [Terminal 3]
      - rostopic pub /lidar_current std_msgs/Bool "data: true"
  - CAMERA
    - roslaunch camera_pose_estimator camera_pose_estimator_launch.launch

<br/>

---
<br/>

- camera_pose_estimator
  - RGB Camera에 대한 Pose Estimation
  - Penga 1080p Webcam

- depth_pose_estimator
  - RGB-D Camera에 대한 Pose Estimation
  - Ros Astra Camera Stereo_s_u3
  - Arucoboard Version

- depth_chessboard_pose_estimator
  - RGB-D Camera에 대한 Pose Estimation
  - Ros Astra Camera Stereo_s_u3
  - Chessboard Version

- lidar_pose_estimator
  - 2D LiDAR에 대한 Pose Estimation
  - LSM10 LiDAR

<br/>

---
<br/>

** Checklist **

- [X] 깃허브 calibration-room 레포지토리를 신설합니다.
- [X] Pull Request 적용으로 Main Branch에 대한 Merging 권한을 제한합니다.
- [X] Sub Branch 제작자는 Branch 이름이 자신의 깃허브 아이디가 되도록 합니다.
- [ ] Sub Branch가 Main Branch로 Merge 될 때마다 Minor Version을 1씩 올립니다. ex) 0.1.0 ~~> 0.2.0

<br/>

---
<br/>

- REFERENCES

  - https://learnopencv.com/introduction-to-epipolar-geometry-and-stereo-vision/
  - https://learnopencv.com/depth-perception-using-stereo-camera-python-c/
  - https://betterprogramming.pub/understand-point-clouds-a-simple-ground-detection-algorithm-71aaa0dd2b2d
  - https://limhyungtae.github.io/2021-09-10-ROS-Point-Cloud-Library-(PCL)-2.-%ED%98%95%EB%B3%80%ED%99%98-toROSMsg,-fromROSMsg/
  - https://stackoverflow.com/questions/71443095/how-to-project-a-chessboard-in-camera-coordinates-to-real-world-coordinates-in-o
  - https://github.com/sxyu/rgbdrec/blob/master/src/DepthCamera.cpp
  - https://stackoverflow.com/questions/71443095/how-to-project-a-chessboard-in-camera-coordinates-to-real-world-coordinates-in-o
  - https://saint-swithins-day.tistory.com/56
  - https://github.com/korejan/rigid_transform_3D_cpp/blob/master/rigid_transform_3D.cpp
  - http://nghiaho.com/?page_id=671
  - https://www.crocus.co.kr/1288
  - https://stackoverflow.com/questions/7731742/square-detection-doesnt-find-squares
  - http://daddynkidsmakers.blogspot.com/2015/08/blog-post_59.html