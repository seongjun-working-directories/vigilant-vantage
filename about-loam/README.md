# About Loam
> J. Zhang and S. Singh: LOAM: Lidar Odometry and Mapping in Real- time

<br/>

- study
  - code_study
    - https://github.com/HKUST-Aerial-Robotics/A-LOAM
  - paper_study
    - J. Zhang and S. Singh: LOAM: Lidar Odometry and Mapping in Real- time

<br/>

- kitti
  - REFERENCE
    - https://idorobotics.com/2019/05/29/converting-the-kitti-dataset-to-rosbags/
    - https://www.cvlibs.net/datasets/kitti/eval_odometry.php
  - SEQUENCE
    - $ wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_10_03_drive_0027/2011_10_03_drive_0027_sync.zip
    - $ wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_10_03_calib.zip
    - $ unzip 2011_10_03_drive_0027_sync.zip
    - $ unzip 2011_10_03_calib.zip
    - $ sudo apt install python3-pip
    - $ sudo pip install numpy --upgrade
    - $ sudo apt install python3-virtualenv
    - $ cd ~/vigilant-vantage/about-loam/kitti/
    - $ virtualenv -p /usr/bin/python3 my_project
    - $ source my_project/bin/activate
    - $ sudo pip install kitti2bag
    - $ sudo pip install rospkg catkin_pkg
    - $ sudo pip install pillow --upgrade
    - $ sudo pip install scikit-image
    - $ sudo pip install pykitti
    - $ sudo pip install opencv-python
    - $ pip3 install pycryptodomex
    - $ pip install gnupg
    - $ cd ~/vigilant-vantage/about-loam/kitti/data
    - $ kitti2bag -t 2011_10_03 -r 0027 raw_synced
      - 단, 해당 명령을 수행하기 이전, ~/vigilant-vantage/about-loam/kitti/my_project/bin/kitti2bag.py 파일에서
      - 코드를 '''\
        import sys\
        import pykitti\
        \# try:\
        \# except ImportError as e:\
        \#     print('Could not load module \'pykitti\'. Please run `pip install pykitti`')\
        \#     sys.exit(1)\
        '''와 같이 변경해야 함
    - $ sudo mv kitti_2011_10_03_drive_0027_synced.bag ../bag
    - $ rosbag play kitti_2011_10_03_drive_0027_synced.bag
    - \# This will create a bag file named kitti_2011_10_03_drive_0027_synced.bag.