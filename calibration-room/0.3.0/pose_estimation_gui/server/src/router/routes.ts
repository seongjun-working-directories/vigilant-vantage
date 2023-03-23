import express, { Router } from 'express';
import type { Request, Response } from 'express';
import { exec } from 'child_process';
import fs from 'fs';
// import rosnodejs from 'rosnodejs'

const router = express.Router();
// const node_handle = rosnodejs.nh;

router.get('/launch_depth', async (req: Request, res: Response) => {
  exec('roslaunch depth_pose_estimator depth_pose_estimator_launch.launch', (err, stdout, stderr) => {
    console.log(err);
    console.log(stdout);
    console.log(stderr);
  });
});

router.get('/launch_lidar', async (req: Request, res: Response) => {
  exec('roslaunch lidar_pose_estimator lidar_pose_estimator_launch.launch', (err, stdout, stderr) => {
    console.log(err);
    console.log(stdout);
    console.log(stderr);
  });
});

router.get('/launch_camera', async (req: Request, res: Response) => {
  exec('roslaunch camera_pose_estimator camera_pose_estimator_launch.launch', (err, stdout, stderr) => {
    console.log(err);
    console.log(stdout);
    console.log(stderr);
  });
});

router.get('/launch_all', async (req: Request, res: Response) => {
  exec('roslaunch depth_pose_estimator all_pose_estimator_launch.launch', (err, stdout, stderr) => {
    console.log(err);
    console.log(stdout);
    console.log(stderr);
  });
});

router.get('/launch_chessboard', async (req: Request, res: Response) => {
  exec('roslaunch depth_chessboard_pose_estimator depth_chessboard_pose_estimator_launch.launch', (err, stdout, stderr) => {
    console.log(err);
    console.log(stdout);
    console.log(stderr);
  });
});

router.get('/publish_lidar_reference', async (req: Request, res: Response) => {
  exec('rostopic pub /lidar_reference std_msgs/Bool "data: true"', (err, stdout, stderr) => {
    console.log(err);
    console.log(stdout);
    console.log(stderr);
  });
});

router.get('/publish_lidar_current', async (req: Request, res: Response) => {
  exec('rostopic pub /lidar_current std_msgs/Bool "data: true"', (err, stdout, stderr) => {
    console.log(err);
    console.log(stdout);
    console.log(stderr);
  });
});

router.post('/store_pose', async (req: Request, res: Response) => {
  const {
    depth_x, depth_y, depth_z,
    depth_roll, depth_pitch, depth_yaw,
    lidar_x, lidar_y, lidar_z,
    lidar_roll, lidar_pitch, lidar_yaw,
    camera_x, camera_y, camera_z,
    camera_roll, camera_pitch, camera_yaw,
  } = req.body;

  let content = '';

  const depth_pose_data_xyz =
  `depth_x : ${depth_x}, depth_y : ${depth_y}, depth_z : ${depth_z}\n`;
  const depth_pose_data_rpy =
  `depth_roll : ${depth_roll}, depth_pitch : ${depth_pitch}, depth_yaw : ${depth_yaw}\n`;
  const depth_pose_data = depth_pose_data_xyz.concat(depth_pose_data_rpy);

  const lidar_pose_data_xyz =
  `lidar_x : ${lidar_x}, lidar_y : ${lidar_y}, lidar_z : ${lidar_z}\n`;
  const lidar_pose_data_rpy =
  `lidar_roll : ${lidar_roll}, lidar_pitch : ${lidar_pitch}, lidar_yaw : ${lidar_yaw}\n`;
  const lidar_pose_data = lidar_pose_data_xyz.concat(lidar_pose_data_rpy);

  const camera_pose_data_xyz =
    `camera_x : ${camera_x}, camera_y : ${camera_y}, camera_z : ${camera_z}\n`;
  const camera_pose_data_rpy =
    `camera_roll : ${camera_roll}, camera_pitch : ${camera_pitch}, camera_yaw : ${camera_yaw}\n`;
  const camera_pose_data = camera_pose_data_xyz.concat(camera_pose_data_rpy);

  content = content.concat(depth_pose_data).concat(lidar_pose_data).concat(camera_pose_data);

  await fs.writeFile('estimated_pose.txt', content, err => {
    if (err) {
      console.error(err);
    }
  });

  // [TEST]
  console.log('Estimated Pose(All) Successfully Stored!');
});

export { router };