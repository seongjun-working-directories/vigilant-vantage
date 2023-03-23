import express, { Router } from 'express';
import type { Request, Response } from 'express';
import { exec } from 'child_process';
import fs from 'fs';

const router = express.Router();

router.get('/on', async (request: Request, response: Response) => {
  exec('roslaunch depth_pose_estimator integrated_pose_estimator_launch.launch', (err, stdout, stderr) => {
    console.log(err);
    console.log(stdout);
    console.error(stderr);
  });
  exec('rosrun tf2_web_republisher tf2_web_republisher', (err, stdout, stderr) => {
    console.log(err);
    console.log(stdout);
    console.error(stderr);
  });
});

router.get('/publish_reference', async (request: Request, response: Response) => {
  exec('rostopic pub /lidar_front_reference std_msgs/Bool "data: true"', (err, stdout, stderr) => {
    console.log(err);
    console.log(stdout);
    console.error(stderr);
  });
  exec('rostopic pub /lidar_rear_reference std_msgs/Bool "data: true"', (err, stdout, stderr) => {
    console.log(err);
    console.log(stdout);
    console.error(stderr);
  });
});

router.get('/publish_front_current', async (request: Request, response: Response) => {
  exec('rostopic pub /lidar_front_current std_msgs/Bool "data: true"', (err, stdout, stderr) => {
    console.log(err);
    console.log(stdout);
    console.error(stderr);
  });
});

router.get('/publish_rear_current', async (request: Request, response: Response) => {
  exec('rostopic pub /lidar_rear_current std_msgs/Bool "data: true"', (err, stdout, stderr) => {
    console.log(err);
    console.log(stdout);
    console.error(stderr);
  });
});

router.post('/store', async (request: Request, response: Response) => {
  const {
    depth_x, depth_y, depth_z, depth_roll, depth_pitch, depth_yaw,
    front_lidar_x, front_lidar_y, front_lidar_z,
    front_lidar_roll, front_lidar_pitch, front_lidar_yaw,
    rear_lidar_x, rear_lidar_y, rear_lidar_z,
    rear_lidar_roll, rear_lidar_pitch, rear_lidar_yaw,
    camera_x, camera_y, camera_z, camera_roll, camera_pitch, camera_yaw
  } = request.body;

  const urdf =
    `<robot name="cona">\n`
    + `\n`
    + `\t<link name="base_link"/>\n`
    +	`\t<link name="laser1"/>\n`
    +	`\t<link name="laser2"/>\n`
    +	`\t<link name="laser3"/>\n`
    +	`\t<link name="laser_multi"/>\n`
    +	`\t<link name="camera_link"/>\n`
    + `\n`
    + `\t<joint name="robot2laser1" type="fixed">\n`
		+ `\t\t<parent link="base_link"/>\n`
    + `\t\t<child link="laser1"/>\n`
    + `\t\t<origin xyz="${front_lidar_x} ${front_lidar_y} ${front_lidar_z}" rpy="${front_lidar_roll} ${front_lidar_pitch} ${front_lidar_yaw}" />\n`
    + `\t</joint>\n`
    + `\n`
    + `\t<joint name="robot2laser2" type="fixed">\n`
    + `\t\t<parent link="base_link"/>\n`
    + `\t\t<child link="laser2"/>\n`
    + `\t\t<origin xyz="${rear_lidar_x} ${rear_lidar_y} ${rear_lidar_z}" rpy="${rear_lidar_roll} ${rear_lidar_pitch} ${rear_lidar_yaw}" />\n`
    + `\t</joint>\n`
    + `\n`
    + `\t<joint name="robot2laser3" type="fixed">\n`
    + `\t\t<parent link="base_link"/>\n`
    + `\t\t<child link="laser3"/>\n`
		+ `\t\t<origin xyz="0.0 0.0 0.0" rpy="0 0.0 0" />\n`
    + `\t</joint>\n`
    + `\n`
    + `\t<joint name="robot2laser_multi" type="fixed">\n`
    + `\t\t<parent link="base_link" />\n`
    + `\t\t<child link="laser_multi" />\n`
    + `\t\t<origin xyz="0 0 0" rpy="0 0 0" />\n`
    + `\t</joint>\n`
    + `\n`
    + `\t<joint name="robot2depth" type="fixed">\n`
    + `\t\t<parent link="base_link"/>\n`
    + `\t\t<child link="camera_link"/>\n`
    + `\t\t<origin xyz="${depth_x} ${depth_y} ${depth_z}" rpy="${depth_roll} ${depth_pitch} ${depth_yaw}" />\n`
    + `\t</joint>\n`
    + `\n`
    + `</robot>`;

    fs.writeFileSync('cona.urdf', urdf);

    console.log(`cona.urdf 가 성공적으로 생성되었습니다.`);
});

router.get('/off', async (request: Request, response: Response) => {
  exec('pkill roslaunch', (err, stdout, stderr) => {
    console.log(err);
    console.log(stdout);
    console.error(stderr);
  });
});

export { router };