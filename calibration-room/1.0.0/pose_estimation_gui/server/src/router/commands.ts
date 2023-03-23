import { Router } from 'express';
import type { Request, Response } from 'express';
import { exec } from 'child_process';

const router = Router();

router.get('/on', async (request: Request, response: Response) => {
  exec('roslaunch depth_pose_estimator classic_pose_estimator_launch.launch', (err, stdout, stderr) => {
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
  
router.get('/off', async (request: Request, response: Response) => {
  exec('rosnode kill /tf2_web_republisher', (err, stdout, stderr) => {
    console.log(err);
    console.log(stdout);
    console.error(stderr);
  });
  exec('pkill roslaunch', (err, stdout, stderr) => {
    console.log(err);
    console.log(stdout);
    console.error(stderr);
  });
});

export { router };