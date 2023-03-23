import { Router } from 'express';
import type { Request, Response } from 'express';
import { exec } from 'child_process';

const router = Router();

router.get('/all_lidar_reference', async (request: Request, response: Response) => {
  exec('rostopic pub /lidar_01_reference std_msgs/Bool "data: true"', (err, stdout, stderr) => {
    console.log(err);
    console.log(stdout);
    console.error(stderr);
  });
  exec('rostopic pub /lidar_02_reference std_msgs/Bool "data: true"', (err, stdout, stderr) => {
    console.log(err);
    console.log(stdout);
    console.error(stderr);
  });
});

router.get('/lidar_1_current', async (request: Request, response: Response) => {
  exec('rostopic pub /lidar_01_current std_msgs/Bool "data: true"', (err, stdout, stderr) => {
    console.log(err);
    console.log(stdout);
    console.error(stderr);
  });
});

router.get('/lidar_2_current', async (request: Request, response: Response) => {
  exec('rostopic pub /lidar_02_current std_msgs/Bool "data: true"', (err, stdout, stderr) => {
    console.log(err);
    console.log(stdout);
    console.error(stderr);
  });
});

export { router };