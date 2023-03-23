import { Router } from 'express';
import type { Request, Response } from 'express';
import fs from 'fs';
import { networkInterfaces } from 'os';
import { exec } from 'child_process';

const router = Router();

router.get('/send', async (request: Request, response: Response) => {
  exec('scp -rp cona.urdf cona@192.168.2.2:~/Desktop', (err, stdout, stderr) => {
    console.log(err);
    console.log(stdout);
    console.error(stderr);
  });
});

router.post('/store', async (request: Request, response: Response) => {
  const {
    depth_1_x, depth_1_y, depth_1_z, depth_1_roll, depth_1_pitch, depth_1_yaw,
    lidar_1_x, lidar_1_y, lidar_1_z, lidar_1_roll, lidar_1_pitch, lidar_1_yaw,
    lidar_2_x, lidar_2_y, lidar_2_z, lidar_2_roll, lidar_2_pitch, lidar_2_yaw,
    camera_1_x, camera_1_y, camera_1_z, camera_1_roll, camera_1_pitch, camera_1_yaw
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
    + `\t<link name="rgb_link"/>\n`
    + `\n`
    + `\t<joint name="robot2laser1" type="fixed">\n`
		+ `\t\t<parent link="base_link"/>\n`
    + `\t\t<child link="laser1"/>\n`
    + `\t\t<origin xyz="${lidar_1_x} ${lidar_1_y} ${lidar_1_z}" rpy="${lidar_1_roll} ${lidar_1_pitch} ${lidar_1_yaw}" />\n`
    + `\t</joint>\n`
    + `\n`
    + `\t<joint name="robot2laser2" type="fixed">\n`
    + `\t\t<parent link="base_link"/>\n`
    + `\t\t<child link="laser2"/>\n`
    + `\t\t<origin xyz="${lidar_2_x} ${lidar_2_y} ${lidar_2_z}" rpy="${lidar_2_roll} ${lidar_2_pitch} ${lidar_2_yaw}" />\n`
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
    + `\t\t<origin xyz="${depth_1_x} ${depth_1_y} ${depth_1_z}" rpy="${depth_1_roll} ${depth_1_pitch} ${depth_1_yaw}" />\n`
    + `\t</joint>\n`
    + `\n`
    + `\t<joint name="robot2rgb" type="fixed">\n`
    + `\t\t<parent link="base_link"/>\n`
    + `\t\t<child link="rgb_link"/>\n`
    + `\t\t<origin xyz="${camera_1_x} ${camera_1_y} ${camera_1_z}" rpy="${camera_1_roll} ${camera_1_pitch} ${camera_1_yaw}" />\n`
    + `\t</joint>\n`
    + `\n`
    + `</robot>`;

    fs.writeFileSync('cona.urdf', urdf);

    console.log(`cona.urdf 가 성공적으로 생성되었습니다.`);
});

router.get('/get-ip', async (request: Request, response: Response) => {
  try {
    const nets = networkInterfaces();
    let ip = '';

    if (nets) {
      for (const attribute of Object.keys(nets)) {
        if ((nets[`${attribute}`] as any)[0].address.startsWith('192')) {
          ip = (nets[`${attribute}`] as any)[0].address;
        }
        if ((nets[`${attribute}`] as any)[1].address.startsWith('192')) {
          ip = (nets[`${attribute}`] as any)[1].address;
        }
      }
    }

    return response.status(200).json({
      ip: ip
    });
  }
  catch (err) {
    return response.status(500).json({
      data: null
    });
  }
});

router.get('/get-cr-settings', async (request: Request, response: Response) => {
  try {
    const data = fs.readFileSync('assets/cr-settings.json', 'utf-8');
    return response.status(200).json({
      data: data
    });
  }
  catch (err) {
    return response.status(500).json({
      data: null
    });
  }
});

router.post('/cr-settings', async (request: Request, response: Response) => {
  try {
    fs.writeFileSync('assets/cr-settings.json', request.body.settings);
    return response.status(200).json({
      data: true
    });
  }
  catch (err) {
    return response.status(500).json({
      data: false
    });
  }
});

export { router };