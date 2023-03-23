<template>
  <div class="mt-5">
    <div class="row mb-3 d-flex justify-content-center">
      <button type="button" class="col-2 mx-1 btn btn-primary v-mar"  style="width: 12em; height: 4em;" @click="()=>mode=1"><strong>6-DOF DATA</strong></button>
      <button type="button" class="col-2 mx-1 btn btn-primary v-mar" style="width: 12em; height: 4em;" @click="()=>mode=2"><strong>SENSOR IMAGE</strong></button>
      <!-- [DEPRECATED]
        <button type="button" class="col-2 mx-1 btn btn-dark v-mar" style="width: 12em; height: 4em;" @click="all_lidar_reference"><strong>ALL LIDAR<br/>REFERENCE</strong></button>
      -->
      <button type="button" class="col-2 mx-1 btn btn-dark v-mar" style="width: 12em; height: 4em;" @click="lidar_1_current"><strong>LIDAR 1<br/>CURRENT</strong></button>
      <button type="button" class="col-2 mx-1 btn btn-dark v-mar" style="width: 12em; height: 4em;" @click="lidar_2_current"><strong>LIDAR 2<br/>CURRENT</strong></button>
    </div>

    <div v-if="mode == 1" class="row mb-3 d-flex justify-content-center">
      <div class="row mt-4 mb-3">
        <div class="col-md-12">
          <h2>6-DOF Data</h2>
        </div>
      </div>
      <div class="row d-flex justify-content-center">
        <div class="col-md-4 background">
          <div class="bordered mb-2">
            <div class="row">
              <h3 class="no-top">Depth 1</h3>
            </div>
            <div class="row">
              <DofComponent :sensor="depth_1" />
            </div>
          </div>
          <div class="bordered mb-2">
            <div class="row">
              <h3 class="no-top">Camera 1</h3>
            </div>
            <div class="row">
              <DofComponent :sensor="camera_1" />
            </div>
          </div>
          <!-- <div class="bordered mb-2" style="background: rgba(0, 0, 0, 0.2)">
            <div class="row">
              <h3 class="no-top">Depth 2</h3>
            </div>
            <div class="row">
              <DofComponent :sensor="depth_2" />
            </div>
          </div> -->
        </div>
          
        <div class="col-md-4">
          <div class="bordered mb-2">
            <div class="row">
              <h3 class="no-top">Lidar 1</h3>
            </div>
            <div class="row">
              <DofComponent :sensor="lidar_1" />
            </div>
          </div>
          <div class="bordered mb-2">
            <div class="row">
              <h3 class="no-top">Lidar 2</h3>
            </div>
            <div class="row">
              <DofComponent :sensor="lidar_2" />
            </div>
          </div>
        </div>

        <div class="col-md-4">
          <!-- <div class="bordered mb-2">
            <div class="row">
              <h3 class="no-top">Camera 1</h3>
            </div>
            <div class="row">
              <DofComponent :sensor="camera_1" />
            </div>
          </div> -->
          <div class="bordered mb-2">
            <button class="btn btn-secondary" style="width: 100%; height: 14.1em;" @click="store_pose">
              <span><h5>캘리브레이션 결과 추출</h5></span>
            </button>
          </div>
          <div class="bordered mb-2">
            <button class="btn btn-secondary" style="width: 100%; height: 14.1em;" @click="send_pose">
              <span><h5>추출된 파일 전송</h5></span>
            </button>
          </div>
        </div>
      </div>
    </div>
    
    <div v-else-if="mode == 2" class="row mb-3 d-flex justify-content-center">
      <div class="row mt-5">
        <div class="col-md-12">
          <h2>Sensor Data</h2>
        </div>
      </div>
      
      <CarouselComponent :ros="ros" :sensors="sensors" :topics="topics" :images="images" />
    </div>
  </div>
</template>

<script setup>
import CarouselComponent from '../components/CarouselComponent.vue';
import DofComponent from '../components/DofComponent.vue';
import { ref } from 'vue';
import ROSLIB from 'roslib';
import axios from 'axios';

const mode = ref(1);

// *** ros *** //
const ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090'
});

ros.on('connection', () => {
  console.log('Connected to websocket server.');
});

ros.on('error', (error) => {
  console.log('Error to connect websocket server: ', error);
});

ros.on('close', () => {
  console.log('Connection to websocket server closed.');
});

// *** variables *** //
const camera_1 = { x: ref('0'), y: ref('0'), z: ref('0'), roll: ref('0'), pitch: ref('0'), yaw: ref('0') };
const depth_1 = { x: ref('0'), y: ref('0'), z: ref('0'), roll: ref('0'), pitch: ref('0'), yaw: ref('0') };
const depth_2 = { x: ref('0'), y: ref('0'), z: ref('0'), roll: ref('0'), pitch: ref('0'), yaw: ref('0') };
const lidar_1 = { x: ref('0'), y: ref('0'), z: ref('0'), roll: ref('0'), pitch: ref('0'), yaw: ref('0') };
const lidar_2 = { x: ref('0'), y: ref('0'), z: ref('0'), roll: ref('0'), pitch: ref('0'), yaw: ref('0') };

const sensors = [depth_1, depth_2, lidar_1, lidar_2, camera_1];

const topics = {
  depth_1_image_1: ROSLIB.Topic,
  depth_1_image_2: ROSLIB.Topic,
  depth_2_image_1: ROSLIB.Topic,
  depth_2_image_2: ROSLIB.Topic,
  lidar_1_image_1: ROSLIB.Topic,
  lidar_1_image_2: ROSLIB.Topic,
  lidar_2_image_1: ROSLIB.Topic,
  lidar_2_image_2: ROSLIB.Topic,
  camera_1_image_1: ROSLIB.Topic,
  camera_1_image_2: ROSLIB.Topic,
};

const images = {
  depth_1_image: { image_1: ref(''), image_2: ref('') },
  depth_2_image: { image_1: ref(''), image_2: ref('') },
  lidar_1_image: { image_1: ref(''), image_2: ref('') },
  lidar_2_image: { image_1: ref(''), image_2: ref('') },
  camera_1_image: { image_1: ref(''), image_2: ref('') }
};

topics.depth_1_image_1 = new ROSLIB.Topic({
  ros: ros,
  name: '/depth_01_image_01',
  messageType: 'sensor_msgs/CompressedImage'
});

topics.depth_1_image_2 = new ROSLIB.Topic({
  ros: ros,
  name: '/depth_01_image_02',
  messageType: 'sensor_msgs/CompressedImage'
});

topics.depth_2_image_1 = new ROSLIB.Topic({
  ros: ros,
  name: '/depth_02_image_01',
  messageType: 'sensor_msgs/CompressedImage'
});

topics.depth_2_image_2 = new ROSLIB.Topic({
  ros: ros,
  name: '/depth_02_image_02',
  messageType: 'sensor_msgs/CompressedImage'
});

topics.camera_1_image_1 = new ROSLIB.Topic({
  ros: ros,
  name: '/camera_01_image_01',
  messageType: 'sensor_msgs/CompressedImage'
});

topics.camera_1_image_2 = new ROSLIB.Topic({
  ros: ros,
  name: '/camera_01_image_02',
  messageType: 'sensor_msgs/CompressedImage'
});

topics.lidar_1_image_1 = new ROSLIB.Topic({
  ros: ros,
  name: '/lidar_01_image_01',
  messageType: 'sensor_msgs/CompressedImage'
});

topics.lidar_1_image_2 = new ROSLIB.Topic({
  ros: ros,
  name: '/lidar_01_image_02',
  messageType: 'sensor_msgs/CompressedImage'
});

topics.lidar_2_image_1 = new ROSLIB.Topic({
  ros: ros,
  name: '/lidar_02_image_01',
  messageType: 'sensor_msgs/CompressedImage'
});

topics.lidar_2_image_2 = new ROSLIB.Topic({
  ros: ros,
  name: '/lidar_02_image_02',
  messageType: 'sensor_msgs/CompressedImage'
});

topics.depth_1_image_1.subscribe((message) => {
  images.depth_1_image.image_1.value = 'data:image/jpg;base64,' + message.data;
});

topics.depth_1_image_2.subscribe((message) => {
  images.depth_1_image.image_2.value = 'data:image/jpg;base64,' + message.data;
});

topics.depth_2_image_1.subscribe((message) => {
  images.depth_2_image.image_1.value = 'data:image/jpg;base64,' + message.data;
});

topics.depth_2_image_2.subscribe((message) => {
  images.depth_2_image.image_2.value = 'data:image/jpg;base64,' + message.data;
});

topics.camera_1_image_1.subscribe((message) => {
  images.camera_1_image.image_1.value = 'data:image/jpg;base64,' + message.data;
});

topics.camera_1_image_2.subscribe((message) => {
  images.camera_1_image.image_2.value = 'data:image/jpg;base64,' + message.data;
});

topics.lidar_1_image_1.subscribe((message) => {
  images.lidar_1_image.image_1.value = 'data:image/jpg;base64,' + message.data;
});

topics.lidar_1_image_2.subscribe((message) => {
  images.lidar_1_image.image_2.value = 'data:image/jpg;base64,' + message.data;
});

topics.lidar_2_image_1.subscribe((message) => {
  images.lidar_2_image.image_1.value = 'data:image/jpg;base64,' + message.data;
});

topics.lidar_2_image_2.subscribe((message) => {
  images.lidar_2_image.image_2.value = 'data:image/jpg;base64,' + message.data;
});

// *** axios *** //
const store_pose = async () => {
  await axios({
    url: 'http://localhost:5000/store',
    method: 'post',
    data: {
      depth_1_x : depth_1.x.value,
      depth_1_y : depth_1.y.value,
      depth_1_z : depth_1.z.value,
      depth_1_roll : depth_1.roll.value,
      depth_1_pitch : depth_1.pitch.value,
      depth_1_yaw : depth_1.yaw.value,
      lidar_1_x : lidar_1.x.value,
      lidar_1_y : lidar_1.y.value,
      lidar_1_z : lidar_1.z.value,
      lidar_1_roll : lidar_1.roll.value,
      lidar_1_pitch : lidar_1.pitch.value,
      lidar_1_yaw : lidar_1.yaw.value,
      lidar_2_x : lidar_2.x.value,
      lidar_2_y : lidar_2.y.value,
      lidar_2_z : lidar_2.z.value,
      lidar_2_roll : lidar_2.roll.value,
      lidar_2_pitch : lidar_2.pitch.value,
      lidar_2_yaw : lidar_2.yaw.value,
      camera_1_x : camera_1.x.value,
      camera_1_y : camera_1.y.value,
      camera_1_z : camera_1.z.value,
      camera_1_roll : camera_1.roll.value,
      camera_1_pitch : camera_1.pitch.value,
      camera_1_yaw : camera_1.yaw.value,
    }
  });
};

const send_pose = async () => {
  await axios({
    url: 'http://localhost:5000/send',
    method: 'get',
    data: {
      // YET
    }
  });
};

/* [DEPRECATED]
const all_lidar_reference = async () => {
  await axios({
    url: 'http://localhost:5000/all_lidar_reference',
    data: {
      // TODO : None
    }
  });
};
*/

const lidar_1_current = async () => {
  await axios({
    url: 'http://localhost:5000/lidar_1_current',
    data: {
      // TODO : None
    }
  });
};

const lidar_2_current = async () => {
  await axios({
    url: 'http://localhost:5000/lidar_2_current',
    data: {
      // TODO : None
    }
  });
};

const quaternion2euler = (sensor, message)=>{
  sensor.x.value = message.transforms[0].transform.translation.x.toString().substring(0, 6);
  sensor.y.value = message.transforms[0].transform.translation.y.toString().substring(0, 6);
  sensor.z.value = message.transforms[0].transform.translation.z.toString().substring(0, 6);

  let qx, qy, qz, qw;
  qx = message.transforms[0].transform.rotation.x;
  qy = message.transforms[0].transform.rotation.y;
  qz = message.transforms[0].transform.rotation.z;
  qw = message.transforms[0].transform.rotation.w;

  let t0, t1, t2, t3, t4;

  t0 = 2.0 * (qw*qx + qy*qz);
  t1 = 1.0 - 2.0 * (qx*qx + qy*qy);
  t2 = 2.0 * (qw*qy - qz*qx);
  if (t2 > 1.0) t2 = 1.0;
  if (t2 < -1.0) t2 = -1.0;
  t3 = 2.0 * (qw*qz + qx*qy);
  t4 = 1.0 - 2.0 * (qy*qy + qz*qz);

  sensor.roll.value = Math.atan2(t0, t1).toString().substring(0, 6);
  sensor.pitch.value = Math.asin(t2).toString().substring(0, 6);
  sensor.yaw.value = Math.atan2(t3, t4).toString().substring(0, 6);
}

const tf_listener = new ROSLIB.Topic({
  ros: ros,
  name: '/tf',
  messageType: 'tf2_msgs/TFMessage'
});

tf_listener.subscribe((message) => {
  if (message.transforms[0].child_frame_id === 'camera_01_tf') {
    quaternion2euler(camera_1, message);
  }
  if (message.transforms[0].child_frame_id === 'depth_01_tf') {
    quaternion2euler(depth_1, message);
  }
  if (message.transforms[0].child_frame_id === 'depth_02_tf') {
    quaternion2euler(depth_2, message);
  }
  if (message.transforms[0].child_frame_id === 'lidar_01_tf') {
    lidar_1.x.value = (-1) * message.transforms[0].transform.translation.y.toString().substring(0, 6);
    lidar_1.y.value = message.transforms[0].transform.translation.x.toString().substring(0, 6);
    // [INVALID] lidar_1.z.value = message.transforms[0].transform.translation.z.toString().substring(0, 6);
    // [INVALID] lidar_1.roll.value = message.transforms[0].transform.rotation.x.toString().substring(0, 6);
    // [INVALID] lidar_1.pitch.value = message.transforms[0].transform.rotation.y.toString().substring(0, 6);
    lidar_1.yaw.value = message.transforms[0].transform.rotation.z.toString().substring(0, 6);
  }
  if (message.transforms[0].child_frame_id === 'lidar_02_tf') {
    lidar_2.x.value = message.transforms[0].transform.translation.x.toString().substring(0, 6);
    lidar_2.y.value = message.transforms[0].transform.translation.y.toString().substring(0, 6);
    // [INVALID] lidar_2.z.value = message.transforms[0].transform.translation.z.toString().substring(0, 6);
    // [INVALID] lidar_2.roll.value = message.transforms[0].transform.rotation.x.toString().substring(0, 6);
    // [INVALID] lidar_2.pitch.value = message.transforms[0].transform.rotation.y.toString().substring(0, 6);
    lidar_2.yaw.value = (message.transforms[0].transform.rotation.z + 3.14).toString().substring(0, 6);
  }
});
</script>

<style scoped>
body {
  background-color: #444 !important;
  background-size: cover !important;
  overflow: scroll !important;
}
h2, h5, p {
  color: #f2f2f2 !important;
}
.bootstrap-select.btn-group .dropdown-menu li a:focus {
  border: 0;
  outline: none !important;
}
.bordered {
  background-color: #fff;
  padding: 15px;
  border: 1px solid #ccc;
  border-radius: .6rem;
}
.v-mar {
  margin: 15px 0;
}
.no-top {
  margin-top: 0;
}
.container {
  margin-bottom: 10rem;
}
label {
  font-weight: normal;
}

.background {
  position: relative;
}

.layer {
  background-color: rgba(0, 0, 0, 0.2);
  position: absolute;
}
</style>