<template>
  <form>
    <div class="container">

      <div class="row mb-3">
        <div class="col-md-7">
          <router-link to="/">
            <h2>CALIBRATION ROOM <small><small>_ 0.3.0 version</small></small></h2>
          </router-link>
          <h5>Author - Park Seongjun [ hamishcode@gmail.com ]</h5>
          <p><em>Last publish date: 2023-01-10</em></p>
        </div>
        <div class="col-md-5">
          <button type="button" class="btn btn-light v-mar"  @click="store_pose"><strong>STORE POSE</strong></button>&nbsp;
          <button type="button" class="btn btn-light v-mar" @click="publish_lidar_reference"><strong>LIDAR REF</strong></button>&nbsp;
          <button type="button" class="btn btn-light v-mar" @click="publish_lidar_current"><strong>LIDAR CUR</strong></button>
        </div>
      </div>

      <div class="row">
        <div id="carouselExampleCaptions" class="carousel slide" data-bs-ride="false">
          <div class="carousel-indicators">
            <button type="button" data-bs-target="#carouselExampleCaptions" data-bs-slide-to="0" class="active" aria-current="true" aria-label="Slide 1"></button>
            <button type="button" data-bs-target="#carouselExampleCaptions" data-bs-slide-to="1" aria-label="Slide 2"></button>
            <button type="button" data-bs-target="#carouselExampleCaptions" data-bs-slide-to="2" aria-label="Slide 3"></button>
          </div>
          <div class="carousel-inner">
            <div class="carousel-item active">
              <div class="row">
                <div class="col-md-6">
                  <div class="bordered">
                    <img :src="depth_image_1" style="width: 100%; height: 100%" />
                  </div>
                  <div class="bordered">
                    <img :src="depth_image_2" style="width: 100%; height: 100%" />
                  </div>
                </div>
                <div class="col-md-6">
                  <div class="bordered">
                    <div class="bordered" style="height: 20em;">
                      <h1><br/>Layout for RVIZ</h1>
                      <h3><br/>This Layout will be soon updated...</h3>
                    </div>
                  </div>
                </div>
              </div>
              <div class="row" style="height:6em"></div>
              <div class="carousel-caption d-none d-md-block">
                <h5>Astra Camera(stereo_s_u3)</h5>
              </div>
            </div>

            <div class="carousel-item">
              <div class="row">
                <div class="col-md-6">
                  <div class="bordered">
                    <img :src="lidar_image_1" style="width: 100%; height: 100%" />
                  </div>
                  <div class="bordered">
                    <img :src="lidar_image_2" style="width: 100%; height: 100%" />
                  </div>
                </div>
                <div class="col-md-6">
                  <div class="bordered">
                    <div class="bordered" style="height: 20em;">
                    <h1><br/>Layout for RVIZ</h1>
                    <h3><br/>This Layout will be soon updated...</h3>
                  </div>
                </div>
              </div>
              <div class="row" style="height:6em"></div>
              </div>
              <div class="carousel-caption d-none d-md-block">
                <h5>LS Lidar(lsm10_v2)</h5>
              </div>
            </div>

            <div class="carousel-item">
              <div class="row">
                <div class="col-md-6">
                  <div class="bordered">
                    <img :src="camera_image_1" style="width: 100%; height: 100%" />
                  </div>
                  <div class="bordered">
                    <img :src="camera_image_2" style="width: 100%; height: 100%" />
                  </div>
                </div>
                <div class="col-md-6">
                  <div class="bordered">
                    <div class="bordered" style="height: 20em;">
                      <h1><br/>Layout for RVIZ</h1>
                      <h3><br/>This Layout will be soon updated...</h3>
                    </div>
                  </div>
                </div>
                <div class="row" style="height:6em"></div>
              </div>
              <div class="carousel-caption d-none d-md-block">
                <h5>Penga Camera(web_cam_1080p)</h5>
              </div>
            </div>
          </div>

          <button class="carousel-control-prev" type="button" data-bs-target="#carouselExampleCaptions" data-bs-slide="prev">
            <span class="carousel-control-prev-icon" aria-hidden="true"></span>
            <span class="visually-hidden">Previous</span>
          </button>
          <button class="carousel-control-next" type="button" data-bs-target="#carouselExampleCaptions" data-bs-slide="next">
            <span class="carousel-control-next-icon" aria-hidden="true"></span>
            <span class="visually-hidden">Next</span>
          </button>
        </div>
      </div>

      <div class="row" style="margin-top: 20px; margin-bottom: 10px;">
        <div class="col-md-12">
          <h2>Estimated Pose</h2>
        </div>
      </div>

      <div class="row">
        <div class="col-md-4">
          <div class="bordered">

            <div class="row">
              <div class="col-md-12">
                <h3 class="no-top">Astra Camera<br/><small>(stereo_s_u3)</small></h3>
              </div>
            </div>
            <div class="row">
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">depth_x</label>
                  <input type="text" class="form-control" id="" :value="depth_x">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">depth_y</label>
                  <input type="text" class="form-control" id="" :value="depth_y">
                </div>
              </div>
            </div>
            <div class="row">
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">depth_z</label>
                  <input type="text" class="form-control" id="" :value="depth_z">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">depth_roll</label>
                  <input type="text" class="form-control" id="" :value="depth_roll">
                </div>
              </div>
            </div>
            <div class="row">
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">depth_pitch</label>
                  <input type="text" class="form-control" id="" :value="depth_pitch">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">depth_yaw</label>
                  <input type="text" class="form-control" id="" :value="depth_yaw">
                </div>
              </div>
            </div>
          </div>
        </div>

        <div class="col-md-4">
          <div class="bordered">
            <div class="row">
              <div class="col-md-12">
                <h3 class="no-top">LS Lidar<br/><small>(lsm10_v2)</small></h3>
              </div>
            </div>
            <div class="row">
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">lidar_x</label>
                  <input type="text" class="form-control" id="" :value="lidar_x">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">lidar_y</label>
                  <input type="text" class="form-control" id="" :value="lidar_y">
                </div>
              </div>
            </div>
            <div class="row">
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">lidar_z</label>
                  <input type="text" class="form-control" id="" :value="lidar_z">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">lidar_roll</label>
                  <input type="text" class="form-control" id="" :value="lidar_roll">
                </div>
              </div>
            </div>
            <div class="row">
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">lidar_pitch</label>
                  <input type="text" class="form-control" id="" :value="lidar_pitch">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">lidar_yaw</label>
                  <input type="text" class="form-control" id="" :value="lidar_yaw">
                </div>
              </div>
            </div>
          </div>
        </div>

        <div class="col-md-4">
          <div class="bordered">
            <div class="row">
              <div class="col-md-12">
                <h3 class="no-top">Penga Camera<br/><small>(web_cam_1080p)</small></h3>
              </div>
            </div>
            <div class="row">
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">camera_x</label>
                  <input type="text" class="form-control" id="" :value="camera_x">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">camera_y</label>
                  <input type="text" class="form-control" id="" :value="camera_y">
                </div>
              </div>
            </div>
            <div class="row">
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">camera_z</label>
                  <input type="text" class="form-control" id="" :value="camera_z">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">camera_roll</label>
                  <input type="text" class="form-control" id="" :value="camera_roll">
                </div>
              </div>
            </div>
            <div class="row">
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">camera_pitch</label>
                  <input type="text" class="form-control" id="" :value="camera_pitch">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">camera_yaw</label>
                  <input type="text" class="form-control" id="" :value="camera_yaw">
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>

      <!--
        <div class="row mt-3">
          <div class="col-md-12">
            <button type="button" class="btn btn-lg btn-primary v-mar" @click="depth_launcher">Depth Camera Launcher</button>&nbsp;
            <button type="button" class="btn btn-lg btn-danger v-mar" @click="lidar_launcher">2D Lidar Launcher</button>&nbsp;
            <button type="button" class="btn btn-lg btn-warning v-mar" @click="camera_launcher">Color Camera Launcher</button>&nbsp;
            <button type="button" class="btn btn-lg btn-success v-mar" @click="integrated_launcher">Integrated Launcher</button>
          </div>
        </div>
      -->
    </div>
  </form>
</template>

<script lang="ts" setup>
import { ref } from 'vue';
import axios from 'axios';
import ROSLIB from 'roslib';
import fs from 'fs';


// *** ros 기본 세팅 관련 코드 *** //
const ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090'
});

ros.on('connection', () => {
  console.log('Connected to websocket server.');
});

ros.on('error', (error: any) => {
  console.log('Error to connect websocket server: ', error);
});

ros.on('close', () => {
  console.log('Connection to websocket server closed.');
});


// *** depth 관련 변수 *** //
let depth_x = ref('0');
let depth_y = ref('0');
let depth_z = ref('0');
let depth_roll = ref('0');
let depth_pitch = ref('0');
let depth_yaw = ref('0');

let depth_image_1 = ref('');
let depth_image_2 = ref('');


// *** lidar 관련 변수 *** //
let lidar_x = ref('0');
let lidar_y = ref('0');
let lidar_z = ref('unknown');
let lidar_roll = ref('unknown');
let lidar_pitch = ref('unknown');
let lidar_yaw = ref('0');

let lidar_image_1 = ref('');
let lidar_image_2 = ref('');


// *** camera 관련 변수 *** //
let camera_x = ref('0');
let camera_y = ref('0');
let camera_z = ref('0');
let camera_roll = ref('0');
let camera_pitch = ref('0');
let camera_yaw = ref('0');

let camera_image_1 = ref('');
let camera_image_2 = ref('');


// *** depth, lidar, camera 에서 송신하는 이미지에 대한 함수 *** //
const depth_image_listener_1 = new ROSLIB.Topic({
  ros: ros,
  name: '/depth_image_message_1',
  messageType: 'sensor_msgs/CompressedImage'
});

depth_image_listener_1.subscribe((message: any) => {
  depth_image_1.value = 'data:image/jpg;base64,' + message.data;
});

const depth_image_listener_2 = new ROSLIB.Topic({
  ros: ros,
  name: '/depth_image_message_2',
  messageType: 'sensor_msgs/CompressedImage'
});

depth_image_listener_2.subscribe((message: any) => {
  depth_image_2.value = 'data:image/jpg;base64,' + message.data;
});

const lidar_image_listener_1 = new ROSLIB.Topic({
  ros: ros,
  name: '/lidar_image_message_1',
  messageType: 'sensor_msgs/CompressedImage'
});

lidar_image_listener_1.subscribe((message: any) => {
  lidar_image_1.value = 'data:image/jpg;base64,' + message.data;
});

const lidar_image_listener_2 = new ROSLIB.Topic({
  ros: ros,
  name: '/lidar_image_message_2',
  messageType: 'sensor_msgs/CompressedImage'
});

lidar_image_listener_2.subscribe((message: any) => {
  lidar_image_2.value = 'data:image/jpg;base64,' + message.data;
});

const camera_image_listener_1 = new ROSLIB.Topic({
  ros: ros,
  name: '/camera_image_message_1',
  messageType: 'sensor_msgs/CompressedImage'
});

camera_image_listener_1.subscribe((message: any) => {
  camera_image_1.value = 'data:image/jpg;base64,' + message.data;
});

const camera_image_listener_2 = new ROSLIB.Topic({
  ros: ros,
  name: '/camera_image_message_2',
  messageType: 'sensor_msgs/CompressedImage'
});

camera_image_listener_2.subscribe((message: any) => {
  camera_image_2.value = 'data:image/jpg;base64,' + message.data;
});


// *** depth, lidar, camera 에서 송신하는 tf에 대한 함수 *** //
const tf_listener = new ROSLIB.Topic({
  ros: ros,
  name: '/tf',
  messageType: 'tf2_msgs/TFMessage'
});

tf_listener.subscribe((message: any) => {
  if (message.transforms[0].child_frame_id === 'camera_tf') {
    camera_x.value = message.transforms[0].transform.translation.x.toString();
    camera_y.value = message.transforms[0].transform.translation.y;
    camera_z.value = message.transforms[0].transform.translation.z;
    camera_roll.value = message.transforms[0].transform.rotation.x;
    camera_pitch.value = message.transforms[0].transform.rotation.y;
    camera_yaw.value = message.transforms[0].transform.rotation.z;
  }
  if (message.transforms[0].child_frame_id === 'depth_tf') {
    depth_x.value = message.transforms[0].transform.translation.x.toString();
    depth_y.value = message.transforms[0].transform.translation.y.toString();
    depth_z.value = message.transforms[0].transform.translation.z.toString();
    depth_roll.value = message.transforms[0].transform.rotation.x.toString();
    depth_pitch.value = message.transforms[0].transform.rotation.y.toString();
    depth_yaw.value = message.transforms[0].transform.rotation.z.toString();
  }
  if (message.transforms[0].child_frame_id === 'lidar_tf') {
    lidar_x.value = message.transforms[0].transform.translation.x.toString();
    lidar_y.value = message.transforms[0].transform.translation.y.toString();
    // [INVALID] lidar_z.value = message.transforms[0].transform.translation.z.toString();
    // [INVALID] lidar_roll.value = message.transforms[0].transform.rotation.x.toString();
    // [INVALID] lidar_pitch.value = message.transforms[0].transform.rotation.y.toString();
    lidar_yaw.value = message.transforms[0].transform.rotation.z.toString();
  }
});


// *** 라이다 모드를 REF로 변경하는 함수 *** //
const publish_lidar_reference = async () => {
  await axios({
    url: 'http://localhost:5000/publish_lidar_reference',
    method: 'get',
    data: {
      // TODO : None
    }
  });
};


// *** 라이다 모드를 CUR로 변경하는 함수 *** //
const publish_lidar_current = async () => {
  await axios({
    url: 'http://localhost:5000/publish_lidar_current',
    method: 'get',
    data: {
      // TODO : None
    }
  });
};


// *** 계산된 pose 데이터를 저장하는 함수 *** //
const store_pose = async () => {
  await axios({
    url: 'http://localhost:5000/store_pose',
    method: 'post',
    data: {
      depth_x : depth_x.value,
      depth_y : depth_y.value,
      depth_z : depth_z.value,
      depth_roll : depth_roll.value,
      depth_pitch : depth_pitch.value,
      depth_yaw : depth_yaw.value,
      lidar_x : lidar_x.value,
      lidar_y : lidar_y.value,
      lidar_z : lidar_z.value,
      lidar_roll : lidar_roll.value,
      lidar_pitch : lidar_pitch.value,
      lidar_yaw : lidar_yaw.value,
      camera_x : camera_x.value,
      camera_y : camera_y.value,
      camera_z : camera_z.value,
      camera_roll : camera_roll.value,
      camera_pitch : camera_pitch.value,
      camera_yaw : camera_yaw.value,
    }
  });
};
</script>

<style>
body {
  background-color: #444 !important;
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
</style>