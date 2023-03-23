<template>
  <form>
    <div class="container">

      <div class="row mb-3">
        <div class="col-md-7">
          <router-link to="/">
            <h2>CALIBRATION ROOM - DEPTH<small><small>_ <span style="color: red;">chessboard*</span></small></small></h2>
          </router-link>
          <h5>Author - Park Seongjun [ hamishcode@gmail.com ]</h5>
          <p><em>Last publish date: 2023-01-10</em></p>
        </div>
        <div class="col-md-5">
          <button type="button" class="btn btn-light v-mar"  @click="store_pose"><strong>STORE POSE</strong></button>&nbsp;
          <button type="button" class="btn btn-light v-mar" @click="intrinsic_launcher"><strong>CALC INTRINSIC PARAM</strong></button>
        </div>
      </div>

      <div class="row">
        <div class="col-md-6">
          <div class="bordered">
            <img :src="depth_chessboard_image_1" style="width: 100%; height: 100%" />
          </div>
          <div class="bordered">
            <img :src="depth_chessboard_image_2" style="width: 100%; height: 100%" />
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
                  <input type="text" class="form-control" id="" :value="depth_chessboard_x">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">depth_y</label>
                  <input type="text" class="form-control" id="" :value="depth_chessboard_y">
                </div>
              </div>
            </div>
            <div class="row">
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">depth_z</label>
                  <input type="text" class="form-control" id="" :value="depth_chessboard_z">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">depth_roll</label>
                  <input type="text" class="form-control" id="" :value="depth_chessboard_roll">
                </div>
              </div>
            </div>
            <div class="row">
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">depth_pitch</label>
                  <input type="text" class="form-control" id="" :value="depth_chessboard_pitch">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">depth_yaw</label>
                  <input type="text" class="form-control" id="" :value="depth_chessboard_yaw">
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
                  <input type="text" class="form-control" id="" :value="depth_only">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">lidar_y</label>
                  <input type="text" class="form-control" id="" :value="depth_only">
                </div>
              </div>
            </div>
            <div class="row">
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">lidar_z</label>
                  <input type="text" class="form-control" id="" :value="depth_only">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">lidar_roll</label>
                  <input type="text" class="form-control" id="" :value="depth_only">
                </div>
              </div>
            </div>
            <div class="row">
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">lidar_pitch</label>
                  <input type="text" class="form-control" id="" :value="depth_only">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">lidar_yaw</label>
                  <input type="text" class="form-control" id="" :value="depth_only">
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
                  <input type="text" class="form-control" id="" :value="depth_only">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">camera_y</label>
                  <input type="text" class="form-control" id="" :value="depth_only">
                </div>
              </div>
            </div>
            <div class="row">
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">camera_z</label>
                  <input type="text" class="form-control" id="" :value="depth_only">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">camera_roll</label>
                  <input type="text" class="form-control" id="" :value="depth_only">
                </div>
              </div>
            </div>
            <div class="row">
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">camera_pitch</label>
                  <input type="text" class="form-control" id="" :value="depth_only">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">camera_yaw</label>
                  <input type="text" class="form-control" id="" :value="depth_only">
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  </form>s
</template>

<script lang="ts" setup>
import { ref } from 'vue';
import axios from 'axios';
import ROSLIB from 'roslib';

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
let depth_chessboard_x = ref('0');
let depth_chessboard_y = ref('0');
let depth_chessboard_z = ref('0');
let depth_chessboard_roll = ref('0');
let depth_chessboard_pitch = ref('0');
let depth_chessboard_yaw = ref('0');

let depth_chessboard_image_1 = ref('');
let depth_chessboard_image_2 = ref('');

const depth_only = 'None';


// *** depth 관련 함수 *** //
const depth_image_listener_1 = new ROSLIB.Topic({
  ros: ros,
  name: '/depth_chessboard_image_message_1',
  messageType: 'sensor_msgs/CompressedImage'
});

depth_image_listener_1.subscribe((message: any) => {
  depth_chessboard_image_1.value = 'data:image/jpg;base64,' + message.data;
});

const depth_image_listener_2 = new ROSLIB.Topic({
  ros: ros,
  name: '/depth_chessboard_image_message_2',
  messageType: 'sensor_msgs/CompressedImage'
});

depth_image_listener_2.subscribe((message: any) => {
  depth_chessboard_image_2.value = 'data:image/jpg;base64,' + message.data;
});


// *** depth_tf 관련 변수 *** //
const tf_listener = new ROSLIB.Topic({
  ros: ros,
  name: '/tf',
  messageType: 'tf2_msgs/TFMessage'
});


// *** depth_chessboard_tf 관련 함수 *** //
tf_listener.subscribe((message: any) => {
  if (message.transforms[0].child_frame_id === 'depth_chessboard_tf') {
    depth_chessboard_x.value = message.transforms[0].transform.translation.x.toString();
    depth_chessboard_y.value = message.transforms[0].transform.translation.y;
    depth_chessboard_z.value = message.transforms[0].transform.translation.z;
    depth_chessboard_roll.value = message.transforms[0].transform.rotation.x;
    depth_chessboard_pitch.value = message.transforms[0].transform.rotation.y;
    depth_chessboard_yaw.value = message.transforms[0].transform.rotation.z;
  }
});


// *** 계산된 pose 데이터를 저장하는 함수 *** //
const store_pose = async () => {
  await axios({
    url: 'http://localhost:5000/store_pose',
    method: 'post',
    data: {
      depth_x : depth_chessboard_x.value,
      depth_y : depth_chessboard_y.value,
      depth_z : depth_chessboard_z.value,
      depth_roll : depth_chessboard_roll.value,
      depth_pitch : depth_chessboard_pitch.value,
      depth_yaw : depth_chessboard_yaw.value,
      lidar_x : depth_only,
      lidar_y : depth_only,
      lidar_z : depth_only,
      lidar_roll : depth_only,
      lidar_pitch : depth_only,
      lidar_yaw : depth_only,
      camera_x : depth_only,
      camera_y : depth_only,
      camera_z : depth_only,
      camera_roll : depth_only,
      camera_pitch : depth_only,
      camera_yaw : depth_only,
    }
  });
};


// *** 카메라 내부 파라미터를 저장하는 함수 *** //
const intrinsic_launcher = async () => {
  // NOT IMPLEMENTED YET
};
</script>

<style scoped>
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