<template>
  <form>
    <div class="container">

      <div class="row mb-3">
        <div class="col-md-7">
          <router-link to="/">
            <h2>CALIBRATION ROOM - LIDAR<small><small>_ 0.3.0 version</small></small></h2>
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
                  <input type="text" class="form-control" id="" :value="lidar_only">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">depth_y</label>
                  <input type="text" class="form-control" id="" :value="lidar_only">
                </div>
              </div>
            </div>
            <div class="row">
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">depth_z</label>
                  <input type="text" class="form-control" id="" :value="lidar_only">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">depth_roll</label>
                  <input type="text" class="form-control" id="" :value="lidar_only">
                </div>
              </div>
            </div>
            <div class="row">
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">depth_pitch</label>
                  <input type="text" class="form-control" id="" :value="lidar_only">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">depth_yaw</label>
                  <input type="text" class="form-control" id="" :value="lidar_only">
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
                  <input type="text" class="form-control" id="" :value="lidar_only">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">camera_y</label>
                  <input type="text" class="form-control" id="" :value="lidar_only">
                </div>
              </div>
            </div>
            <div class="row">
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">camera_z</label>
                  <input type="text" class="form-control" id="" :value="lidar_only">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">camera_roll</label>
                  <input type="text" class="form-control" id="" :value="lidar_only">
                </div>
              </div>
            </div>
            <div class="row">
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">camera_pitch</label>
                  <input type="text" class="form-control" id="" :value="lidar_only">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">camera_yaw</label>
                  <input type="text" class="form-control" id="" :value="lidar_only">
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  </form>
</template>


<script lang="ts" setup>
import { ref } from 'vue';
import axios from 'axios';
import ROSLIB from 'roslib';


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


// *** lidar 관련 변수 *** //
let lidar_x = ref('0');
let lidar_y = ref('0');
let lidar_z = ref('Unknown');
let lidar_roll = ref('Unknown');
let lidar_pitch = ref('Unknown');
let lidar_yaw = ref('0');

let lidar_image_1 = ref('');
let lidar_image_2 = ref('');

const lidar_only = 'None'


// *** lidar 관련 함수 *** //
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


// *** lidar_tf 관련 변수 *** //
const tf_listener = new ROSLIB.Topic({
  ros: ros,
  name: '/tf',
  messageType: 'tf2_msgs/TFMessage'
});


// *** lidar_tf 관련 함수 *** //
tf_listener.subscribe((message: any) => {
  if (message.transforms[0].child_frame_id === 'lidar_tf') {
    lidar_x.value = message.transforms[0].transform.translation.x.toString();
    lidar_y.value = message.transforms[0].transform.translation.y;
    // [INVALID] lidar_z.value = message.transforms[0].transform.translation.z.toString();
    // [INVALID] lidar_roll.value = message.transforms[0].transform.rotation.x.toString();
    // [INVALID] lidar_pitch.value = message.transforms[0].transform.rotation.y.toString();
    lidar_yaw.value = message.transforms[0].transform.rotation.z;

  }
});


// *** lidar_mode를 REF로 변경하는 함수 *** //
const publish_lidar_reference = async () => {
  await axios({
    url: 'http://localhost:5000/publish_lidar_reference',
    method: 'get',
    data: {
      // TODO : None
    }
  });
};


// *** lidar_mode를 CUR로 변경하는 함수 *** //
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
      depth_x : lidar_only,
      depth_y : lidar_only,
      depth_z : lidar_only,
      depth_roll : lidar_only,
      depth_pitch : lidar_only,
      depth_yaw : lidar_only,
      lidar_x : lidar_x.value,
      lidar_y : lidar_y.value,
      lidar_z : lidar_z.value,
      lidar_roll : lidar_roll.value,
      lidar_pitch : lidar_pitch.value,
      lidar_yaw : lidar_yaw.value,
      camera_x : lidar_only,
      camera_y : lidar_only,
      camera_z : lidar_only,
      camera_roll : lidar_only,
      camera_pitch : lidar_only,
      camera_yaw : lidar_only,
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