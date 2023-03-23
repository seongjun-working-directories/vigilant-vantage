<template>
  <form>
    <div class="container">
      <HeaderComponent />

      <div class="row mb-3 d-flex justify-content-center">
        <button type="button" class="col-2 btn btn-primary v-mar"  @click="store_pose"><strong>STORE<br/>POSE</strong></button>&nbsp;&nbsp;
        <button type="button" class="col-2 btn btn-dark v-mar" @click="lidar_reference"><strong>ALL LIDAR<br/>REFERENCE</strong></button>&nbsp;&nbsp;
        <button type="button" class="col-2 btn btn-dark v-mar" @click="front_lidar_current"><strong>FRONT LIDAR<br/>CURRENT</strong></button>&nbsp;&nbsp;
        <button type="button" class="col-2 btn btn-dark v-mar" @click="rear_lidar_current"><strong>REAR LIDAR<br/>CURRENT</strong></button>
      </div>

      <div class="row">
        <div id="carouselExampleCaptions" class="carousel slide" data-bs-ride="false">
          <div class="carousel-indicators">
            <button type="button" data-bs-target="#carouselExampleCaptions" data-bs-slide-to="0" class="active" aria-current="true" aria-label="Slide 1"></button>
            <button type="button" data-bs-target="#carouselExampleCaptions" data-bs-slide-to="1" aria-label="Slide 2"></button>
            <button type="button" data-bs-target="#carouselExampleCaptions" data-bs-slide-to="2" aria-label="Slide 3"></button>
            <button type="button" data-bs-target="#carouselExampleCaptions" data-bs-slide-to="3" aria-label="Slide 4"></button>
          </div>
          <div class="carousel-inner">
            <div class="carousel-item active">
              <div class="row">
                <div class="col-md-6">
                  <div class="bordered" style="background-color: #444;">
                    <img :src="depth_image_1" style="width: 100%; height: 100%" />
                  </div>
                </div>
                <div class="col-md-6">
                  <div class="bordered" style="background-color: #444;">
                    <img :src="depth_image_2" style="width: 100%; height: 100%" />
                  </div>
                </div>
              </div>
              <div class="row" style="height:6em"></div>
              <div class="carousel-caption d-none d-md-block">
                <h5>Astra Camera (Stereo_s_u3)</h5>
              </div>
            </div>

            <div class="carousel-item">
              <div class="row">
                <div class="col-md-6">
                  <div class="bordered" style="background-color: #444;">
                    <img :src="front_lidar_image_1" style="width: 100%; height: 100%" />
                  </div>
                </div>
                <div class="col-md-6">
                  <div class="bordered" style="background-color: #444;">
                    <img :src="front_lidar_image_2" style="width: 100%; height: 100%" />
                  </div>
              </div>
              <div class="row" style="height:6em"></div>
              </div>
              <div class="carousel-caption d-none d-md-block">
                <h5>LS LiDAR (LSM10 | LS01B) [FRONT]</h5>
              </div>
            </div>

            <div class="carousel-item">
              <div class="row">
                <div class="col-md-6">
                  <div class="bordered" style="background-color: #444;">
                    <img :src="rear_lidar_image_1" style="width: 100%; height: 100%" />
                  </div>
                </div>
                <div class="col-md-6">
                  <div class="bordered" style="background-color: #444;">
                    <img :src="rear_lidar_image_2" style="width: 100%; height: 100%" />
                  </div>
              </div>
              <div class="row" style="height:6em"></div>
              </div>
              <div class="carousel-caption d-none d-md-block">
                <h5>LS LiDAR (LSM10 | LS01B) [REAR]</h5>
              </div>
            </div>

            <div class="carousel-item">
              <div class="row">
                <div class="col-md-6">
                  <div class="bordered" style="background-color: #444;">
                    <img :src="camera_image_1" style="width: 100%; height: 100%" />
                  </div>
                </div>
                <div class="col-md-6">
                  <div class="bordered" style="background-color: #444;">
                    <img :src="camera_image_2" style="width: 100%; height: 100%" />
                  </div>
                </div>
                <div class="row" style="height:6em"></div>
              </div>
              <div class="carousel-caption d-none d-md-block">
                <h5>SVPRO (USB Web Camera HD)</h5>
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

      <div class="row" style="margin-top: 20px; margin-bottom: 12px;">
        <div class="col-md-12">
          <h2>Estimated Pose</h2>
        </div>
      </div>

      <div class="row d-flex justify-content-center">
        <div class="col-md-6">
          <div class="bordered">
            <div class="row">
              <div class="col-md-12">
                <h3 class="no-top">Astra Camera <small>(Stereo_s_u3)</small></h3>
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

        <div class="col-md-6">
          <div class="bordered">
            <div class="row">
              <div class="col-md-12">
                <h3 class="no-top">SVPRO <small>(USB Web Camera HD)</small></h3>
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
      <div class="row m-2"></div>
      <div class="row d-flex justify-content-center">
        <div class="col-md-6">
          <div class="bordered">
            <div class="row">
              <div class="col-md-12">
                <h3 class="no-top">LS Lidar <small>(LSM10 | LS01B) [FRONT]</small></h3>
              </div>
            </div>
            <div class="row">
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">front_lidar_x</label>
                  <input type="text" class="form-control" id="" :value="front_lidar_x">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">front_lidar_y</label>
                  <input type="text" class="form-control" id="" :value="front_lidar_y">
                </div>
              </div>
            </div>
            <div class="row">
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">front_lidar_z</label>
                  <input type="text" class="form-control" id="" :value="front_lidar_z">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">front_lidar_roll</label>
                  <input type="text" class="form-control" id="" :value="front_lidar_roll">
                </div>
              </div>
            </div>
            <div class="row">
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">front_lidar_pitch</label>
                  <input type="text" class="form-control" id="" :value="front_lidar_pitch">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">front_lidar_yaw</label>
                  <input type="text" class="form-control" id="" :value="front_lidar_yaw">
                </div>
              </div>
            </div>
          </div>
        </div>

        <div class="col-md-6">
          <div class="bordered">
            <div class="row">
              <div class="col-md-12">
                <h3 class="no-top">LS Lidar <small>(LSM10 | LS01B) [REAR]</small></h3>
              </div>
            </div>
            <div class="row">
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">rear_lidar_x</label>
                  <input type="text" class="form-control" id="" :value="rear_lidar_x">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">rear_lidar_y</label>
                  <input type="text" class="form-control" id="" :value="rear_lidar_y">
                </div>
              </div>
            </div>
            <div class="row">
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">rear_lidar_z</label>
                  <input type="text" class="form-control" id="" :value="rear_lidar_z">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">rear_lidar_roll</label>
                  <input type="text" class="form-control" id="" :value="rear_lidar_roll">
                </div>
              </div>
            </div>
            <div class="row">
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">rear_lidar_pitch</label>
                  <input type="text" class="form-control" id="" :value="rear_lidar_pitch">
                </div>
              </div>
              <div class="col-md-6">
                <div class="form-group">
                  <label for="">rear_lidar_yaw</label>
                  <input type="text" class="form-control" id="" :value="rear_lidar_yaw">
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

<script setup>
import HeaderComponent from '../components/HeaderComponent.vue';
import { ref } from 'vue';
import ROSLIB from 'roslib';
import axios from 'axios';

// *** variables *** //
let depth_x = ref('0');
let depth_y = ref('0');
let depth_z = ref('0');
let depth_roll = ref('0');
let depth_pitch = ref('0');
let depth_yaw = ref('0');
let depth_image_1 = ref('');
let depth_image_2 = ref('');

let front_lidar_x = ref('0');
let front_lidar_y = ref('0');
let front_lidar_z = ref('0');
let front_lidar_roll = ref('0');
let front_lidar_pitch = ref('0');
let front_lidar_yaw = ref('0');
let front_lidar_image_1 = ref('');
let front_lidar_image_2 = ref('');

let rear_lidar_x = ref('0');
let rear_lidar_y = ref('0');
let rear_lidar_z = ref('0');
let rear_lidar_roll = ref('0');
let rear_lidar_pitch = ref('0');
let rear_lidar_yaw = ref('0');
let rear_lidar_image_1 = ref('');
let rear_lidar_image_2 = ref('');

let camera_x = ref('0');
let camera_y = ref('0');
let camera_z = ref('0');
let camera_roll = ref('0');
let camera_pitch = ref('0');
let camera_yaw = ref('0');
let camera_image_1 = ref('');
let camera_image_2 = ref('');

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

// *** axios *** //
const store_pose = async () => {
  await axios({
    url: 'http://localhost:5000/store',
    method: 'post',
    data: {
      depth_x : depth_x.value,
      depth_y : depth_y.value,
      depth_z : depth_z.value,
      depth_roll : depth_roll.value,
      depth_pitch : depth_pitch.value,
      depth_yaw : depth_yaw.value,
      front_lidar_x : front_lidar_x.value,
      front_lidar_y : front_lidar_y.value,
      front_lidar_z : front_lidar_z.value,
      front_lidar_roll : front_lidar_roll.value,
      front_lidar_pitch : front_lidar_pitch.value,
      front_lidar_yaw : front_lidar_yaw.value,
      rear_lidar_x : rear_lidar_x.value,
      rear_lidar_y : rear_lidar_y.value,
      rear_lidar_z : rear_lidar_z.value,
      rear_lidar_roll : rear_lidar_roll.value,
      rear_lidar_pitch : rear_lidar_pitch.value,
      rear_lidar_yaw : rear_lidar_yaw.value,
      camera_x : camera_x.value,
      camera_y : camera_y.value,
      camera_z : camera_z.value,
      camera_roll : camera_roll.value,
      camera_pitch : camera_pitch.value,
      camera_yaw : camera_yaw.value,
    }
  });
};

const lidar_reference = async () => {
  await axios({
    url: 'http://localhost:5000/publish_reference',
    data: {
      // TODO : None
    }
  });
};

const front_lidar_current = async () => {
  await axios({
    url: 'http://localhost:5000/publish_front_current',
    data: {
      // TODO : None
    }
  });
};

const rear_lidar_current = async () => {
  await axios({
    url: 'http://localhost:5000/publish_rear_current',
    data: {
      // TODO : None
    }
  });
};

const depth_image_listener_1 = new ROSLIB.Topic({
  ros: ros,
  name: '/depth_image_message_1',
  messageType: 'sensor_msgs/CompressedImage'
});

depth_image_listener_1.subscribe((message) => {
  depth_image_1.value = 'data:image/jpg;base64,' + message.data;
});

const depth_image_listener_2 = new ROSLIB.Topic({
  ros: ros,
  name: '/depth_image_message_2',
  messageType: 'sensor_msgs/CompressedImage'
});

depth_image_listener_2.subscribe((message) => {
  depth_image_2.value = 'data:image/jpg;base64,' + message.data;
});

const front_lidar_image_listener_1 = new ROSLIB.Topic({
  ros: ros,
  name: '/lidar_front_image_message_1',
  messageType: 'sensor_msgs/CompressedImage'
});

front_lidar_image_listener_1.subscribe((message) => {
  front_lidar_image_1.value = 'data:image/jpg;base64,' + message.data;
});

const front_lidar_image_listener_2 = new ROSLIB.Topic({
  ros: ros,
  name: '/lidar_front_image_message_2',
  messageType: 'sensor_msgs/CompressedImage'
});

front_lidar_image_listener_2.subscribe((message) => {
  front_lidar_image_2.value = 'data:image/jpg;base64,' + message.data;
});

const rear_lidar_image_listener_1 = new ROSLIB.Topic({
  ros: ros,
  name: '/lidar_rear_image_message_1',
  messageType: 'sensor_msgs/CompressedImage'
});

rear_lidar_image_listener_1.subscribe((message) => {
  rear_lidar_image_1.value = 'data:image/jpg;base64,' + message.data;
});

const rear_lidar_image_listener_2 = new ROSLIB.Topic({
  ros: ros,
  name: '/lidar_rear_image_message_2',
  messageType: 'sensor_msgs/CompressedImage'
});

rear_lidar_image_listener_2.subscribe((message) => {
  rear_lidar_image_2.value = 'data:image/jpg;base64,' + message.data;
});

const camera_image_listener_1 = new ROSLIB.Topic({
  ros: ros,
  name: '/camera_image_message_1',
  messageType: 'sensor_msgs/CompressedImage'
});

camera_image_listener_1.subscribe((message) => {
  camera_image_1.value = 'data:image/jpg;base64,' + message.data;
});

const camera_image_listener_2 = new ROSLIB.Topic({
  ros: ros,
  name: '/camera_image_message_2',
  messageType: 'sensor_msgs/CompressedImage'
});

camera_image_listener_2.subscribe((message) => {
  camera_image_2.value = 'data:image/jpg;base64,' + message.data;
});

const tf_listener = new ROSLIB.Topic({
  ros: ros,
  name: '/tf',
  messageType: 'tf2_msgs/TFMessage'
});

tf_listener.subscribe((message) => {
  if (message.transforms[0].child_frame_id === '/camera_tf') {
    camera_x.value = message.transforms[0].transform.translation.x.toString();
    camera_y.value = message.transforms[0].transform.translation.y;
    camera_z.value = message.transforms[0].transform.translation.z;
    camera_roll.value = message.transforms[0].transform.rotation.x;
    camera_pitch.value = message.transforms[0].transform.rotation.y;
    camera_yaw.value = message.transforms[0].transform.rotation.z;
  }
  if (message.transforms[0].child_frame_id === '/depth_tf') {
    depth_x.value = message.transforms[0].transform.translation.x.toString();
    depth_y.value = message.transforms[0].transform.translation.y.toString();
    depth_z.value = message.transforms[0].transform.translation.z.toString();
    depth_roll.value = message.transforms[0].transform.rotation.x.toString();
    depth_pitch.value = message.transforms[0].transform.rotation.y.toString();
    depth_yaw.value = message.transforms[0].transform.rotation.z.toString();
  }
  if (message.transforms[0].child_frame_id === '/lidar_front_tf') {
    front_lidar_x.value = message.transforms[0].transform.translation.x.toString();
    front_lidar_y.value = message.transforms[0].transform.translation.y.toString();
    // [INVALID] front_lidar_z.value = message.transforms[0].transform.translation.z.toString();
    // [INVALID] front_lidar_roll.value = message.transforms[0].transform.rotation.x.toString();
    // [INVALID] front_lidar_pitch.value = message.transforms[0].transform.rotation.y.toString();
    front_lidar_yaw.value = message.transforms[0].transform.rotation.z.toString();
  }
  if (message.transforms[0].child_frame_id === '/lidar_rear_tf') {
    rear_lidar_x.value = message.transforms[0].transform.translation.x.toString();
    rear_lidar_y.value = message.transforms[0].transform.translation.y.toString();
    // [INVALID] rear_lidar_z.value = message.transforms[0].transform.translation.z.toString();
    // [INVALID] rear_lidar_roll.value = message.transforms[0].transform.rotation.x.toString();
    // [INVALID] rear_lidar_pitch.value = message.transforms[0].transform.rotation.y.toString();
    rear_lidar_yaw.value = message.transforms[0].transform.rotation.z.toString();
  }
});

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