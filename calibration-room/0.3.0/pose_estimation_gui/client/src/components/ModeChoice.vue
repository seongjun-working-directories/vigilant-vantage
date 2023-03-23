<template>
  <div class="all">
    <div class="row mb-3">
      <router-link to="/">
        <h2>CALIBRATION ROOM <small><small>_ 0.3.0 version</small></small></h2>
      </router-link>
      <h5>Author - Park Seongjun [ hamishcode@gmail.com ]</h5>
      <p><em>Last publish date: 2023-01-10</em></p>
    </div>

    <!-- integrated modal -->
    <button type="button" class="btn btn-lg btn-warning m-2" data-bs-toggle="modal" style="width: 5em" data-bs-target="#integrated_modal" @click="integrated_launcher">
      Integrated Pose Estimation
    </button>
    <div class="modal fade" id="integrated_modal" tabindex="-1" aria-labelledby="integrated_modal_label" aria-hidden="true">
      <div class="modal-dialog modal-dialog-centered">
        <div class="modal-content">
          <div class="modal-header">
            <h1 class="modal-title fs-5" id="integrated_modal_label" style="color: black !important">Integrated Pose Estimation Launched!</h1>
            <button type="button" class="btn-close" data-bs-dismiss="modal" aria-label="Close"></button>
          </div>
          <div class="modal-body">
            Click `Check Estimation` to check the pose data.
          </div>
          <div class="modal-footer">
            <button type="button" class="btn btn-secondary" data-bs-dismiss="modal">Close</button>
            <router-link to="/all_pose_estimation" class="btn btn-primary">
              Check Estimation
            </router-link>
          </div>
        </div>
      </div>
    </div>

    <!-- depth modal -->
    <button type="button" class="btn btn-lg btn-danger m-2" data-bs-toggle="modal" style="width: 5em" data-bs-target="#depth_modal" @click="depth_launcher">
      Depth Pose Estimation
    </button>
    <div class="modal fade" id="depth_modal" tabindex="-1" aria-labelledby="depth_modal_label" aria-hidden="true">
      <div class="modal-dialog modal-dialog-centered">
        <div class="modal-content">
          <div class="modal-header">
            <h1 class="modal-title fs-5" id="depth_modal_label" style="color: black">Depth Pose Estimation Launched!</h1>
            <button type="button" class="btn-close" data-bs-dismiss="modal" aria-label="Close"></button>
          </div>
          <div class="modal-body">
            Click `Check Estimation` to check the pose data.
          </div>
          <div class="modal-footer">
            <button type="button" class="btn btn-secondary" data-bs-dismiss="modal">Close</button>
            <router-link to="/depth_pose_estimation" class="btn btn-primary">
              Check Estimation
            </router-link>
          </div>
        </div>
      </div>
    </div>

    <!-- <br/> -->

    <!-- lidar modal -->
    <button type="button" class="btn btn-lg btn-info m-2" data-bs-toggle="modal" style="width: 5em" data-bs-target="#lidar_modal" @click="lidar_launcher">
      Lidar Pose Estimation
    </button>
    <div class="modal fade" id="lidar_modal" tabindex="-1" aria-labelledby="lidar_modal_label" aria-hidden="true">
      <div class="modal-dialog modal-dialog-centered">
        <div class="modal-content">
          <div class="modal-header">
            <h1 class="modal-title fs-5" id="lidar_modal_label" style="color: black">Lidar Pose Estimation Launched!</h1>
            <button type="button" class="btn-close" data-bs-dismiss="modal" aria-label="Close"></button>
          </div>
          <div class="modal-body">
            Click `Check Estimation` to check the pose data.<br/>
            (*CAUTION : You should reload the page)
          </div>
          <div class="modal-footer">
            <button type="button" class="btn btn-secondary" data-bs-dismiss="modal">Close</button>
            <router-link to="/lidar_pose_estimation" class="btn btn-primary">
              Check Estimation
            </router-link>
          </div>
        </div>
      </div>
    </div>

    <!-- camera modal -->
    <button type="button" class="btn btn-lg btn-success m-2" data-bs-toggle="modal" style="width: 5em" data-bs-target="#camera_modal" @click="camera_launcher">
      Camera Pose Estimation
    </button>
    <div class="modal fade" id="camera_modal" tabindex="-1" aria-labelledby="camera_modal_label" aria-hidden="true">
      <div class="modal-dialog modal-dialog-centered">
        <div class="modal-content">
          <div class="modal-header">
            <h1 class="modal-title fs-5" id="camera_modal_label" style="color: black">Camera Pose Estimation Launched!</h1>
            <button type="button" class="btn-close" data-bs-dismiss="modal" aria-label="Close"></button>
          </div>
          <div class="modal-body">
            Click `Check Estimation` to check the pose data.
          </div>
          <div class="modal-footer">
            <button type="button" class="btn btn-secondary" data-bs-dismiss="modal">Close</button>
            <router-link to="/camera_pose_estimation" class="btn btn-primary">
              Check Estimation
            </router-link>
          </div>
        </div>
      </div>
    </div>

    <br />
    <p class="mt-4 mb-1" style="color: black">
      Working With Chessboard?&nbsp;&nbsp;&nbsp;
      <router-link to="/chessboard_pose_estimation" @click="chessboard_launcher">
        CLICK ME
      </router-link>
    </p>
    <span>
      (* Default Board Version Is ARUCOBAORD)
    </span>
  </div>
</template>

<script lang="ts" setup>
import axios from 'axios';

// *** 일괄 센서 관련 서버 통신 함수 *** //
const integrated_launcher = async () => {
  await axios({
    url: 'http://localhost:5000/launch_all',
    method: 'get',
    data: {
      // TODO : None
    }
  });
};

// *** depth 관련 서버 통신 함수 *** //
const depth_launcher = async () => {
  await axios({
    url: 'http://localhost:5000/launch_depth',
    method: 'get',
    data: {
      // TODO : Chessboard, Arucoboard Version Division
    }
  });
};

// *** lidar 관련 서버 통신 함수 *** //
const lidar_launcher = async () => {
  await axios({
    url: 'http://localhost:5000/launch_lidar',
    method: 'get',
    data: {
      // TODO : None
    }
  });
};


// *** camera 관련 서버 통신 함수 *** //
const camera_launcher = async () => {
  await axios({
    url: 'http://localhost:5000/launch_camera',
    method: 'get',
    data: {
      // TODO : Chessboard, Arucoboard Version Division
    }
  });
};


// *** chessboard 버전의 depth 관련 서버 통신 함수 *** //
const chessboard_launcher = async () => {
  await axios({
    url: 'http://localhost:5000/launch_chessboard',
    method: 'get',
    data: {
      // TODO : None
    }
  });
};
</script>

<style>
html, body {
  height: 100% !important;
}

body {
  background-color: #444 !important;
  display: flex !important;
  justify-content: center !important;
  align-items:center !important;
}

h1, h2, h5, p {
  color: #f2f2f2 !important;
}

.all .btn {
  display: inline-block;
  margin: .1em;
  min-width: 10em;
}

h1 {
  font-size: 20px;
  font-weight: 300;
  margin: 1em 0;
  text-align: center;
}
</style>