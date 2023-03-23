<template>
  <div class="all">
    <div class="container">
			<div class="margin-bottom-basic mb-4">
				<h5>센서의 6-DOF 값을 계산하여 URDF 로 생성합니다.</h5>
				<span style="color:grey;">
          작동 상태는 '
          <span v-if="connection" style="color: green">{{ connection_text }}</span>
          <span v-else style="color: red">{{ connection_text }}</span>
          '이며, 연결된 아이피는
          {{ (ip) ? ip + ' 입니다.' : '없습니다.' }}
        </span>
			</div>
    </div>

    <button type="button" class="btn btn-lg btn-success m-2" style="width: 5em; height: 10em;" @click="launch_on">
      <b>LAUNCH ON</b>
    </button>
    <button type="button" class="btn btn-lg btn-danger m-2" style="width: 5em; height: 10em;" @click="launch_off">
      <b>LAUNCH OFF</b>
    </button>
    <router-link to="/calibration">
      <button type="button" class="btn btn-lg btn-info m-2" style="width: 5em; height: 10em;">
        <b>SHOW RESULT</b>
      </button>
    </router-link>
    <a href="/webviz.html">
      <button type="button" class="btn btn-lg btn-warning m-2" style="width: 5em; height: 10em;">
        <b>WEBVIZ</b>
      </button>
    </a>
    
    <div class="mt-3">
      (* 프로그램에 대한 상세 코드는
      <a target=”_blank” href="https://github.com/hamish-official/calibration-room.git">여기</a>에서 확인 가능합니다.)<br/>
    </div>
  </div>
</template>

<script setup>
import axios from 'axios';
import { ref } from 'vue';
import ROSLIB from 'roslib';

const connection = ref(false);
const connection_text = ref('Disconnected');
const ip = ref('');

const getIp = async () => {
  const data = await axios({
    url: 'http://localhost:5000/get-ip',
    data: {
      // TODO : None
    }
  });

  ip.value = data.data.ip;
};

getIp();

// *** ros *** //
const ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090'
});

ros.on('connection', () => {
  console.log('Connected to websocket server.');
  connection.value = true;
  connection_text.value = 'Connected';
});

ros.on('error', (error) => {
  console.log('Error to connect websocket server: ', error);
  connection.value = false;
  connection_text.value = 'ERROR';
});

ros.on('close', () => {
  console.log('Connection to websocket server closed.');
  connection.value = false;
  connection_text.value = 'Disconnected';
});

const launch_on = async () => {
  setTimeout(function() {
    window.location.reload(true);
  }, 3000);
  await axios({
    url: 'http://localhost:5000/on',
    data: {
      // TODO : None
    }
  });
};

const launch_off = async () => {
  await axios({
    url: 'http://localhost:5000/off',
    data: {
      // TODO : None
    }
  });
};
</script>

<style>
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