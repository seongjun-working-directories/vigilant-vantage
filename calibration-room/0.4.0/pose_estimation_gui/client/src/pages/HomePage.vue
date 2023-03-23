<template>
  <div class="all">
    <HeaderComponent />

    <button type="button" class="btn btn-lg btn-success m-2" style="width: 5em" @click="launch_on">
      LAUNCH ON
    </button>

    <button type="button" class="btn btn-lg btn-danger m-2" style="width: 5em" @click="launch_off">
      LAUNCH OFF
    </button>

    <br/>

    <router-link to="/calibration" type="button" class="btn btn-lg btn-info m-2" style="width: 5em">
      SHOW RESULT
    </router-link>

    <a href="/webviz.html" class="btn btn-lg btn-warning m-2" style="width: 5em">
      WEBVIZ
    </a>

    <br />
    <p class="mt-4 mb-1" style="color: black">
      If you need any help,&nbsp;&nbsp;
      <router-link to="/help">
        CLICK ME
      </router-link>
    </p>
    <span>
      (* 클릭하시면, 매뉴얼 페이지로 이동합니다.)
    </span>
  </div>
</template>

<script setup>
import axios from 'axios';
import HeaderComponent from '../components/HeaderComponent.vue';

const launch_on = async () => {
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