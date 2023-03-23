<template>
  <nav class="navbar navbar-expand-lg navbar-dark bg-dark fixed-top" id="mainNav">
    <div class="container">
      <router-link class="navbar-brand js-scroll-trigger" to="/">
        <span style="color:white">
          CALIBRATION ROOM
        </span>
      </router-link>
      <button class="navbar-toggler" type="button" data-bs-toggle="collapse" data-bs-target="#navbarResponsive" aria-controls="navbarResponsive" aria-expanded="false" aria-label="Toggle navigation">
        <span class="navbar-toggler-icon"></span>
      </button>
      
      <!-- <div class="collapse navbar-collapse" id="navbarResponsive"> -->
      <div class="collapse navbar-collapse justify-content-end" id="navbarResponsive">
        <ul class="navbar-nav ml-auto">
          <li class="nav-item">
            <router-link class="nav-link js-scroll-trigger" to="/help">Instructions</router-link>
          </li>
          <li class="nav-item">
            <router-link class="nav-link js-scroll-trigger" to="/settings">Settings</router-link>
          </li>
        </ul>
      </div>
    </div>
  </nav>
</template>