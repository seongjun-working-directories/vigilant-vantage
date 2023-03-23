<template>
  <div class="row mt-3">
    <div id="carouselExampleCaptions" class="carousel slide" data-bs-ride="false">
      <div class="carousel-indicators">
        <button type="button" data-bs-target="#carouselExampleCaptions" data-bs-slide-to="0" class="active" aria-current="true" aria-label="Slide 1"></button>
        <button type="button" data-bs-target="#carouselExampleCaptions" data-bs-slide-to="1" aria-label="Slide 2"></button>
        <button type="button" data-bs-target="#carouselExampleCaptions" data-bs-slide-to="2" aria-label="Slide 3"></button>
      </div>
      <div class="carousel-inner">
        <div class="carousel-item active">
          <p>Depth 1</p>
          <div class="row">
            <div class="col-md-6">
              <div class="bordered" style="background-color: #444;">
                <img :src="images.depth_1_image.image_1.value" style="width: 100%; height: 100%" />
              </div>
            </div>
            <div class="col-md-6">
              <div class="bordered" style="background-color: #444;">
                <img :src="images.depth_1_image.image_2.value" style="width: 100%; height: 100%" />
              </div>
            </div>
            <div class="row d-flex justify-content-center mt-3 mb-5" style="background-color: white;">
              x: {{  sensors[0].x.value }} , y: {{  sensors[0].y.value }} , z: {{ sensors[0].z.value }} , 
              roll: {{  sensors[0].roll.value }} , pitch: {{  sensors[0].pitch.value }} , yaw: {{  sensors[0].yaw.value }}
            </div>
          </div>
          <!-- [DEPRECATED]
            <p>Depth 2</p>
            <div class="row">
              <div class="col-md-6">
                <div class="bordered" style="background-color: #444;">
                  <img :src="images.depth_2_image.image_1.value" style="width: 100%; height: 100%" />
                </div>
              </div>
              <div class="col-md-6">
                <div class="bordered" style="background-color: #444;">
                  <img :src="images.depth_2_image.image_2.value" style="width: 100%; height: 100%" />
                </div>
              </div>
              <div class="row d-flex justify-content-center mt-3 mb-5" style="background-color: white;">
                x: {{  sensors[1].x.value }} , y: {{  sensors[1].y.value }} , z: {{ sensors[1].z.value }} , 
                roll: {{  sensors[1].roll.value }} , pitch: {{  sensors[1].pitch.value }} , yaw: {{  sensors[1].yaw.value }}
              </div>
            </div>
          -->
          <!-- <div class="carousel-caption d-none d-md-block">
            <h5>Depth</h5>
          </div> -->
        </div>

        <div class="carousel-item">
          <p>Lidar 1</p>
          <div class="row">
            <div class="col-md-6">
              <div class="bordered" style="background-color: #444;">
                <img :src="images.lidar_1_image.image_1.value" style="width: 100%; height: 80%" />
              </div>
            </div>
            <div class="col-md-6">
              <div class="bordered" style="background-color: #444;">
                <img :src="images.lidar_1_image.image_2.value" style="width: 100%; height: 80%" />
              </div>
            </div>
            <div class="row d-flex justify-content-center mt-3 mb-5" style="background-color: white;">
              x: {{  sensors[2].x.value }} , y: {{  sensors[2].y.value }} , z: {{ sensors[2].z.value }} , 
              roll: {{  sensors[2].roll.value }} , pitch: {{  sensors[2].pitch.value }} , yaw: {{  sensors[2].yaw.value }}
            </div>
          </div>
          <p>Lidar 2</p>
          <div class="row">
            <div class="col-md-6">
              <div class="bordered" style="background-color: #444;">
                <img :src="images.lidar_2_image.image_1.value" style="width: 100%; height: 100%" />
              </div>
            </div>
            <div class="col-md-6">
              <div class="bordered" style="background-color: #444;">
                <img :src="images.lidar_2_image.image_2.value" style="width: 100%; height: 100%" />
              </div>
            </div>
            <div class="row d-flex justify-content-center mt-3 mb-5" style="background-color: white;">
              x: {{  sensors[3].x.value }} , y: {{  sensors[3].y.value }} , z: {{ sensors[3].z.value }} , 
              roll: {{  sensors[3].roll.value }} , pitch: {{  sensors[3].pitch.value }} , yaw: {{  sensors[3].yaw.value }}
            </div>
          </div>
          <!-- <div class="carousel-caption d-none d-md-block">
            <h5>Lidar</h5>
          </div> -->
        </div>

        <div class="carousel-item">
          <p>Camera 1</p>
          <div class="row">
            <div class="col-md-6">
              <div class="bordered" style="background-color: #444;">
                <img :src="images.camera_1_image.image_1.value" style="width: 100%; height: 100%" />
              </div>
            </div>
            <div class="col-md-6">
              <div class="bordered" style="background-color: #444;">
                <img :src="images.camera_1_image.image_2.value" style="width: 100%; height: 100%" />
              </div>
            </div>
            <div class="row d-flex justify-content-center mt-3 mb-5" style="background-color: white;">
              x: {{  sensors[4].x.value }} , y: {{  sensors[4].y.value }} , z: {{ sensors[4].z.value }} , 
              roll: {{  sensors[4].roll.value }} , pitch: {{  sensors[4].pitch.value }} , yaw: {{  sensors[4].yaw.value }}
            </div>
          </div>
          <!-- <div class="carousel-caption d-none d-md-block">
            <h5>Camera</h5>
          </div> -->
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
</template>

<script setup>
import ROSLIB from 'roslib';

const props = defineProps({
  ros: ROSLIB.Ros,
  sensors: Array,
  images: Object,
  topics: Object,
});
</script>

<style scoped>
.bordered {
  background-color: #fff;
  padding: 10px;
  border: 2px solid #ccc;
  border-radius: .4rem;
}
</style>