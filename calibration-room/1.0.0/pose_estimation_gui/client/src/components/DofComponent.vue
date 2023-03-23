<template>
  <div class="row">
    <div class="col-md-6">
      <div class="form-group">
        <label for="">x</label>
        <input type="text" class="form-control" id="" :value="props?.sensor?.x.value">
      </div>
    </div>
    
		<div class="col-md-6">
      <div class="form-group">
        <label for="">y</label>
        <input type="text" class="form-control" id="" :value="props?.sensor?.y.value">
      </div>
    </div>
  </div>

  <div class="row">
    <div class="col-md-6">
      <div class="form-group">
        <label for="">z</label>
        <input type="text" class="form-control" id="" :value="props?.sensor?.z.value">
      </div>
    </div>
    
		<div class="col-md-6">
      <div class="form-group">
        <label for="">roll</label>
        <input type="text" class="form-control" id="" :value="props?.sensor?.roll.value">
      </div>
    </div>
  </div>
  
	<div class="row">
    <div class="col-md-6">
      <div class="form-group">
        <label for="">pitch</label>
        <input type="text" class="form-control" id="" :value="props?.sensor?.pitch.value">
      </div>
    </div>
    <div class="col-md-6">
      <div class="form-group">
        <label for="">yaw</label>
        <input type="text" class="form-control" id="" :value="props?.sensor?.yaw.value">
      </div>
    </div>
  </div>
</template>

<script lang="ts">
import { Ref } from 'vue';

class Sensor {
	x: Ref<string>;
	y: Ref<string>;
	z: Ref<string>;
	roll: Ref<string>;
	pitch: Ref<string>;
	yaw: Ref<string>;
};
</script>

<script lang="ts" setup>
const props = defineProps({
  sensor: Sensor
});
</script>