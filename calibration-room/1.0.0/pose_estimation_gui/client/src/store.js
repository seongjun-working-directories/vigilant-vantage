import { createStore } from 'vuex';
import { ref } from 'vue';
import axios from 'axios';

let data;

const setStore = async () => {
  data = await axios({
    url: 'http://localhost:5000/get-cr-settings',
    method: 'get',
    data: {
      // TODO : None
    }
  });
}


await setStore();

export const store = createStore({
  state: {
    camera1_activate: ref((data.data.data.camera1_activate) ? data.data.data.camera1_activate : true),
    depth1_activate: ref((data.data.data.depth1_activate) ? data.data.data.depth1_activate : true),
    depth2_activate: ref((data.data.data.depth2_activate) ? data.data.data.depth2_activate : false),
    depth3_activate: ref((data.data.data.depth3_activate) ? data.data.data.depth3_activate : false),
    lidar1_activate: ref((data.data.data.lidar1_activate) ? data.data.data.lidar1_activate : true),
    lidar2_activate: ref((data.data.data.lidar2_activate) ? data.data.data.lidar2_activate : true),
    lidar3_activate: ref((data.data.data.lidar3_activate) ? data.data.data.lidar3_activate : false),
  }
});