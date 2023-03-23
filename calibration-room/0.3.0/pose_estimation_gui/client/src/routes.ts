import * as VueRouter from 'vue-router';
import PoseEstimationGui from './components/PoseEstimationGui.vue';
import DepthPoseEstimationGui from './components/DepthPoseEstimationGui.vue';
import LidarPoseEstimationGui from './components/LidarPoseEstimationGui.vue';
import CameraPoseEstimationGui from './components/CameraPoseEstimationGui.vue';
import ChessboardPoseEstimationGui from './components/ChessboardPoseEstimationGui.vue';
import ModeChoice from './components/ModeChoice.vue';


const routes = [
  {
    path: '/',
    component: ModeChoice
  },
  {
    path: '/all_pose_estimation',
    component: PoseEstimationGui
  },
  {
    path: '/depth_pose_estimation',
    component: DepthPoseEstimationGui
  },
  {
    path: '/lidar_pose_estimation',
    component: LidarPoseEstimationGui
  },
  {
    path: '/camera_pose_estimation',
    component: CameraPoseEstimationGui
  },
  {
    path: '/chessboard_pose_estimation',
    component: ChessboardPoseEstimationGui
  }
];

const router = VueRouter.createRouter({
  history: VueRouter.createWebHashHistory(), routes
});

export { router };