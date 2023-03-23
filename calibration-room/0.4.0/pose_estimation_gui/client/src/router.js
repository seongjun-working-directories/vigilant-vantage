import * as VueRouter from 'vue-router';
import Homepage from './pages/HomePage.vue';
import CalibrationPage from './pages/CalibrationPage.vue';
import HelpPage from './pages/HelpPage.vue';

const routes = [
  {
    path: '/',
    component: Homepage
  },
  {
    path: '/calibration',
    component: CalibrationPage
  },
  {
    path: '/help',
    component: HelpPage
  },
];

const router = VueRouter.createRouter({
  history: VueRouter.createWebHashHistory(), routes
});

export { router };