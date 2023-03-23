import { createApp } from 'vue'
import './style.css'
import App from './App.vue'
import { router } from './routes'

import './../node_modules/bootstrap/dist/css/bootstrap.min.css';
import './../node_modules/bootstrap/dist/js/bootstrap.bundle.min.js';

createApp(App).use(router).mount('#app')
