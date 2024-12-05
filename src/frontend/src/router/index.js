import { createRouter, createWebHistory } from 'vue-router'
import HomeView from '../views/HomeView.vue'
import Control from '../components/Control.vue'
const router = createRouter({
  history: createWebHistory(import.meta.env.BASE_URL),
  routes: [
    {
      path: '/',
      name: "Home",
      props: true,
      component:() => import('../components/Navigation.vue')
  },
  {
    path: '/Control',
    name: 'Control',
    props: true,
    component: Control,
  },
  {
    path: '/config',
    name: 'Config',
    props: true,
    component: () => import('../components/Config.vue')
  },
  {
    path: '/Navigation',
    name: 'Navigation',
    props: true,
    component: () => import('../components/Navigation.vue')
  },
  {
    path: '/Mapping',
    name: 'Mapping',
    props: true,
    component: () => import('../components/Mapping.vue')
  },
  {
    path: '/MapEdit',
    name: 'MapEdit',
    props: true,
    component: () => import('../components/MapEdit.vue')
  },
  {
    path: '/RobotState',
    name: 'RobotState',
    props: true,
    component: () => import('../components/RobotState.vue')
  
  },
  {
    path: '/VersionSwitch',
    name: 'VersionSwitch',
    props: true,
    component: () => import('../components/VersionSwitch.vue')
  },
  {
    path: '/Log',
    name: 'Log',
    props: true,
    component: () => import('../components/Log.vue')
  }
  ],

})

export default router
