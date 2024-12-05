
import { createApp } from 'vue'
import { createPinia } from 'pinia'
import { Joystick } from 'vue-joystick-component'
import App from './App.vue'
import router from './router'
import { createStore } from 'vuex'; // 导入 Vuex store
import ElementPlus from 'element-plus';
import { useStore } from 'vuex';
import 'element-plus/dist/index.css';
// import { Location } from '@element-plus/icons-vue/dist/types'
import { ElIcon } from 'element-plus'

import { Location,Menu,Document,Setting } from '@element-plus/icons-vue'

const app = createApp(App)




function getPublicIP() {
    console.log("Getting public IP address");
  
    const href = window.location.href;
    // 使用正则表达式匹配 IP 地址格式
    const ipMatch = href.match(/\b\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}\b/);
    
    let publicIP = null; // 初始化 publicIP 变量
  
    if (ipMatch) {
      // 提取到的 IP 地址
      publicIP = ipMatch[0]; 
      // console.log("Main IP:", publicIP);
    } else {
      console.log("No valid IP address found in the URL.");
      alert("无法获取到当前网址的 IP!");
    }
    
    return publicIP; // 返回 publicIP
  }
  

const store = createStore({
    state:{
        publicIP: null
    },
    mutations:{
        setPublicIP(state,ip){
            state.publicIP = ip
        }
    }
}
)
const publiciP =getPublicIP()


store.commit('setPublicIP',getPublicIP())
app.component('Joystick', Joystick)
app.component('ElIcon', ElIcon)
app.component('Location', Location)
app.component('Menu', Menu)
app.component('Document', Document)
app.component('Setting', Setting)
app.use(createPinia())
app.use(router)
app.use(store)
app.use(publiciP)
app.use(ElementPlus)
app.mount('#app')

export default Joystick