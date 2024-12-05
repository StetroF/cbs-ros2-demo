<script setup>
import { onMounted } from 'vue';
import { useStore } from 'vuex'; // 导入 Vuex store
import { computed } from 'vue';
import { mapState } from 'vuex';
import router from './router';

onMounted(() => {


  const store = useStore();
  const publicIP = computed(() => store.state.publicIP); // 计算属性，获取 publicIP
});

</script>


<template>
  <div class="common-layout">
    <el-container>
      <!-- 侧边栏 -->
      <el-aside width="200px" class="aside-container">
        <el-col style="width:100%" >
          <img src="./components/icons/icon_web/logo.png" alt="Logo" class="logo" style="height:35px;margin-top: 35px;"/>
          <!-- <h3 class="mb-2">菜单栏</h3> -->
          <el-menu class="aside-menu" style="margin-top: 40px;">

            <router-link to="/" class="no-underline">
              <el-menu-item index="1">
                <template #title>
                  <img src ='./components/icons/icon_web/icon_任务下发.png' class='aside-icon'/>
                  <span clas="gray-blue">主界面</span>
                </template>
              </el-menu-item>
            </router-link>
            
            <!-- <router-link to="/Mapping" class="no-underline">
              <el-menu-item index="5">
                <template #title>
                  <img src ='./components/icons/icon_web/icon_建图.png' class='aside-icon'/>
                  <span>建图</span>
                </template>
              </el-menu-item>
            </router-link>

            <router-link to="/MapEdit" class="no-underline">
              <el-menu-item index="6">
                <template #title>
                  <img src ='./components/icons/icon_web/icon_参数设置.png' class='aside-icon'/>
                  <span>地图编辑</span>
                </template>
              </el-menu-item>
            </router-link>


            <router-link to="/config" class="no-underline">
              <el-menu-item index="4">
                <template #title>
                  <img src ='./components/icons/icon_web/icon_参数设置.png' class='aside-icon'/>
                  <span>参数设置</span>
                </template>
              </el-menu-item>
            </router-link>

            <router-link to="/RobotState" class="no-underline">
              <el-menu-item index="3">
                <template #title>
                  <img src ='./components/icons/icon_web/icon_导航栏.png' class='aside-icon'/>
                  <span>硬件质检</span>
                </template>
              </el-menu-item>
            </router-link>




            <router-link to="/VersionSwitch" class="no-underline">
              <el-menu-item index="7">
                <template #title>
                  <img src ='./components/icons/icon_web/icon_版本切换.png' class='aside-icon'/>
                  <span>版本切换</span>
                </template>
              </el-menu-item>
            </router-link>


            <router-link to="/Log" class="no-underline">
              <el-menu-item index="8">
                <template #title>
                  <img src ='./components/icons/icon_web/icon_版本切换.png' class='aside-icon'/>
                  <span>日志下载</span>
                </template>
              </el-menu-item>
            </router-link> -->
            

            <!-- <router-link to="/Calibrate" class="no-underline">
              <el-menu-item index="8">
                <template #title>
                  <img src ='./components/icons/icon_web/icon_版本切换.png' class='aside-icon'/>
                  <span>标定（开发中）</span>
                </template>
              </el-menu-item>
            </router-link> -->

          </el-menu>



        </el-col>



      </el-aside>
      <!-- 头部栏 -->
      <el-container>
        <el-header class = "header-container">
          <img src="./components/icons/header-icon/signal_4.png" alt="SignalIcon" class="custom-icon" />
          <img src = "./components/icons/header-icon/battery_100.png" alt="BatteryIcon" class="custom-icon" />
          <img src = "./components/icons/header-icon/icon_maximum.png" alt="MaximumIcon" class="custom-icon" />
        </el-header>
        <!-- 主体内容 -->
        <el-main :style="{ padding: '5px' }">
          <keep-alive>
            <router-view />
          </keep-alive>
        </el-main>
      </el-container>
      
    </el-container>
  </div>

</template>

<!-- <style src = "./assets/app.css"/> -->
<style scoped>
.aside-container{
  border-right: 2px solid #E8E8E8;
}
.header-container {
  border-bottom: 2px solid #E8E8E8; /* 分隔线 */
  display: flex; /* 使用 flex 布局 */
  justify-content: flex-end; /* 右对齐 */
  align-items: center; /* 垂直居中 */
  padding: 0 20px; /* 内边距 */
}

.header-icons {
  display: flex;
  gap: 15px; /* 图标间距 */
}

.custom-icon {
  width: 40px; /* 自定义图标宽度 */
  height: 40px; /* 自定义图标高度 */
  cursor: pointer; /* 鼠标悬停时显示为指针 */
}

.aside-icon{
  width: 25px; /* 自定义图标宽度 */
  height: 25px; /* 自定义图标高度 */
  cursor: pointer; /* 鼠标悬停时显示为指针 */
  margin-right: 15px; /* 图标右侧间距 */
}
/* .aside-menu{
  width: 195px;
} */
.no-underline {
  text-decoration: none; /* 取消下划线 */

}
.gray-blue {
  color: #ad1272;

}
.test-color{
  color: #1370c7;
}
</style>