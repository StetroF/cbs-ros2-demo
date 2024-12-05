<template>
  <div style="flex-direction: row" >
      <h1>硬件质检</h1>
      <hr><br><br>
      <div style="display: flex;">
        <div class ="Control" style="margin-right: 100px;">

          <div class="form-group">
            <label for="rotationInput">语音播放:</label>
            <input type="text" id="rotationInput" v-model="ttsStr" class="form-control d-inline" style="width: 200px; margin-left: 10px;">
            <button type="button" class="btn btn-primary btn-sm" @click="setTTS(ttsStr)" style="margin-left: 10px;">点击播放</button>
          </div>

          <div class="container" style="flex-direction: column">
            <div class="status-row">
              <span class="status-label">急停状态：</span>
              <button class="status-value" 
                  :style="{ color: currentEmergencyStatus ? 'red' : 'gray', opacity: currentEmergencyStatus ? 1 : 0.5 }" 
                  disabled>急停</button>
              <button class="status-value" 
                  :style="{ color: !currentEmergencyStatus ? 'green' : 'gray', opacity: !currentEmergencyStatus ? 1 : 0.5, margin: '0 10px' }" 
                  disabled>正常</button>
            </div>

            <div class="status-row">
              <span class="status-label">暂停状态：</span>
              <button class="status-value" 
                  :style="{ color: currentPauseStatus ? 'orange' : 'gray', opacity: currentPauseStatus ? 1 : 0.5 }" 
                  disabled>暂停</button>
              <button class="status-value" 
                  :style="{ color: !currentPauseStatus ? 'green' : 'gray', opacity: !currentPauseStatus ? 1 : 0.5, margin: '0 10px' }" 
                  disabled>正常</button>
            </div>

            <div class="status-row">
              <span class="status-label">复位状态：</span>
              <button class="status-value" 
                  :style="{ color: currentResetStatus ? 'blue' : 'gray', opacity: currentResetStatus ? 1 : 0.5 }" 
                  disabled>未复位</button>
              <button class="status-value" 
                  :style="{ color: !currentResetStatus ? 'green' : 'gray', opacity: !currentResetStatus ? 1 : 0.5, margin: '0 10px' }" 
                  disabled>已复位</button>
            </div>
          </div>
          <div class="container" style="flex-direction: column">
            <!-- 按钮和输入栏 1: doRotation -->
            <div class="form-group">
              <label for="rotationInput">角度:</label>
              <input type="number" id="rotationInput" v-model="angle" class="form-control d-inline" style="width: 100px; margin-left: 10px;">
              <button type="button" class="btn btn-primary btn-sm" @click="doRotation(angle)" style="margin-left: 100px;">旋转</button>
            </div>
            ---------------------------------------------------------<br/>
            <!-- 按钮和输入栏 2: moveTask -->
            <div class="form-group">
              <label for="distInput">距离:</label>
              <input type="number" id="distInput" v-model="distance" class="form-control d-inline" style="width: 75px; margin-left: 10px;">
              <label for="backwardInput" style="margin-left: 20px;">倒退:</label>
              <select id="backwardInput" v-model="backward" class="form-control d-inline" style="width: 50px; margin-left: 10px;">
                <option :value="true">是</option>
                <option :value="false">否</option>
              </select>
              <button type="button" class="btn btn-success btn-sm" @click="moveTask(distance, backward)" style="margin-left: 10px;">移动</button>
            </div>

          </div>

          <!-- 其他部分代码保留不变 -->

          <div class="container" style="flex-direction: column">
            <button v-if="movethread === null" type="button" class="btn btn-primary btn-sm" @click="startMove()">开始移动</button>
            <button v-else type="button" class="btn btn-primary btn-sm" @click="stopMove()">停止移动</button>
            <Joystick
              :size="100"
              base-color="pink"
              stick-color="purple"
              :throttle="100"
              @set-velocity="updateVelocity"
            />
          </div>
          </div>

        <div class="container"style='flex-direction: column;align-items: center;width:750px'>
          <h2>硬件数据</h2>
          <RealTimeChart :data="odometryData"></RealTimeChart>
        </div>

      </div>

  </div>
</template>

<script scoped>
import Joystick from './Joystick.vue';
import axios from 'axios';
import { useStore } from 'vuex';
import { computed, ref } from 'vue';
import RealTimeChart from "./Chart.vue";
export default {
  components: {
    Joystick,
    RealTimeChart
  },
  data() {
    return {
      velocity: [0.0, 0.0],
      movethread: null,
      publicIP: null, // 用于存储公共 IP 地址
      angle: 0,       // 旋转角度
      distance: 0,    // 移动距离
      backward: false,  // 是否倒退
      currentEmergencyStatus: '急停',
      currentPauseStatus: '暂停',
      currentResetStatus: '未复位',
      robotIOWebSocket: null, // 机器人 IO WebSocket 连接
      odometryData: [], // 机器人 odometry 数据
    };
  },
  mounted() {
    this.subscribeRobotIO();
    this.testOdom(); // 启动模拟数据更新
  },
  methods: {
    testOdom() {
      setInterval(() => {
        const now = new Date().toISOString().slice(0, 19).replace('T', ' '); // 获取当前时间

        const newData = {
          time: now, // 添加当前时间字段
          r: Math.random() * 1.57,
          p: Math.random() * 1.57,
          yaw: Math.random() * 1.57,
        };

        if (this.odometryData.length > 200) {
          this.odometryData.shift(); // 移除数组的第一个元素，保持数据量
        }

        this.odometryData.push(newData); // 更新 odometryData
      }, 100); // 每100ms更新一次数据
    },
    moveTask(distance, backward) {
      const floatDistance = parseFloat(distance);
      
      if (isNaN(floatDistance)) {
        console.error("Invalid distance value, must be a number");
        return;
      }
      
      const data = { dist: floatDistance, backward: backward };
      
      axios.post(`http://${this.publicIP}:5000/RobotControl/moveTask`, data)
        .then(response => {
          console.log("Move task response:", response.data);
          alert("任务发送成功");
        })
        .catch(error => {
          console.error("Error in move task request:", error);
        });
    },

    subscribeRobotIO() {
      if (!this.robotIOWebSocket) {
        this.robotIOWebSocket = new WebSocket(`ws://${this.publicIP}:5000/robot_io_websocket`);
        this.robotIOWebSocket.onopen = () => {
          console.log("Robot IO WebSocket connected");
        };
        this.robotIOWebSocket.onmessage = (event) => {
          const data = JSON.parse(event.data);
          this.currentEmergencyStatus = data.emergencyIO;
          this.currentPauseStatus = data.pauseIO;
          this.currentResetStatus = data.resetIO;
        };
        this.robotIOWebSocket.onclose = () => {
          console.log("Robot IO WebSocket closed");
          this.robotIOWebSocket = null;
        };
      }
    },
    unscsibscribeRobotIO() {
      if (this.robotIOWebSocket) {
        this.robotIOWebSocket.close();
      }
    },
    // 其他方法...
  },
  created() {
    const store = useStore();
    const publicIP = computed(() => store.state.publicIP);
    this.publicIP = publicIP.value;
    console.log("Nav IP:", this.publicIP);
  },
  beforeDestroy() {
    if (this.movethread != null) {
      clearInterval(this.movethread);
    }
  },
  beforeUnmount() {
    this.unscsibscribeRobotIO();
  },
};
</script>

<style scoped>
.container {

  margin-top: 15px;
  border: 2px solid #ccc; /* 边框样式 */
  border-radius: 8px; /* 圆角 */
  padding: 20px;
  width:100%;
  background-color: #f9f9f9; /* 背景颜色 */
  margin-left: 10px; /* 可根据需要调整间距 */
  width: 310px; /* 固定宽度，可以根据需要调整 */
}
.status-row {
  margin: 10px 0; /* 每行之间的间距 */
}
.status-label {
  font-weight: bold; /* 状态标签加粗 */
}
.row{
  flex-direction: row;
}
</style>
