<template>
  <div class="row">
    <div class="col-sm-10">
      <h1>导航栏</h1>
      <hr><br><br>

      <!-- 按钮和输入栏 1: doRotation -->
      <div class="form-group">
        <label for="rotationInput">角度:</label>
        <input type="number" id="rotationInput" v-model="angle" class="form-control d-inline" style="width: 200px; margin-left: 10px;">
        <button type="button" class="btn btn-primary btn-sm" @click="doRotation(angle)" style="margin-left: 10px;">旋转</button>
      </div>
      <br>

      <!-- 按钮和输入栏 2: moveTask -->
      <div class="form-group">
        <label for="distInput">距离:</label>
        <input type="number" id="distInput" v-model="distance" class="form-control d-inline" style="width: 200px; margin-left: 10px;">
        <label for="backwardInput" style="margin-left: 20px;">倒退:</label>
        <select id="backwardInput" v-model="backward" class="form-control d-inline" style="width: 200px; margin-left: 10px;">
          <option :value="true">是</option>
          <option :value="false">否</option>
        </select>
        <button type="button" class="btn btn-success btn-sm" @click="moveTask(distance, backward)" style="margin-left: 10px;">移动</button>
      </div>
      <br>

      <li>
      <!-- 取消任务按钮 -->
      <button type="button" class="btn btn-danger btn-sm" @click="cancelTask()">取消任务(还没实现)</button>
      </li> 
      <!-- 其他部分代码保留不变 -->
      <br><br>
      <br><br>


      <li>
        <button v-if="movethread === null" type="button" class="btn btn-primary btn-sm" @click="startMove()">开始移动</button>
        <button v-else type="button" class="btn btn-primary btn-sm" @click="stopMove()">停止移动</button>
      </li>

      <li>
        <Joystick
          :size="100"
          base-color="pink"
          stick-color="purple"
          :throttle="100"
          @set-velocity="updateVelocity"
        />
      </li>

      <!-- 表格部分保留不变 -->
      <table class="table table-hover">
        <thead>
          <tr>
            <th scope="col">Title</th>
            <th scope="col">Author</th>
            <th scope="col">Read?</th>
            <th></th>
          </tr>
        </thead>
        <tbody>
          <tr v-for="(book, index) in books" :key="index">
            <td>{{ book.title }}</td>
            <td>{{ book.author }}</td>
            <td>
              <span v-if="book.read">Yes</span>
              <span v-else>No</span>
            </td>
            <td>
              <button type="button" class="btn btn-warning btn-sm">Update</button>
              <button type="button" class="btn btn-danger btn-sm">Delete</button>
            </td>
          </tr>
        </tbody>
      </table>
    </div>
  </div>
</template>

<script scoped>
import Joystick from './Joystick.vue';
import axios from 'axios';
import { useStore } from 'vuex';
import { computed } from 'vue';
export default {
  components: {
    Joystick,
  },
  data() {
    return {
      books: [],
      velocity: [0.0, 0.0],
      movethread: null,
      publicIP: null, // 用于存储公共 IP 地址
      angle: 0,       // 旋转角度
      distance: 0,    // 移动距离
      backward: false  // 是否倒退
    };
  },

  methods: {
    startMove() 
    {
        this.intervalId = setInterval(this.sendVelocity, 100); // 每100毫秒发送一次
        axios.post(`http://${this.publicIP}:5000/RobotControl/setManualMode`, {
        manualMode: true
      })
      this.movethread = 1
    },
    stopMove() {
      this.intervalId = setInterval(this.sendVelocity, 100); // 每100毫秒发送一次
        axios.post(`http://${this.publicIP}:5000/RobotControl/setManualMode`, {
        manualMode: false
      })
      this.movethread = null
    },
    buttonClick() {
      alert("Button Clicked");
    },
  
    updateVelocity(velocity) {
      //当没有点击 "k开始移动时"，不发送请求
      if (this.movethread == null)
      {
        return;
      }
      // 在这里处理传递来的速度数据
      console.log('收到的速度:', velocity);
      axios.post(`http://${this.publicIP}:5000/RobotControl/setvel`, {
        velocity: velocity,
      });
    },
    getPublicIP() {
      console.log("Getting public IP address");
      const href = window.location.href;
  
      // 使用正则表达式匹配 IP 地址格式
      const ipMatch = href.match(/\b\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}\b/);
      
      if (ipMatch) {
        // 提取到的 IP 地址
        this.publicIP = ipMatch[0]; 
        console.log("Public IP:", this.publicIP);
      } else {
        console.log("No valid IP address found in the URL.");
        alert("无法获取到当前网址的ip!");
      }
    },
    // 机器人旋转函数
    doRotation(angle) {
      const floatAngle = parseFloat(angle); // 将输入的角度转换为浮点数
      
      if (isNaN(floatAngle)) {
        console.error("Invalid angle value, must be a number");
        return; // 如果不是有效的数字，则返回
      }
      
      console.log("data:", floatAngle);

      // 发送包含角度的对象
      axios.post(`http://${this.publicIP}:5000/RobotControl/doRotation`, { angle: floatAngle })
        .then(response => {
          console.log("Rotation response:", response.data); // 打印后端响应
          alert("任务发送成功");
        })
        .catch(error => {
          console.error("Error in rotation request:", error); // 打印错误信息
        });
    },
    // 机器人执行odom任务函数
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
    // 机器人取消任务函数
    cancelTask() {
      axios.get(`http://${this.publicIP}:5000/RobotControl/cancelTask`) // 发送 GET 请求到 /cancelTask
        .then(response => {
          console.log("Cancel task response:", response.data); // 打印后端响应
          alert("任务已取消");
        })
        .catch(error => {
          console.error("Error in cancel task request:", error); // 打印错误信息
        });
    },

  },

  created() {


    const store = useStore()
    const publicIP = computed(()=>store.state.publicIP)
    this.publicIP = publicIP.value
    console.log("Nav IP:",
      this.publicIP
    )
  },

  beforeDestroy() {
    if (this.movethread != null) {
      clearInterval(this.movethread);
    }
  },
};
</script>
