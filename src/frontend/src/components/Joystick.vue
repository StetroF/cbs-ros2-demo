<template>
    <Joystick
      :size="size"
      :base-color="baseColor"
      :stick-color="stickColor"
      :throttle="throttle"
      @start="handleStart"
      @stop="handleStop"
      @move="handleMove"
    />
  </template>

<script>
import Joystick from 'vue-joystick-component';
import { useStore } from 'vuex';
import { computed, resolveComponent } from 'vue';
import axios from 'axios';

export default {
    components: {
      Joystick,
    },
    props: {
      size: {
        type: Number,
        default: 100,
      },
      baseColor: {
        type: String,
        default: 'pink',
      },
      stickColor: {
        type: String,
        default: 'purple',
      },
      throttle: {
        type: Number,
        default: 100,
      },
    },
    data() {
      return {
        velocity: [0.0, 0.0], // 初始化速度
        intervalId: null, // 定时器ID
        publicIP:null
      };
    },
    created() {
        const store = useStore();
        const ip = computed(() => store.state.publicIP);
        this.publicIP = ip;
    },
    methods: {
      start(){
        axios.post(`http://${this.publicIP}:5000/RobotControl/setManualMode`, {
        manualMode: true
      })

      },
      handleStart() {
        // console.log('Joystick started');
        // 启动定时器以定期发送当前速度
        this.intervalId = setInterval(this.sendVelocity, 100); // 每100毫秒发送一次
      //   axios.post(`http://${this.publicIP}:5000/RobotControl/setManualMode`, {
      //   manualMode: true
      // })
      },
      handleStop() {
        // console.log('Joystick stopped');
        this.velocity = [0.0, 0.0];
        this.sendVelocity(); // 发送停止速度
        clearInterval(this.intervalId); // 清除定时器
      //   console.log("设置手动模式为false")
      //   axios.post(`http://${this.publicIP}:5000/RobotControl/setManualMode`, {
      //   manualMode: false
      // })
        
      },
      handleMove({ x, y, direction, distance }) {
          // 设置当前速度
          this.velocity = [y, -x];
          console.log('Joystick move', { y, x, direction, distance });
        
          // 计算当前速度的大小
          const speedMagnitude = Math.sqrt(Math.pow(this.velocity[0], 2) + Math.pow(this.velocity[1], 2));

          // 限制最大速度为 0.4
          // if (speedMagnitude > 0.4) {
          //     const scale = 0.4 / speedMagnitude; // 计算缩放比例
          //     this.velocity[0] *= scale; // 调整 x 速度
          //     this.velocity[1] *= scale; // 调整 y 速度
          // }

          // 输出当前速度
          console.log("当前速度: ", this.velocity);

          // 发送当前速度
          this.sendVelocity();
      },

      sendVelocity() {
        // 这里可以将 velocity 发送到机器人的控制接口，例如使用 axios
        this.$emit('set-velocity', this.velocity);
      },
    },
    beforeDestroy() {
      // 组件销毁前清除定时器
      clearInterval(this.intervalId);
    },
  };
  </script>
  
  <style scoped>
  /* 你可以在这里添加样式 */
  </style>