<template>
  <div class="container">
      <!-- 左边的地图 -->
      <div ref="mapContainer" class="map-container"></div>
      <!-- 右边的任务栏和摇杆 -->
      <div v-if="showold===true" style="flex-direction: column;">
        <div class="control-container" style="height: 400px">

          <div class='row-button'>
            <el-table :data="historyRobotPose" style="margin-bottom: 5px;">
              <el-table-column prop="x" label="X" width="150" />
              <el-table-column prop="y" label="Y" width="150" />
              <el-table-column prop="z" label="角度" width="150" />
              <el-table-column prop="score" label="匹配分数" width="150" />
            </el-table>

          </div>
          <div class='row-button'>
            <div class= 'row1'>

              <div class="row-button">
                <el-button color="#626aef" v-if="movethread === null" type="el-button" class="btn btn-primary btn-sm" @click="startMove()">开始移动</el-button>
                <el-button color="#626aef" v-else type="el-button" class="btn btn-primary btn-sm " @click="stopMove()">停止移动</el-button>
                
                <el-button color="#4caf50" type="default" class="btn btn-primary btn-sm" @click="relocalize" style="margin-left: 15px;">手动定位</el-button>

                <!-- 条件渲染 -->
                <el-button v-if="showCalibrationButton" color="#626aef" type="default" class="btn btn-success btn-sm" @click="startCalibration" style="margin-left: 15px;">开始校准</el-button>              
                
                </div>
                <!-- </div> -->
                <br></br>
                <br></br>
                <Joystick
                  :size="125"
                  base-color="#D3D3D3"
                  stick-color="#403c70"
                  :throttle="100"
                  @set-velocity="updateVelocity"
                />
                <br></br>
                <div class='row-button' style="flex-direction: column;">

                  <label>最大线速度</label>
                  <el-input-number v-model="maxLinearVel" :precision="2" :step="0.1" :min="0.1" :max="1.5" placeholder="m/s" style="margin-left: 15px;"/>
                  <label>最大角速度</label>
                  <el-input-number v-model="maxAngularVel" :precision="2" :step="0.1" :min="0.1" :max="1.5" placeholder="rad/s" style="margin-left: 15px;"/>
                </div>
                </div>
            <div class= 'row2'> <!-- 显示事件码列表 -->
                  <el-table empty-text="暂无异常事件码" :data="evtCodelist" style="margin-left: 15px;width: 400px"max-height="200"  >
                    <el-table-column prop="code" label="异常事件码" width="100" >
                      <template #default="scope">
                        <span :style="getCodeStyle(scope.row.priority)">
                          {{ scope.row.code }}
                        </span>
                      </template>
                    </el-table-column>
                    <el-table-column prop="brief" label="描述" width="100" />
                    <el-table-column prop="suggestion" label="细节" width="100" />
                    <el-table-column prop="time" label="时间戳" width="100" />
                  </el-table>
            </div>
          </div>



        </div>
        <div class="task-container" style="height: 400px;"> <!-- 新的容器 -->
          <h2>任务栏</h2>
            <el-button color="#626aef" @click="startTask">选择任务点</el-button>
            <el-button color="#d3dadf" @click="clearTaskNodeLayer">清除任务点</el-button>
            <el-button type='primary' @click="setFilterPolygon" style="margin-left:50px;border-radius: 10px; ">设置过滤区域</el-button>
          <br /><br />          

          <label for="taskType">选择任务类型:</label>

          <div class="task-point-container">
            <el-radio-group v-model="taskType">
              <el-radio v-for="(item, index) in taskOptions" 
                        :key="index" 
                        :label="item.value">
                {{ item.label }}
              </el-radio>
            </el-radio-group>

          </div>
          <label fo='avoidanceMode'>避障模式(当设置为绕障时，所有途径路线都是绕障):</label>
          <div class="task-point-container" style="width:150px">
            <el-radio-group v-model="avoidanceMode">
              <el-radio :label="0">停障</el-radio>
              <el-radio :label="1">避障</el-radio>
            </el-radio-group>
          </div>

          <br /><br />
            <template v-if="taskType === 5">
              <el-input-number controls-position="right" v-model="angle" :precision="2" :step="0.1" :max="3.14" placeholder="操作角度(弧度)"/>
            </template>
            <el-button v-if="taskType === 5" color="#626aef" @click="sendTask( taskType,angle)">下发任务</el-button>  
            <el-button v-else color="#626aef" @click="sendTask(taskType)">下发任务</el-button>
              

            <el-button color="#d3dadf" @click="cancelTask()">取消所有任务</el-button>

      </div>
    </div>

  </div>
</template>


<script>
import axios from 'axios';
import { useStore } from 'vuex';
import { computed, resolveComponent, toHandlers } from 'vue';
import 'leaflet/dist/leaflet.css'
import L from 'leaflet'
import 'leaflet-rotatedmarker'
import '@elfalem/leaflet-curve';
import Swal from 'sweetalert2';
import Joystick from './Joystick.vue';
import { reactive, onMounted } from 'vue';
import { Subject } from 'rxjs';
import { debounceTime } from 'rxjs/operators';
import BSpline from 'b-spline';

// const visualNodes = {
//   name: "john",
//   easy: function(){
//     console.log("easy")
//   }
// }

export default {
  components:{
    Joystick
  },
  data() {
    return {
      showold:false,
      maxLinearVel:0.6,
      maxAngularVel:0.4,
      avoidanceMode: 0,
      movethread:null, //绑定是否开始移动的按键
      publicIP: null,
      nodes: [],
      canvasWidth: 1600,
      canvasHeight: 500,
      robotPose: reactive({ x: 0.0, y: 0.0, z: 0.0 }),// Z表示角度
      historyRobotPose: reactive([{x :0.0,y :0.0, z:0.0}]),
      robotVel:{ x: 0.0, y: 0.0, z: 0.0}, //Z表示角速度
      robotSvg: `<svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" fill="green" class="bi bi-backpack-fill" viewBox="0 0 16 16">
                    <path d="M5 13v-3h4v.5a.5.5 0 0 0 1 0V10h1v3z" />
                    <path d="M6 2v.341C3.67 3.165 2 5.388 2 8v5.5A2.5 2.5 0 0 0 4.5 16h7a2.5 2.5 0 0 0 2.5-2.5V8a6 6 0 0 0-4-5.659V2a2 2 0 1 0-4 0m2-1a1 1 0 0 1 1 1v.083a6 6 0 0 0-2 0V2a1 1 0 0 1 1-1m0 3a4 4 0 0 1 3.96 3.43.5.5 0 1 1-.99.14 3 3 0 0 0-5.94 0 .5.5 0 1 1-.99-.14A4 4 0 0 1 8 4M4.5 9h7a.5.5 0 0 1 .5.5v4a.5.5 0 0 1-.5.5h-7a.5.5 0 0 1-.5-.5v-4a.5.5 0 0 1 .5-.5" />
                  </svg>`,

      robotVelocity: {
        linear: 0,
        yaw: 0,
      },

      map_info:{
        resolution: 0.05,
        width: null,
        height: null,
        scale: 1.0, //相对于canvas的缩放比
        edges:[],
      },
      test_x :1,
      map: null,
      robotMarker: null|L.Marker,//显示机器人位置
      nodeLayer: null,//显示各个node的图层
      edgeLayer: null,//显示各个edge的图层
      lidarLayer: null,//显示雷达数据
      taskNodeLayer:null, //存储选中的task起点和终点
      localPathLayer: null, // Leaflet 图层
      mapLayer:null, // 存储地图图层
      closestNode: null, // 用于存储最近的节点
      taskPoint:{
        startPoint:null,
        endPoint:null
      },
      nodeStyle:{
        originalStyle:{
          radius: 1.5,
          color: 'black',
          fillColor: 'black',
          fillOpacity: 1
        },
        startPointStyle:{
          radius:8.0,
          color: 'green',
          fillColor: 'green'
        },
        endPointStyle:{
          radius: 8.0,
          color: 'blue',
          fillColor: 'blue'
        },
        highlightStyle:{
          radius: 5,
          color: 'red',
          fillColor: 'black'
        },
        chargerPointStyle:{
          radius: 5,
          color: '#FFA500', // 替换为温暖的橙色
          fillColor: '#FFA500', // 柔和的暖黄色
          fillOpacity: 0.8,
        },
        deliverPointStyle:{
          radius: 5,
          color: 'purple',
          fillColor: 'purple'
        }
      },
      // 存储显示“起始点”，“终点”的文本
      taskPointTip:{
        startTip:null,
        endTip:null
      },
      lidarData: null,
      lidarWebsocket : null,
      robotPoseWebsocket: null,
      eventCodeWebsocket: null,
      localPathWebsocket: null,
      taskOptions: [
        { label: '取货任务', value: 1 },
        { label: '移动任务', value: 2 },
        { label: '送货任务', value: 3 },
        { label: '充电任务', value: 4 },
        { label: '操作任务', value: 5 }
      ],
      taskType: 2, // 绑定的任务类型
      elementInfo:null,
      instructionPoint:[],
      showCalibrationButton: false,
      draggableRobot:null, //绑定机器人拖动事件
      evtCodelist: reactive([]),
      localPath : reactive([]),
      localPathSubject: new Subject(),
    }
  },
  mounted(){
    this.initializeMap()
    this.fetchMapData().then(() => 
    {
      this.fetchMapImage()

      this.localPathSubject.pipe(debounceTime(100)).subscribe
      (localPath => {
        this.localPath = localPath; // 更新本地路径
        this.updateLocalPath(); // 更新图层
      });
    })
    return this.historyRobotPose
  },
  created() {
    const store = useStore();
    const ip = computed(() => store.state.publicIP);
    this.publicIP = ip;
    
    window.addEventListener('keydown', this.highlightNearestNode);
    this.subscribeLidar()
    this.subscribeRobotPose()
    this.subscribeEventCode()
    this.subscribeLocalPath()

    //初始化函数，请求python层每次切换到这个界面时，都会重新加载一遍history_map，保证在编辑地图后，python保存的是最新的node和edge信息
    axios.post(`http://${this.publicIP}:5000/Task/init_node`)

    this.listenKeyboard()
  },
  beforeUnmount() {
        this.unscribeLidar()
        this.unscribeLidarRobotPose()
        this.unscribeEventCode()
        this.unscribeLocalPath()
      },
  methods: {
    setFilterPolygon(){
      console.log('开始举升标定')
      const polygon = 
        [
          {
            x: 19,
            y: 14
          },
          {
            x: 23.9,
            y: 13.8
          },
          {
            x: 24.0,
            y: 16.35
          },
          {
            x: 19,
            y: 16.28
          }
        ]
      
      const setFilterPolygonRequest = `http://${this.publicIP}:5000/setFilterLidarPolygon`;
      axios.post(setFilterPolygonRequest,polygon).then(response => {
        console.log('设置过滤区域响应:',response.data)
        Swal.fire({
          title: "设置过滤·区域成功",
          text: response.data.message,
          icon: response.data.status ? "success" : "error",
          // confirmButtonText: "确定",
          timer: 1500,
          showCloseButton: false,
          position: "top",
          backdrop: false,
          heightAuto: true,
        })

          }).catch(error => {
            Swal.fire({
              title: "设置过滤区域失败",
              text: "请检查网络连接或联系管理员",
              icon: "error",
              // confirmButtonText: "确定",
              timer: 1500,
              showCloseButton: false,
              position: "top",
              backdrop: false,
              heightAuto: true,
            })
          });
    },
    getCodeStyle(priority) {
      // console.log('priority:', typeof priority)
      switch (priority) {
        case "2":
          return { backgroundColor: 'orange', color: 'white', padding: '5px', borderRadius: '4px' };
        case "3":
          return { backgroundColor: 'red', color: 'white', padding: '5px', borderRadius: '4px' };
        default:
          return {}; // 无特殊样式
      }
    },
    updateLocalPath() {
      if (this.localPath.poses.length > 0) {
        this.localPathLayer.clearLayers(); // 清空之前的图层

        this.localPath.poses.forEach((pose, index) => {
          // 仅在 index 为 0, 4, 8, ... 时绘制点
          if (index % 4 === 0) {
            const x = pose.position.x / this.map_info.resolution * this.map_info.scale;
            const y = pose.position.y / this.map_info.resolution * this.map_info.scale;

            // 绘制点
            L.circleMarker([y, x], {
              radius: 0.5, // 点的大小
              color: 'green', // 点的颜色
              fillColor: 'green', // 填充颜色
              fillOpacity: 0.5,
            }).addTo(this.localPathLayer);
          }
        });
      }
    },
    listenKeyboard(){
        const keysPressed = {};

        function updateMovement() {
            if (keysPressed['w'] && keysPressed['d']) {
                this.updateVelocity([this.maxLinearVel, -this.maxAngularVel]); // 前进并左转
            } else if (keysPressed['w'] && keysPressed['a']) {
                this.updateVelocity([this.maxLinearVel, this.maxAngularVel]); // 前进并右转
            } else if (keysPressed['s'] && keysPressed['d']) {
                this.updateVelocity([-this.maxLinearVel, this.maxAngularVel]); // 后退并左转
            } else if (keysPressed['s'] && keysPressed['a']) {
                this.updateVelocity([-this.maxLinearVel, -this.maxAngularVel]); // 后退并右转
            } else if (keysPressed['w']) {
                this.updateVelocity([this.maxLinearVel, 0]); // 仅前进
            } else if (keysPressed['s']) {
                this.updateVelocity([-this.maxLinearVel, 0]); // 仅后退
            } else if (keysPressed['a']) {
                this.updateVelocity([0, this.maxAngularVel]); // 仅左转
            } else if (keysPressed['d']) {
                this.updateVelocity([0, -this.maxAngularVel]); // 仅右转
            } else {
                this.updateVelocity([0, 0]); // 停止
            }
        }

        function pollKeyboardState() {
            // 持续检测按键状态
            updateMovement.call(this);
            requestAnimationFrame(pollKeyboardState.bind(this));//轮询按键状态，而不是根据press判断，保证当你W--WD--W时，最后一个W会判断为按下状态
        }

        // 使用按键事件来更新 keysPressed 对象
        window.addEventListener('keydown', (event) => {
            keysPressed[event.key] = true;
        });

        window.addEventListener('keyup', (event) => {
            delete keysPressed[event.key];
        });

        // 启动轮询
        pollKeyboardState.call(this);
    },
    subscribeLocalPath() {
      if (!this.localPathWebsocket) {
        this.localPathWebsocket = new WebSocket(`ws://${this.publicIP}:5000/local_path_websocket`);
        this.localPathWebsocket.onopen = () => {
          console.log('LocalPath WebSocket 连接成功');
        };
        this.localPathWebsocket.onmessage = (event) => {
          const localPath = JSON.parse(event.data);
          this.localPathSubject.next(localPath); // 将路径数据传递给 Subject
          // console.log('LocalPath:', localPath);
        };
        this.localPathWebsocket.onerror = (error) => {
          console.error('WebSocket 发生错误:', error);
        };
        this.localPathWebsocket.onclose = () => {
          console.log('WebSocket 连接关闭');
        };
      }
    },
  
    unscribeLocalPath(){
      if(this.localPathWebsocket){
        this.localPathWebsocket.close();
        this.localPathWebsocket = null;
      }
    },
    updateInstructionPoint(){
      if(this.instructionPoint.length > 0){
        const startPoint = this.instructionPoint[0];
        const endPoint = this.instructionPoint[this.instructionPoint.length-1];
        this.taskPoint.startPoint = startPoint;
        this.taskPoint.endPoint = endPoint;
        this.updateTaskPointTip();
      }
    },
    subscribeEventCode(){
      if(!this.eventCodeWebsocket){
        this.eventCodeWebsocket = new WebSocket(`ws://${this.publicIP}:5000/event_code_websocket`);
        this.eventCodeWebsocket.onopen = () => {
          console.log('EventCode WebSocket 连接成功');
        };
        this.eventCodeWebsocket.onmessage = (event) => {
          const eventCode = JSON.parse(event.data);
          this.evtCodelist = eventCode
        };
        this.eventCodeWebsocket.onerror = (error) => {
          console.error('WebSocket 发生错误:', error);
        };
        this.eventCodeWebsocket.onclose = () => {
          console.log('WebSocket 连接关闭');
        };
      }
    },
    unscribeEventCode(){
      if(this.eventCodeWebsocket){
        this.eventCodeWebsocket.close();
        this.eventCodeWebsocket = null;
      }
    },
    cancelTask(){
      const task = {
        endPoint: "123",
        instruction: 5,
        angle: 0,
      }
      const requestUrl = `http://${this.publicIP}:5000/Task/sendTask`;
      const response = axios.post(requestUrl, task).then(response => {
        console.log('取消任务响应:',response.data)
        Swal.fire({
          title: "取消任务",
          text: response.data.message,
          icon: response.data.status ? "success" : "error",
          // confirmButtonText: "确定",
          timer: 1500,
          showCloseButton: false,
          position: "top",
          backdrop: false,
          heightAuto: true,
        })
      }).catch(error => {
        console.error('取消任务失败:', error);
      });
    },
    startCalibration(){
      const calibratioUrl = `http://${this.publicIP}:5000/startCalibration`;
      const calibrationRequest ={
        x: this.robotPose.x,
        y: this.robotPose.y,
        yaw: this.robotPose.z,
      }
      console.log('校准请求:',calibrationRequest)
      const response = axios.post(calibratioUrl, calibrationRequest).then(response => {
        console.log('校准响应:',response.data)
        Swal.fire({
          title: "校准结果",
          text: response.data.message,
          icon: response.data.status ? "success" : "error",
          // confirmButtonText: "确定",
          timer: 1500,
          showCloseButton: false,
          position: "top",
          backdrop: false,
          heightAuto: true,
        })
        if (response.data.status){
          this.map.removeLayer(this.directionPoint)
          this.subscribeRobotPose()
          this.draggableRobot.disable()}

      }).catch(error => {
        console.error('校准失败:', error);
      });
      this.showCalibrationButton = false;

    },
    relocalize(){
      this.showCalibrationButton = true;
      console.log('手动定位')
      if (this.robotMarker) {
        this.unscribeLidarRobotPose()
        this.createDirectionElements()
      }
    },
    unscribeLidarRobotPose(){
      if (this.robotPoseWebsocket) {
        this.robotPoseWebsocket.close();
        this.robotPoseWebsocket = null;
        }
    },
    subscribeRobotPose(){
      if (!this.robotPoseWebsocket){
        this.robotPoseWebsocket = new WebSocket(`ws://${this.publicIP}:5000/Robot/robot_pose_websocket`);
        this.robotPoseWebsocket.onopen = () => {
          console.log('RobotPose WebSocket 连接成功');
        };
        this.robotPoseWebsocket.onmessage = (event) => {
          const robotPose = JSON.parse(event.data);
          console.log('Robot pose:', robotPose)
          
          // this.robotPose = robotPose.pose;
          // this.robotVel = robotPose.velocity;

          // this.historyRobotPose[0] = { //historyRobotPose用于更新显示的表单的数据
          //   x: parseFloat(robotPose.pose.x).toFixed(4),
          //   y: parseFloat(robotPose.pose.y).toFixed(4),
          //   z: parseFloat(robotPose.pose.z).toFixed(4), 
          //   score: parseFloat(robotPose.score).toFixed(2)
          // };
          // this.updateRobotPosition();
          // this.handleSetVelocity()
        };
        this.robotPoseWebsocket.onerror = (error) => {
          console.error('WebSocket 发生错误:', error);
        };
        this.robotPoseWebsocket.onclose = () => {
          console.log('WebSocket 连接关闭');
        };

      }
    },
    handleSetVelocity() {
      // 更新data中的robotVelocity对象
      const v_x = this.robotVel.x;
      const v_y = this.robotVel.y;

      const squareSum = v_x ** 2 + v_y ** 2;
      const vel_x = Math.sqrt(squareSum).toFixed(2);
      const yaw = (this.robotVel.z).toFixed(2);
      this.robotVelocity = {
        linear: vel_x,
        yaw: yaw
      };
      // console.log('WebSocket 速度显示');
    },

    unscribeLidar() {
      if (this.lidarWebsocket) {
        this.lidarWebsocket.close();
        this.lidarWebsocket = null;
      }
    },
    subscribeLidar() {
        if (!this.lidarWebsocket) {
            this.lidarWebsocket = new WebSocket(`ws://${this.publicIP}:5000/lidar_websocket`);

            this.lidarWebsocket.onopen = () => {
                console.log('Lidar WebSocket 连接成功');
            };
            this.lidarMarkers = [];
            this.lidarWebsocket.onmessage = (event) => {
                // 解析接收到的雷达数据
                const lidarData = JSON.parse(event.data);
                this.lidarData = lidarData;
                // console.log('收到雷达数据')
                // 如果 lidarLayer 不存在，则创建它
                if (!this.lidarLayer) {
                    this.lidarLayer = L.layerGroup().addTo(this.map);
                }

                // 更新标记
                lidarData.forEach((point, index) => {
                    let { x, y, intensity } = point;

                    const transformX = x * Math.cos(this.robotPose.z) - y * Math.sin(this.robotPose.z) + this.robotPose.x;
                    const transformY = x * Math.sin(this.robotPose.z) + y * Math.cos(this.robotPose.z) + this.robotPose.y;

                    x = transformX / this.map_info.resolution * this.map_info.scale;
                    y = transformY / this.map_info.resolution * this.map_info.scale;

                    // 计算颜色
                    const r = Math.round(255 * (1 - intensity));
                    const g = 0;
                    const b = Math.round(255 * intensity);
                    const color = `rgb(${r}, ${g}, ${b})`;

                    // 创建或更新标记
                    if (this.lidarMarkers[index]) {
                        // 更新已存在的标记
                        const marker = this.lidarMarkers[index];
                        marker.setLatLng([y, x]);
                        marker.setStyle({ fillColor: color, color: color });
                    } else {
                        // 新建标记并存储
                        const marker = L.circleMarker([y, x], {
                            radius: 2,
                            fillColor: color,
                            color: color,
                            weight: 1,
                            opacity: 1,
                            fillOpacity: 0.5,
                        }).addTo(this.lidarLayer);
                        this.lidarMarkers[index] = marker; // 存储引用
                    }
                });

                // 如果 lidarLayer 还没有添加到地图上，则添加
                if (!this.map.hasLayer(this.lidarLayer)) {
                    this.lidarLayer.addTo(this.map);
                }
            };

            this.lidarWebsocket.onerror = (error) => {
                console.error('WebSocket 发生错误:', error);
            };

            this.lidarWebsocket.onclose = () => {
                console.log('WebSocket 连接关闭');
            };

            // 初始化 markers 数组
            this.lidarMarkers = [];
        }
    },
    sendTask(taskType,angle){

      console.log('下发任务:',taskType,angle) ;  
      if (!this.taskPoint.endPoint){
        alert('请选择任务终点!');
        return;
      }
      console.log('任务终点:', this.taskPoint.endPoint);
      const task = {
        endPoint: this.taskPoint.endPoint.point.node_id,
        instruction: taskType,
        avoidanceMode: this.avoidanceMode
      }
      task.angle = typeof angle != 'undefined' ? angle : 0.0;
      


      console.log('任务:', task);
      const response = axios.post(`http://${this.publicIP}:5000/Task/sendTask`, task).then(response => {
        console.log('响应: ',response.data)
        Swal.fire({
          title: "任务下发结果",
          text: response.data.message,
          icon: response.data.status ? "success" : "error",
          // confirmButtonText: "确定",
          timer: 1500,
          showCloseButton: false,
          position: "top",
          backdrop: false,
          heightAuto: true,
        })
        // alert(response.data.message)
      }).catch(error => {
        console.error('任务下发失败:', error);
      });

      // alert('任务下发成功!');
    },

    getRobotPose() {
      setInterval(async () => {
        try {

          // console.log('Get robot pose');
          const response = await axios.get(`http://${this.publicIP}:5000/RobotControl/getRobotPose`);
          const robotPose = JSON.parse(response.data);
          this.robotPose = robotPose.pose;
          // console.log('Robot pose:', this.robotPose);
          this.updateRobotPosition();
        } catch (error) {
          console.error('Get robot pose error:', error);
        }
      }, 100);
    },
    initializeMap() {
      // console.log("初始化地图")
        this.map = L.map(this.$refs.mapContainer, {
          center: [0, 0], // Adjust based on your map coordinates
          zoom: 0,
          crs: L.CRS.Simple, // Simple CRS for custom maps
        });
  
        // Add an empty layer for nodes
        this.nodesLayer = L.layerGroup().addTo(this.map);
        this.edgeLayer = L.layerGroup().addTo(this.map);
        this.taskNodeLayer = L.layerGroup().addTo(this.map);
        this.lidarLayer = L.layerGroup().addTo(this.map);
        this.tooltipLayer = L.layerGroup().addTo(this.map);
        this.localPathLayer = L.layerGroup().addTo(this.map);
    },
    // 绘制地图到leaflet上
    async fetchMapImage() {
      try {
        const response = await axios.get(`http://${this.publicIP}:5000/Map/GetMapImage`, { responseType: 'blob' });
        console.log('收到地图响应')
        const imageBlob = response.data;
        const img = new Image();
        const imageURL = URL.createObjectURL(imageBlob);

        img.src = imageURL;
        img.onload = () => {
          const originalWidth = img.width;
          const originalHeight = img.height;

          // 获取Leaflet容器的大小
          const containerSize = this.map.getSize();
          this.containerSize = {width: containerSize.x, height: containerSize.y}
          const desireX = containerSize.x;
          const desireY = containerSize.y;

          // 计算缩放比例，使图片按比例缩放到容器大小
          const scaleX = desireX / originalWidth;
          const scaleY = desireY / originalHeight;
          const scale = Math.min(scaleX, scaleY);
          this.map_info = {
            ...this.map_info,
             width: originalWidth,
             height: originalHeight,
             scale: scale,
          }

          // 计算缩放后的图片尺寸
          const scaledWidth = originalWidth * scale;
          const scaledHeight = originalHeight * scale;

          // 使用缩放后的宽高设置地图边界（[y, x]）
          const bounds = [[0, 0], [scaledHeight, scaledWidth]];

          // 添加图像覆盖层，并将地图缩放到合适的边界
          // L.imageOverlay(imageURL, bounds).addTo(this.map);
          this.mapLayer = L.imageOverlay(imageURL, bounds);

          this.map.addLayer(this.mapLayer);
          this.map.fitBounds(bounds);
          this.drawNodes()
          this.drawEdges()
          // this.getRobotPose()
          // console.log(`Map image loaded: ${scaledWidth}x${scaledHeight}`);
        };
      } catch (error) {
        console.error('Failed to load map image:', error);
      }
    },


    async fetchMapData() {
      try {
        const response = await axios.post(`http://${this.publicIP}:5000/Map/GetMapData`);
        this.elementInfo = response.data.elements_info?JSON.parse(response.data.elements_info).elementInfo:null
        // console.log('elementInfo:', this.elementInfo)
        this.map_info = {
          ...this.map_info,
          resolution: JSON.parse(response.data.map_json).resolution,
        }
        this.map_info.edges = JSON.parse(response.data.map_edges).map_edges
        // console.log('边信息:',(this.map_info.edges))
        const json_node = JSON.parse(response.data.node);
        this.nodes = json_node.node;
      } catch (error) {
        console.error('Failed to load map data:', error);
      }
    },
    //显示普通点，充点电，储位点
    async drawNodes() {
      // console.log('此时的elementinfo: ', this.elementInfo);

      // 遍历 elementInfo 和 nodes 匹配节点，构建 instructionPoint 列表
      this.nodes.forEach(node => {
        let matched = false;
        if(this.elementInfo)
        {
          this.elementInfo.forEach(element => {
          const element_x = element.featureParam.featureOperationInfo.targetPoint.x;
          const element_y = element.featureParam.featureOperationInfo.targetPoint.y;

          const node_x = node.x;
          const node_y = node.y;

          if (Math.abs(element_x - node_x)<0.01 && Math.abs(element_y - node_y)<0.01) {
            // console.log('找到匹配的节点:', node);
            let originStyle;
            if (element.type === 1) {
              originStyle = this.nodeStyle.chargerPointStyle;
            } else if (element.type === 0) {
              originStyle = this.nodeStyle.deliverPointStyle; // 储位点样式
            } else {
              originStyle = this.nodeStyle.originalStyle; // 默认样式
            }

            this.instructionPoint.push({
              type: element.type,
              x: (element_x / this.map_info.resolution) * this.map_info.scale,
              y: (element_y / this.map_info.resolution) * this.map_info.scale,
              originStyle: originStyle,
              label: element.type === 1 ? '充点电' : (element.type === 0 ? '储位点' : undefined), // 只有当type为0或1时设置label，其他情况为undefined
              node_id: node.node_id,
            });

            matched = true;
          }
        });
        }
        

        // 如果没有匹配的节点，设置默认样式的 instructionPoint
        if (!matched) {
          this.instructionPoint.push({
            type: 2,
            x: (node.x / this.map_info.resolution) * this.map_info.scale,
            y: (node.y / this.map_info.resolution) * this.map_info.scale,
            originStyle: this.nodeStyle.originalStyle,
            node_id: node.node_id,
          });
        }
      });

      // console.log('节点信息:', this.instructionPoint);

      // 基于 instructionPoint 渲染 marker
      this.instructionPoint.forEach(point => {
        const x = point.x;
        const y = point.y;

        const marker = L.circleMarker([y, x], {
          radius: point.originStyle.radius || 1.5,
          color: point.originStyle.color || 'black',
          fillColor: point.originStyle.fillColor || 'black',
          fillOpacity: point.originStyle.fillOpacity || 1,
        });

        marker.point = point; // 保存节点信息
        marker.originalStyle = point.originStyle; // 保存原始样式
        marker.node_id = point.node_id; // 保存节点id
        // 如果是充电点或储位点，添加文字标签
        if (point.label) {
          const tooltip = L.tooltip({
            permanent: true,    // 永久显示
            direction: 'top',  // 在左侧显示
            className: 'custom-tooltip', // 自定义类名样式
            offset: [-10, 0],   // 调整与节点的距离
          })
            .setContent(point.label)  // 设置标签内容
            .setLatLng([y, x])  // 设置标签位置
          this.tooltipLayer.addLayer(tooltip);
          }

        // 添加 marker 到图层
        this.nodesLayer.addLayer(marker);
        this.taskNodeLayer.addLayer(marker);
      });
    },


    //当鼠标从某个高亮点移开时，恢复默认样式
    clearTaskNodeLayer() {
      // 重置任务点
      this.taskPoint = {
        startPoint: null,
        endPoint: null,
      };

      // 恢复节点的原始样式
      this.taskNodeLayer.eachLayer((node) => {
        const originalStyle = this.getInstructionPointStyle(node);
        node.setStyle(originalStyle);
      });

      // 移除起始点和终点的 tooltip
      if (this.taskPointTip.startTip) {
        this.map.removeLayer(this.taskPointTip.startTip);
        this.taskPointTip.startTip = null;
      }

      if (this.taskPointTip.endTip) {
        this.map.removeLayer(this.taskPointTip.endTip);
        this.taskPointTip.endTip = null;
      }

      // 取消事件监听
      this.map.off('mousemove', this.highLightMoveNode); // 取消高亮节点
      this.map.off('click', this.hightLightTaskNode); // 取消高亮任务节点

      this.startTask(); // 开始任务
    },
    // 高亮节点移动事件
    highLightMoveNode(event) {
      const distTolerance = 50;

      const isMatchingPoint = (node, targetPoint) => {
        return (
          targetPoint !== null &&
          node._latlng.lat === targetPoint._latlng.lat &&
          node._latlng.lng === targetPoint._latlng.lng
        );
      };

      const mousePoint = this.map.latLngToLayerPoint(event.latlng); // 鼠标的像素坐标
      let closestDistance = Infinity;
      let closetNode = null;

      // 遍历所有 markers，找到最近的节点
      this.nodesLayer.eachLayer((node) => {
        const markerPoint = this.map.latLngToLayerPoint(node.getLatLng()); // node 的像素坐标
        const pixelDistance = mousePoint.distanceTo(markerPoint);

        if (pixelDistance < closestDistance) {
          closestDistance = pixelDistance;
          closetNode = node;
        }
      });

      // 处理高亮逻辑
      if (closestDistance < distTolerance && closetNode) {
        if (this.closestNode && this.closestNode !== closetNode) {
          // 恢复之前高亮节点的样式为其原始样式
          const previousStyle = this.getInstructionPointStyle(this.closestNode);
          this.closestNode.setStyle(previousStyle);
        }

        // 设置新的高亮样式
        closetNode.setStyle(this.nodeStyle.highlightStyle);
        this.closestNode = closetNode; // 更新当前高亮的节点
      } else {
        // 如果没有符合条件的节点，恢复之前高亮的节点为其原始样式
        if (this.closestNode) {
          const previousStyle = this.getInstructionPointStyle(this.closestNode);
          this.closestNode.setStyle(previousStyle);
          this.closestNode = null;
        }
      }

      // 处理 taskNodeLayer 中的节点样式
      this.taskNodeLayer.eachLayer((node) => {
        if (isMatchingPoint(node, this.taskPoint.endPoint)) {
          node.setStyle(this.nodeStyle.endPointStyle);
        }
      });
    },

    // 获取对应的 instructionPoint 的样式
    getInstructionPointStyle(node) {
      const instruction = this.instructionPoint.find(
        (point) =>
          point.x === node._latlng.lng && point.y === node._latlng.lat
      );
      return instruction ? instruction.originStyle : this.nodeStyle.originalStyle;
    },


    hightLightTaskNode(event) {
      this.clearTaskNodeLayer(); // 清除任务图层

      if (this.closestNode) {
        // 设置终点（假设起始点逻辑在其他地方处理）
        if (this.taskPoint.endPoint === null) {
          this.taskPoint.endPoint = this.closestNode;
          console.log('结束点:', this.taskPoint.endPoint);
          // console.log('选中节点:', this.taskPoint.endPoint.node.node_id);

          // 设置结束点样式
          this.closestNode.setStyle(this.nodeStyle.endPointStyle);

          // 添加终点标签
          this.taskPointTip.endTip = L.tooltip({
            permanent: true, 
            direction: 'top',
            className: 'end-tooltip',
            offset: [0, -10],
          })
            .setContent('终点')
            .setLatLng(this.closestNode.getLatLng())
            .addTo(this.map);
        } else {
          // 如果已经设置了结束点，恢复之前的样式
          const previousStyle = this.getInstructionPointStyle(this.taskPoint.endPoint);
          this.taskPoint.endPoint.setStyle(previousStyle); 
          this.taskPoint.endPoint = this.closestNode;

          // console.log('更新结束点:', this.taskPoint.endPoint.node.node_id);

          // 更新样式为结束点样式
          this.closestNode.setStyle(this.nodeStyle.endPointStyle);

          // 更新终点标签位置
          this.taskPointTip.endTip.setLatLng(this.closestNode.getLatLng());
        }
      }
    },
    // 绑定按键，当按键触发，且鼠标距离node小于50px时，高亮显示node
    startTask() {
      console.log('开始高亮');

      this.map.on('mousemove', this.highLightMoveNode) //高亮离鼠标最近的节点
      // 绑定事件，当鼠标按下时，分别确定task的起始点和结束点
      this.map.on('click', this.hightLightTaskNode); //高亮选中的任务节点
    },


    updateRobotPosition() {
      const x = this.robotPose.x /this.map_info.resolution * this.map_info.scale
      const y = this.robotPose.y /this.map_info.resolution * this.map_info.scale
      const z = this.robotPose.z;
      // console.log("更新机器人位置:",x,y,z)
      if (this.robotMarker)
      {
        // this.robotMarker.
        this.robotMarker.setLatLng([y+4, x]);
        this.robotMarker.setRotationAngle(-(z * 180) / Math.PI); // Convert radians to degrees
      }
      else
      {
        const icon = L.divIcon({
          html: `
            <svg xmlns="http://www.w3.org/2000/svg" 
                width="24" 
                height="24" 
                fill="blue" 
                class="bi bi-backpack-fill" 
                viewBox="0 0 16 16" 
                style="transform: rotate(90deg);">
              <path d="M5 13v-3h4v.5a.5.5 0 0 0 1 0V10h1v3z" />
              <path d="M6 2v.341C3.67 3.165 2 5.388 2 8v5.5A2.5 2.5 0 0 0 4.5 16h7a2.5 2.5 0 0 0 2.5-2.5V8a6 6 0 0 0-4-5.659V2a2 2 0 1 0-4 0m2-1a1 1 0 0 1 1 1v.083a6 6 0 0 0-2 0V2a1 1 0 0 1 1-1m0 3a4 4 0 0 1 3.96 3.43.5.5 0 1 1-.99.14 3 3 0 0 0-5.94 0 .5.5 0 1 1-.99-.14A4 4 0 0 1 8 4M4.5 9h7a.5.5 0 0 1 .5.5v4a.5.5 0 0 1-.5.5h-7a.5.5 0 0 1-.5-.5v-4a.5.5 0 0 1 .5-.5" />
            </svg>`,
          className: '',
        });

  
        this.robotMarker = L.marker([y, x], { icon, rotationAngle: -(z * 180) / Math.PI }).addTo(this.map);


      }
    },
    createDirectionElements() {
      const { x, y, z: yaw } = this.robotPose;
      const { resolution, scale } = this.map_info;

      // 计算机器人位置在地图上的坐标
      const robotX = (x / resolution) * scale;
      const robotY = (y / resolution) * scale;

      // 创建旋转后的矩形顶点（根据 yaw 旋转）
      const rectangleSize = { width: 28, height: 28 }; // 宽度 24，高度 28
      const rectangleBounds = this.getRotatedRectangleBounds(
        robotX, robotY, rectangleSize, yaw
      );
      const lineLength = 15 / scale;
      const endX = robotX + lineLength * Math.cos(yaw);
      const endY = robotY + lineLength * Math.sin(yaw);

      this.directionPoint = L.circleMarker([endY, endX], {
        radius: 5,
        color: 'green',
      }).addTo(this.map);
      // this.directionLine = L.polyline([[robotY, robotX], [endY, endX]], {
      //   color: 'green',
      //   weight: 4,
      // }).addTo(this.map);
      // 绑定方向点的拖拽事件：控制矩形旋转
      this.bindRotationControl(this.directionPoint);

      // 绑定矩形拖拽事件：移动机器人
      this.bindRectangleDrag(this.robotMarker);
    },

    // 计算旋转矩形的顶点坐标
    getRotatedRectangleBounds(centerX, centerY, size, yaw) {
      const { width, height } = size;
      const halfWidth = width / 2;
      const halfHeight = height / 2;

      // 计算矩形四个顶点的相对位置
      const corners = [
        [-halfHeight, -halfWidth],
        [-halfHeight, halfWidth],
        [halfHeight, halfWidth],
        [halfHeight, -halfWidth]
      ];

      // 根据 yaw 旋转顶点
      return corners.map(([dy, dx]) => {
        const rotatedX = centerX + dx * Math.cos(yaw) - dy * Math.sin(yaw);
        const rotatedY = centerY + dx * Math.sin(yaw) + dy * Math.cos(yaw);
        return [rotatedY, rotatedX]; // Leaflet 坐标系为 (lat, lng)
      });
    },

    // 绑定方向点的拖拽事件，实现旋转
    bindRotationControl(directionPoint) {
        const draggable = new L.Draggable(directionPoint.getElement());
        draggable.enable();

        draggable.on('drag', (e) => {
            // 获取当前鼠标位置的地理坐标
            const clickedLatLng = this.map.mouseEventToLatLng(e.originalEvent);
            const point = this.map.latLngToContainerPoint(clickedLatLng);

            // 获取 mapLayer 的边界
            const bounds = this.mapLayer.getBounds();
            const downleft = this.map.latLngToContainerPoint(bounds.getSouthWest());

            // 计算相对于 mapLayer 的坐标
            const relativeX = point.x - downleft.x;
            const relativeY = downleft.y - point.y;

            // 计算新的 yaw 角度
            const centerX = this.robotPose.x / this.map_info.resolution * this.map_info.scale;
            const centerY = this.robotPose.y / this.map_info.resolution * this.map_info.scale;
            const newYaw = Math.atan2(relativeY - centerY, relativeX - centerX);

            this.robotPose.z = newYaw; // 更新机器人 yaw
            this.updateRobotPosition();

        });

    },

    bindRectangleDrag(roboMarker){
      this.draggableRobot = new L.Draggable(roboMarker.getElement());
      this.draggableRobot.enable();

      // 监听拖动事件
      this.draggableRobot.on('drag', (e) => {

          const clickedLatLng = this.map.mouseEventToLatLng(e.originalEvent);
          const point = this.map.latLngToContainerPoint(clickedLatLng);

          // 获取 mapLayer 的边界
          const bounds = this.mapLayer.getBounds();
          const downleft = this.map.latLngToContainerPoint(bounds.getSouthWest());

          // 计算相对于 mapLayer 的坐标
          const relativeX = point.x - downleft.x;
          const relativeY = downleft.y - point.y;
          
          const updateRobotX = relativeX / this.map_info.scale * this.map_info.resolution;
          const updateRobotY = relativeY / this.map_info.scale * this.map_info.resolution;

          const offset_x = (updateRobotX - this.robotPose.x)/this.map_info.resolution * this.map_info.scale;
          const offset_y = (updateRobotY - this.robotPose.y)/this.map_info.resolution * this.map_info.scale;

          this.robotPose.x = updateRobotX
          this.robotPose.y = updateRobotY
          console.log('当前机器人位置:', this.robotPose.x, this.robotPose.y);
          this.updateRobotPosition()
          this.directionPoint.setLatLng([this.directionPoint.getLatLng().lat + offset_y, this.directionPoint.getLatLng().lng + offset_x]);
      
        });

    },

    // 更新方向线的位置
    updateDirectionLine() {
      const { x, y, z: yaw } = this.robotPose;
      const { resolution, scale } = this.map_info;

      const robotX = (x / resolution) * scale;
      const robotY = (y / resolution) * scale;

      const lineLength = 15 / scale;
      const endX = robotX + lineLength * Math.cos(yaw);
      const endY = robotY + lineLength * Math.sin(yaw);

      this.directionLine.setLatLngs([[robotY, robotX], [endY, endX]]);
      this.directionPoint.setLatLng([endY, endX]);
    },

        // 绘制edges
    drawEdges(){
      this.map_info.edges.forEach(edge => {

        // 处理起点
        const startX = edge.startPoint.x / this.map_info.resolution * this.map_info.scale;
        const startY = edge.startPoint.y / this.map_info.resolution * this.map_info.scale;

        const startMarker = L.circleMarker([startY, startX], { 
          radius: 1.5, 
          color: 'black', 
          fillColor: 'black',
          fillOpacity: 1 
        });

        startMarker.originalStyle = {
          radius: 1.5,
          color: 'black',
          fillColor: 'black'
        }; // 保存原始样式

        const endX = edge.endPoint.x /this.map_info.resolution * this.map_info.scale
        const endY = edge.endPoint.y /this.map_info.resolution * this.map_info.scale

        // 1.绘制边
        if (edge.edgeType == 'line')
      {
        const line = L.polyline([[startY, startX], [endY, endX]], { color: 'black', weight: 2, opacity: 0.4 }).addTo(this.edgeLayer);
      }
      else if (edge.edgeType === 'bcurve') {
        const controlPoint1 = this.transformPoint(edge.controlPoints[0]);
        const controlPoint2 = this.transformPoint(edge.controlPoints[1]);

        const controlPoint3 = this.calculateControlPoint(startX, startY, controlPoint1)
        const controlPoint4 = this.calculateControlPoint(endX, endY, controlPoint2)
        console.log('控制点：', controlPoint1, controlPoint2, controlPoint3, controlPoint4)
        const bcurve = this.drawBCurve([
          {x:controlPoint3[0],y:controlPoint3[1]},
          {x:startX,y:startY},
          {x:controlPoint1[0],y:controlPoint1[1]},
          {x:controlPoint2[0],y:controlPoint2[1]},
          {x:endX,y:endY},
          {x:controlPoint4[0],y:controlPoint4[1]}]
        )
        bcurve.setStyle({color:'black'})
        this.edgeLayer.addLayer(bcurve)
      
      }
      else if (edge.edgeType === 'curve') {
          const startX = edge.startPoint.x / this.map_info.resolution * this.map_info.scale;
          const startY = edge.startPoint.y / this.map_info.resolution * this.map_info.scale;
          const endX = edge.endPoint.x / this.map_info.resolution * this.map_info.scale;
          const endY = edge.endPoint.y / this.map_info.resolution * this.map_info.scale;
          // 转换控制点为 Leaflet 坐标
          const controlPoints = edge.controlPoints.map(cp => [
            cp.y / this.map_info.resolution * this.map_info.scale,
            cp.x / this.map_info.resolution * this.map_info.scale,
          ]);

          // 构造贝塞尔曲线
          const bezierPoints = [[startY, startX]];
          controlPoints.forEach(cp => bezierPoints.push(cp));
          bezierPoints.push([endY, endX]);
          // 确保路径数据格式正确
          const pathData = ['M', bezierPoints[0]];

          // 使用 C 指令构建贝塞尔曲线路径
          for (let i = 1; i < bezierPoints.length - 2; i += 2) {
            pathData.push(
              'C', bezierPoints[i], bezierPoints[i + 1], bezierPoints[i + 2]
            );
          }


          // 绘制贝塞尔曲线
          L.curve(pathData, { color: 'black', weight: 2, opacity: 0.4 }).addTo(this.edgeLayer);
        }
      }
    )
    },

    updateVelocity(velocity) {
        if (this.movethread == null) {
          return;
        }

        const clampVelocity = (vel, maxVel) => Math.max(Math.min(vel, maxVel), -maxVel);

        velocity[0] = clampVelocity(velocity[0], this.maxLinearVel);
        velocity[1] = clampVelocity(velocity[1], this.maxAngularVel);

        console.log('收到的速度:', velocity);

        axios.post(`http://${this.publicIP}:5000/RobotControl/setvel`, {
          velocity: velocity,
        });
    },
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
    transformPoint(point) {
        // 算法转到leaflet坐标系
        //输入：算法层坐标
        //输出：leaflet坐标
        const transformedX = point.x / this.map_info.resolution * this.map_info.scale;
        const transformedY = point.y / this.map_info.resolution * this.map_info.scale;
        return [transformedX, transformedY];
    },
    transformPointReverse(point) {
      // leaflet坐标转到算法层坐标
      //输入：leaflet坐标
      //输出：算法层坐标
        const transformedX = point.x / this.map_info.scale * this.map_info.resolution;
        const transformedY = point.y / this.map_info.scale * this.map_info.resolution;
        return { x: transformedX, y: transformedY };
    },
    calculateControlPoint(baseX, baseY, targetPoint) {
        const dx = targetPoint[0] - baseX;
        const dy = targetPoint[1] - baseY;
        
        // 计算控制点的坐标，保持与起点或终点的相同距离，但方向相反
        const controlPointX = baseX - dx;
        const controlPointY = baseY - dy;

        return [controlPointX, controlPointY];
    },
    drawBCurve(bCurvePoints) {
        //console.log('绘制B样条曲线',bCurvePoints)
        const points = bCurvePoints.map(point => [point.x, point.y]); // [latitude, longitude] 格式
        const degree = 3;
        // 生成样条曲线上的点集
        const splinePoints = [];
        const numPoints = 500; // 设置曲线的细腻度，点越多曲线越平滑
        for (let i = 0; i <= numPoints; i++) {
          const t = i / numPoints;
          const point = BSpline(t, degree, points);
          splinePoints.push(point);
        }

        // 将生成的点转为 Leaflet 所需的格式
        const latlngs = splinePoints.map(point => [point[1], point[0]]); // [latitude, longitude] 格式

        // 在 Leaflet 中绘制 B 样条曲线
        const bcurve = L.polyline(latlngs, {
          color: 'blue',
          opacity: 0.6,
          weight: 2
        })
        // 添加到图层中
        return bcurve
    },

  },
};
</script>
<style scoped>
.container {
  display: flex;
  flex-direction: row;
}
.map-container {
    width: 1700px; /*TODO 这里由于设置了固定像素导致无法自适应 */
    height: 800px;
    /* border: 1px solid #ccc; */
    position: relative;
    border-right: 2px solid #ccc;
  }
.task-point-container {
  border: 2px solid #ccc; /* 边框样式 */
}
.control-container {
  padding: 10px; 
  background-color: #f9f9f9; /* 背景颜色 */
  width: auto;
  margin-bottom: 10px;
  flex-direction: column;
  /* align-items: center; */
  display:flex;
}
.row-button {
  display: flex; /* 行方向排列按钮 */
  justify-content: flex-start; /* 按钮左对齐 */
  width:100%;
}

.task-container {

  border-radius: 8px; /* 圆角 */
  padding: 20px; /* 内边距 */
  background-color: #f9f9f9; /* 背景颜色 */
}

.task {
  height: 100%;
  display: flex; /* 使内部内容居中 */
  flex-direction: column;
}

.gray-color{
  color: #d3dadf;
}

</style>