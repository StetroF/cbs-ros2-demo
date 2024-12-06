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
        origin:null
      },
      test_x :1,
      map: null,
      instructionPoint: [],
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
            radius: 2.0,
            color: 'black',
            fillColor: 'black',
            fillOpacity: 1
          },
          startPointStyle:{
            radius:4.0,
            color: 'green',
            fillColor: 'green'
          },
          endPointStyle:{
            radius: 4.0,
            color: 'blue',
            fillColor: 'blue'
          },
          highlightStyle:{
            radius: 3.5,
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

      robotMarkerSubject:new Subject(),
    }
  },
  mounted(){
    this.initializeMap()
    this.fetchMapData().then(() => 
    {
      this.fetchMapImage()

      // this.localPathSubject.pipe(debounceTime(100)).subscribe
      // (localPath => {
      //   this.localPath = localPath; // 更新本地路径
      //   this.updateLocalPath(); // 更新图层
      // });
    })
    return this.historyRobotPose
  },
  created() {
    const store = useStore();
    const ip = computed(() => store.state.publicIP);
    this.publicIP = ip;
    
    window.addEventListener('keydown', this.highlightNearestNode);
    this.subscribeRobotPose()

    //初始化函数，请求python层每次切换到这个界面时，都会重新加载一遍history_map，保证在编辑地图后，python保存的是最新的node和edge信息
    // axios.post(`http://${this.publicIP}:5000/Task/init_node`)

    this.listenKeyboard()
    this.robotMarkerSubject.subscribe(robotPose => {
      this.updateRobotMarker(robotPose)})
  },
  beforeUnmount() {
        this.unsubscribeRobotPose()
      },
  methods: {


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
    drawEdges() {
      // 清空现有的边缘图层
          this.edgeLayer.clearLayers();
          // if (!this.map_info.edges)
          // {
          //   alert('当前没有获取到边!')
          //     return;
          // }
          if (!this.map_info.edges) {
            return;}
          this.map_info.edges.forEach(edge => 
          {
            const [visLine,seletableObj]=this.addEdge(edge)
            this.edgeLayer.addLayer(visLine)
            this.edgeLayer.addLayer(seletableObj)
          
          })

      },  

    unsubscribeRobotPose(){
      if (this.robotPoseWebsocket){
        this.robotPoseWebsocket.close();
        this.robotPoseWebsocket = null;
      }
    },
    subscribeRobotPose(){
      this.robotMarkerSubject = new Subject();
      this.robotMarker = {};
      if (!this.robotPoseWebsocket){
        this.robotPoseWebsocket = new WebSocket(`ws://${this.publicIP}:5000/Robot/robot_pose_websocket`);
        this.robotPoseWebsocket.onopen = () => {
          console.log('RobotPose WebSocket 连接成功');
        };
        this.robotPoseWebsocket.onmessage = (event) => {
          const robotPose = JSON.parse(event.data);
          
          this.robotMarkerSubject.next(robotPose)

        };
        this.robotPoseWebsocket.onerror = (error) => {
          console.error('WebSocket 发生错误:', error);
        };
        this.robotPoseWebsocket.onclose = () => {
          console.log('WebSocket 连接关闭');
        };

      }
    },
    updateRobotMarker(robotPoses){
      // console.log('更新机器人位置:',robotPoses)
      for(let robot_id in robotPoses){
        
        let gazebo_robotpose = robotPoses[robot_id]
        // console.log('机器人角度:',gazebo_robotpose)
        // console.log('origin:',this.map_info.origin)

        let mapRobotPose = this.transformPoint(gazebo_robotpose)



        if (!this.robotMarker[robot_id]){
          this.robotMarker[robot_id] = L.marker([mapRobotPose[1], mapRobotPose[0]], {
            icon: L.divIcon({
              html: this.robotSvg,
              className: 'robot-marker',
              iconSize: [30, 30], 
              iconAnchor: [15, 15],
              })
          }).addTo(this.map);
          this.robotMarker[robot_id].setRotationAngle(-(gazebo_robotpose[2] *180/Math.PI -90))
        }
        else{
          this.robotMarker[robot_id].setLatLng([mapRobotPose[1], mapRobotPose[0]]);
          this.robotMarker[robot_id].setRotationAngle(-(gazebo_robotpose[2] * 180/Math.PI-90))

        }
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
          // console.log(`Map image loaded: ${scaledWidth}x${scaledHeight}`);
        };
      } catch (error) {
        console.error('Failed to load map image:', error);
      }
    },


    async fetchMapData() {
      try {
        const response = await axios.get(`http://${this.publicIP}:5000/Map/GetMapData`);
        // this.elementInfo = response.data.elements_info?JSON.parse(response.data.elements_info).elementInfo:null
        // console.log('elementInfo:', this.elementInfo)
        console.log('收到地图数据响应',JSON.parse(response.data.map_json))
        this.map_info = {
          ...this.map_info,
          resolution: JSON.parse(response.data.map_json).resolution,
          origin: JSON.parse(response.data.map_json).origin,
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
    drawNodes() {
        // 清空现有的节点图层
        this.nodesLayer.clearLayers();
        if (this.taskNodeLayer) {
            this.taskNodeLayer.clearLayers(); // 如果 taskNodeLayer 也需要清空
        }

        this.instructionPoint = []; // 重置 instructionPoint 列表

        // 清除现有的工具提示
        if (this.tooltipLayer) {
            this.tooltipLayer.clearLayers(); // 清除工具提示图层
        }

        // 遍历 elementInfo 和 nodes 匹配节点，构建 instructionPoint 列表
        if (!this.nodes){
          console.log('没有节点数据')
            return;
        }
        this.nodes.forEach(node => {
            console.log('node:', node)  
            let matched = false;
            if (this.elementInfo)
             { this.elementInfo.forEach(element => {
                  const element_x = element.featureParam.featureOperationInfo.targetPoint.x;
                  const element_y = element.featureParam.featureOperationInfo.targetPoint.y;

                  const node_x = node.x;
                  const node_y = node.y;

                  if (Math.abs(element_x - node_x)<0.01 && Math.abs(element_y - node_y)<0.01) {
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
                          label: element.type === 1 ? '充点电' : (element.type === 0 ? '储位点' : undefined),
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
            const markerTooltip = L.tooltip({
                permanent: true,    // 永久显示
                direction: 'top',  // 在上方显示
                className: 'custom-tooltip', // 自定义类名样式
                offset: [0, -10],   // 调整与节点的距离
            })
            .setContent(point.node_id)  // 设置标签内容
            .setLatLng([y, x]);  // 设置标签位置
            marker.tooltip = markerTooltip;
            this.tooltipLayer.addLayer(markerTooltip); // 添加到工具提示图层

            if (point.label) {
                const tooltip = L.tooltip({
                    permanent: true,    // 永久显示
                    direction: 'top',  // 在上方显示
                    className: 'custom-tooltip', // 自定义类名样式
                    offset: [0, -10],   // 调整与节点的距离
                })
                .setContent(point.node_id)  // 设置标签内容
                .setLatLng([y, x]);  // 设置标签位置
                marker.tooltip = tooltip;
                this.tooltipLayer.addLayer(tooltip); // 添加到工具提示图层
            }

            // 创建一个透明的 DivIcon 用于增大拖动判定范围
            const dragIcon = L.divIcon({
                className: 'transparent-drag-area', // 定义透明图标的样式
                iconSize: [20, 20],                 // 设置透明区域的大小
                iconAnchor: [10, 10],               // 图标锚点
            });
            const dragMarker = L.marker([y, x], {
                icon: dragIcon,
                draggable: false,                    // 默认禁止拖动
                opacity: 0,                         // 透明度设为 0
            }).addTo(this.map);


            marker.point = point; // 保存节点信息
            marker.originalStyle = point.originStyle; // 保存原始样式
            marker.relativeEdges = []
            marker.dragMarker = dragMarker
            // 将拖动事件同步到实际的 circleMarker 上
            marker.dragMarker.on('drag', (event) => {
              const markerLatLng = event.target.getLatLng();
              const point = this.map.latLngToContainerPoint(markerLatLng);
              // 获取 mapLayer 的边界
              const bounds = this.imageOverlay.getBounds();
              const downleft = this.map.latLngToContainerPoint(bounds.getSouthWest());

              
              // 计算相对于 mapLayer 的坐标
              let relativeX = point.x - downleft.x;
              let relativeY = downleft.y - point.y;

              const zoom = this.map.getZoom();
              relativeX = relativeX / (Math.pow(2, zoom))
              relativeY = relativeY / (Math.pow(2, zoom))
              this.nodeSubject.next({
                  node_id: marker.point.node_id,
                  x: relativeX,
                  y: relativeY,
                  delete:false
                })

            });
            // 添加 marker 到图层
            this.nodesLayer.addLayer(marker);
        });
        const isMatchingPoint = (node, targetPoint) => {
          return (
            targetPoint !== null &&
            node._latlng.lat === targetPoint._latlng.lat &&
            node._latlng.lng === targetPoint._latlng.lng
          );
        };

        this.taskNodeLayer.eachLayer((node) => {
          if (isMatchingPoint(node, this.taskPoint.startPoint)){
            node.setStyle(this.nodeStyle.startPointStyle);
          }
          if (isMatchingPoint(node, this.taskPoint.endPoint)) {
            node.setStyle(this.nodeStyle.endPointStyle);
          }
        })
        // 确保工具提示图层被添加到地图上（如果尚未添加）
        if (!this.map.hasLayer(this.tooltipLayer)) {
            this.tooltipLayer.addTo(this.map);
        }
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
        const transformedX = point[0] / this.map_info.resolution * this.map_info.scale;
        const transformedY = point[1] / this.map_info.resolution * this.map_info.scale;
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
    addEdge(edge){
        // if(this.nodeLayer){

          const startX = edge.startPoint.x / this.map_info.resolution * this.map_info.scale;
          const startY = edge.startPoint.y / this.map_info.resolution * this.map_info.scale;
          const endX = edge.endPoint.x / this.map_info.resolution * this.map_info.scale;
          const endY = edge.endPoint.y / this.map_info.resolution * this.map_info.scale;
          const edge_id = edge.edgeId





          this.nodesLayer.eachLayer((node) => {
            if (Math.abs(node.point.x - startX) < 15 && Math.abs(node.point.y - startY) < 15
              || Math.abs(node.point.x - endX) < 15 && Math.abs(node.point.y - endY) < 15){
                node.relativeEdges.push(edge_id)
            }})

          // 1.绘制边
          if (edge.edgeType === 'line') {
              const line = L.polyline([[startY, startX], [endY, endX]], {
                  color: 'black',
                  weight: 2,
                  opacity: 0.4
              });
              
              line.edgeType = 'line';
              line.edgeId = edge_id;
              line.width = edge.width;
              line.velocity = edge.velocity;
              line.topoType = edge.topoType;
              line.allowBackward = edge.allowBackward;
              line.dockingType = edge.dockingType;
              line.avoidanceMode = edge.avoidanceMode;
              line.safetyZoneType = edge.safetyZoneType;
              line.requireApply = edge.requireApply;
              line.elementId = edge.elementId;
              line.navigationMode = edge.navigationMode

              line.startPoint = { x: startX, y: startY };
              line.endPoint = { x: endX, y: endY };
              // 创建一个透明且更粗的 polyline 覆盖层
              const selectableObj = L.polyline([[startY, startX], [endY, endX]], {
                  color: 'black',
                  weight: 20, // 更粗的线条，方便捕捉鼠标事件
                  opacity: 0.0 // 完全透明
              });

              line.selectableObj = selectableObj
              // 将源线和透明线都添加到图层
              return [line,selectableObj]

          }

          else if (edge.edgeType === 'bcurve') {
            const controlPoint1 = this.transformPoint(edge.controlPoints[0]);
            const controlPoint2 = this.transformPoint(edge.controlPoints[1]);

            const controlPoint3 = this.calculateControlPoint(startX, startY, controlPoint1)
            const controlPoint4 = this.calculateControlPoint(endX, endY, controlPoint2)
            // console.log('控制点：', controlPoint1, controlPoint2, controlPoint3, controlPoint4)
            const bcurve = this.drawBCurve([
              {x:controlPoint3[0],y:controlPoint3[1]},
              {x:startX,y:startY},
              {x:controlPoint1[0],y:controlPoint1[1]},
              {x:controlPoint2[0],y:controlPoint2[1]},
              {x:endX,y:endY},
              {x:controlPoint4[0],y:controlPoint4[1]}]
            )
            bcurve.setStyle({color:'black'})
            const selectableObj = this.drawBCurve([
              {x:controlPoint3[0],y:controlPoint3[1]},
              {x:startX,y:startY},
              {x:controlPoint1[0],y:controlPoint1[1]},
              {x:controlPoint2[0],y:controlPoint2[1]},
              {x:endX,y:endY},
              {x:controlPoint4[0],y:controlPoint4[1]}],
            )
            selectableObj.setStyle({opacity:0.0,weight:20,color:'black'})
            bcurve.selectableObj = selectableObj

            bcurve.startPoint = {x:startX,y:startY}
            bcurve.endPoint = {x:endX,y:endY}
            bcurve.topoType = edge.topoType
            bcurve.width = edge.width
            bcurve.velocity = edge.velocity
            bcurve.allowBackward = edge.allowBackward
            bcurve.dockingType = edge.dockingType
            bcurve.avoidanceMode = edge.avoidanceMode
            bcurve.safetyZoneType = edge.safetyZoneType
            bcurve.requireApply = edge.requireApply
            bcurve.elementId = edge.elementId
            bcurve.navigationMode = edge.navigationMode

            bcurve.controlPoints = [
              controlPoint1,
              controlPoint2,
            ]

            bcurve.edgeId = edge_id
            bcurve.edgeType = 'bcurve'
            return [bcurve,selectableObj]

          
          }
          else if (edge.edgeType === 'curve') 
          {

              // 转换控制点为 Leaflet 坐标
              const controlPoints = edge.controlPoints.map(cp => [
                  cp.y / this.map_info.resolution * this.map_info.scale,
                  cp.x / this.map_info.resolution * this.map_info.scale,
              ]);

              // 构造贝塞尔曲线
              const bezierPoints = [[startY, startX]];
              controlPoints.forEach(cp => bezierPoints.push(cp));
              bezierPoints.push([endY, endX]);


              const numPoints = 200; // 调整近似的精度

              const polylinePoints = [];
              for (let t = 0; t <= 1; t += 1 / numPoints) {
                  const pt = calculateCubicBezier(bezierPoints, t);
                  polylinePoints.push(pt);
              }
              const curve = L.polyline(polylinePoints, { color: 'black', weight: 2, opacity: 0.4 });
              curve.startPoint = {x:startX,y:startY}
              curve.endPoint = {x:endX,y:endY}
              curve.topoType = edge.topoType
              curve.width = edge.width
              curve.velocity = edge.velocity
              curve.allowBackward = edge.allowBackward
              curve.dockingType = edge.dockingType
              curve.avoidanceMode = edge.avoidanceMode
              curve.safetyZoneType = edge.safetyZoneType
              curve.requireApply = edge.requireApply
              curve.elementId = edge.elementId  
              curve.navigationMode = edge.navigationMode

              curve.edgeId = edge_id
              curve.edgeType = 'curve'
              curve.controlPoints = controlPoints

              const selectableObj = L.polyline(polylinePoints, { color: 'black', weight: 20, opacity: 0.0 });
              selectableObj.setStyle({opacity:0.0,weight:20,color:'black'})
              curve.selectableObj = selectableObj

              return [curve,selectableObj]
          }
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