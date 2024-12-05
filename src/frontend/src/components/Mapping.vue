<template>
    <div class="container">
        
        <div ref="mapContainer" class="map-container"></div>
        <div style="flex-direction: column;width:300px">
          <div class="control-container" style="height: 400px">
            <div class="row-button">
              <h2>摇杆</h2>
            </div>
            <!-- <div class="row" style = "flex-direction: row;"> -->
              <div class="row-button">
                <el-button color="#626aef" v-if="movethread === null" type="el-button" class="btn btn-primary btn-sm" @click="startMove()">开始移动</el-button>
                <el-button color="#626aef" v-else type="el-button" class="btn btn-primary btn-sm " @click="stopMove()">停止移动</el-button>
                

              </div>
            <!-- </div> -->
            <br></br>
            <br></br>
            <Joystick
              :size="150"
              base-color="#D3D3D3"
              stick-color="#403c70"
              :throttle="100"
              @set-velocity="updateVelocity"
            />
            <div class='row-button' style="flex-direction: column;">

              <label>最大线速度</label>
              <el-input-number v-model="maxLinearVel" :precision="2" :step="0.1" :min="0.1" :max="1.5" placeholder="m/s" style="margin-left: 15px;"/>
              <label>最大角速度</label>
              <el-input-number v-model="maxAngularVel" :precision="2" :step="0.1" :min="0.1" :max="1.5" placeholder="rad/s" style="margin-left: 15px;"/>
            </div>

          </div>
          <div class="control-container"style="height:450px;align-items: start"> 
            <h2>建图栏</h2>
            
              <el-button v-if="mappingResponse" color="#626aef" disabled type="el-button" class="btn btn-primary btn-sm" @click="stopMapping()">开始建图</el-button>
              <el-button v-else color="#626aef" type="el-button" class="btn btn-primary btn-sm" @click="startMapping()">开始建图</el-button>

              <div>
                <el-radio v-model="recordMappingData" :label="true">记录建图数据</el-radio>
                <el-radio v-model="recordMappingData" :label="false">不记录建图数据</el-radio>
              </div>
            <br />

            <label style="font-weight: bold; margin-bottom: 10px;">地图名字:</label>
            <input style="height: 20px;width: 100%;" type="text" id="MapName" v-model="MapName" min="1" />
            
            <!-- 是否导出的单选框 -->
            <label style="font-weight: bold; margin-bottom: 10px;">是否导出:</label>
            <el-radio-group v-model="shouldExport">
              <el-radio :label="true">是</el-radio>
              <el-radio :label="false">否</el-radio>
            </el-radio-group>
            <br /><br />

            <el-button type="primary" @click="saveMap(MapName)">保存地图</el-button>
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

  
  export default {
    components:{
      Joystick
    },
    data() {
      return {
        maxLinearVel: 0.3,
        maxAngularVel: 0.2,
        recordMappingData: true,
        movethread:null, //绑定是否开始移动的按键
        publicIP: null,
        nodes: [],
        canvasWidth: 1600,
        canvasHeight: 500,
        shouldExport: false,
        robotPose: { x: 0.0, y: 0.0, z: 0.0 },// Z表示角度
        robotSvg: `<svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" fill="green" class="bi bi-backpack-fill" viewBox="0 0 16 16">
                      <path d="M5 13v-3h4v.5a.5.5 0 0 0 1 0V10h1v3z" />
                      <path d="M6 2v.341C3.67 3.165 2 5.388 2 8v5.5A2.5 2.5 0 0 0 4.5 16h7a2.5 2.5 0 0 0 2.5-2.5V8a6 6 0 0 0-4-5.659V2a2 2 0 1 0-4 0m2-1a1 1 0 0 1 1 1v.083a6 6 0 0 0-2 0V2a1 1 0 0 1 1-1m0 3a4 4 0 0 1 3.96 3.43.5.5 0 1 1-.99.14 3 3 0 0 0-5.94 0 .5.5 0 1 1-.99-.14A4 4 0 0 1 8 4M4.5 9h7a.5.5 0 0 1 .5.5v4a.5.5 0 0 1-.5.5h-7a.5.5 0 0 1-.5-.5v-4a.5.5 0 0 1 .5-.5" />
                    </svg>`,
        map_info:{
          resolution: 0.05,
          width: null,
          height: null,
          scale: 1.0, //相对于canvas的缩放比
          edges:[],
        },
        map: null,
        robotMarker: null|L.Marker,//显示机器人位置
        lidarLayer: null,//显示雷达数据
        mapLayer: null ,

        lidarData: null,
        lidarWebsocket : null,
        mapSocket:null,
        mappingResponse: false,
      }
    },
    mounted(){
      this.initializeMap()

        // this.switchGlobalLidarMode()
        // this.fetchMapData().then(() => 
        // {
        // })
        // this.subscribeLidar()
        // this.subscribeRobotPose()
    },
    setup(){
        // created()
    },
    created() {
      // this.initializeMap()

      const store = useStore();
      const ip = computed(() => store.state.publicIP);
      this.publicIP = ip;
      
      window.addEventListener('keydown', this.highlightNearestNode);
      this.listenKeyboard()
    },
      beforeUnmount() {
        this.unscribeLidar()
        this.unscribeLidarRobotPose()
        this.unscscribeMap()
        this.switchGlobalLidarMode()
      },
    methods: {
      listenKeyboard(){
        const keysPressed = {};

        function updateMovement() {
            if (keysPressed['w'] && keysPressed['d']) {
                this.updateVelocity([0.3, -0.3]); // 前进并左转
            } else if (keysPressed['w'] && keysPressed['a']) {
                this.updateVelocity([0.3, 0.3]); // 前进并右转
            } else if (keysPressed['s'] && keysPressed['d']) {
                this.updateVelocity([-0.3, 0.3]); // 后退并左转
            } else if (keysPressed['s'] && keysPressed['a']) {
                this.updateVelocity([-0.3, -0.3]); // 后退并右转
            } else if (keysPressed['w']) {
                this.updateVelocity([0.3, 0]); // 仅前进
            } else if (keysPressed['s']) {
                this.updateVelocity([-0.3, 0]); // 仅后退
            } else if (keysPressed['a']) {
                this.updateVelocity([0, 0.3]); // 仅左转
            } else if (keysPressed['d']) {
                this.updateVelocity([0, -0.3]); // 仅右转
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

        switchGlobalLidarMode(){
            axios.post(`http://${this.publicIP}:5000/switchGlobalLidarMode`).then(response => {
                console.log(response.data);
            })
        },
        saveMap(MapName) {
          console.log("开始保存地图");
          console.log('请求:', {
              mapName: MapName,
              export: this.shouldExport  // 确保 export 为 true 以请求压缩包
          });
          axios.post(`http://${this.publicIP}:5000/saveMap`, {
              mapName: MapName,
              export: this.shouldExport  // 确保 export 为 true 以请求压缩包
          }, {
              responseType: 'blob'  // 指定响应类型为 blob，用于接收文件流
          }).then(async response => {
              this.mappingResponse = false;

              // 检查是否为文件响应
              const isFileResponse = response.headers['content-type'] === 'application/zip';
              if (isFileResponse) {
                  // 创建 URL 链接下载文件
                  const url = window.URL.createObjectURL(new Blob([response.data]));
                  const link = document.createElement('a');
                  link.href = url;
                  link.setAttribute('download', `${MapName}.zip`);  // 设置下载文件名
                  document.body.appendChild(link);
                  link.click();
                  document.body.removeChild(link);

                  Swal.fire({
                      title: '保存成功!',
                      text: '地图文件已保存并下载。',
                      icon: 'success',
                      confirmButtonText: '确认',
                      allowOutsideClick: false,
                      timer: 1500,
                      showCloseButton: false,
                      position: "top",
                      backdrop: false,
                      heightAuto: true,
                  });
              } else {
                  // 将 blob 数据解析为 JSON
                  const responseData = await response.data.text();
                  const jsonResponse = JSON.parse(responseData);

                  Swal.fire({
                      title: jsonResponse.status ? '保存成功!' : '保存失败!',
                      text: jsonResponse.message,
                      icon: jsonResponse.status ? 'success' : 'error',
                      confirmButtonText: '确认',
                      allowOutsideClick: false,
                      timer: 1500,
                      showCloseButton: false,
                      position: "top",
                      backdrop: false,
                      heightAuto: true,
                  });
              }

              this.unscscribeMap();
              this.fetchMapData().then(() => {
                  this.fetchMapImage();
              });
          }).catch(error => {
              console.log(error);
              Swal.fire({
                  title: '保存失败!',
                  text: '无法保存地图文件。',
                  icon: 'error',
                  confirmButtonText: '确认',
                  allowOutsideClick: false,
                  showCloseButton: false,
                  position: "top",
                  backdrop: false,
                  heightAuto: true,
              });
          });
      },
        async updateMap(base64Image) {
        // 移除现有的图层
            this.map.eachLayer(layer => {
                if (layer instanceof L.ImageOverlay) {
                    this.map.removeLayer(layer);
                }
            });

            // 创建新的图像对象
            const img = new Image();
            const imageURL = `data:image/png;base64,${base64Image}`; // 假设图像格式为PNG

            img.src = imageURL;
            img.onload = () => {
                const originalWidth = img.width;
                const originalHeight = img.height;

                // 获取Leaflet容器的大小
                const containerSize = this.map.getSize();
                const desireX = containerSize.x;
                const desireY = containerSize.y;

                // 计算缩放比例
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
                L.imageOverlay(imageURL, bounds).addTo(this.map);
                this.map.fitBounds(bounds);
            };

            img.onerror = () => {
                console.error('Failed to load image from base64:', imageURL);
            };
        },
        unscscribeMap(){
            if (this.mapSocket) {
                this.mapSocket.close();
                this.mapSocket = null;
            }
        },
        startMapping(){
          if (this.recordMappingData){
            const requsetUrl = `http://${this.publicIP}:5000/recordMappingData`;
            axios.post(requsetUrl, {
              request: this.recordMappingData,
            })
          }
         this.subscribeMap()
         this.fetchMapData().then(() => 
         {
         this.subscribeRobotPose()
         this.subscribeLidar()
         })
        },
        subscribeMap(){
            if (!this.mapSocket){
                this.mappingResponse = true;
                this.mapSocket = new WebSocket(`ws://${this.publicIP}:5000/map_websocket`);
                this.mapSocket.onopen = () => {
                    console.log('Map WebSocket 连接成功');
                    Swal.fire({
                        title: '建图响应!',
                        text: '开始建图!',
                        icon:'success',
                        confirmButtonText: '确认',
                        allowOutsideClick: false,
                        timer: 1500,
                        showCloseButton: false,
                        position: "top",
                        backdrop: false,
                        heightAuto: true,
                    })
                };
                this.mapSocket.onmessage = (event) => {
                  console.log('Map data received:');
                    const map_data = JSON.parse(event.data);
                    const image = map_data.map
                    this.updateMap(image);
                };
                this.mapSocket.onerror = (error) => {
                    console.error('WebSocket 发生错误:', error);
                };
                this.mapSocket.onclose = () => {
                    console.log('WebSocket 连接关闭');
                };
            }   
        },
        unscribeLidarRobotPose(){
            console.log("取消订阅机器人位置")
            if (this.robotPoseWebsocket) {
                this.robotPoseWebsocket.close();
                this.robotPoseWebsocket = null;
                }
            },
        subscribeRobotPose(){
        if (!this.robotPoseWebsocket){
            this.robotPoseWebsocket = new WebSocket(`ws://${this.publicIP}:5000/robot_pose_websocket`);
            this.robotPoseWebsocket.onopen = () => {
            console.log('RobotPose WebSocket 连接成功');
            };
            this.robotPoseWebsocket.onmessage = (event) => {
            const robotPose = JSON.parse(event.data);
            this.robotPose = robotPose.pose;
            console.log('机器人位置:',this.robotPose)
            this.updateRobotPosition();
            };
            this.robotPoseWebsocket.onerror = (error) => {
            console.error('WebSocket 发生错误:', error);
            };
            this.robotPoseWebsocket.onclose = () => {
            console.log('WebSocket 连接关闭');
            };
        }

        
        },
      unscribeLidar() {
        console.log("取消订阅雷达数据")
        if (this.lidarWebsocket) {
          this.lidarWebsocket.close();
          this.lidarWebsocket = null;
        }
      },
      subscribeLidar() {
        if (! this.lidarWebsocket)
        {
          this.lidarWebsocket = new WebSocket(`ws://${this.publicIP}:5000/lidar_websocket`);
            
            this.lidarWebsocket.onopen = () => {
                console.log('雷达WebSocket 连接成功');
            };
  
            this.lidarWebsocket.onmessage = (event) => {
        // 解析接收到的雷达数据
                // console.log('Lidar data received:');
                const lidarData = JSON.parse(event.data); // 确保数据是有效的 JSON 格式
                // console.log('Lidar data:',lidarData);
                this.lidarData = lidarData;
                console.log()
                // 清空之前的雷达层（可选）
                if (this.lidarLayer) {
                    this.lidarLayer.clearLayers();  // 如果已经有点云数据，先清空
                }
  
                lidarData.forEach(point => {
                    let { x, y, intensity } = point; // 假设每个点包含 x, y 和 intensity 属性
                    const transformX = x * Math.cos(this.robotPose.z) - y * Math.sin(this.robotPose.z) + this.robotPose.x;
                    const transformY = x * Math.sin(this.robotPose.z) + y * Math.cos(this.robotPose.z) + this.robotPose.y;

                    x = transformX / this.map_info.resolution * this.map_info.scale;
                    y = transformY / this.map_info.resolution * this.map_info.scale;

                    const marker = L.circleMarker([y, x], {
                        radius: 2,  // 根据需要调整半径
                        fillColor: `rgba(255, 0, 0, ${intensity / 255})`, // 将强度转换为颜色
                        color: 'red',
                        weight: 1,
                        opacity: 1,
                        fillOpacity: 0.5
                    });
  
                    // 将标记添加到 lidarLayer
                    marker.addTo(this.lidarLayer);
                });
  
                // 启用 lidarLayer（如果还没有添加到地图上）
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
        }
  
      },
      initializeMap() {
        // console.log("初始化地图")
        this.map = L.map(this.$refs.mapContainer, {
          center: [0, 0], // Adjust based on your map coordinates
          zoom: 0,
          crs: L.CRS.Simple, // Simple CRS for custom maps
        });
  
          // Add an empty layer for nodes
          this.lidarLayer = L.layerGroup().addTo(this.map);
      },
      // 绘制地图到leaflet上
      async fetchMapImage() {
        try {
          const response = await axios.get(`http://${this.publicIP}:5000/Map/GetMapImage`, { responseType: 'blob' });
          const imageBlob = response.data;
          const img = new Image();
          const imageURL = URL.createObjectURL(imageBlob);
  
          img.src = imageURL;
          img.onload = () => {
            const originalWidth = img.width;
            const originalHeight = img.height;
  
            // 获取Leaflet容器的大小
            const containerSize = this.map.getSize();
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
            L.imageOverlay(imageURL, bounds).addTo(this.map);
            this.map.fitBounds(bounds);
          };
        } catch (error) {
          console.error('Failed to load map image:', error);
        }
      },
  
  
      async fetchMapData() {
        try {
          const response = await axios.post(`http://${this.publicIP}:5000/Map/GetMapData`);
          this.map_info = {
            ...this.map_info,
            resolution: JSON.parse(response.data.map_json).resolution,
          }
          this.map_info.edges = JSON.parse(response.data.map_edges).map_edges
          const json_node = JSON.parse(response.data.node);
          this.nodes = json_node.node;
        } catch (error) {
          console.error('Failed to load map data:', error);
        }
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
      // 绘制edges
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


    },
  };
  </script>
  <style scoped>
  .container {
    display: flex;
    flex-direction: row;

  }
  .row-button {
  display: flex; /* 行方向排列按钮 */
  justify-content: flex-start; /* 按钮左对齐 */
  width:100%;
}

  .map-container {
    width: 1300px; /*TODO 这里由于设置了固定像素导致无法自适应 */
    height: 800px;
    /* border: 1px solid #ccc; */
    position: relative;
    border-right: 2px solid #ccc;
    margin-right: 10px;
    
  }
  
    .task-container {
    border: 2px solid #ccc; /* 边框样式 */
    border-radius: 8px; /* 圆角 */
    padding: 20px; /* 内边距 */
    background-color: #f9f9f9; /* 背景颜色 */
    margin-left: 20px; /* 可根据需要调整间距 */
    width: 200px; /* 固定宽度，可以根据需要调整 */
  }
  
.control-container {
  width:100%;
  background-color: #f9f9f9; /* 背景颜色 */
  flex-grow: 1; /* 占据剩余宽度 */
  margin-bottom: 10px;
  flex-direction: column;
  align-items: center;
  display:flex;
  /* width: 200px; 不再需要这个固定宽度 */
}

  </style>