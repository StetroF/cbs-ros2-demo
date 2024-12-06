<template>
  <div class="container">
    <!-- Toolbar -->
    <div 
      class="toolbar" 
      style="
        position: absolute;
        top: 75px;
        left: 50%;
        transform: translateX(-50%);
        width: 500px;
        height: 50px;
        background-color: rgba(255, 255, 255, 0.8);
        display: flex;
        align-items: center;
        justify-content: space-around;
        border-radius: 10px;
        box-shadow: 0 2px 5px rgba(0, 0, 0, 0.2);
      "
    >
      <img 
        src="../assets/select.svg"
        alt="点图标" 
        class="tool-icon" 
        title="选择"

        :class="{ active: activeTool === 'select' }" 
        @click="toggleTool('select')" 
      />
      <img 
        src="../assets/point.svg"
        alt="点图标" 
        title="添加点"
        class="tool-icon" 
        :class="{ active: activeTool === 'node' }" 
        @click="toggleTool('node')" 
      />

      <img 
        src="../assets/line.svg"
        alt="线图标" 
        title="添加边"
        class="tool-icon" 
        :class="{ active: activeTool === 'edge' }" 
        @click="toggleTool('edge')" 
      />
      <img 
        src="../assets/drag.svg" 
        alt="线图标" 
        title="拖拽"
        class="tool-icon" 
        :class="{ active: activeTool === 'drag' }" 
        @click="toggleTool('drag')" 
      />

      <img 
        src="../assets/charger.svg" 
        alt="线图标" 
        title="拖拽"
        class="tool-icon" 
        :class="{ active: activeTool === 'element' }" 
        @click="toggleTool('element')" 
      />
    </div>

    <!-- Map Container -->
    <div ref="mapContainer" class="map-container"></div>

    <!-- Task Section -->
    <div style="flex-direction: column;">
      <div class="task-container" style="height: 350px;width:350px;margin-top: 60px;">
        <h2>地图选择</h2>
        <br />

        <label for="mapSelect">选择地图:</label>
        <select id="mapSelect" v-model="selectedMap" @change="onMapSelected()">
          <option v-for="map in mapList" :key="map" :value="map">
            {{ map }}
          </option>
        </select>
        <br /><br />

          <el-button type="primary" @click="saveMap()"style="margin-left:0px">保存地图</el-button>
          <el-button color="#626aef" type='success' @click="onSwitchUsingMap()" style="margin-left: 85px;">切换为当前使用地图</el-button>
        <!-- 元素添加区，用于添加特殊点如充点电，储位点，操作点 -->
        <div v-if='showElement' style='margin-top: 15px;'>

          <h2>元素添加区</h2>
          <div class='row-display' style="display: flex;margin-right:30px">
            <img src='../assets/charger.svg' alt="充电电" title = "充电点" style="width: 50px;height: 50px;margin-top: 10px;margin-left: 10px;"
            :class="{ active: activeElement === 'charger' }" @click="onSelectElement('charger')" />
            <img src="../assets/delivery.svg" alt='储位点' title='储位点' style="width: 50px;height: 50px;margin-top: 10px;margin-left: 10px;"
            :class="{ active: activeElement === 'delivery' }" @click="onSelectElement('delivery')" />
            <img src="../assets/gripper.svg" alt='操作点' title='操作点' style="width: 50px;height: 50px;margin-top: 10px;margin-left: 10px;"
            :class="{ active: activeElement === 'gripper' }" @click="onSelectElement('gripper')" />
          </div>
        </div>

      </div>
      <!-- 曲线编辑栏 -->
      <div class="task-container" style="height: 485px;width:350px; margin-top: 30px" v-if='showSelectEdge'>
        <h2>参数设置</h2>
        <!-- <div v-if = "showSelectEdge" style="display: flex;flex-direction: column;"> -->
          <div class="select-option">
            <div class="row-display">
              <label>拓扑配型:</label>
              <el-select style="width:100px;" v-model="selectedEdgeConfig.topoType" placeholder="请选择拓扑类型">
                <el-option label="软边" :value="1"></el-option>
                <el-option label="硬边" :value="2"></el-option>

              </el-select>
            </div>
            <div class="row-display">
              <label style="margin-right: 30px;">宽度  :</label>
              <el-input style='width:75px;' v-model="selectedEdgeConfig.width" placeholder="宽度"></el-input>
            </div>

            <div class="row-display">
              <label style="margin-right: 30px;">行驶速度  :</label>
              <el-input style='width:75px;' v-model="selectedEdgeConfig.velocity" placeholder="速度"></el-input>
            </div>
            
            
            <div class='row-display'>
              <label>曲线类型： </label>
              <el-select style ="width:200px" v-model="selectedEdgeConfig.edgeType" placeholder="直线" @change="onEdgeTypeChange()">
                <el-option label="直线" value="line"></el-option>
                <el-option label="贝塞尔曲线(开发中)" value="curve"></el-option>
                <el-option label="B样条曲线" value="bcurve"></el-option>

              </el-select>
            </div>

            <div class='row-display'>
              <label>正向导航模式： </label>
              <el-select style ="width:200px" v-model="selectedEdgeConfig.navigationMode" placeholder="正向导航模式">
                <el-option label="ODOM" :value="1"></el-option>
                <el-option label="SLAM" :value="2"></el-option>
                <el-option label="激光特征" :value="3"></el-option>
                <el-option label="室外" :value="5"></el-option>
                <el-option label="反光板" :value="6"></el-option>
                <el-option label="栈板导航" :value="7"></el-option>
              </el-select>
            </div>

            <div class='row-display'>
              <label>避障模式 </label>
              <el-select style ="width:200px" v-model="selectedEdgeConfig.avoidanceMode" placeholder="避障模式">
                <el-option label="停障" :value="0"></el-option>
                <el-option label="绕障" :value="1"></el-option>
                <el-option label="特征对桩" :value="2"></el-option>
                <el-option label="里程计出桩" :value="3"></el-option>
              </el-select>
            </div>

            <div v-if = 'selectedEdgeConfig.edgeType === "bcurve"'>
              <div class='row-display'>
                <label>控制点1角度:</label>
                <el-input  style='width:120px;margin-right: 30px;' v-model.number="visualAngle.angle1" placeholder="控制点角度1"></el-input>
                <label>(Enter修改)</label>
              </div>
              <div class='row-display'>
                <label>控制点2角度:</label>
                <el-input  style='width:120px;margin-right: 30px;' v-model.number="visualAngle.angle2" placeholder="控制点角度2"></el-input>
                <label>(-180~180)</label>
              </div>

            </div>

            <div class="row-display">
                <el-button color="#626aef" @click="onSaveEdge()" >确定</el-button>
                <!-- <el-button  @click="onCancelSaveEdge()">取消</el-button> -->
              </div>
          </div>



      </div>

      <!-- Node编辑栏 -->
      <div class ='task-container'v-if='showSelectNode' style="display: flex; flex-direction: column;width:350px" @keyup.enter="onSaveNode()">
          
          <div class="select-option">
            <div class="row-display" style="margin-top: 10px;">
              <label>x值:</label>
              <el-input style='width:150px;' v-model="displayNode.x" placeholder="x"></el-input>
            </div>
            <div class="row-display">
              <label>y值:</label>
              <el-input style='width:150px;' v-model="displayNode.y" placeholder="y"></el-input>
            </div>
            <div class="row-display" style="margin-top: 60px; margin-left: 10px;">
              <el-button color="#626aef" @click="onSaveNode()">确定</el-button>
              <el-button @click="onCancelSaveNode()">取消</el-button>
              <el-button type="danger" @click="onSaveNode(true)" style="margin-left: 50px;">删除节点</el-button>
            </div>
            <div class="row-display" style="margin-top: 10px;display: flex;">
              <label>确定: Enter</label>
            </div>
            <div class="row-display" style="display: flex;">
              <label sytle="margin-left: 10px;">删除: Delete</label>
            </div>
          </div>

        </div>
    </div>
  </div>
</template>
  
  <script>
  import axios from 'axios';
  import { useStore } from 'vuex';
  import { computed, normalizeStyle, resolveComponent, toHandlers } from 'vue';
  import 'leaflet/dist/leaflet.css'
  import L, { control } from 'leaflet'
  import { Subject } from 'rxjs';
  import 'leaflet-rotatedmarker'
  import '@elfalem/leaflet-curve';
  import Swal from 'sweetalert2';
  import { v4 as uuidv4 } from 'uuid';
  import BSpline from 'b-spline';
  import rotateIconUrl from '@/assets/rotate.svg';
  import rotateIconUrl2 from '@/assets/rotate_2.svg';
  import arrowIconUrl from '@/assets/arrow.svg';
  import calculateCubicBezier from './api'
  import chargerImg from '@/assets/battery-empty-svgrepo-com.svg'
  import gripperImg from '@/assets/gripper.svg'
  import deliveryImg from '@/assets/delivery.svg'

  export default {

    data() {
      return {
        activeElement: 'charger',
        showElement: false,
        publicIP: null,
        nodes: [],
        bCurbePointSubject:null,
        map_info:{
          resolution: 0.05,
          width: null,
          height: null,
          scale: 1.0, //相对于canvas的缩放比
          edges:[],
        },
        map: null,
        nodeLayer: null,//显示各个node的图层
        edgeLayer: null,//显示各个edge的图层
        closetNode: null, // 用于存储最近的节点
        tooltipLayer:null,
        taskNodeLayer:null, //存储选中的task起点和终点
        dottledLayer:null,//虚线层
        selectedEdgeLayer:null, //用于存储地图编辑时显示的边
        controlPointLayer:null,
        elementLayer:null,

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
        // 存储显示“起始点”，“终点”的文本
        taskPointTip:{
          startTip:null,
          endTip:null
        },
        taskType: 2, // 绑定的任务类型
        elementInfo:null,
        instructionPoint:[],
        mapList: [],      // 地图列表
        selectedMap: '',   // 当前选中的地图
        imageOverlay: null,  // 用于存储当前图层
        activeTool: null, // 当前激活的工具（'node' 或 'edge'）
        selectedEdge:null,
        showSelectEdge:false,
        initialZoom: null,//初始地图缩放比
        initialDownLeft:null,
        selectedEdgeConfig:{
          edgeId: "",
          allowBackward: false,
          requireApply: false,
          topoType: 1,
          edgeType: "line",
          width: 2.0,
          
          
          dockingType:0,
          avoidanceMode: 1,
          safetyZoneType: 0,
          navigationMode: 2,
          velocity: 1.2,
          
          startPoint:null,
          endPoint:null,
          elementId: "",
          controlPoints:null,
          bCurvePoints: {
            startPoint: null,
            controlPoint1:null,
            controlPoint2:null,
            controlPoint3:{
              k:0.0
            },
            controlPoint4:{
              k:0.0
            },
            endPoint:null
          },
        },
        showSelectNode: false,
        displayNode: {
          x:null,
          y:null
        
        }, // 用于显示选中节点的属性设置
        visualAngle:
        {
          angle1:0.0,
          angle2:0.0
        },//用于显示两个控制点的角度
        bCurbePointSubject: new Subject(),
        nodeSubject: new Subject(),
        elementSubject:null,
      }
    },

    mounted(){
      this.initializeMap()
      this.fetchMapData().then(() => 
      {
        this.fetchMapImage()

        this.bCurbePointSubject = new Subject()
        this.elementSubject = new Subject()
        this.nodeSubject = new Subject()


        this.subscribeElement()



        //! node拖拽事件，与工具栏的drag相关
        this.nodeSubject.subscribe((msg) => {
          // console.log('nodeSubject:',msg)
          this.nodesLayer.eachLayer((node) => {
            if (node.point.node_id === msg.node_id )
            {
              console.log('找到对应node:',node)
              const relativeEdges = node.relativeEdges
              if (relativeEdges)
              {
                console.log('找到对应edge:',relativeEdges)
                relativeEdges.forEach((edgeId)=>
                {
                  // console.log('edgeId:',edgeId)
                    
                  this.edgeLayer.eachLayer((edge) => 
                  {
                    // console.log('edge:',edge.edgeId)
                    if (edge.edgeId === edgeId)
                    {
                      if (Math.abs(edge.startPoint.x - node.point.x) < 15 && Math.abs(edge.startPoint.y - node.point.y) < 15)
                      {//1.重新划线
                        edge.startPoint = {x:msg.x,y:msg.y}

                        this.redrawEdge(edge,[msg.y,msg.x],[edge.endPoint.y,edge.endPoint.x])
                        this.map_info.edges.forEach((edgeInfo) => 
                        {//2更新对应json数据
                          if (edgeInfo.edgeId === edge.edgeId)
                          {
                            // console.log('更新edgeInfo:',edgeInfo)
                            const transformHostPoint = this.transformPointReverse({x:msg.x,y:msg.y})
                            // console.log('transformHostPoint:',transformHostPoint)
                            edgeInfo.startPoint.x = transformHostPoint.x
                            edgeInfo.startPoint.y = transformHostPoint.y
                          }
                        })
                      }
                      else if (Math.abs(edge.endPoint.x - node.point.x) < 15 && Math.abs(edge.endPoint.y - node.point.y) < 15)
                      {//1.重新划线
                        edge.endPoint = {x:msg.x,y:msg.y}
                        this.redrawEdge(edge,[edge.startPoint.y,edge.startPoint.x],[msg.y,msg.x])
                        this.map_info.edges.forEach((edgeInfo) => 
                        {//2.更新对应json数据
                          if (edgeInfo.edgeId === edge.edgeId)
                          {
                            console.log('更新edgeInfo:',edgeInfo)
                            const transformHostPoint = this.transformPointReverse({x:msg.x,y:msg.y})
                            // console.log('transformHostPoint:',transformHostPoint)
                            edgeInfo.endPoint.x = transformHostPoint.x
                            edgeInfo.endPoint.y = transformHostPoint.y
                          }
                        })
                      }
                      if(msg.delete){
                        this.edgeLayer.removeLayer(edge)
                        this.map_info.edges = this.map_info.edges.filter((edgeInfo) => edgeInfo.edgeId!== edge.edgeId)
                      }
                    }
                  })
                })

              }
              //更新储位点，充点电提醒栏坐标
              if (node.tooltip)
              {
                node.tooltip.setLatLng([msg.y, msg.x])
              }
              node.point.x = msg.x
              node.point.y = msg.y
              //更新现实的任务点坐标
              node.setLatLng([msg.y, msg.x]);       // 更新 circleMarker 的位置
              if (node.dragMarker) {node.dragMarker.setLatLng([msg.y, msg.x]);} // 更新 dragMarker 的位置
              console.log('更新node:',node.point)
              ///更新源数据，用于保存到json中
              const nodeInfo = this.nodes.find((nodeInfo)=> nodeInfo.node_id === msg.node_id)
              if (nodeInfo)
              {
                const {x,y} = this.transformPointReverse({x:msg.x,y:msg.y})
                Object.assign(nodeInfo,{x,y})
              }

              if(msg.delete)
              {
                this.nodesLayer.removeLayer(node)
                this.nodes = this.nodes.filter((nodeInfo) => nodeInfo.node_id !== node.point.node_id)
              }
            }
        })
        })



        //controlPoint1的平移会影响controlPoint2的位置，同理，controlPoint2的旋转也会影响controlPoint1的位置，使他们始终在一个圆的直径的两个端点上
        this.bCurbePointSubject.subscribe((msg)=>{
          // console.log('订阅更改数据:',msg)
          if(this.selectedEdge.seletableObj){this.edgeLayer.removeLayer(this.selectedEdge.seletableObj)}
          this.edgeLayer.removeLayer(this.selectedEdge)

          if (msg.controlPoint1){
            // 如果是控制点1位置更新，则同步更新控制点3
            const distance = Math.sqrt((msg.controlPoint1.x - this.bCurvePoints.startPoint.x)**2 + (msg.controlPoint1.y - this.bCurvePoints.startPoint.y)**2)
            this.bCurvePoints.controlPoint1= {
              ...this.bCurvePoints.controlPoint1,
              x: msg.controlPoint1.x,
              y: msg.controlPoint1.y,
              distance: distance
            }
            this.bCurvePoints.controlPoint3.distance = distance
            const controlPoint3 = this.calculateControlPoint(this.bCurvePoints.startPoint.x, this.bCurvePoints.startPoint.y, [this.bCurvePoints.controlPoint1.x, this.bCurvePoints.controlPoint1.y]);
            this.bCurvePoints.controlPoint3 ={
              ...this.bCurvePoints.controlPoint3,
              x: controlPoint3[0],
              y: controlPoint3[1],
            }
            this.bCurvePoints.controlPoint3.marker.setLatLng([controlPoint3[1], controlPoint3[0]])
            // console.log('更新控制点3: ',controlPoint3)
            console.log('更新控制点1距离: ',this.bCurvePoints.controlPoint1.distance)
            console.log('此时控制点2距离:',this.bCurvePoints.controlPoint2.distance)
          }

          else if (msg.controlPoint2){
            // 如果是控制点2位置更新，则同步更新控制点4
            const distance = Math.sqrt((msg.controlPoint2.x - this.bCurvePoints.endPoint.x)**2 + (msg.controlPoint2.y - this.bCurvePoints.endPoint.y)**2)
            this.bCurvePoints.controlPoint2 = {
              ...this.bCurvePoints.controlPoint2,
              x: msg.controlPoint2.x,
              y: msg.controlPoint2.y,
              distance: distance
            }
            // this.controlPoint1 = {x: this.bCurvePoints.controlPoint1.x, y: this.bCurvePoints.controlPoint1.y}
            const controlPoint4 = this.calculateControlPoint(this.bCurvePoints.endPoint.x, this.bCurvePoints.endPoint.y, [this.bCurvePoints.controlPoint2.x, this.bCurvePoints.controlPoint2.y]);
            this.bCurvePoints.controlPoint4 = {
              ...this.bCurvePoints.controlPoint4,
              x: controlPoint4[0],
              y: controlPoint4[1]
            }
            this.bCurvePoints.controlPoint4.marker.setLatLng([controlPoint4[1], controlPoint4[0]])
            // console.log('更新控制点4: ',controlPoint4)
            console.log('更新控制点2距离: ',this.bCurvePoints.controlPoint2.distance)
            console.log('此时控制点1距离:',this.bCurvePoints.controlPoint1.distance)
          }
          else if (msg.controlPoint3){

            const k = ((msg.controlPoint3.y - this.bCurvePoints.startPoint.y) / (msg.controlPoint3.x - this.bCurvePoints.startPoint.x))
            const b = (msg.controlPoint3.y - k * msg.controlPoint3.x)
            this.bCurvePoints.controlPoint3 ={
              ...this.bCurvePoints.controlPoint3,
              x: msg.controlPoint3.x,
              y: msg.controlPoint3.y,
              k: k,
              b: b
            }
            const deltaY = this.bCurvePoints.startPoint.y-msg.controlPoint3.y 
            const deltaX = this.bCurvePoints.startPoint.x-msg.controlPoint3.x

            // 使用 Math.atan2 来计算角度
            const angle = Math.atan2(deltaY, deltaX) * 180 / Math.PI;
            this.visualAngle.angle1 = parseFloat(angle.toFixed(4));
            // this.$set(this.bCurvePoints.controlPoint3, 'k', k);
            this.bCurvePoints.controlPoint3.marker.setLatLng([msg.controlPoint3.y, msg.controlPoint3.x])
            const controlPoint1 = this.calculateControlPoint(this.bCurvePoints.startPoint.x, this.bCurvePoints.startPoint.y, [this.bCurvePoints.controlPoint3.x, this.bCurvePoints.controlPoint3.y]);
            console.log('计算后的控制点1:',controlPoint1)
            this.bCurvePoints.controlPoint1 ={
              ...this.bCurvePoints.controlPoint1,
              x: controlPoint1[0],
              y: controlPoint1[1]
            }
            // this.controlPoint1 = {x: controlPoint1[0], y: controlPoint1[1]}
            this.bCurvePoints.controlPoint1.marker.setLatLng([controlPoint1[1], controlPoint1[0]])
            
          }
          else if (msg.controlPoint4){
            const k = ((msg.controlPoint4.y - this.bCurvePoints.endPoint.y) / (msg.controlPoint4.x - this.bCurvePoints.endPoint.x))
            const b = (msg.controlPoint4.y - k * msg.controlPoint4.x)
            this.bCurvePoints.controlPoint4 ={
              ...this.bCurvePoints.controlPoint4,
              x: msg.controlPoint4.x,
              y: msg.controlPoint4.y,
              k: k,
              b: b
            }
            const deltaY = this.bCurvePoints.endPoint.y-msg.controlPoint4.y 
            const deltaX = this.bCurvePoints.endPoint.x-msg.controlPoint4.x
            const angle = Math.atan2(deltaY, deltaX) * 180 / Math.PI; // 计算角度，范围在 -180 到 180 之间
            this.visualAngle.angle2 = parseFloat(angle.toFixed(4));


            const controlPoint2 = this.calculateControlPoint(this.bCurvePoints.endPoint.x, this.bCurvePoints.endPoint.y, [this.bCurvePoints.controlPoint4.x, this.bCurvePoints.controlPoint4.y]);
            this.bCurvePoints.controlPoint2 ={
              ...this.bCurvePoints.controlPoint2,
              x: controlPoint2[0],
              y: controlPoint2[1]
            }
            // this.controlPoint2 = {x: controlPoint2[0], y: controlPoint2[1]}
            this.bCurvePoints.controlPoint2.marker.setLatLng([controlPoint2[1], controlPoint2[0]])
          }
          this.$nextTick(() => {
            // 在视图更新后执行
            this.$forceUpdate(); // 或者在这里强制更新视图
          });
          this.selectedEdgeConfig.controlPoints = [
            this.transformPointReverse(this.bCurvePoints.controlPoint1),
            this.transformPointReverse(this.bCurvePoints.controlPoint2),
            ]

          const bcurve =this.drawBCurve([this.bCurvePoints.controlPoint3,this.bCurvePoints.startPoint,this.bCurvePoints.controlPoint1,this.bCurvePoints.controlPoint2,this.bCurvePoints.endPoint,this.bCurvePoints.controlPoint4])
          // this.selectedEdge = bcurve
          this.selectedEdgeLayer.clearLayers()
          this.selectedEdgeLayer.addLayer(bcurve)
          this.drawDottledLine()
        

        })

      })

    },
    created() {
      const store = useStore();
      const ip = computed(() => store.state.publicIP);
      this.publicIP = ip;
      this.getMapList()

      this.changeControlPointAngle = this.changeControlPointAngle.bind(this);
      window.addEventListener('keydown', this.changeControlPointAngle)
      
    },
    beforeUnmount() {
        }, 
    methods: {
      subscribeElement(){
        this.elementSubject.subscribe((msg) => 
        {
          
          // console.log('elementSubject:',msg)
          let newAngle = null
          let newPoint = null
          if (msg.type === 'rotate')
          {

            this.elementIcon.angle = Math.atan2(msg.point[1]-this.elementIcon.point[1],msg.point[0]-this.elementIcon.point[0])

            // console.log('angle:',this.elementIcon.angle* 180 / Math.PI)
            this.elementIcon.setRotationAngle(-this.elementIcon.angle* 180 / Math.PI + 90)
            this.elementLayer.removeLayer(this.linetoCircle)
            this.linetoCircle.setLatLngs([[msg.point[1],msg.point[0]],[this.elementIcon.point[1],this.elementIcon.point[0]]])
              // [[msg.point[1],msg.point[0]],[this.elementIcon.point[1],this.elementIcon.point[0]]],

            this.linetoCircle.point = msg.point
            this.elementIcon.circularMarker.point = msg.point
            this.elementLayer.addLayer(this.linetoCircle)

            newAngle = (this.elementIcon.angle + Math.PI) % (2 * Math.PI)
            newAngle = newAngle> (Math.PI)? newAngle -(Math.PI*2) : newAngle

          }
          else if (msg.type === 'move')
          {
            const offsetX = msg.point[0] - this.elementIcon.point[0]
            const offsetY = msg.point[1] - this.elementIcon.point[1]
            this.elementIcon.point = [msg.point[0],msg.point[1]]
            console.log('move:',this.elementIcon.point)

            this.elementIcon.marker.setLatLng([msg.point[1],msg.point[0]])//重新绘制特殊点的node的位置
            Object.assign(this.elementIcon.marker.point,{x:msg.point[0],y:msg.point[1]})//更新特殊点的node的位置
            this.edgeLayer.removeLayer(this.elementLine)
            


            this.elementLine.setLatLngs([[this.elementIcon.marker.point.y,this.elementIcon.marker.point.x],[this.elementIcon.predock_marker.point.y,this.elementIcon.predock_marker.point.x]])
            //不是直接重新赋值，而是重新绘制
            this.edgeLayer.addLayer(this.elementLine)
            //更新json数据(edge)
            const transformHostPoint = this.transformPointReverse({x:msg.point[0],y:msg.point[1]})
            const EdgeId = this.map_info.edges.find((edgeInfo) => edgeInfo.edgeId === this.elementLine.edgeId)
            Object.assign(EdgeId.startPoint,{x:transformHostPoint.x,y:transformHostPoint.y})
            this.edgeLayer.eachLayer((edge) => {
              if (edge.edgeId === EdgeId.edgeId)
              {
                edge.startPoint = {x:msg.point[0],y:msg.point[1]}
              }})

            //更新json数据(node)
            const NodeId = this.nodes.find((nodeInfo) => nodeInfo.node_id === this.elementIcon.marker.point.node_id)
            console.log("找到node:",NodeId)
            if (NodeId){
              Object.assign(NodeId,{x:transformHostPoint.x,y:transformHostPoint.y})

            }

            //更新旋转图标位置即this.elementIcon.circularMarker
            const newCircieY = this.elementIcon.circularMarker.point[1] + offsetY
            const newCircieX = this.elementIcon.circularMarker.point[0] + offsetX
            this.elementIcon.circularMarker.setLatLng([newCircieY, newCircieX])
            this.elementIcon.circularMarker.point = [newCircieX, newCircieY]

            this.linetoCircle.setLatLngs([[this.elementIcon.point[1],this.elementIcon.point[0]],[newCircieY, newCircieX]])
            
            newPoint = transformHostPoint
          }


          //更新element_info的json
          const elementInfo = this.elementInfo.find((elementInfo) => elementInfo.id === msg.element_id)
          if (elementInfo)
          {
            if(newAngle){
              elementInfo.featureParam.featurePose.angle = newAngle
            }
            if(newPoint){
              elementInfo.featureParam.featureOperationInfo.targetPoint = newPoint
            }

            //更新角度和四个points
            const center = elementInfo.featureParam.featureOperationInfo.targetPoint
              const angle = elementInfo.featureParam.featurePose.angle
              const dist = 0.4 //默认对接的是一个正方形，边长为0.8

              const p1 ={
                x: center.x - dist * (Math.cos(angle) + Math.sin(angle)),
                y: center.y + dist * (Math.cos(angle) - Math.sin(angle))
              }
              const p2 = {
                x: center.x + dist * (Math.cos(angle) - Math.sin(angle)),
                y: center.y + dist * (Math.cos(angle) + Math.sin(angle))
              }
              const p3 = {
                x: center.x + dist * (Math.cos(angle) + Math.sin(angle)),
                y: center.y - dist * (Math.cos(angle) - Math.sin(angle))
              }
              const p4 = {
                x: center.x - dist * (Math.cos(angle) - Math.sin(angle)),
                y: center.y - dist * (Math.cos(angle) + Math.sin(angle))
              }
              const points = [p1,p2,p3,p4]
              elementInfo.featureParam.featurePose.points = points
              console.log('更新后的四个点:',p1.x,p1.y,p2.x,p2.y,p3.x,p3.y,p4.x,p4.y)
              console.log('更新后的目标点:',elementInfo.featureParam.featureOperationInfo.targetPoint.x,elementInfo.featureParam.featureOperationInfo.targetPoint.y)
          }
        })

      },
      onAddElement(){
        console.log('添加元素')

      },
      onSelectElement(element){
        if(!this.activeElement != element){this.activeElement = element} 
        console.log('选中元素:',this.activeElement)
        this.map.on('click', this.handleAddElement)
      },
      async onSwitchUsingMap(){
        const request = {
          map_name: this.selectedMap
        }
        console.log('切换地图请求:',request)
        const requestUrl = `http://${this.publicIP}:5000/Task/switchUsingMap`
        console.log('切换地图请求url:',requestUrl)
        axios.post(requestUrl, request).then((response) =>
         {
          Swal.fire({
            title: '切换成功',
            text: response.data.message,
            icon: response.data.status? 'success' : 'error',
            confirmButtonText: '确定',
            timer: 3000
          })
        })
      },
      async onEdgeTypeChange() {
        this.dottledLayer.clearLayers()
        this.controlPointLayer.clearLayers()
        this.selectedEdgeLayer.clearLayers()
        
        const removedEdge = this.selectedEdge
        console.log('移除原本的边',removedEdge)
        const removedSelectableObj = this.selectedEdge.selectableObj
        if(removedSelectableObj)this.edgeLayer.removeLayer(removedSelectableObj)
        this.edgeLayer.removeLayer(removedEdge)
        const edgeId = this.selectedEdge.edgeId?this.selectedEdge.edgeId:this.generateRandomString()
        const edge = this.selectedEdgeConfig
        if (this.selectedEdgeConfig.edgeType === 'bcurve' && !this.selectedEdgeConfig.controlPoints)
        {
          this.selectedEdgeConfig.controlPoints = []
          this.selectedEdgeConfig.controlPoints.push(
          {
            x:(this.selectedEdgeConfig.startPoint.x +this.selectedEdgeConfig.endPoint.x)/2,
            y:(this.selectedEdgeConfig.startPoint.y +this.selectedEdgeConfig.endPoint.y)/2+1.0
          })
          this.selectedEdgeConfig.controlPoints.push(
            {
            x:(this.selectedEdgeConfig.startPoint.x +this.selectedEdgeConfig.endPoint.x)/2,
            y:(this.selectedEdgeConfig.startPoint.y +this.selectedEdgeConfig.endPoint.y)/2-1.0
          })
            
        }
        const [visLine,selectableObj]=this.addEdge(this.selectedEdgeConfig)
        this.selectedEdgeLayer.addLayer(visLine)
        this.selectedEdgeLayer.addLayer(selectableObj)
        console.log('当前曲线类型：{}',this.selectedEdgeConfig)

        if(this.selectedEdgeConfig.edgeType === 'bcurve'){this.drawDraggableAndMovableIcon(this.selectedEdgeConfig.startPoint, this.selectedEdgeConfig.endPoint);}
    },
    //绘制可以旋转的控制icon和可以平移的控制icon，分别绑定控制点34(负责旋转)和12(负责平移)
    drawDraggableAndMovableIcon(startPoint,endPoint,ctrlP1,ctrlP2){
        this.dottledLayer.clearLayers()
        this.controlPointLayer.clearLayers()

        
        // 计算控制点
        const [transStartX, transStartY] = this.transformPoint(startPoint);
        const [transEndX, transEndY] = this.transformPoint(endPoint);


        // 计算控制点1,2的位置 ，他们只允许在虚线上,这里的问号表示，当你有传入控制点,那么就用
        // 传入的控制点显示b样条曲线(用于编辑已经生成了b样条)
        // 如果没有传入(即第一次绘制b样条时，那么就会自动计算控制点)
        const controlPoint1 = ctrlP1?ctrlP1:[(transStartX + transEndX) / 2, (transStartY + transEndY) / 2 + 20];
        const controlPoint2 = ctrlP2?ctrlP2:[(transStartX + transEndX) / 2, (transStartY + transEndY) / 2 - 20];

        this.selectedEdgeConfig.controlPoints = [
          this.transformPointReverse({x:controlPoint1[0],y:controlPoint1[1]}),
          this.transformPointReverse({x:controlPoint2[0],y:controlPoint2[1]}),
        ]

        // 计算控制点3和控制点4
        const controlPoint3 = this.calculateControlPoint(transStartX, transStartY, controlPoint1);
        const controlPoint4 = this.calculateControlPoint(transEndX, transEndY, controlPoint2);

        // 更新 selectedEdgeConfig 的控制点
        // this.selectedEdgeConfig.controlPoints = [controlPoint1, controlPoint2, controlPoint3, controlPoint4];

        // 绘制从控制点到起点和终点的虚线
        const lineToStart1 = L.polyline([[transStartY, transStartX], [controlPoint1[1], controlPoint1[0]]], {
            color: 'black',
            weight: 2,
            opacity: 0.4,
            dashArray: '5, 5'
        })
        this.dottledLayer.addLayer(lineToStart1)

        const lineToStart3 = L.polyline([[transStartY, transStartX], [controlPoint3[1], controlPoint3[0]]], {
            color: 'black',
            weight: 2,
            opacity: 0.4,
            dashArray: '5, 5'
        })
        this.dottledLayer.addLayer(lineToStart3)
        const lineToEnd2 = L.polyline([[transEndY, transEndX], [controlPoint2[1], controlPoint2[0]]], {
            color: 'black',
            weight: 2,
            opacity: 0.4,
            dashArray: '5, 5'
        })
        this.dottledLayer.addLayer(lineToEnd2)
        const lineToEnd4 = L.polyline([[transEndY, transEndX], [controlPoint4[1], controlPoint4[0]]], {
            color: 'black',
            weight: 2,
            opacity: 0.4,
            dashArray: '5, 5'
        })  
        this.dottledLayer.addLayer(lineToEnd4)
        
        //保存斜率，距离，用于实时更新，保证你在平移后再旋转贝塞尔曲线时，是根据最新的斜率和距离计算
        const k1 = parseFloat(((controlPoint1[1] - transStartY) / (controlPoint1[0] - transStartX)).toFixed(4));
        const b1 = parseFloat((controlPoint1[1] - k1 * controlPoint1[0]).toFixed(4));
        const k2 = parseFloat(((controlPoint2[1] - transEndY) / (controlPoint2[0] - transEndX)).toFixed(4));
        const b2 = parseFloat((controlPoint2[1] - k2 * controlPoint2[0]).toFixed(4));
        const distance = Math.sqrt((transEndX - controlPoint2[0]) ** 2 + (transEndY - controlPoint2[1]) ** 2);

        const angle1 = Math.atan2(controlPoint1[1] - transStartY, controlPoint1[0] - transStartX) * 180 / Math.PI;
        const angle2 = Math.atan2(controlPoint2[1] - transEndY, controlPoint2[0] - transEndX) * 180 / Math.PI;
        //this.visualAngle表示前端显示控制点的角度，绑定的Enter键
        this.visualAngle = {
            angle1: parseFloat(angle1.toFixed(4)),
            angle2: parseFloat(angle2.toFixed(4)),
        };

        this.bCurvePoints = {
          startPoint: {
            x: transStartX,
            y: transStartY,
          },
          controlPoint1: {
            x: controlPoint1[0],
            y: controlPoint1[1],
            distance: distance,
            marker: null
          },
          
          controlPoint2: {
            x: controlPoint2[0],
            y: controlPoint2[1],
            distance: distance,
            marker: null
          },
          controlPoint3: {
            x: controlPoint3[0],
            y: controlPoint3[1],
            k: k1,
            b: b1,
          },
          
          controlPoint4: {
            x: controlPoint4[0],
            y: controlPoint4[1],
            k: k2,
            b: b2,
          },
          endPoint: {
            x: transEndX,
            y: transEndY,
          }
        }
        const rotateIcon = L.icon({
            iconUrl: rotateIconUrl,
            iconSize: [32, 32],                // 图标的大小（宽度, 高度）
            iconAnchor: [16, 32],              // 图标的锚点位置，通常是图标的底部中心
            popupAnchor: [0, -32]              // 弹窗的锚点位置，相对于图标的位置
        });
        const arrowIcon = L.icon({
            iconUrl: arrowIconUrl,
            iconSize: [36, 20],                // 图标的大小（宽度, 高度）
            iconAnchor: [16, 32],              // 图标的锚点位置，通常是图标的底部中心
            popupAnchor: [0, -32]              // 弹窗的锚点位置，相对于图标的位置
        });
        this.bCurvePoints.controlPoint1.marker = L.marker([controlPoint1[1], controlPoint1[0]], {
            icon: arrowIcon,
            draggable: true,
        })
        this.controlPointLayer.addLayer(this.bCurvePoints.controlPoint1.marker)
        this.bCurvePoints.controlPoint2.marker = L.marker([controlPoint2[1], controlPoint2[0]], {
            icon: arrowIcon,
            draggable: true,
        })
        this.controlPointLayer.addLayer(this.bCurvePoints.controlPoint2.marker)
        this.bCurvePoints.controlPoint3.marker = L.marker([controlPoint3[1], controlPoint3[0]], {
            draggable: true,
            icon: rotateIcon
        })  
        this.controlPointLayer.addLayer(this.bCurvePoints.controlPoint3.marker)
        this.bCurvePoints.controlPoint4.marker = L.marker([controlPoint4[1], controlPoint4[0]], {
            draggable: true,
            icon: rotateIcon
        })
        this.controlPointLayer.addLayer(this.bCurvePoints.controlPoint4.marker)

        

        this.bCurvePoints.controlPoint1.marker.on('drag', (event) => {
            const currentMarker = event.target;
            const [newX,newY] = this.keepOnLine(currentMarker,this.bCurvePoints.controlPoint3.k,this.bCurvePoints.controlPoint3.b);
            currentMarker.setLatLng([newY, newX]);  // 更新位置
            this.bCurbePointSubject.next({controlPoint1:{x:newX,y:newY}})
        });
        this.bCurvePoints.controlPoint2.marker.on('drag', (event) => {
            const currentMarker = event.target;
            const [newX,newY] = this.keepOnLine(currentMarker,this.bCurvePoints.controlPoint4.k,this.bCurvePoints.controlPoint4.b);
            currentMarker.setLatLng([newY, newX]);  // 更新位置
            this.bCurbePointSubject.next({controlPoint2:{x:newX,y:newY}})
        });
        this.bCurvePoints.controlPoint3.marker.on('drag', (event) => {
            const currentMarker = event.target;
            const [newX,newY] = this.circularMotion(currentMarker, [transStartX,transStartY], this.bCurvePoints.controlPoint1.distance);
            currentMarker.setLatLng([newY, newX]);  // 更新位置
            this.bCurbePointSubject.next({controlPoint3:{x:newX,y:newY}})
        });
        this.bCurvePoints.controlPoint4.marker.on('drag', (event) => {
            const currentMarker = event.target;
            const [newX,newY] = this.circularMotion(currentMarker, [transEndX,transEndY], this.bCurvePoints.controlPoint2.distance);
            currentMarker.setLatLng([newY, newX]);  // 更新位置
            this.bCurbePointSubject.next({controlPoint4:{x:newX,y:newY}})
        });

    },
    // 绘制从起点到终点的虚线
    drawDottledLine(){
      this.dottledLayer.clearLayers()
        const lineToStart1 = L.polyline([[this.bCurvePoints.startPoint.y, this.bCurvePoints.startPoint.x], [this.bCurvePoints.controlPoint1.y, this.bCurvePoints.controlPoint1.x]], {
            color: 'black',
            weight: 2,
            opacity: 0.4,
            dashArray: '5, 5'
        })
        this.dottledLayer.addLayer(lineToStart1)

        const lineToStart3 = L.polyline([[this.bCurvePoints.startPoint.y, this.bCurvePoints.startPoint.x], [this.bCurvePoints.controlPoint3.y, this.bCurvePoints.controlPoint3.x]], {
            color: 'black',
            weight: 2,
            opacity: 0.4,
            dashArray: '5, 5'
        })
        this.dottledLayer.addLayer(lineToStart3)
        const lineToEnd2 = L.polyline([[this.bCurvePoints.endPoint.y, this.bCurvePoints.endPoint.x], [this.bCurvePoints.controlPoint2.y, this.bCurvePoints.controlPoint2.x]], {
            color: 'black',
            weight: 2,
            opacity: 0.4,
            dashArray: '5, 5'
        })
        this.dottledLayer.addLayer(lineToEnd2)
        const lineToEnd4 = L.polyline([[this.bCurvePoints.endPoint.y, this.bCurvePoints.endPoint.x], [this.bCurvePoints.controlPoint4.y, this.bCurvePoints.controlPoint4.x]], {
            color: 'black',
            weight: 2,
            opacity: 0.4,
            dashArray: '5, 5'
        })  
        this.dottledLayer.addLayer(lineToEnd4)

    },
    keepOnLine(marker,k1,b1) {
        ////保证marker一定只能在这条给定直线上移动
        const markerLatLng = marker.getLatLng();
        const point = this.map.latLngToContainerPoint(markerLatLng);
        // 获取 mapLayer 的边界
        const bounds = this.imageOverlay.getBounds();
        const downleft = this.map.latLngToContainerPoint(bounds.getSouthWest());

        // 计算相对于 mapLayer 的坐标
        let relativeX = point.x - downleft.x;
        let relativeY = downleft.y - point.y;


        const currentZoom = this.map.getZoom();
        relativeX = relativeX / (Math.pow(2, currentZoom))
        relativeY = relativeY / (Math.pow(2, currentZoom))
        // console.log('缩放比例：', currentZoom);
        // const relativeX = (point.x - downleft.x) / scaleFactor + this.initialDownLeft.x;
        // const relativeY = (downleft.y - point.y) / scaleFactor + this.initialDownLeft.y;
        // 计算投影点,将marker投影到原来的直线上
        const distance = (k1 * relativeX - relativeY + b1) / Math.sqrt(k1 * k1 + 1);

        const xP = relativeX - distance * (k1 / Math.sqrt(k1 * k1 + 1));
        const yP = relativeY + distance * (1 / Math.sqrt(k1 * k1 + 1));

        // console.log('直线k1,b1: ', [k1, b1], ' 当前位置： ', [relativeX, relativeY], ' 投影位置： ', [xP, yP]);

        return [xP, yP];
    },
    circularMotion(marker, anchorPoint, fixedDistance) {
      ////保证marker在圆周上运动
        const newPos = marker.getLatLng();
        const angle = Math.atan2(newPos.lat - anchorPoint[1], newPos.lng - anchorPoint[0]);

        // 计算新的坐标，保持与anchorPoint的固定距离
        const newPointX = anchorPoint[0] + fixedDistance * Math.cos(angle);
        const newPointY = anchorPoint[1] + fixedDistance * Math.sin(angle);

        // 更新marker位置
        // marker.setLatLng([newPointY, newPointX]);
        // console.log('重置位置:',newPointX,newPointY)
        return [newPointX, newPointY];
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
        // console.log('绘制B样条曲线',bCurvePoints)
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
          color: 'black',
          opacity: 0.6,
          weight: 2
        })
        // 添加到图层中
        return bcurve
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


      async saveMap(){
        console.log('保存地图')
        const request = {
          map_name: this.selectedMap,
          edges: this.map_info.edges,
          nodes: this.nodes,
          element_info: this.elementInfo,
        }

        console.log('保存地图请求：',request)
        const response = axios.post(`http://${this.publicIP}:5000/MapEdit/SaveMap`,request)
        .then(response => {
             console.log('保存地图成功:',response.data)
             Swal.fire({
                 title: "",
                 text: response.data.message,
                 icon: response.data.status ? "success" : "error",
                 timer: 1500,
                 showCloseButton: false,
                 position: "top",
                 backdrop: false,
                 heightAuto: true,
             })
         })
        .catch(error => {
             console.error('保存地图失败:', error);
             Swal.fire({
                 title: "",
                 text: "保存地图失败",
                 icon: "error",
                 timer: 1000,
                 showCloseButton: false,
                 position: "top",
                 backdrop: false,
                 heightAuto: true,
             })
         })
        },
      redrawEdge(edge,[startY,startX],[endY,endX]){
        if (edge.edgeType === 'line') {
          console.log('更新坐标:',[startX,startY],[endX,endY])
          edge.setLatLngs([[startY, startX], [endY, endX]])
          edge.selectableObj.setLatLngs([[startY, startX], [endY, endX]])
          // edge.endPoint = {x:endX,y:endY}            
        }
        else if(edge.edgeType === 'bcurve'){
          let [controlPoint1,controlPoint2] = edge.controlPoints
          const controlPoint3 = this.calculateControlPoint(startX, startY, controlPoint1)
          const controlPoint4 = this.calculateControlPoint(endX, endY, controlPoint2)

          const bCurvePoints = [
              {x:controlPoint3[0],y:controlPoint3[1]},
              {x:startX,y:startY},
              {x:controlPoint1[0],y:controlPoint1[1]},
              {x:controlPoint2[0],y:controlPoint2[1]},
              {x:endX,y:endY},
              {x:controlPoint4[0],y:controlPoint4[1]}]
          
          const points = bCurvePoints.map(point => [point.x, point.y]); // [latitude, longitude] 格式
          const degree = 3;
          // 生成样条曲线上的点集
          const splinePoints = [];
          const numPoints = 50; // 设置曲线的细腻度，点越多曲线越平滑
          for (let i = 0; i <= numPoints; i++) {
            const t = i / numPoints;
            const point = BSpline(t, degree, points);
            splinePoints.push(point);
          }

          // 将生成的点转为 Leaflet 所需的格式
          const latlngs = splinePoints.map(point => [point[1], point[0]]); // [latitude, longitude] 格式
          edge.setLatLngs(latlngs)
          edge.selectableObj.setLatLngs(latlngs)
        }
        else if(edge.edgeType === 'curve'){
          const bezierPoints = [[startY, startX],...edge.controlPoints, [endY, endX]]
          const numPoints = 50; // 调整近似的精度
          const polylinePoints = [];
          for (let t = 0; t <= 1; t += 1 / numPoints) {
            const pt = calculateCubicBezier(bezierPoints, t);
            polylinePoints.push(pt);
          }
          edge.setLatLngs(polylinePoints)
        }
        

      },
      // 添加单条edge函数
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
      ///将this.selectedEdgeConfig转换为通用格式
      generate_edge_json_config(){
        const template_edges = {
          edgeId: this.selectedEdgeConfig.edgeId,
          allowBackward : this.selectedEdgeConfig.allowBackward,
          startPoint: {
            node_id: this.selectedEdgeConfig.startPoint.node_id,
            x: this.selectedEdgeConfig.startPoint.x,
            y: this.selectedEdgeConfig.startPoint.y,
          },
          topoType: this.selectedEdgeConfig.topoType,
          edgeType: this.selectedEdgeConfig.edgeType,
          endPoint: {
            node_id: this.selectedEdgeConfig.endPoint.node_id,
            x: this.selectedEdgeConfig.endPoint.x,
            y: this.selectedEdgeConfig.endPoint.y,
          },
          width: this.selectedEdgeConfig.width,
          dockingType: this.selectedEdgeConfig.dockingType,
          avoidanceMode: this.selectedEdgeConfig.avoidanceMode,
          safetyZoneType: this.selectedEdgeConfig.safetyZoneType,
          navigationMode: this.selectedEdgeConfig.navigationMode,
          velocity: this.selectedEdgeConfig.velocity,
          elementId: this.selectedEdgeConfig.elementId,
          requireApply: this.selectedEdgeConfig.requireApply,
        }
        console.log('生成通用边配置：',template_edges)
        if (this.selectedEdgeConfig.edgeType === 'bcurve') {
          template_edges.controlPoints = [
            this.selectedEdgeConfig.controlPoints[0],
            this.selectedEdgeConfig.controlPoints[1],
          
          ]}
        return template_edges
      },
      onCancelSaveEdge(){
        
        this.showSelectEdge = false
        this.dottledLayer.clearLayers() // 清除虚线
        this.controlPointLayer.clearLayers() // 清除控制点图层
        if (this.taskPointTip.endTip){this.taskPointTip.endTip.remove();}
        if(this.taskPointTip.startTip){this.taskPointTip.startTip.remove();}
      },
      onSaveEdge(){
        this.selectedEdgeLayer.clearLayers()
        console.log('当前taskPoint：',this.taskPoint)

        // console.log('保存边缘配置：'+JSON.stringify(this.selectedEdgeConfig))
        // this.map_info.edges = this.map_info.edges.filter((edgeInfo) => edgeInfo.edgeId!== this.selectedEdgeConfig.edgeId)
        
        this.map_info.edges.push(this.generate_edge_json_config())
        console.log('添加Node后:',this.nodes)

        console.log('删除掉edge: ',this.selectedEdge)

        const edge_id = this.map_info.edges.find((edge) => edge.edgeId === this.selectedEdgeConfig.edgeId).edgeId

        if (this.selectedEdge){
          if (this.selectedEdge.selectableObj)
          {
            console.log('删除掉selectableObj: ',this.selectedEdge.selectableObj)

            this.edgeLayer.removeLayer(this.selectedEdge.selectableObj)
          }

          // console.log('找到被替换的edge_id: ',edge_id)
          this.edgeLayer.removeLayer(this.selectedEdge)
        } 


        console.log("找到被替换的edge_id: ",edge_id)
        
        const [visLine,seletableObj]= this.addEdge(this.map_info.edges.find((edge) => edge.edgeId === this.selectedEdgeConfig.edgeId))//只重新画被选中的边
        this.edgeLayer.addLayer(visLine)
        this.edgeLayer.addLayer(seletableObj)

        this.dottledLayer.clearLayers() // 清除虚线
        this.controlPointLayer.clearLayers() // 清除控制点图层

        if (this.taskPointTip.endTip){this.taskPointTip.endTip.remove();}
        if(this.taskPointTip.startTip){this.taskPointTip.startTip.remove();}
        this.showSelectEdge = false//当点击确定，编辑栏隐藏


        //重新触发一次选中
        //原因：当编辑一条曲线之后，这条曲线由于重新绘制导致不在原有的layer中，导致不可再次编辑
        //因此重新触发一次toggletool
        if (this.activeTool === 'select')
        {
          this.toggleTool('null')
          this.toggleTool('select')
        }
        this.selectedEdgeConfig.edgeType = 'line'//重新置为默认的line，下次选中其他线时显示的仍然是line
      },
      generateRandomString() {
        return uuidv4().replace(/\-/g, '').slice(0, 16);
      },
      removeEventListeners() {
        this.map.off('mousemove'); // 取消高亮节点
        this.map.off('click');

        this.map.off('keydown');
        this.nodesLayer.eachLayer((node) => { // 取消节点的拖动
          if(node.dragMarker){
            node.dragMarker.dragging.disable()}       
          })
        this.edgeLayer.eachLayer((edge) => { // 取消边的拖动
          if(edge.selectableObj){
            edge.selectableObj.setStyle({opacity:0.0})
            edge.selectableObj.off('mouseover')
            edge.selectableObj.off('click')
          }
        })
      },
      hightlightNearestEdge(){
        
      },
      toggleTool(tool) {
        // this.map.dragging.enable() // 允许拖动地图
        this.showSelectNode = false
        this.showSelectEdge = false;
        this.showElement = false;
        this.clearTaskNodeLayer() //清除已选中的节点
        this.removeEventListeners() //清除原有绑定的事件监听
        //清除掉绘制线的图层
        this.dottledLayer.clearLayers();
        this.controlPointLayer.clearLayers()
        this.taskPoint.startPoint = null;
        this.taskPoint.endPoint = null;
        

        // 如果点击的是同一个工具，则取消激活
        this.activeTool = this.activeTool === tool ? null : tool;
        if (this.activeTool) {
          this.addEventListenerForActiveTool(this.activeTool);
        }
      },

      addEventListenerForActiveTool(tool) {
        if (tool === 'node') {
          this.map.on('click', this.handleAddNode);
        } 
        else if (tool === 'edge')
        {
          this.map.on('mousemove',(event)=>
          {
            this.highLightMoveNode(event,10)
          })
          this.map.on('click',this.highLightTaskNode)
        }
        else if (tool ==='drag') 
        {
          console.log('激活选中节点')
          this.map.on('mousemove',this.highLightMoveNode)
          // this.map.dragging.disable() 
          this.activateNodeDgag()
        }
        else if (tool === 'select')
        {

          this.map.on('mousemove',(event)=>
          {
            this.highLightMoveNode(event,10)

          })
          
          this.map.on('click',this.onSelectObject)
          this.map.on('keydown', this.handleKeyDown);
          this.handleSelectEdge()

        }
        else if(tool==='element'){
          console.log('激活元素编辑')
          this.showElement = true
          this.activeElement='charger'
          this.map.on('click',this.handleAddElement)
        }
      },
      handleAddElement(event){
        const element_id = this.generateRandomString()


        if (this.elementLayer.getLayers().length > 0) {this.elementLayer.clearLayers();}
        const clickedLatLng = this.map.mouseEventToLatLng(event.originalEvent);
        const point = this.map.latLngToContainerPoint(clickedLatLng);

        // 获取 imageOverlay 的边界
        const bounds = this.imageOverlay.getBounds();
        const downleft = this.map.latLngToContainerPoint(bounds.getSouthWest());

        // 计算相对于 mapLayer 的坐标
        let relativeX = point.x - downleft.x;
        let relativeY = downleft.y - point.y;
        // console.log(`鼠标坐标: (${relativeX}, ${relativeY})`);

        const zoom = this.map.getZoom();
        // console.log(`当前缩放级别: ${zoom}`);
        relativeX = relativeX / (Math.pow(2, zoom));
        relativeY = relativeY / (Math.pow(2, zoom));



        ///!!! 1.画特殊点图标
        let icon = null
        let elementType = 3
        // 判断 activeElement
        if (this.activeElement === 'charger') {
          console.log('激活充电桩')
          icon = L.divIcon({
            html: `<img src="${chargerImg}" alt="charger" width="30" height="30">`,
            className: 'charger-icon',
            iconSize: [30, 30],
            iconAnchor: [15, 15] // 将锚点设置为图标中心
          });
          elementType = 1
        } else if (this.activeElement === 'gripper') {
          console.log('激活夹爪')
          icon = L.divIcon({
            html: `<img src="${gripperImg}" alt="gripper" width="30" height="30">`,
            className: 'gripper-icon',
            iconSize: [30, 30],
            iconAnchor: [15, 15] // 将锚点设置为图标中心
          });
          elementType = 2
        }
        else if (this.activeElement === 'delivery') {
          elementType = 0
          icon = L.divIcon({
            html: `<img src="${deliveryImg}" alt="delivery" width="30" height="30">`,
            className: 'delivery-icon',
            iconSize: [30, 30],
            iconAnchor: [15, 15] // 将锚点设置为图标中心
          });
        }
        console.log('当前icon:', icon)
        this.elementIcon = L.marker([relativeY, relativeX], { icon: icon ,draggable: true});
        this.elementIcon.on('drag', (event) => 
        {
          const currentMarker = event.target;
          const newPos = currentMarker.getLatLng();
          this.elementSubject.next({
            element_id:element_id,
            type:'move',
            point: [newPos.lng, newPos.lat],
          })
        })
        this.elementLayer.addLayer(this.elementIcon);
        this.elementIcon.point = [relativeX, relativeY]

        ///!!! 2.画第一个node(充点电或者储位点内的node)
        const element_node_id = this.generateRandomString()
        const topo_type = 0
        const transformHostX = relativeX / this.map_info.scale * this.map_info.resolution
        const transformHostY = relativeY / this.map_info.scale * this.map_info.resolution
        const node_info = {
            node_id: element_node_id,
            x: transformHostX,
            y: transformHostY,
            topo_type: topo_type,
        }
        this.nodes.push(node_info)
        this.instructionPoint.push({
          type: 3,
          x: relativeX,
          y: relativeY,
          originStyle: this.nodeStyle.originalStyle,
          label: undefined,
          node_id: element_node_id,
        })

        const marker = L.circleMarker([relativeY, relativeX], 
          this.nodeStyle.originalStyle
        );
        marker.point = {
          x: relativeX,
          y: relativeY,
          node_id: element_node_id
        }

        marker.relativeEdges = []
        this.nodesLayer.addLayer(marker);
        console.log('生成node:',node_info)
        this.elementIcon.marker = marker


        ///!!! 3.画第第二个node，连接node1和node2
        const predock_node_id = this.generateRandomString()
        const predoctHostX = transformHostX + 1.2
        const predoctHostY = transformHostY

        const predock_node_info = {
            node_id: predock_node_id,
            x: predoctHostX,  
            y: predoctHostY,
            topo_type: 1,
        }
        this.nodes.push(predock_node_info)
        this.instructionPoint.push({
          type: 2,
          x: predoctHostX,
          y: predoctHostY, 
          originStyle: this.nodeStyle.originalStyle,
          label: undefined,
          node_id: predock_node_id,
        })

        const predockLeafLetPoint = this.transformPoint({x: predoctHostX,y: predoctHostY})


        const predock_marker = L.circleMarker([predockLeafLetPoint[1],predockLeafLetPoint[0]], 
          this.nodeStyle.originalStyle
        );
        //!!! 3.1 给predock_marker添加拖动事件
        const dragIcon = L.divIcon({
                className: 'transparent-drag-area', 
                iconSize: [20, 20],                 // 设置透明区域的大小
                iconAnchor: [10, 10],               // 图标锚点
            });
        const dragMarker = L.marker([predockLeafLetPoint[1], predockLeafLetPoint[0]], {
                icon: dragIcon,
                draggable: true,                    // 默认禁止拖动
                opacity: 0,                         // 透明度设为 0
            }).addTo(this.map);
        predock_marker.dragMarker = dragMarker
        predock_marker.dragMarker.on('drag', (event) => {
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
              node_id: predock_marker.point.node_id,
              x: relativeX,
              y: relativeY,
              delete:false
            })

        });


        predock_marker.point = {
          x:predockLeafLetPoint[0],
          y:predockLeafLetPoint[1],
          node_id: predock_node_id
        }
        this.elementIcon.predock_marker = predock_marker
        predock_marker.relativeEdges = []
        this.nodesLayer.addLayer(predock_marker);
        console.log('生成node:',predock_node_info)

        //!!! 3.2 构造edge，生成连线
        const constructEdge = {
          startPoint:{
            x: transformHostX,
            y: transformHostY,
          },
          endPoint: {
            x: predoctHostX,
            y: predoctHostY,
          },
          edgeId: this.generateRandomString(),
          topoType: 0,
          edgeType: 'line',
          velocity:0.3,
          topoType: 0,
          allowBackward:false,
          width:2,
          dockingType: 0,
          avoidanceMode: 2, //2：FEATURE_INFO
          safetyZoneType:0,
          requireApply:false,
          elementId: "",
          navigationMode: 3,
        }

        if (this.activeElement === 'charger'){
          constructEdge.safetyZoneType = 1
        }
        else if (this.activeElement === 'delivery' || this.activeElement === 'gripper'){
          constructEdge.safetyZoneType = 2
        }
        const [elementLine,selesctableObj] = this.addEdge(constructEdge)
        this.elementLine = elementLine
        this.edgeLayer.addLayer(elementLine)
        this.edgeLayer.addLayer(selesctableObj)
        this.map_info.edges.push(constructEdge)

        ///!!! 4.画旋转Marker，这个node是一个marker用于旋转充点电角度
        const angleHostX = transformHostX
        const angleHostY = transformHostY + 1.8

        const transformAnglePoint = this.transformPoint({x: angleHostX,y: angleHostY})
        const rotateIcon = L.icon({
            iconUrl: rotateIconUrl2,
            iconSize: [24, 24],                // 图标的大小（宽度, 高度）
            iconAnchor: [12, 12],              // 图标的锚点位置，通常是图标的底部中心
            popupAnchor: [0, -24]              // 弹窗的锚点位置，相对于图标的位置
        });

        this.linetoCircle = L.polyline(
          [[relativeY,relativeX],
          [transformAnglePoint[1],transformAnglePoint[0]]],
          {color: 'black',weight:2,dashArray: '5, 5'}
        )
        this.linetoCircle.point = transformAnglePoint
        this.elementLayer.addLayer(this.linetoCircle)




        const circularMitionMarker = L.marker([transformAnglePoint[1], transformAnglePoint[0]], {
            icon: rotateIcon,
            draggable: true, // 允许拖动
        })
        circularMitionMarker.angle = 0.0
        this.elementLayer.addLayer(circularMitionMarker)
        circularMitionMarker.on('drag', (event) => {
          const currentMarker = event.target;
            const [newX,newY] = this.circularMotion(currentMarker, this.elementIcon.point,1.5*1.2*this.map_info.scale / this.map_info.resolution);
            currentMarker.setLatLng([newY, newX]);  // 更新位置
            this.elementSubject.next({
              element_id:element_id,
              type:'rotate',
              point: [newX, newY],
            })
        });
        circularMitionMarker.point = transformAnglePoint
        this.elementIcon.circularMarker = circularMitionMarker


        //TODO
        //1. 构造elementInfo
        //2. 重新根据elementInfo画充电点|储位点|操作点
        const angle = 0.0 //默认向右开放，即前端的PI,但是json里面的角度似乎是参考x轴负方向为0，因此这里加一个pi变成0
        const p1 = {
          x: transformHostX - 0.4 * (Math.cos(angle) + Math.sin(angle)),
          y: transformHostY + 0.4 * (Math.cos(angle) - Math.sin(angle))
        }
        const p2 = {
          x: transformHostX + 0.4 * (Math.cos(angle) - Math.sin(angle)),
          y: transformHostY + 0.4 * (Math.cos(angle) + Math.sin(angle))
        }
        const p3 = {
          x: transformHostX + 0.4 * (Math.cos(angle) + Math.sin(angle)),
          y: transformHostY - 0.4 * (Math.cos(angle) - Math.sin(angle))
        }
        const p4 = {
          x: transformHostX + 0.4 * (Math.sin(angle) - Math.cos(angle)),
          y: transformHostY - 0.4 * (Math.cos(angle) + Math.sin(angle))
        }

        const constructElementInfo = 
        {
          id: element_id,
          type: elementType,
          featureParam:
          {
            featureFigureType:0,
            featureOperationInfo:
            {
              dockingMode:0,
              targetPoint: 
              {
                x: transformHostX,
                y: transformHostY,
              }
            },
            featurePose:
            {
              angle : angle,
              length: 0.8,
              location:{
                x: transformHostX,
                y: transformHostY
              },
              points:[
                  p1,p2,p3,p4
              ],
              qrCode: "",
              width: 0.8,
            }
          }

        }
        this.elementInfo.push(constructElementInfo)
        // console.log('生成elementInfo:',constructElementInfo)
        console.log('添加elementInfo:',this.elementInfo)
        this.activeElement = null;
        this.map.off('click',this.handleAddElement)
      },
      //绑定Enter健，按下时修改控制点角度
      changeControlPointAngle(event) {
          if (event.key === 'Enter' && this.showSelectEdge) {
            // if(typeof(this.visualAngle.angle1)!='number' ||typeof(this.visualAngle.angle2)!='number'){
            //   alert('请不要输入非数字字符!')
            //   return
            // }
            console.log('-------------------------------------------')
            // console.log('修改控制点角度:', [this.visualAngle.angle1, this.visualAngle.angle2]);

            const dist1 = this.bCurvePoints.controlPoint1.distance;
            const dist2 = this.bCurvePoints.controlPoint2.distance;
            // 将角度转换为 -180 到 180 范围
            let angle1 = this.visualAngle.angle1 % 360;  // 保证在 [0, 360) 范围内
            let angle2 = this.visualAngle.angle2 % 360;  // 保证在 [0, 360) 范围内
            // console.log('角度1：', angle1, '角度2：', angle2);
            // 如果角度为负值，将其转换到 [0, 360)
            if (angle1 < 0) angle1 += 360;
            if (angle2 < 0) angle2 += 360;

            // 将角度调整到 -180 到 180 的范围
            if (angle1 > 180) angle1 -= 360;
            if (angle2 > 180) angle2 -= 360;

            // 将角度从度数转换为弧度
            angle1 = angle1 * (Math.PI / 180);
            angle2 = angle2 * (Math.PI / 180);

            console.log('角度1：', angle1, '角度2：', angle2);

            const calcStartPoint = this.transformPoint(this.selectedEdgeConfig.startPoint);
            const calcEndPoint = this.transformPoint(this.selectedEdgeConfig.endPoint);
            // 计算新的坐标
            const newX1 = calcStartPoint[0] + dist1 * Math.cos(angle1);
            const newY1 = calcStartPoint[1] + dist1 * Math.sin(angle1);
            const newX2 = calcEndPoint[0] + dist2 * Math.cos(angle2);
            const newY2 = calcEndPoint[1] + dist2 * Math.sin(angle2);

            const controlPonit3 = this.calculateControlPoint(this.bCurvePoints.startPoint.x, this.bCurvePoints.startPoint.y,[newX1, newY1])
            const controlPoint4 = this.calculateControlPoint(this.bCurvePoints.endPoint.x, this.bCurvePoints.endPoint.y,[newX2, newY2])
            
            
            this.bCurbePointSubject.next({
              controlPoint3:
              {
                x: controlPonit3[0],
                y: controlPonit3[1]
              },
            })
            this.bCurbePointSubject.next({
              controlPoint4:
              {
                x: controlPoint4[0],
                y: controlPoint4[1]
              },
            })

          }
      },

      handleSelectEdge(){

        this.edgeLayer.eachLayer((edge)=>{
            if(edge.selectableObj){
              edge.selectableObj.setStyle({opacity:0.035,weight:28,color:'red'})
              edge.selectableObj.on('mouseover', () => {
                edge.setStyle({ color: 'red' });
              });
              edge.selectableObj.on('mouseout', () => {
                edge.setStyle({ color: 'black' });
              });
              edge.selectableObj.on('click', () => {
                this.showSelectNode = false
                this.selectedEdge = edge
                this.taskPoint = 
                {
                  startPoint: 
                  {
                    point:
                    {
                      x: edge.startPoint.x,
                      y: edge.startPoint.y,
                    },
                  },
                  endPoint:
                  {
                    point:
                    {
                        x:edge.endPoint.x,
                        y:edge.endPoint.y,
                      }
                  },
                  
                }
                console.log('选中边：',this.selectedEdge)
                this.controlPointLayer.clearLayers() // 清除控制点图层
                this.dottledLayer.clearLayers() // 清除虚线
                const transformHostStartPoint = this.transformPointReverse(edge.startPoint)
                const transformHostEndPoint = this.transformPointReverse(edge.endPoint)
                this.selectedEdgeConfig = {
                  edgeId: edge.edgeId,
                  allowBackward : edge.allowBackward,
                  startPoint: transformHostStartPoint,
                  endPoint: transformHostEndPoint,

                  topoType: edge.topoType,
                  edgeType: edge.edgeType,
                  width: edge.width,
                  dockingType: edge.dockingType,
                  avoidanceMode: edge.avoidanceMode,
                  safetyZoneType: edge.safetyZoneType,
                  navigationMode: edge.navigationMode,
                  velocity: edge.velocity,
                  elementId: edge.elementId,
                  requireApply: edge.requireApply,
                }
                this.showSelectEdge = true

                if (edge.edgeType === 'bcurve') {
                  if(edge.controlPoints){
                    this.selectedEdgeConfig.controlPoints = [
                      this.transformPointReverse({x:edge.controlPoints[0][0],y:edge.controlPoints[0][1]}),
                      this.transformPointReverse({x:edge.controlPoints[1][0],y:edge.controlPoints[1][1]})
                    ]
                  }

                  // 计算控制点1,2的位置,给 this.controlPoint1赋值
                  const controlPoint1 = this.selectedEdge.controlPoints[0];
                  const controlPoint2 = this.selectedEdge.controlPoints[1];

                  this.drawDraggableAndMovableIcon(this.selectedEdgeConfig.startPoint,this.selectedEdgeConfig.endPoint,controlPoint1,controlPoint2)
                  console.log('此时的b样条曲线点',this.bCurvePoints)

                }
              })
            }

          })

      },
      handleKeyDown(event) {
        console.log('按下按键：', event)
        if (event.originalEvent.key === 'Delete') {
          console.log('删除节点')
          this.onSaveNode(true)
        }
      },
      updateNodeInput(){
        if (this.closetNode){
          return [this.closetNode.point.x,this.closetNode.point.y]
        }
      },
      onSelectObject(event){
        if (this.closetNode){
          const transfromHostPoint = this.transformPointReverse(this.closetNode.point)
          this.displayNode= {
            x: transfromHostPoint.x.toFixed(4),
            y: transfromHostPoint.y.toFixed(4),
            node_id: this.closetNode.point.node_id,
          }
          this.showSelectNode = true
          this.showSelectEdge = false
          console.log('显示点信息:')
        }
        // console.log('地图初始缩放比例:',this.initialZoom)
        const currentZoom = this.map.getZoom();
        console.log('当前缩放比例:',currentZoom)


      },
      onSaveNode(del=false){

        const transformMapPoint = this.transformPoint(this.displayNode)
        console.log('保存节点配置：'+JSON.stringify(this.displayNode))
        console.log('保存节点配置：'+JSON.stringify(transformMapPoint))
        this.nodeSubject.next({
          x: transformMapPoint[0],
          y: transformMapPoint[1],
          node_id:this.displayNode.node_id,
          delete: del
        })

        // this.showSelectNode = false
      },
      onCancelSaveNode(){
        this.showSelectNode = false
      },
      activateNodeDgag(){
          this.nodesLayer.eachLayer((node) => {
            if(node.dragMarker){
            node.dragMarker.dragging.enable()}
            //不是直接让node允许拖动，而是让其内部的dragMarker允许拖动
            //因为dragMarker(透明)更大一点方便选中
          })
      },
      async switchViualMap(mapName) {
        console.log('切换地图');
        try {
            const response = await axios.post(`http://${this.publicIP}:5000/MapEdit/SwitchMap`, 
                { map_name: mapName },
                { responseType: 'blob' } // 设置响应类型为 Blob
            );

            const imageBlob = response.data;
            const imageURL = URL.createObjectURL(imageBlob);
            const img = new Image();
                // 清除现有的图层（如果存在）
            if (this.imageOverlay) {
                await this.map.removeLayer(this.imageOverlay);
            }
            img.src = imageURL;
            img.onload = () => {
                const originalWidth = img.width;
                const originalHeight = img.height;

                // 获取 Leaflet 容器的大小
                const containerSize = this.map.getSize();
                const desireX = containerSize.x;
                const desireY = containerSize.y;

                // 计算缩放比例，使图片按比例缩放到容器大小
                const scaleX = desireX / originalWidth;
                const scaleY = desireY / originalHeight;
                const scale = Math.min(scaleX, scaleY);

                // 存储地图信息
                this.map_info = {
                    ...this.map_info,
                    width: originalWidth,
                    height: originalHeight,
                    scale: scale,
                };

          // 计算缩放后的图片尺寸
          const scaledWidth = originalWidth * scale;
          const scaledHeight = originalHeight * scale;

          // 使用缩放后的宽高设置地图边界（[y, x]）
          const bounds = [[0, 0], [scaledHeight, scaledWidth]];

          // 如果已有图层，移除它
          if (this.imageOverlay) {
              this.map.removeLayer(this.imageOverlay);
          }

          // 使用 setTimeout 确保地图状态更新不会冲突
          setTimeout(() => {
              // 添加新的图像覆盖层
              this.imageOverlay = L.imageOverlay(imageURL, bounds).addTo(this.map);

              // 调整地图边界到新图层大小
              this.map.fitBounds(bounds);

              // 刷新地图容器，避免渲染问题
              this.map.invalidateSize();
          }, 100); // 延迟 100ms 执行
      };        
      } catch (error) {
          console.error('切换地图失败:', error);
      }
    },
    async onMapSelected() {
        console.log('选择的地图：', this.selectedMap);
        if (this.nodeLayer){
            this.nodeLayer.clearLayers()
        }
        if (this.edgeLayer){
            this.edgeLayer.clearLayers();
        }
        await this.switchViualMap(this.selectedMap)
        await this.switchMapData(this.selectedMap)
        this.drawNodes()
        this.drawEdges()

        //找到node和edge的对应关系
      },
      async getMapList() {
        try 
        {
            // console.log('获取地图列表');
            const response = await axios.get(`http://${this.publicIP}:5000/MapEdit/GetMapList`);
            this.mapList = response.data; // 假设 response.data 是一个地图列表
            // console.log('获取到的地图列表：', this.mapList);
        } 
        catch (error) {
            console.error('获取地图列表失败：', error);
            // 可以在这里添加用户提示，例如使用 alert 或其他方式
        }
    },
    async switchMapData(mapName){
        try {
          const response = await axios.post(`http://${this.publicIP}:5000/MapEdit/SwitchMapData`,
            {map_name: mapName},
          );
          this.elementInfo = (response.data.elements_info)?JSON.parse(response.data.elements_info).elementInfo:[]
          // console.log('elementInfo:', this.elementInfo)
          console.log('切换地图数据成功:', response.data);
          this.map_info = {
            ...this.map_info,
            resolution: JSON.parse(response.data.map_json).resolution,
          }
        this.map_info.edges = (response.data.map_edges)?JSON.parse(response.data.map_edges).map_edges:[]
        const json_node = (response.data.node)?JSON.parse(response.data.node):null;
        this.nodes = (response.data.node)?json_node.node:[];
        } catch (error) {
          console.error('Failed to load map data:', error);
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
        this.nodesLayer = L.layerGroup().addTo(this.map); // 初始化节点图层
        this.edgeLayer = L.layerGroup().addTo(this.map); // 初始化边缘图层
        this.imageOverlay = L.layerGroup().addTo(this.map); // 确保初始化为 null
        this.taskNodeLayer = L.layerGroup().addTo(this.map); // 初始化任务节点图层
        this.tooltipLayer = L.layerGroup().addTo(this.map); // 初始化工具提示图层
        this.dottledLayer = L.layerGroup().addTo(this.map); // 初始化虚线图层
        this.controlPointLayer = L.layerGroup().addTo(this.map); // 初始化控制点图层
        this.selectedEdgeLayer = L.layerGroup().addTo(this.map); // 初始化选中边图层
        this.elementLayer = L.layerGroup().addTo(this.map); // 初始化元素图层
      },
      handleAddNode(event) {
        this.showSelectNode = true;
        const clickedLatLng = this.map.mouseEventToLatLng(event.originalEvent);
        const point = this.map.latLngToContainerPoint(clickedLatLng);

        // 获取 imageOverlay 的边界
        const bounds = this.imageOverlay.getBounds();
        const downleft = this.map.latLngToContainerPoint(bounds.getSouthWest());

        // 计算相对于 mapLayer 的坐标
        let relativeX = point.x - downleft.x;
        let relativeY = downleft.y - point.y;
        console.log(`鼠标坐标: (${relativeX}, ${relativeY})`);

        const zoom = this.map.getZoom();
        console.log(`当前缩放级别: ${zoom}`);
        relativeX = relativeX / Math.pow(2, zoom);
        relativeY = relativeY / Math.pow(2, zoom);

        // **改动1：生成顺序编号的 node_id**
        const node_id = this.generateSequentialNodeId();
        const topo_type = 2;
        const transformHostX = (relativeX / this.map_info.scale) * this.map_info.resolution;
        const transformHostY = (relativeY / this.map_info.scale) * this.map_info.resolution;

        this.displayNode = {
          x: transformHostX.toFixed(4),
          y: transformHostY.toFixed(4),
          node_id: node_id,
        };
        const node_info = {
          node_id: node_id,
          x: transformHostX,
          y: transformHostY,
          topo_type: topo_type,
        };
        this.nodes.push(node_info);
        this.instructionPoint.push({
          type: 3,
          x: relativeX,
          y: relativeY,
          originStyle: this.nodeStyle.originalStyle,
          label: undefined,
          node_id: node_id,
        });

        const marker = L.circleMarker([relativeY, relativeX], this.nodeStyle.originalStyle);
        const dragIcon = L.divIcon({
          className: 'transparent-drag-area', // 定义透明图标的样式
          iconSize: [20, 20], // 设置透明区域的大小
          iconAnchor: [10, 10], // 图标锚点
        });
        const dragMarker = L.marker([relativeY, relativeX], {
          icon: dragIcon,
          draggable: false, // 默认禁止拖动
          opacity: 0, // 透明度设为 0
        }).addTo(this.map);
        marker.point = {
          x: relativeX,
          y: relativeY,
          node_id: node_id,
        }; // 保存节点信息
        marker.originalStyle = point.originStyle; // 保存原始样式
        marker.relativeEdges = [];
        marker.dragMarker = dragMarker;

        // 将拖动事件同步到实际的 circleMarker 上
        marker.dragMarker.on('drag', (event) => {
          const markerLatLng = event.target.getLatLng();
          const point = this.map.latLngToContainerPoint(markerLatLng);
          const bounds = this.imageOverlay.getBounds();
          const downleft = this.map.latLngToContainerPoint(bounds.getSouthWest());

          let relativeX = point.x - downleft.x;
          let relativeY = downleft.y - point.y;

          const zoom = this.map.getZoom();
          relativeX = relativeX / Math.pow(2, zoom);
          relativeY = relativeY / Math.pow(2, zoom);

          this.nodeSubject.next({
            node_id: marker.point.node_id,
            x: relativeX,
            y: relativeY,
            delete: false,
          });
        });

        // 添加 marker 到图层
        this.nodesLayer.addLayer(marker);

        console.log('生成node:', node_info);
      },

      generateSequentialNodeId() {
        // 如果 this.nodes 为空，则从 P1 开始
        if (!this.nodes) {
          this.nodes = [];
          return 'P1';
        }

        // 获取已有节点的最大编号
        const existingIds = this.nodes
          .map(node => node.node_id)
          .filter(id => id.startsWith('P'))
          .map(id => parseInt(id.slice(1))) // 提取数字部分
          .filter(num => !isNaN(num)); // 过滤非数字

        // 获取最大编号，如果没有节点，则默认从 P1 开始递增
        const maxId = existingIds.length > 0 ? Math.max(...existingIds) : 0;
        return `P${maxId + 1}`;
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
            this.imageOverlay = L.imageOverlay(imageURL, bounds).addTo(this.map);
            this.map.fitBounds(bounds);
            this.drawNodes()
            this.drawEdges()

            
            this.initialZoom = this.map.getZoom() // 记录初始缩放级别
            this.initialDownLeft = this.map.latLngToContainerPoint(this.imageOverlay.getBounds().getSouthWest());

            // console.log(`Map image loaded: ${scaledWidth}x${scaledHeight}`);
          };
        } catch (error) {
          console.error('Failed to load map image:', error);
        }
      },
      addToolbar() {
        const toolbar = L.control({ position: 'topleft' });

        toolbar.onAdd = () => {
        const div = L.DomUtil.create('div', 'leaflet-bar');
        div.style.width = '500px';
        div.style.height = '50px';
        div.style.backgroundColor = 'rgba(255, 255, 255, 0.8)';
        div.style.display = 'flex';
        div.style.alignItems = 'center';
        div.style.justifyContent = 'space-around';
        div.style.borderRadius = '10px';
        div.style.boxShadow = '0 2px 5px rgba(0, 0, 0, 0.2)';

        const pointIcon = L.DomUtil.create('img', '', div);
        pointIcon.src = 'https://cdn-icons-png.flaticon.com/512/32/32339.png';
        pointIcon.style.width = '30px';
        pointIcon.style.height = '30px';
        pointIcon.style.cursor = 'pointer';

        const lineIcon = L.DomUtil.create('img', '', div);
        lineIcon.src = 'https://cdn-icons-png.flaticon.com/512/32/32278.png';
        lineIcon.style.width = '30px';
        lineIcon.style.height = '30px';
        lineIcon.style.cursor = 'pointer';

        return div;
        };

        toolbar.addTo(this.map);
    },

  
    async fetchMapData() {
      try {
        const response = await axios.get(`http://${this.publicIP}:5000/Map/GetMapData`);
        // this.elementInfo = response.data.elements_info?JSON.parse(response.data.elements_info).elementInfo:null
        // console.log('elementInfo:', this.elementInfo)
        // console.log('收到地图数据响应',JSON.parse(response.data))
        this.selectedMap = response.data.map_name

        this.map_info = {
          ...this.map_info,
          resolution: JSON.parse(response.data.map_json).resolution,
          origin: JSON.parse(response.data.map_json).origin,
        }
        this.map_info.edges = JSON.parse(response.data.map_edges).map_edges?JSON.parse(response.data.map_edges).map_edges :[]
        console.log('边信息:',(this.map_info.edges))
        const json_node = JSON.parse(response.data.node);
        this.nodes = json_node.node;
        // console.log('node信息',this.nodes)
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

            if (point.label) {
                const tooltip = L.tooltip({
                    permanent: true,    // 永久显示
                    direction: 'top',  // 在上方显示
                    className: 'custom-tooltip', // 自定义类名样式
                    offset: [0, -10],   // 调整与节点的距离
                })
                .setContent(point.label)  // 设置标签内容
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
        this.map.off('click', this.highLightTaskNode); // 取消高亮任务节点
  
        this.startTask(); // 开始任务
      },
      startTask() {
        // console.log('开始高亮');

        this.map.on('mousemove', this.highLightMoveNode) //高亮离鼠标最近的节点
        // 绑定事件，当鼠标按下时，分别确定task的起始点和结束点
        this.map.on('click', this.highLightTaskNode); //高亮选中的任务节点
        },
      // 高亮节点移动事件
      highLightMoveNode(event,dist = 30) {
        
        const distTolerance = dist;
  
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
          if (this.closetNode && this.closetNode !== closetNode) {
            // 恢复之前高亮节点的样式为其原始样式
            const previousStyle = this.getInstructionPointStyle(this.closetNode);
            this.closetNode.setStyle(previousStyle);
          }
  
          // 设置新的高亮样式
          closetNode.setStyle(this.nodeStyle.highlightStyle);
          this.closetNode = closetNode; // 更新当前高亮的节点
        } else {
          // 如果没有符合条件的节点，恢复之前高亮的节点为其原始样式
          if (this.closetNode) {
            const previousStyle = this.getInstructionPointStyle(this.closetNode);
            this.closetNode.setStyle(previousStyle);
            this.closetNode = null;
          }
        }
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
  
  
      highLightTaskNode(event) {
  
        if (this.closetNode) {
          if (this.taskPoint.startPoint === null) {
            this.taskPoint.startPoint = this.closetNode;
            console.log('起点:', this.taskPoint.startPoint);

            // 设置结束点样式
            this.taskPoint.startPoint.setStyle(this.nodeStyle.startPointStyle);

            // 添加终点标签
            this.taskPointTip.startTip = L.tooltip({
              permanent: true, 
              direction: 'top',
              className: 'end-tooltip',
              offset: [0, -10],
            })
              .setContent('起点')
              .setLatLng(this.closetNode.getLatLng())
              .addTo(this.map);
          }
          else if (this.taskPoint.endPoint === null) {

            if (this.closetNode === this.taskPoint.startPoint) {
              Swal.fire({
                title: "终点选择错误",
                text: "起点和终点不能相同,请重新选择终点",
                icon: "error",
                timer: 1500,
                showCloseButton: false,
                position: "top",
                backdrop: false,
                heightAuto: true,
              })
              return;
            }
            this.taskPoint.endPoint = this.closetNode;
            console.log('结束点:', this.taskPoint.endPoint);
            // console.log('选中节点:', this.taskPoint.endPoint.node.node_id);
  
            // 设置结束点样式
            this.closetNode.setStyle(this.nodeStyle.endPointStyle);
  
            // 添加终点标签
            this.taskPointTip.endTip = L.tooltip({
              permanent: true, 
              direction: 'top',
              className: 'end-tooltip',
              offset: [0, -10],
            })
              .setContent('终点')
              .setLatLng(this.closetNode.getLatLng())
              .addTo(this.map);
            // 生成一条起始点到终点的边
            const strartX = this.taskPoint.startPoint.point.x 
            const strartY = this.taskPoint.startPoint.point.y 
            const endX = this.taskPoint.endPoint.point.x 
            const endY = this.taskPoint.endPoint.point.y
            this.selectedEdge = L.polyline([[strartY, strartX], [endY, endX]], { color: 'black', weight: 3, opacity: 0.5 })
            this.selectedEdge.edgeId = this.generateRandomString()
            this.selectedEdge.relativeNode = [this.taskPoint.startPoint.point.node_id, this.taskPoint.endPoint.point.node_id]
            this.selectedEdge.startPoint = this.taskPoint.startPoint.point
            this.selectedEdge.endPoint = this.taskPoint.endPoint.point
            // this.selectedEdge.addTo(this.edgeLayer)
            // this.edgeLayer.addLayer(this.selectedEdge); // 使用 addLayer 将线添加到 edgeLayer
            console.log('画边:', this.selectedEdge);
            this.showSelectEdge = true;

            this.selectedEdgeConfig.startPoint = {
              node_id: this.selectedEdge.startPoint.node_id,
              x: this.taskPoint.startPoint.point.x / this.map_info.scale * this.map_info.resolution,
              y: this.taskPoint.startPoint.point.y / this.map_info.scale * this.map_info.resolution
            }
            this.selectedEdgeConfig.endPoint = {
              node_id: this.selectedEdge.endPoint.node_id,
              x:this.taskPoint.endPoint.point.x / this.map_info.scale * this.map_info.resolution,
              y:this.taskPoint.endPoint.point.y / this.map_info.scale * this.map_info.resolution}

            this.selectedEdgeConfig.edgeId = this.selectedEdge.edgeId;
            const [visLine,seletableObj] = this.addEdge(this.selectedEdgeConfig)
            this.selectedEdgeLayer.addLayer(visLine)
            this.selectedEdgeLayer.addLayer(seletableObj)

            this.changeControlPointAngle = this.changeControlPointAngle.bind(this);
          } else {
            // 重新选择两个点
            this.showSelectEdge = false;
            if (this.taskPointTip.endTip){this.taskPointTip.endTip.remove();}
            if(this.taskPointTip.startTip){this.taskPointTip.startTip.remove();}

            this.edgeLayer.removeLayer(this.selectedEdge);
            this.taskPoint.startPoint.setStyle(this.taskPoint.startPoint.point.originalStyle);
            this.taskPoint.endPoint.setStyle(this.taskPoint.endPoint.point.originalStyle);
            this.dottledLayer.clearLayers();
            this.controlPointLayer.clearLayers()
            this.taskPoint.startPoint = null;
            this.taskPoint.endPoint = null;
            this.highLightTaskNode()

            
          }
        }
      },
        // 绘制edges
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
  
    },
  };
  </script>
  <style scoped>
  .container {
    display: flex;
    flex-direction: row;
    /* width: 100vw; */
    /* height: 100vh; */
    /* position: relative; */
  }
    
  .map-container {
    margin-top: 60px;
      width: 1200px; /*TODO 这里由于设置了固定像素导致无法自适应 */
      height: 800px;
      border: 1px solid #ccc;
      position: relative;
    }
  
  
  .task-container {
    border: 2px solid #ccc; /* 边框样式 */
    border-radius: 8px; /* 圆角 */
    padding: 10px; /* 内边距 */
    background-color: #f9f9f9; /* 背景颜色 */
    margin-left: 5px; /* 可根据需要调整间距 */
    width: 200px; /* 固定宽度，可以根据需要调整 */
  }
  
  .task {
    height: 100%;
    display: flex; /* 使内部内容居中 */
    flex-direction: column;
  }
  
  .tool-icon {
  width: 30px; 
  height: 30px; 
  cursor: pointer; 
  transition: transform 0.2s, box-shadow 0.2s;
  position: relative;
}

.tool-icon.active {
  transform: scale(1.2); /* 放大效果 */
  filter: brightness(1.2); /* 增加亮度 */
  box-shadow: 0 0 10px rgba(0, 0, 0, 0.5); /* 添加阴影效果 */
}

.tool-icon.active::before {
  content: "";
  position: absolute;
  top: -5px;
  left: -5px;
  right: -5px;
  bottom: -5px;
  border: 2px solid black; /* 边框颜色变为黑色 */
  border-radius: 5px; /* 可以根据需要调整边框圆角 */
  pointer-events: none; /* 确保点击仍能触发图标本身的事件 */
}
.select-option{
  margin-left: 0px;
  /* border: 1px solid #ccc; */
  border-radius: 5px;
  flex-direction: column;
  /* display: flex; */
  height:400px;


}
.select-option > * {
  font-size: 18px;
  margin-bottom: 8px; /* 设置子元素底部的间距 */
}
.row-display{
  display: flex;
  flex-direction: row;
}
.col-display{
  display: flex;
  flex-direction: column;
}

.custom-marker-icon {
    cursor: pointer;
}
.transparent-drag-area {
    background-color: transparent;
    border-radius: 50%;
}
.active {
    border: 2px solid #007bff; /* 你可以根据需求调整颜色 */
    box-shadow: 0 0 10px rgba(0, 123, 255, 0.5); /* 添加阴影效果 */
    border-radius: 4px; /* 如果想要圆角，这里可以定义一个值 */
}
  </style>