<template>
  <div class="grid">
    <div class="file-selector">
      <div class="selector-container">
        <select v-model="selectedFile">
          <option value="" disabled selected>请选择需要更新的版本</option>

          <!-- 遍历 files 数组并展示每个文件名 -->
          <option v-for="(file, index) in files" :key="index">{{ file }}</option>
        </select>
        
        <!-- 新增的下拉栏 -->
        <select v-model="selectedParameter" style="margin-left: 10px;">
          <option value="" disabled selected>选择车型</option>
          <option value="300">300</option>
          <option value="500">500</option>
          <option value="1200">1200</option>
        </select>
        
        <el-button 
        type="primary" 
        style="margin-left: 10px;" 
        @click="updateVersion()" 
        :loading="shouLoadings"
      >
        {{ shouLoadings ? '更新中...' : '更新版本' }}
      </el-button>
      </div>
    </div>
  </div>
</template>

<script>
import { useStore } from 'vuex';
import { computed } from 'vue';
import axios from 'axios';
import Swal from 'sweetalert2';

export default {
  data() {
    return {
      files: [], // 存储提取的文件名
      selectedFile: '', // 选中的文件
      selectedParameter: '', // 选中的参数
      publicIP: null, // 存储 public IP
      shouLoadings: false, // 控制加载动画的显示
    };
  },
  created() {
    const store = useStore();
    const ip = computed(() => store.state.publicIP);
    this.publicIP = ip.value;

    // 加载版本列表
    this.loadPackageVersion();
  },
  methods: {
    // 提取文件名并加载到 files 数组
    async loadPackageVersion() {
      try {
        const response = await axios.get(`http://${this.publicIP}:2345/ftp-files`);
        console.log('获取到的版本列表: ', response.data);

        // 遍历并提取每个对象的键名
        this.files = response.data.files.map((item) => Object.keys(item)[0]);
      } catch (error) {
        console.error('加载版本列表失败:', error);
      }
    },

    // 处理获取版本的请求
    async updateVersion() {
    try{
      if (!this.selectedFile) {
        Swal.fire({
          // title: 
          text: '请选择需要更新的版本!',
          icon: "error",
          // confirmButtonText: "确定",
          showCloseButton: false,
          position: "top",
          backdrop: false,
          heightAuto: true,
          timer: 1500,
        })  
        return
      }
      console.log('开始更新版本!')
      this.shouLoadings = true;
      const updateRequestUrl = `http://${this.publicIP}:2345/install/${this.selectedFile}?robot_model=${this.selectedParameter}`;
        console.log('请求的 URL: ', updateRequestUrl)
        const response = await axios.get(updateRequestUrl);
        console.log('更新版本的响应: ', response.data);
        if( response.data.status === 'success' ){
          Swal.fire({
            // title: 
            text: '版本更新成功!',
            icon: "success",
            // confirmButtonText: "确定",
            showCloseButton: false,
            position: "top",
            backdrop: false,
            heightAuto: true,
          })
        }
        else{
          Swal.fire({
            // title: 
            text:'更新版本失败，请联系管理员!',
            icon: "error",
            // confirmButtonText: "确定",
            showCloseButton: false,
            position: "top",
            backdrop: false,
            heightAuto: true,
          })
        }
        this.shouLoadings = false;
      // console.log('选中的文件: ', this.selectedFile, '选中的参数: ', this.selectedParameter)
      // const robotModelCheckRequest = `http://${this.publicIP}:2345/install_check/${this.selectedParameter}`;
      // const checkResponse = await axios.get(robotModelCheckRequest);
      // if (checkResponse.data.status === 'success'){
      //   // const updateRequestUrl = `http://${this.publicIP}:2345/install/${this.selectedFile}?robot_model=${this.selectedParameter}`;
      //   // console.log('请求的 URL: ', updateRequestUrl)
      //   // const response = await axios.get(updateRequestUrl);
        
      //   console.log('更新版本的响应: ', response.data);
      //   Swal.fire({
      //   title: "版本更新",
      //   text: response.data.message,
      //   icon: response.data.status ? "success" : "error",
      //   // confirmButtonText: "确定",
      //   timer: 1500,
      //   showCloseButton: false,
      //   position: "top",
      //   backdrop: false,
      //   heightAuto: true,
      // })

        
      // }
      // else{
      //   console.log('机器人车型检查失败: ', checkResponse.data)
      //   Swal.fire({
      //   title: "机器人车型检查",
      //   text: checkResponse.data.message,
      //   icon: "error",
      //   // confirmButtonText: "确定",
      //   timer: 1500,
      //   showCloseButton: false,
      //   position: "top",
      //   backdrop: false,
      //   heightAuto: true,
      // })

      // }

    }
    catch(error){
      console.error('更新版本失败:', error);
      Swal.fire({
        // title: 
        text:'更新版本超时，请联系管理员!',
        icon: "error",
        // confirmButtonText: "确定",
        showCloseButton: false,
        position: "top",
        backdrop: false,
        heightAuto: true,
      })
      this.shouLoadings = false;
    }
  }
  },
};
</script>

<style scoped>
.grid {
  padding: 20px;
  flex-direction: column;
}

.file-selector {
  width: 250px;
  margin-bottom: 20px;
}

.selector-container {
  display: flex; /* 使用 Flexbox 布局 */
  align-items: center; /* 垂直居中对齐 */
}

select {
  flex: 1; /* 让下拉菜单占满可用空间 */
  padding: 8px;
  font-size: 16px;
}

el-button {
  margin-left: 10px; /* 按钮与下拉菜单之间的间距 */
}
</style>
