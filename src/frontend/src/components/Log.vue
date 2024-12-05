<template>
    <div class="container">
      <div class="file-selector">
        <el-radio-group v-model="selectedLogType" @change="updateFileList" style='margin-right: 30px'>
            <el-radio label="app_log">App日志</el-radio>
            <el-radio label="host_log">Host日志</el-radio>
        </el-radio-group>

        <el-select v-model="selectedFile" size="large" style="width: 330px">
          <el-option v-for="file in files" :key="file" :label="file" :value="file">{{ file }}</el-option>
        </el-select>
        <el-button v-if="!isDownloading" color="#626aef" @click="download" style="margin-left: 10px; height: 35px;">下载</el-button>
        <el-button v-else color="#626aef" @click="isDownloading=false" disabled="isDownloading" style="margin-left: 10px; height: 35px;">下载中。。。</el-button>
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
            files: [],  // 用于存储从后端获取的文件名
            selectedFile: '',  // 选中的文件名
            selectedFileContent: '',  // 当前选中文件的内容
            parsedConfig: {},  // 解析后的文件配置
            publicIP: null,  // 存储从 store 获取的 IP 地址
            selectedLogType: 'app_log',  // 默认选中的日志类型
            app_log: null,
            host_log: null,
            isDownloading: false,  // 添加状态以跟踪下载过程
        };
    },
    created() {
        const store = useStore();
        const ip = computed(() => store.state.publicIP);
        this.publicIP = ip.value; // 确保获取实际值
  
        // 加载配置文件列表
        this.loadLog().then(() => {
            // 加载配置文件内容
            this.updateFileList(); // 默认加载 app_log 列表
        });
    },
    methods: {
        updateFileList() {
            // 根据选中的日志类型更新文件列表
            if (this.selectedLogType === 'app_log') {
                this.files = this.app_log || [];
            } else if (this.selectedLogType === 'host_log') {
                this.files = this.host_log || [];
            }
            // 重置选中的文件和解析的配置
            this.selectedFile = '';
            this.parsedConfig = {};
        },

        // 调用后端 API 加载配置文件列表
        async loadLog() {
            try {
                const res = await axios.post(`http://${this.publicIP}:5000/Log/loadLog`);
                this.app_log = JSON.parse(res.data).app_log;
                this.host_log = JSON.parse(res.data).host_log;
                console.log(this.app_log);
            } catch (error) {
                console.error("Error loading log:", error);
            }
        },

        async download() {
            try {
                if (!this.selectedFile) {
                    Swal.fire({
                        title: "文件下载错误",
                        text: "请选择一个文件!",
                        icon: "error",
                        timer: 1500,
                        showCloseButton: false,
                        position: "top",
                        backdrop: false,
                        heightAuto: true,
                    });
                    return; // 如果没有选择文件，退出方法
                }
                
                this.isDownloading = true; // 设置为下载中
                console.log('请求下载：', this.selectedFile);
                
                const response = await axios.post(`http://${this.publicIP}:5000/Log/downLoadLog`, {
                    log_file: this.selectedFile,
                }, {
                    responseType: 'blob', // 指定响应类型为 blob
                });
                
                // 创建一个 URL 对象
                const url = window.URL.createObjectURL(new Blob([response.data]));
                const link = document.createElement('a');
                link.href = url;
                link.setAttribute('download', this.selectedFile); // 设置下载文件名
                document.body.appendChild(link);
                link.click();
                link.parentNode.removeChild(link); // 清理 DOM
            } catch (error) {
                console.error("Error downloading file:", error);
                Swal.fire({
                    title: "下载失败",
                    text: "请重试或检查文件是否存在。",
                    icon: "error",
                    confirmButtonText: "确定",
                });
            } finally {
                this.isDownloading = false; // 下载完成后恢复按钮
            }
        }
    }
};
</script>

<style scoped>
  .container {
    margin-top: 20px;
    margin-left: 30px;
    flex-direction: column;
    margin-right: 50px;
  }
  
  .file-selector {
    margin-bottom: 20px; /* 为文件选择器和网格之间增加间距 */
  }
  
  .config-container {
    display: grid;
    grid-template-columns: 1fr; /* 确保每个 section 只有一列 */
    gap: 20px; /* 添加间距 */
  }
  
  .section {
    border: 2px solid #ccc; /* 添加边框 */
    padding: 10px; /* 添加内边距 */
    border-radius: 5px; /* 圆角边框 */
  }
  
  .section-title {
    color: black;
    font-size: 1.6em;
    font-weight: bold;
    margin-bottom: 15px; /* 在标题和内容之间添加间距 */
  }
  .input-background {
      background-color: #e1e3ee;
      border: 0px solid #909091;
      border-radius: 5px;
      font-size: 16px;
  }
  .label{
    font-size:20px
  }
  .form-group {
    display: flex;
    margin-top: 15px;
    align-items: center;
    margin-right: 15px;
  }
  
  input {
    margin-left: 10px;
    flex: 1;
  }
  
  el-button {
    margin-left: 10px;
  }
  </style>
  