<template>
  <div class="container">
    <div class="file-selector">
      <el-select v-model="selectedFile"  size="large" style="width: 240px" @change="loadFileContent">
        <el-option v-for="file in files" :key="file" :label="file" :value="file">{{ file }}</el-option>
      </el-select>
      <el-button color="#626aef" @click="updateAllParameters" style="margin-left: 10px ;height: 35px;">Save All Changes</el-button>
    </div>

    <div class="config-container">
      <div v-for="(section, sectionName) in parsedConfig" :key="sectionName" class="section">
        <h4 class="section-title">{{ sectionName }}</h4>
        <div class="form-group" v-for="(value, key) in section" :key="key">
          <label class="label">{{ key }}</label>
          <input class="input-background" type="text" style='height:40px;width: 100%;' v-model="parsedConfig[sectionName][key]" />
          <el-button color="#626aef" @click="updateParameter(sectionName, key)" style ="margin-left: 10px;height:40px">Confirm</el-button>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import { useStore } from 'vuex';
import { computed } from 'vue';
import axios from 'axios';

export default {
  data() {
    return {
      files: [],  // 用于存储从后端获取的文件名
      selectedFile: '',  // 选中的文件名
      selectedFileContent: '',  // 当前选中文件的内容
      parsedConfig: {},  // 解析后的文件配置
      publicIP: null,  // 存储从 store 获取的 IP 地址
    };
  },
  created() {
    const store = useStore();
    const ip = computed(() => store.state.publicIP);
    this.publicIP = ip;

    // 加载配置文件列表
    this.loadConfig();
  },
  methods: {
    // 调用后端 API 加载配置文件列表
    loadConfig() {
      axios.post(`http://${this.publicIP}:5000/Config/loadConfig`)
        .then((res) => {
          console.log('Config:', res.data);
          if (res.data && typeof res.data === 'object') {
            this.files = Object.keys(res.data).sort();  // 获取文件名列表并排序
            this.fileContents = res.data;  // 存储文件内容
            if (this.files.length > 0) {
              this.selectedFile = this.files[0];  // 默认选择第一个文件
              this.loadFileContent();  // 加载第一个文件的内容
            }
          }
        })
        .catch((error) => {
          console.error("Error loading config:", error);
        });
    },

    // 加载选中的文件内容并解析
    loadFileContent() {
      if (this.selectedFile && this.fileContents[this.selectedFile]) {
        this.selectedFileContent = this.fileContents[this.selectedFile];  // 获取选中文件内容
        this.parsedConfig = this.parseIni(this.selectedFileContent);  // 解析 .ini 文件
      }
    },

    // 解析 .ini 文件的内容
    parseIni(content) {
      const lines = content.split('\n');
      const config = {};
      let currentSection = '';

      lines.forEach(line => {
        line = line.trim();
        if (line.startsWith('[') && line.endsWith(']')) {
          currentSection = line.slice(1, -1);
          config[currentSection] = {};
        } else if (line && currentSection && !line.startsWith('#')) {
          const [key, value] = line.split('=').map(item => item.trim());
          config[currentSection][key] = value;
        }
      });

      return config;
    },

    // 更新单个参数
    updateParameter(section, key) {
      // 打印确认信息
      console.log(`Updated ${section}.${key} to: ${this.parsedConfig[section][key]}`);
      
      // 将更新后的值存储到私有变量中
      const updatedValue = this.parsedConfig[section][key];

      // 构建要发送到后端的更新数据
      const dataToUpdate = {
        file: this.selectedFile,
        section: section,
        key: key,
        value: updatedValue
      };

      // 发送请求，将更新的参数保存到后端（可以通过API）
      axios.post(`http://${this.publicIP}:5000/Config/updateParameter`, dataToUpdate)
        .then((response) => {
          // 打印来自后端的响应信息
          console.log('Update response:', response.data);
          alert(`Successfully updated ${section}.${key} to: ${updatedValue}`);
        })
        .catch((error) => {
          // 如果更新失败，则显示错误信息
          console.error('Error updating parameter:', error);
          alert('Error updating parameter. Please try again.');
        });
    },
    // 保存整个配置文件
    updateAllParameters() {
        const updateData = {
          file: this.selectedFile,
          data: this.parsedConfig  // 传递整个配置数据
        };

        axios.post(`http://${this.publicIP}:5000/Config/updateAllParameters`, updateData)
          .then((res) => {
            console.log('Config updated successfully:', res.data);
            alert('Configuration saved!');
          })
          .catch((error) => {
            console.error('Error updating config:', error);
          });
      },
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
