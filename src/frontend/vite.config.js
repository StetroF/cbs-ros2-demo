import { fileURLToPath, URL } from 'node:url';
import { defineConfig } from 'vite';
import vue from '@vitejs/plugin-vue';
import vueJsx from '@vitejs/plugin-vue-jsx';
import VueDevTools from 'vite-plugin-vue-devtools';
import path from 'path';
import fs from 'fs';

// 删除指定目录的函数
function deleteDir(dirPath) {
  if (fs.existsSync(dirPath)) {
    fs.rmSync(dirPath, { recursive: true, force: true });
    console.log(`目录已删除: ${dirPath}`);
  }
}

// 创建指定目录的函数
function createDir(dirPath) {
  if (!fs.existsSync(dirPath)) {
    fs.mkdirSync(dirPath, { recursive: true });
    console.log(`目录已创建: ${dirPath}`);
  }
}

const outDir = path.resolve(__dirname, '../wsrc-app-web');

// https://vitejs.dev/config/
export default defineConfig({
  plugins: [
    vue(),
    vueJsx(),
    VueDevTools(),
    {
      name: 'custom-clean-plugin', // 自定义插件名称
      apply: 'build',
      buildStart() {
        console.log('开始编译...');
        deleteDir(outDir); // 删除目录
        createDir(outDir); // 重新创建目录
      },
      closeBundle() {
        console.log('编译完成...');
      },
    },
  ],
  resolve: {
    alias: {
      '@': fileURLToPath(new URL('./src', import.meta.url)),
    },
  },
  build: {
    outDir, // 输出目录
    rollupOptions: {}, // 可以在这里添加更多 Rollup 配置
  },
  server: {
    host: '0.0.0.0',
    port: 1111,
    proxy: {
      '^/api': 'http://0.0.0.0:5000/',
    },
  },
});
