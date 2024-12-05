<template>
    <div class="chart-container">
      <div ref="chart" class="chart"></div>
    </div>
  </template>
  
  <script>
  import * as echarts from "echarts";
  
  export default {
    name: "RealTimeChart",
    props: {
      data: {
        type: Array,
        required: true,
      },
    },
    data() {
      return {
        chart: null,
        colorMap: {
          r: "#5470C6",   // 蓝色
          p: "#91CC75",   // 绿色
          yaw: "#FAC858", // 黄色
          x: "#EE6666",   // 红色
          y: "#73C0DE",   // 浅蓝色
        },
      };
    },
    watch: {
      data: {
        handler(newData) {
          this.updateChart(newData);
        },
        deep: true,
      },
    },
    mounted() {
      this.initChart();
    },
    methods: {
      initChart() {
        this.chart = echarts.init(this.$refs.chart);
        this.chart.setOption(this.getOption([]));
      },
      updateChart(data) {
        const option = this.getOption(data);
        this.chart.setOption(option);
      },
      getOption(data) {
        const keys = Object.keys(data[0] || {}).filter(
          (key) => key !== "time"
        );
  
        return {
          tooltip: {
            trigger: "axis",
          },
          legend: {
            data: keys,
            top: "5%",
            textStyle: {
              fontSize: 14,
            },
          },
          xAxis: {
            type: "category",
            boundaryGap: false,
            data: data.map((item) => item.time),
          },
          yAxis: {
            type: "value",
          },
          series: keys.map((key) => ({
            name: key,
            type: "line",
            showSymbol: false,
            smooth: true,
            data: data.map((item) => item[key]),
            lineStyle: {
              color: this.colorMap[key],
              width: 2,
            },
          })),
        };
      },
    },
  };
  </script>
  
  <style scoped>
  /* 限制图表容器的宽度和高度 */
  .chart-container {
    width: 800px; /* 限制宽度为600px */
    /* margin: 0 auto;  */
  }
  
  .chart {
    width: 100%;
    height: 400px;
  }
  </style>
  