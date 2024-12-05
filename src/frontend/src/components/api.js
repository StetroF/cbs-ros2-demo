export function calculateCubicBezier(points, t) {
    if (points.length !== 4) {
        throw new Error("对于三阶贝塞尔曲线，必须提供4个控制点。");
    }

    const [P0, P1, P2, P3] = points; // 分别是起点、控制点1、控制点2和终点
    const x = Math.pow(1 - t, 3) * P0[1] + 
              3 * Math.pow(1 - t, 2) * t * P1[1] + 
              3 * (1 - t) * Math.pow(t, 2) * P2[1] + 
              Math.pow(t, 3) * P3[1]; // 计算 x 坐标

    const y = Math.pow(1 - t, 3) * P0[0] + 
              3 * Math.pow(1 - t, 2) * t * P1[0] + 
              3 * (1 - t) * Math.pow(t, 2) * P2[0] + 
              Math.pow(t, 3) * P3[0]; // 计算 y 坐标

    return [y, x]; // 返回点的格式 [lat, lng]
}
export default calculateCubicBezier;