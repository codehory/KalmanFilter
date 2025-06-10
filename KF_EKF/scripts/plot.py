import numpy as np
import matplotlib.pyplot as plt

# 读取数据文件
data = np.loadtxt('../data/output.txt')  # 你保存为 data.txt（或自定义文件名）

# 解包各列
time = (data[:, 0] - data[0, 0]) * 1e-6  # 转换为相对时间（秒）
x, y, vx, vy = data[:, 1], data[:, 2], data[:, 3], data[:, 4]
gt_x, gt_y, gt_vx, gt_vy = data[:, 5], data[:, 6], data[:, 7], data[:, 8]

# ======================= 计算误差 =======================
pos_err = np.sqrt((x - gt_x)**2 + (y - gt_y)**2)
vel_err = np.sqrt((vx - gt_vx)**2 + (vy - gt_vy)**2)

# ======================= 作图 =======================
plt.figure(figsize=(15, 10))

# ---- 位置轨迹图 ----
plt.subplot(2, 2, 1)
plt.plot(x, y, label='Filtered', marker='o', markersize=3)
plt.plot(gt_x, gt_y, label='GT', marker='x', linestyle='--')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Trajectory: Filter vs GT')
plt.legend()
plt.grid(True)

# ---- 速度图 ----
plt.subplot(2, 2, 2)
plt.plot(time, vx, label='vx (Filtered)')
plt.plot(time, gt_vx, label='vx (GT)', linestyle='--')
plt.plot(time, vy, label='vy (Filtered)')
plt.plot(time, gt_vy, label='vy (GT)', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Velocity')
plt.title('Velocity Comparison')
plt.legend()
plt.grid(True)

# ---- 位置误差图 ----
plt.subplot(2, 2, 3)
plt.plot(time, pos_err, label='Position Error')
plt.xlabel('Time (s)')
plt.ylabel('Position Error (m)')
plt.title('Position Error over Time')
plt.grid(True)

# ---- 速度误差图 ----
plt.subplot(2, 2, 4)
plt.plot(time, vel_err, label='Velocity Error')
plt.xlabel('Time (s)')
plt.ylabel('Velocity Error (m/s)')
plt.title('Velocity Error over Time')
plt.grid(True)

plt.tight_layout()
plt.show()

