import serial
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# --- 配置 ---
SERIAL_PORT = 'COM4'  # !!! 修改为你的串口号
BAUD_RATE = 115200
# 地图尺寸 (与 C++ 代码一致)
MAP_WIDTH_M = 5 * 1.2
MAP_HEIGHT_M = 6 * 1.2
CELL_SIZE_M = 1.2

# --- 全局变量 ---
robot_history = {'x': [], 'y': []}
robot_current_pose = {'x': 0, 'y': 0, 'yaw': 0}
target_pos = {'x': 0, 'y': 0}
planned_path = []

# --- 设置绘图 ---
fig, ax = plt.subplots(figsize=(8, 10))
ax.set_aspect('equal')
ax.set_xlim(0, MAP_WIDTH_M)
ax.set_ylim(0, MAP_HEIGHT_M)
ax.set_title('Robot Path Tracking Visualization')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.grid(True)

# 绘制地图网格
for i in range(6): # 5列
    ax.axvline(x=i * CELL_SIZE_M, color='lightgray', linestyle='--')
for i in range(7): # 6行
    ax.axhline(y=i * CELL_SIZE_M, color='lightgray', linestyle='--')

# 绘图对象
path_line, = ax.plot([], [], 'g--', label='Planned Path')
trajectory_line, = ax.plot([], [], 'b-', label='Robot Trajectory')
robot_dot, = ax.plot([], [], 'ro', markersize=8, label='Robot')
robot_arrow, = ax.plot([], [], 'r-', linewidth=2) # 代表机器人朝向
target_dot, = ax.plot([], [], 'c*', markersize=10, label='Target')

ax.legend()

# --- 串口读取 ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
except serial.SerialException as e:
    print(f"Error opening serial port {SERIAL_PORT}: {e}")
    exit()

def read_serial_data():
    """从串口读取并解析一行JSON数据"""
    global planned_path, robot_current_pose, target_pos
    if not ser.is_open:
        return
    try:
        line = ser.readline().decode('utf-8').strip()
        if line:
            data = json.loads(line)
            if 'path' in data:
                planned_path = data['path']
                print(f"Received path with {len(planned_path)} waypoints.")
            elif 'robot' in data:
                robot_current_pose = data['robot']
                target_pos = data['target']
                robot_history['x'].append(robot_current_pose['x'])
                robot_history['y'].append(robot_current_pose['y'])
    except (json.JSONDecodeError, UnicodeDecodeError) as e:
        # 忽略无法解析的行
        pass
    except Exception as e:
        print(f"An unexpected error occurred: {e}")


def update_plot(frame):
    """更新绘图的回调函数"""
    read_serial_data()

    # 更新规划路径
    if planned_path:
        path_x, path_y = zip(*planned_path)
        path_line.set_data(path_x, path_y)

    # 更新机器人轨迹
    if robot_history['x']:
        trajectory_line.set_data(robot_history['x'], robot_history['y'])

    # 更新机器人当前位置和朝向
    x, y, yaw = robot_current_pose['x'], robot_current_pose['y'], robot_current_pose['yaw']
    robot_dot.set_data(x, y)
    
    # 计算朝向箭头的终点
    arrow_len = 0.5 # 箭头长度
    arrow_end_x = x + arrow_len * np.cos(yaw)
    arrow_end_y = y + arrow_len * np.sin(yaw)
    robot_arrow.set_data([x, arrow_end_x], [y, arrow_end_y])

    # 更新目标点
    target_dot.set_data(target_pos['x'], target_pos['y'])

    return path_line, trajectory_line, robot_dot, robot_arrow, target_dot

# --- 运行动画 ---
ani = animation.FuncAnimation(fig, update_plot, blit=True, interval=50, cache_frame_data=False)

print(f"Listening on {SERIAL_PORT}... Close the plot window to stop.")
plt.show()

# --- 清理 ---
if ser.is_open:
    ser.close()
    print("Serial port closed.")

