# 此文件用于上位机接收下位机通过串口发送来的激光雷达坐标
# 然后绘制成XY轨迹图

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import re
import time

plt.rcParams['font.sans-serif'] = ['SimHei']  # 用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False    # 用来正常显示负号

# -------------------------- 全局配置 --------------------------
# 串口参数（
SERIAL_PORT = "COM6"
BAUDRATE = 115200     # 波特率，需与下位机一致
TIMEOUT = 0.1         # 串口超时时间

# 绘图参数
PLOT_X_RANGE = [-0.5, 8]  # X轴范围（单位：米，根据雷达量程调整）
PLOT_Y_RANGE = [-0.5, 12]  # Y轴范围
UPDATE_INTERVAL = 10    # 绘图刷新间隔（毫秒）

# 全局变量（线程安全）
lock = threading.Lock()
x_data = []  # 存储X坐标
y_data = []  # 存储Y坐标
serial_conn = None  # 串口连接对象
is_running = True   # 程序运行标志

# -------------------------- 串口相关函数 --------------------------
def list_serial_ports():
    """列出所有可用串口（方便调试）"""
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

def init_serial():
    """初始化串口连接"""
    global serial_conn
    try:
        serial_conn = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUDRATE,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=TIMEOUT
        )
        print(f"成功连接串口：{SERIAL_PORT}")
        return True
    except Exception as e:
        print(f"串口初始化失败：{e}")
        print(f"可用串口列表：{list_serial_ports()}")
        return False

def parse_lidar_data(raw_data):
    """解析串口原始数据，提取XY坐标
    假设下位机发送格式："X:1.23,Y:4.56\r\n"（可根据实际格式修改）
    """
    # 正则匹配X、Y数值（兼容浮点数/整数）
    pattern = r"X:([-+]?\d+\.?\d*),Y:([-+]?\d+\.?\d*)"
    match = re.search(pattern, raw_data.decode('utf-8', errors='ignore'))
    if match:
        x = float(match.group(1))
        y = float(match.group(2))
        return (x, y)
    return None

def serial_read_thread():
    """串口数据读取线程（避免阻塞绘图）"""
    global is_running
    while is_running:
        if serial_conn and serial_conn.is_open:
            try:
                # 读取一行数据（下位机需按行发送，结尾加\r\n）
                raw_data = serial_conn.readline()
                if raw_data:
                    coord = parse_lidar_data(raw_data)
                    if coord:
                        x, y = coord
                        # 线程安全更新数据
                        with lock:
                            x_data.append(x)
                            y_data.append(y)
                            # [修改] 取消限制，保留所有历史点
                            # if len(x_data) > 1000:
                            #     x_data.pop(0)
                            #     y_data.pop(0)
            except Exception as e:
                print(f"数据读取/解析错误：{e}")
        time.sleep(0.001)  # 降低CPU占用

# -------------------------- 绘图相关函数 --------------------------
def init_plot():
    """初始化绘图窗口"""
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(PLOT_X_RANGE)
    ax.set_ylim(PLOT_Y_RANGE)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Ladar XY Position")
    ax.grid(True, linestyle='--', alpha=0.7)
    # 初始化轨迹线（空数据）
    # b-为连线，b.为散点
    line, = ax.plot([], [], 'b-', markersize=2, label="历史点")
    scatter, = ax.plot([], [], 'r.', markersize=4, label="最新点")
    ax.legend()
    return fig, ax, line, scatter

def update_plot(frame, line, scatter):
    """实时更新绘图（matplotlib动画回调）"""
    with lock:
        # 深拷贝数据，避免绘图时数据被修改
        x = x_data.copy()
        y = y_data.copy()
    # 更新轨迹线和最新点
    line.set_data(x, y)
    if x and y:
        # [Fix] set_data 需要传入序列(list)，即使只有一个点也要用 [] 包裹
        scatter.set_data([x[-1]], [y[-1]])
    return line, scatter

# -------------------------- 主函数 --------------------------
def main():
    global is_running
    # 1. 初始化串口
    if not init_serial():
        return
    # 2. 启动串口读取线程
    read_thread = threading.Thread(target=serial_read_thread, daemon=True)
    read_thread.start()
    # 3. 初始化绘图
    fig, ax, line, scatter = init_plot()
    # 4. 创建动画（实时更新）
    ani = animation.FuncAnimation(
        fig,
        update_plot,
        fargs=(line, scatter),
        interval=UPDATE_INTERVAL,
        blit=True,  # 优化绘图速度
        cache_frame_data=False
    )
    # 5. 显示绘图窗口（阻塞直到关闭窗口）
    try:
        plt.show()
    finally:
        # 程序退出时清理资源
        is_running = False
        read_thread.join(timeout=1)
        if serial_conn and serial_conn.is_open:
            serial_conn.close()
            print("串口已关闭")

if __name__ == "__main__":
    main()

