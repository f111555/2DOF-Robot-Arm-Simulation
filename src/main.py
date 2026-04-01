import numpy as np
import matplotlib.pyplot as plt

def solve_2r_fk(target_angles, l1, l2):    

    t1 = np.radians(target_angles[0])
    t2 = np.radians(target_angles[1])

    t12=t1+t2


    x=l1*np.cos(t1)+l2*np.cos(t12)
    y=l1*np.sin(t1)+l2*np.sin(t12)

    return x,y


import numpy as np

def solve_2r_ik(target_point, l1, l2, config='up'):
    """
    增加 config 参数：'up' 代表肘部向上，'down' 代表肘部向下
    """
    x = target_point[0]
    y = target_point[1]
    dist_sq = x**2 + y**2
    
    # 1. 检查范围（工作空间限制）
    if dist_sq > (l1 + l2)**2 or dist_sq < abs(l1 - l2)**2:
        raise ValueError("目标点超出工作空间")

    # 2. 计算 cos_theta2
    cos_theta2 = (dist_sq - l1**2 - l2**2) / (2 * l1 * l2)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)

    # 3. 根据 config 选择 sin_theta2 的正负号
    if config == 'down':
        sin_theta2 = np.sqrt(1 - cos_theta2**2)  # 正值
    else:
        sin_theta2 = -np.sqrt(1 - cos_theta2**2) # 负值

    theta2 = np.arctan2(sin_theta2, cos_theta2)

    # 4. 计算 theta1
    k1 = l1 + l2 * cos_theta2
    k2 = l2 * sin_theta2
    theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)

    return theta1, theta2

def plot_robot_arm(theta1_rad, theta2_rad, l1, l2, target_point=None,color='b',label=None):
    """
    绘制机械臂单帧图像
    """
    # 1. 计算 FK 得到各关节点坐标 (用于绘图)
    # 注意：这里我们直接用弧度计算，不再调用 solve_2r_fk，
    # 因为我们需要中间关节 (x1, y1) 的坐标。
    x0, y0 = 0, 0
    x1 = l1 * np.cos(theta1_rad)
    y1 = l1 * np.sin(theta1_rad)
    # theta2 是相对角度
    x2 = x1 + l2 * np.cos(theta1_rad + theta2_rad)
    y2 = y1 + l2 * np.sin(theta1_rad + theta2_rad)

    # 2. 绘制机械臂连杆 (Red, O-shape marker, Solid line)
    plt.plot([x0, x1, x2], [y0, y1, y2],marker= 'o', linewidth=3, markersize=8, label=label,color=color)
    
    # 3. 绘制基座 (Black square)
    plt.plot(x0, y0, 'ko', markersize=10)

    # 4. (可选) 如果有目标点，画出目标点 (Blue cross)
    if target_point is not None:
        plt.plot(target_point[0], target_point[1], 'gx', markersize=10, markeredgewidth=3, label='Target')

    
  


def main_simulation():
    # 参数设置
    l1, l2 = 1.0, 0.8
    
    # 定义一段轨迹 (笛卡尔空间)
    # Y轴从 0.5 变到 -0.5，X轴固定在 1.0
    ##num_points = 50
    ##trajectory_y = np.linspace(0.5, -0.5, num_points)
    ##trajectory_x = np.ones(num_points) * 1.0
    
    
    num_points = 100 # 点多一点，圆才圆润
    ##t = np.linspace(0, 2 * np.pi, num_points)

# 建议参数：圆心在 (0.6, 0.4)，半径 0.4
# 注意：要确保圆上的所有点都在机械臂的触及范围 (L1+L2) 内
    ##trajectory_x = 0.6 + 0.4 * np.cos(t)
    ##trajectory_y = 0.4 + 0.4 * np.sin(t)
    t = np.linspace(0, 2 * np.pi, num_points)

# 原始心形公式
    raw_x = 16 * np.sin(t)**3
    raw_y = 13 * np.cos(t) - 5 * np.cos(2*t) - 2 * np.cos(3*t) - np.cos(4*t)

# 缩放并平移（缩小到约 0.5 左右的尺寸，并移动到机械臂前方）
    trajectory_x = 0.6 + raw_x * 0.03 
    trajectory_y = 0.5 + raw_y * 0.03
    # 创建画布
    plt.figure(figsize=(8, 8))
    
    # 开启交互模式 (关键：允许在 plt.show() 之前刷新图像)
    plt.ion() 

    print("开始动画演示...")

    history_x = []
    history_y = []
    
    for i in range(num_points):
        target = [trajectory_x[i], trajectory_y[i]]
        history_x.append(target[0])
        history_y.append(target[1])
        
        
        try:
            # 1. 计算两个姿态
            t1_up, t2_up = solve_2r_ik(target, l1, l2, config='up')
            t1_down, t2_down = solve_2r_ik(target, l1, l2, config='down')
            
            # 2. 清除上一帧内容
            plt.clf() 

            plt.plot(history_x, history_y, 'g--', alpha=0.6, label='Path')
            
            # 3. 绘制两个姿态
            plot_robot_arm(t1_up, t2_up, l1, l2, color='r', label='Elbow Up')
            plot_robot_arm(t1_down, t2_down, l1, l2, color='b', label='Elbow Down')
            
            plt.grid(True, linestyle='--', alpha=0.5)
            plt.axis('equal')
            limit = l1 + l2 + 0.2 # 稍微留一点余量
            plt.xlim(-0.5, limit) # 因为机械臂主要向右伸展，X轴从-0.5开始即可
            plt.ylim(-limit, limit)
            plt.title(f"Robot Arm IK: Elbow Up vs Down (Point {i})")
            plt.legend(loc='upper left') # 显示图例，区分红蓝
            #短暂停顿，控制动画速度 (单位：秒)
            plt.pause(0.01) 
            
        except ValueError as e:
            print(f"坐标点 {target} 报错: {e}")
            continue # 跳过不可达点

    # 动画结束，关闭交互模式并保持最终图像
    plt.ioff()
    print("动画结束。")
    plt.show() # 如果不加这句，窗口可能会在结束时闪退

if __name__ == "__main__":
    # 请确保此时 solve_2r_ik 和 solve_2r_fk 函数已经定义在上方
    main_simulation()

