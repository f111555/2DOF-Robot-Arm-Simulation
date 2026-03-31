import numpy as np
import matplotlib.pyplot as plt

def solve_2r_fk(target_angles, l1, l2):    

    t1 = np.radians(target_angles[0])
    t2 = np.radians(target_angles[1])

    t12=t1+t2


    x=l1*np.cos(t1)+l2*np.cos(t12)
    y=l1*np.sin(t1)+l2*np.sin(t12)

    return x,y


def solve_2r_ik(target_point, l1, l2):
    """
    求解 2R 机械臂逆运动学
    :param target_point: numpy array, [x, y]
    :param l1: 第一段连杆长度
    :param l2: 第二段连杆长度
    :return: theta1, theta2 (弧度)
    """
    x = target_point[0]
    y = target_point[1]
    
    # 计算末端到原点距离的平方
    dist_sq = x**2 + y**2
    
    # 检查目标点是否在工作空间内（能否够得着）
    if dist_sq > (l1 + l2)**2 or dist_sq < abs(l1 - l2)**2:
        raise ValueError("目标点超出了机械臂的可达范围！")

    # 1. 计算 theta2 (这里取“下肘”解，即 theta2 为正)
    # cos_theta2 = (x^2 + y^2 - l1^2 - l2^2) / (2 * l1 * l2)
    cos_theta2 = (dist_sq - l1**2 - l2**2) / (2 * l1 * l2)
    
    # 限制 cos 值在 [-1, 1] 之间，防止浮点数精度带来的错误
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
    
    sin_theta2 = np.sqrt(1 - cos_theta2**2)
    theta2 = np.arctan2(sin_theta2, cos_theta2)

    # 2. 计算 theta1
    k1 = l1 + l2 * cos_theta2
    k2 = l2 * sin_theta2
    theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)

    return theta1, theta2

def plot_robot_arm(theta1_rad, theta2_rad, l1, l2, target_point=None):
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
    plt.plot([x0, x1, x2], [y0, y1, y2], 'ro-', linewidth=4, markersize=10, label='Arm')
    
    # 3. 绘制基座 (Black square)
    plt.plot(x0, y0, 'ks', markersize=12)

    # 4. (可选) 如果有目标点，画出目标点 (Blue cross)
    if target_point is not None:
        plt.plot(target_point[0], target_point[1], 'bx', markersize=15, markeredgewidth=3, label='Target')

    # 5. 美化界面 (关键步骤！)
    plt.grid(True, linestyle='--', alpha=0.5) # 显示网格
    plt.axis('equal') # 保持 XYZ 轴比例一致，防止机械臂变形
    
    # 设置固定的坐标轴范围，防止动画跳动
    limit = l1 + l2 + 0.5
    plt.xlim(-limit/2, limit) # 机械臂主要在第一、四象限
    plt.ylim(-limit, limit)
    plt.title("2-DOF Robot Arm IK Simulation")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")


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
            # 1. 求解 IK 得到角度 (弧度)
            theta1, theta2 = solve_2r_ik(target, l1, l2)
            
            # 2. 清除当前画布的旧图像
            plt.clf() 

            plt.plot(history_x, history_y, 'g--', alpha=0.6, label='Path')
            
            # 3. 绘制新的机械臂姿态
            plot_robot_arm(theta1, theta2, l1, l2, target_point=target)
            
            # 4. 短暂停顿，控制动画速度 (单位：秒)
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

