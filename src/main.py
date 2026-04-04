import numpy as np
import matplotlib.pyplot as plt
import time
from matplotlib.patches import Ellipse

class RobotArm2DOF:
    def __init__(self, l1=1.0, l2=0.8):
        """初始化机械臂的物理参数"""
        self.l1 = l1
        self.l2 = l2
        
    def solve_fk(self, theta1_rad, theta2_rad):
        """正运动学：输入弧度，输出末端坐标 (x, y)"""
        x = self.l1 * np.cos(theta1_rad) + self.l2 * np.cos(theta1_rad + theta2_rad)
        y = self.l1 * np.sin(theta1_rad) + self.l2 * np.sin(theta1_rad + theta2_rad)
        return x, y

    def solve_ik(self, target_point, config='up'):
        """逆运动学：输入目标点 [x, y]，输出弧度 (theta1, theta2)"""
        x, y = target_point
        dist_sq = x**2 + y**2
        
        # 检查是否可达
        if dist_sq > (self.l1 + self.l2)**2 or dist_sq < abs(self.l1 - self.l2)**2:
            raise ValueError(f"Target {target_point} out of reach")

        cos_t2 = (dist_sq - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        cos_t2 = np.clip(cos_t2, -1.0, 1.0)
        
        # 根据配置选择正负号
        sin_t2 = np.sqrt(1 - cos_t2**2) if config == 'up' else -np.sqrt(1 - cos_t2**2)
        theta2 = np.arctan2(sin_t2, cos_t2)
        
        k1 = self.l1 + self.l2 * cos_t2
        k2 = self.l2 * sin_t2
        theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)
        
        return theta1, theta2

    def plot_arm(self, t1, t2, ax, color='b', label=None, alpha=1.0):
        """绘制当前姿态"""
        # 计算关键点坐标
        x0, y0 = 0, 0
        x1 = self.l1 * np.cos(t1)
        y1 = self.l1 * np.sin(t1)
        x2, y2 = self.solve_fk(t1,t2)
        
        # 绘图
        ax.plot([x0, x1, x2], [y0, y1, y2], 'o-', color=color, 
                 linewidth=3, markersize=6, label=label, alpha=alpha)
    
    def get_jacobian(self, t1, t2):
        """计算当前位姿下的雅可比矩阵"""
        j11 = -self.l1 * np.sin(t1) - self.l2 * np.sin(t1 + t2)
        j12 = -self.l2 * np.sin(t1 + t2)
        j21 =  self.l1 * np.cos(t1) + self.l2 * np.cos(t1 + t2)
        j22 =  self.l2 * np.cos(t1 + t2)
        
        return np.array([[j11, j12], 
                         [j21, j22]])
    @staticmethod
    def quintic_scaling(t, T):
        """五次多项式平滑算法：不依赖具体的机械臂实例"""
        tau = np.clip(t / T, 0, 1)
        # 位置因子 s
        s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
        # 速度因子 v (s 的导数)
        v = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / T
        # 加速度因子 a (v 的导数)
        a = (60 * tau - 180 * tau**2 + 120 * tau**3) / (T**2)
        return s, v, a



def main_simulation():
    # 1. 实例化机器人对象
    my_robot = RobotArm2DOF(l1=1.0, l2=0.8)
    
    # 2. 轨迹参数设置
    total_time = 5.0  # 5秒走完
    fps = 30          # 每秒30帧
    num_frames = int(total_time * fps)
    
    # 3. 预生成心形轨迹 (Path)
    t_geom = np.linspace(0, 2 * np.pi, num_frames)
    heart_x = 0.6 + (16 * np.sin(t_geom)**3) * 0.03
    heart_y = 0.5 + (13 * np.cos(t_geom) - 5 * np.cos(2*t_geom) - 2 * np.cos(3*t_geom) - np.cos(4*t_geom)) * 0.03
    
    # 4. 准备画布
    fig = plt.figure(figsize=(12, 6)) # 拓宽画布
    ax_robot = fig.add_subplot(1, 2, 1) # 左边画机器人
    ax_curve = fig.add_subplot(1, 2, 2) # 右边画曲线
    plt.ion()
    
    # 用于记录速度和加速度历史，方便绘图
    hist_x, hist_y = [], []
    time_hist, vel_hist, acc_hist = [], [], []
    det_hist=[]
    last_t1 = 0.0
    last_t2 = 0.0
    
    for i in range(3, 0, -1): # 3秒倒计时够用了
        print(f"倒计时 {i} 秒后开始动画...")
        time.sleep(1)
    
    print("开始播放动画...")

    
    # 4. 主循环：使用 num_frames 驱动
    for i in range(num_frames):
        t_curr = i / fps
       
        # 使用类内部的静态方法计算平滑因子
        s, v, a = RobotArm2DOF.quintic_scaling(t_curr, total_time)
        
        # 查表找到对应的轨迹点
        idx = int(s * (num_frames - 1))
        target = [heart_x[idx], heart_y[idx]]
        
        # 记录已走过的路径
        hist_x.append(target[0])
        hist_y.append(target[1])
        time_hist.append(t_curr)
        vel_hist.append(v)
        acc_hist.append(a)

        
        
        try:
            t1_up, t2_up = my_robot.solve_ik(target, config='up')
            t1_down, t2_down = my_robot.solve_ik(target, config='down')
            J1 = my_robot.get_jacobian(t1_up, t2_up)
            det1 = np.linalg.det(J1)
            det_hist.append(det1)

            # --- 刷新绘图 ---
            ax_robot.cla()
            ax_curve.cla()
            
            if i > 0:
                dt = 1.0 / fps
                d_q = np.array([t1_up - last_t1, t2_up - last_t2]) / dt
                v_end = J1 @ d_q
                vx, vy = v_end[0], v_end[1]
                
                # 获取当前末端坐标
                curr_pos = my_robot.solve_fk(t1_up, t2_up)
                
                # 画箭头：scale 越小箭头越长，width 控制粗细
                ax_robot.quiver(curr_pos[0], curr_pos[1], vx, vy, 
                                color='orange', scale=10, width=0.015, 
                                zorder=5, label='Velocity')
                
                A = J1 @ J1.T
            
                # 2. 计算特征值和特征向量
                vals, vecs = np.linalg.eigh(A)
            
                # 3. 计算椭圆参数
                # 宽度和高度对应特征值的平方根 (放大 0.2 倍方便观察)
                width = np.sqrt(vals[1]) * 0.4
                height = np.sqrt(vals[0]) * 0.4
                # 计算旋转角度 (弧度转角度)
                angle = np.degrees(np.arctan2(vecs[1, 1], vecs[0, 1]))
            
                # 4. 创建并添加椭圆对象
                ell = Ellipse(xy=curr_pos, width=width, height=height, angle=angle,
                          edgecolor='purple', fc='None', lw=2, alpha=0.6, label='Manipulability')
                ax_robot.add_patch(ell)
        
           

        

           
           
            
            
            # 如果你想看一眼数值（建议每 10 帧打印一次，否则太快了）
            if i % 10 == 0:
                print(f"Frame {i}: Jacobian Det1 = {det1:.4f}")
            
            

        

            # 左图：机器人仿真
            ax_robot.plot(hist_x, hist_y, 'g--', alpha=0.4, label='Heart Path')
            my_robot.plot_arm(t1_up, t2_up, ax=ax_robot, color='r', label='Elbow Up')
            my_robot.plot_arm(t1_down, t2_down, ax=ax_robot, color='b', label='Elbow Down', alpha=0.3)
            
            ax_robot.set_title("Robot Space")
            ax_robot.axis('equal')
            ax_robot.set_xlim(-0.5, 2.0); ax_robot.set_ylim(-1.5, 1.5)
            ax_robot.grid(True, alpha=0.3)
            ax_robot.legend(loc='upper left')

            # 右图：动力学监控 
            ax_curve.plot(time_hist, vel_hist, 'b-', label='Velocity')
            ax_curve.plot(time_hist, acc_hist, 'r-', label='Acceleration')
            ax_curve.plot(time_hist, det_hist, 'g:', label='Determinant ')
            
            ax_curve.set_title("Dynamics & Dexterity Monitor")
            ax_curve.set_title("Dynamics Monitor (Quintic)")
            ax_curve.set_xlabel("Time (s)")
            ax_curve.set_xlim(0, total_time)
            ax_curve.set_ylim(-0.2, 1.5) # 根据 total_time 调整范围
            ax_curve.grid(True, alpha=0.3)
            ax_curve.legend()

            last_t1, last_t2 = t1_up, t2_up

            plt.pause(0.001)

        except ValueError:
            continue
    # 动画结束，关闭交互模式并保持最终图像
    plt.ioff()
    print("动画结束。")
    plt.show() # 如果不加这句，窗口可能会在结束时闪退

if __name__ == "__main__":
    # 请确保此时 solve_2r_ik 和 solve_2r_fk 函数已经定义在上方
    main_simulation()

