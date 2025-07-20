import matplotlib.pyplot as plt
import numpy as np

def load_data(file_path):
    """
    从 .txt 文件中读取数据，返回 numpy 数组。
    要求每行 29 个逗号分隔的 float。
    """
    data = []
    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip().rstrip(',')
            if line:
                values = [float(x) for x in line.split(',')]
                if len(values) == 29:
                    data.append(values)
                else:
                    print(f"[警告] 跳过异常行（列数={len(values)}）：{line}")
    return np.array(data)

def plot_joint_motor_speed(joint_file, motor_file, speed_file):
    # 加载三组数据
    joint_data = load_data(joint_file)
    motor_data = load_data(motor_file)
    speed_data = load_data(speed_file)

    # 检查一致性
    if not (joint_data.shape == motor_data.shape == speed_data.shape):
        print("[错误] 三个文件行列数不一致，请检查数据格式。")
        return

    num_motors = joint_data.shape[1]
    rows = (num_motors + 4) // 5
    cols = 5

    fig, axes = plt.subplots(rows, cols, figsize=(18, 4 * rows))
    axes = axes.flatten()

    for i in range(num_motors):
        ax = axes[i]
        ax.plot(joint_data[:, i], label="关节位置", color='blue')
        ax.plot(motor_data[:, i], label="电机输出", color='red', linestyle='--')
        ax.plot(speed_data[:, i], label="电机速度", color='green', linestyle=':')
        ax.set_title(f"Motor {i}")
        ax.set_xlabel("时间步")
        ax.set_ylabel("值")
        ax.grid(True)
        ax.legend(fontsize="small")

    # 删除多余的子图
    for j in range(num_motors, len(axes)):
        fig.delaxes(axes[j])

    plt.tight_layout()
    plt.suptitle("电机通道三线对比图：关节位置、电机输出、电机速度", fontsize=16, y=1.02)
    plt.show()

# 示例调用
if __name__ == "__main__":
    plot_joint_motor_speed(
        joint_file="/home/lj/humanrobot/unitree_g1_control/contorl_algorithm/data/q_cur.txt",
        motor_file="/home/lj/humanrobot/unitree_g1_control/contorl_algorithm/data/tau_out.txt",
        speed_file="/home/lj/humanrobot/unitree_g1_control/contorl_algorithm/data/q_dot_cur.txt",
    )
