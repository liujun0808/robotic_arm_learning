import matplotlib.pyplot as plt
import numpy as np
import matplotlib

# 设置中文字体支持
matplotlib.rcParams['font.sans-serif'] = ['SimHei']      # 中文字体
matplotlib.rcParams['axes.unicode_minus'] = False        # 正负号显示正常


def load_data(file_path):
    data = []
    with open(file_path, 'r') as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip().rstrip(',')
            if line:
                parts = line.split(',')
                try:
                    values = [float(x) if x.strip() != '' else 0.0 for x in parts]
                    if len(values) == 7:
                        data.append(values)
                    else:
                        print(f"[警告] 第 {line_num} 行列数不是 29，跳过。")
                except ValueError:
                    print(f"[错误] 第 {line_num} 行非法数字，跳过：{line}")
    return np.array(data)


def plot_single_group(data, title, color):
    num_channels = data.shape[1]
    rows, cols = 6, 5

    fig, axes = plt.subplots(rows, cols, figsize=(20, 12))
    axes = axes.flatten()

    for i in range(num_channels):
        ax = axes[i]
        ax.plot(data[:, i], label=f"Joint {i}", color=color)
        ax.set_title(f"Joint {i}")
        ax.set_xlabel("time step")
        ax.grid(True)

    # 删除多余子图
    for j in range(num_channels, len(axes)):
        fig.delaxes(axes[j])

    fig.suptitle(title, fontsize=18)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.show()  # 不阻塞
    plt.pause(0.1)  # 等待窗口显示


# def plot_three_windows(joint_file, speed_file, torque_file):
def plot_three_windows(joint_file):
    joint_data = load_data(joint_file)
    # speed_data = load_data(speed_file)
    # torque_data = load_data(torque_file)

    # if not (joint_data.shape == speed_data.shape == torque_data.shape):
    #     print("[错误] 三组数据行列数不一致！")
    #     return

    plot_single_group(joint_data, "Position", "blue")
    # plot_single_group(speed_data, "Velocity", "green")
    # plot_single_group(torque_data, "Torque", "red")

    print("所有窗口已打开，按 Ctrl+C 可退出程序或关闭任意窗口。")
    plt.show()  # 最后阻塞，等待所有图关闭



if __name__ == "__main__":
    plot_three_windows(
        joint_file="/home/lj/project/force_control/control_algorithm/data/q.txt"
        # joint_file="/home/lj/project/force_control/control_algorithm/data/q_err.txt"
        # speed_file="/home/lj/project/force_control/control_algorithm/data/q_err.txt",
        # torque_file="/home/lj/project/force_control/control_algorithm/data/q_err.txt"
    )

