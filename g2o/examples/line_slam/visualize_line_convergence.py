#!/usr/bin/env python3
"""
可视化线特征优化收敛过程 (2D版本)

用法:
  python visualize_line_convergence.py line_convergence_plucker.txt
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

def parse_convergence_file(filename):
    """解析收敛历史文件"""
    gt_lines = []
    iterations = []
    chi2_values = []

    with open(filename, 'r') as f:
        lines = f.readlines()

    i = 0
    while i < len(lines):
        line = lines[i].strip()

        if line.startswith('#') or not line:
            i += 1
            continue

        if line.startswith('GT'):
            num_lines = int(line.split()[1])
            i += 1
            for _ in range(num_lines):
                parts = lines[i].strip().split()
                p1 = np.array([float(parts[1]), float(parts[2]), float(parts[3])])
                p2 = np.array([float(parts[4]), float(parts[5]), float(parts[6])])
                gt_lines.append((p1, p2))
                i += 1

        elif line.startswith('ITERATIONS'):
            i += 1
            while i < len(lines):
                line = lines[i].strip()
                if line.startswith('ITER'):
                    parts = line.split()
                    chi2 = float(parts[2])
                    chi2_values.append(chi2)

                    iter_lines = []
                    i += 1
                    while i < len(lines) and not lines[i].strip().startswith('ITER'):
                        if lines[i].strip() and not lines[i].strip().startswith('#'):
                            parts = lines[i].strip().split()
                            if len(parts) >= 7:
                                p1 = np.array([float(parts[1]), float(parts[2]), float(parts[3])])
                                p2 = np.array([float(parts[4]), float(parts[5]), float(parts[6])])
                                iter_lines.append((p1, p2))
                        i += 1
                    iterations.append(iter_lines)
                else:
                    i += 1
        else:
            i += 1

    return gt_lines, iterations, chi2_values


def visualize_convergence_2d(gt_lines, iterations, chi2_values):
    """2D可视化收敛过程（三视图）"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # 视图配置: (x_idx, y_idx, xlabel, ylabel, title)
    views = [
        (0, 1, 'X', 'Y', 'Top View (X-Y)'),
        (0, 2, 'X', 'Z', 'Side View (X-Z)'),
        (1, 2, 'Y', 'Z', 'Front View (Y-Z)'),
    ]

    num_iters = len(iterations)
    colors = plt.cm.coolwarm(np.linspace(0, 1, num_iters))

    for ax, (xi, yi, xl, yl, title) in zip(axes.flat[:3], views):
        # 绘制真值线（绿色粗线）
        for lid, (p1, p2) in enumerate(gt_lines):
            ax.plot([p1[xi], p2[xi]], [p1[yi], p2[yi]],
                    'g-', linewidth=3, label='GT' if lid == 0 else '')

        # 绘制每次迭代的线
        for iter_idx, iter_lines in enumerate(iterations):
            alpha = 0.2 + 0.8 * (iter_idx / max(num_iters - 1, 1))
            for lid, (p1, p2) in enumerate(iter_lines):
                label = f'Iter {iter_idx}' if lid == 0 and iter_idx in [0, num_iters-1] else ''
                ax.plot([p1[xi], p2[xi]], [p1[yi], p2[yi]],
                        color=colors[iter_idx], alpha=alpha, linewidth=1)

        ax.set_xlabel(xl)
        ax.set_ylabel(yl)
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        ax.axis('equal')

    # Chi2曲线
    ax_chi2 = axes[1, 1]
    ax_chi2.semilogy(range(len(chi2_values)), chi2_values, 'b-o', markersize=4)
    ax_chi2.set_xlabel('Iteration')
    ax_chi2.set_ylabel('Chi2')
    ax_chi2.set_title('Chi2 Convergence')
    ax_chi2.grid(True, alpha=0.3)

    # 添加图例
    axes[0, 0].legend(loc='upper right')

    plt.tight_layout()
    plt.savefig('line_convergence.png', dpi=150)
    print("图像已保存到 line_convergence.png")
    plt.show()


def visualize_with_slider_2d(gt_lines, iterations, chi2_values):
    """带滑块的交互式2D可视化"""
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    plt.subplots_adjust(bottom=0.2)

    views = [
        (0, 1, 'X', 'Y', 'Top View (X-Y)'),
        (0, 2, 'X', 'Z', 'Side View (X-Z)'),
        (1, 2, 'Y', 'Z', 'Front View (Y-Z)'),
    ]

    ax_slider = plt.axes([0.2, 0.05, 0.6, 0.03])
    slider = Slider(ax_slider, 'Iteration', 0, len(iterations)-1, valinit=0, valstep=1)

    def update(val):
        iter_idx = int(slider.val)

        for ax, (xi, yi, xl, yl, title) in zip(axes, views):
            ax.clear()

            # 绘制真值线（绿色）
            for p1, p2 in gt_lines:
                ax.plot([p1[xi], p2[xi]], [p1[yi], p2[yi]], 'g-', linewidth=3)

            # 绘制当前迭代的线（红色）
            if iter_idx < len(iterations):
                for p1, p2 in iterations[iter_idx]:
                    ax.plot([p1[xi], p2[xi]], [p1[yi], p2[yi]], 'r-', linewidth=2)

            ax.set_xlabel(xl)
            ax.set_ylabel(yl)
            ax.set_title(title)
            ax.grid(True, alpha=0.3)
            ax.axis('equal')

        chi2_str = f'{chi2_values[iter_idx]:.2f}' if iter_idx < len(chi2_values) else 'N/A'
        fig.suptitle(f'Iteration {iter_idx}, Chi2 = {chi2_str}')
        fig.canvas.draw_idle()

    slider.on_changed(update)
    update(0)
    plt.show()


def compute_line_errors(gt_lines, iterations):
    """计算每次迭代的线误差"""
    errors = []
    for iter_lines in iterations:
        iter_error = 0
        for lid, (est_p1, est_p2) in enumerate(iter_lines):
            gt_p1, gt_p2 = gt_lines[lid]
            gt_dir = (gt_p2 - gt_p1)
            gt_len = np.linalg.norm(gt_dir)
            gt_dir = gt_dir / gt_len

            est_mid = (est_p1 + est_p2) / 2
            gt_mid = (gt_p1 + gt_p2) / 2
            v = est_mid - gt_mid
            dist = np.linalg.norm(v - np.dot(v, gt_dir) * gt_dir)
            iter_error += dist
        errors.append(iter_error / len(iter_lines))
    return errors


def print_per_line_error(gt_lines, iterations):
    """打印每条线的误差"""
    print("\n每条线的收敛情况 (初始 -> 最终):")
    print("-" * 60)

    initial_lines = iterations[0]
    final_lines = iterations[-1]

    for lid in range(len(gt_lines)):
        gt_p1, gt_p2 = gt_lines[lid]
        gt_mid = (gt_p1 + gt_p2) / 2

        init_p1, init_p2 = initial_lines[lid]
        init_mid = (init_p1 + init_p2) / 2
        init_err = np.linalg.norm(init_mid - gt_mid)

        final_p1, final_p2 = final_lines[lid]
        final_mid = (final_p1 + final_p2) / 2
        final_err = np.linalg.norm(final_mid - gt_mid)

        status = "OK" if final_err < init_err else "WORSE"
        print(f"  Line {lid:2d}: {init_err:6.3f}m -> {final_err:6.3f}m  [{status}]")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        filename = 'line_convergence_plucker.txt'
    else:
        filename = sys.argv[1]

    print(f"读取文件: {filename}")
    gt_lines, iterations, chi2_values = parse_convergence_file(filename)

    print(f"真值线数量: {len(gt_lines)}")
    print(f"迭代次数: {len(iterations)}")
    if chi2_values:
        print(f"Chi2: {chi2_values[0]:.2f} -> {chi2_values[-1]:.2f}")

    # 计算线误差
    line_errors = compute_line_errors(gt_lines, iterations)
    print(f"平均线误差: {line_errors[0]:.4f}m -> {line_errors[-1]:.4f}m")

    # 打印每条线的误差
    print_per_line_error(gt_lines, iterations)

    # 选择可视化模式
    print("\n选择可视化模式:")
    print("  1. 静态图（所有迭代叠加）")
    print("  2. 交互式滑块")

    try:
        choice = input("请选择 [1/2]: ").strip()
    except:
        choice = '1'

    if choice == '2':
        visualize_with_slider_2d(gt_lines, iterations, chi2_values)
    else:
        visualize_convergence_2d(gt_lines, iterations, chi2_values)
