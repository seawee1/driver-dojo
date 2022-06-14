import matplotlib.pyplot as plt
from driver_dojo.examples.utils import plot_normal, plot_uniform, plot_categorical


def plot_intersection():
    fig, axes = plt.subplots(1, 7, figsize=(7 * 3.5, 3.5))

    plot_categorical(axes[0], "num_roads", [0.3, 0.5, 0.2], [3, 4, 5])
    plot_uniform(axes[1], "center_dist", 15.0, 30.0)
    plot_uniform(axes[2], "road_length", 40.0, 79.0)
    plot_uniform(axes[3], "curvature_outer", -0.02, 0.02)
    plot_uniform(axes[4], "curvature_inner", -0.02, 0.02)
    plot_categorical(axes[5], "num_lanes", [0.45, 0.45, 0.1], [1, 2, 3])
    plot_uniform(axes[6], "angle_offset", -125.0, 125.0)
    fig.tight_layout()
    plt.savefig("intersection_dist.pdf")
    plt.show()


def plot_highway_entry():
    fig, axes = plt.subplots(1, 5, figsize=(5 * 3.5, 3.5))
    plot_categorical(axes[0], "num_lanes", [0.1, 0.4, 0.4, 0.1], [1, 2, 3, 4])
    plot_uniform(axes[1], "curvature_a", -0.01, 0.01)
    plot_uniform(axes[2], "curvature_b", -0.005, 0.005)
    plot_uniform(axes[3], "curvature_c", -0.03, 0.03)
    plot_uniform(axes[4], "curvature_entry", -0.03, 0.03)
    fig.tight_layout()
    plt.savefig("highway_entry_dist.pdf")
    plt.show()


def plot_highway_exit():
    fig, axes = plt.subplots(1, 5, figsize=(5 * 3.5, 3.5))
    plot_categorical(axes[0], "num_lanes", [0.1, 0.4, 0.4, 0.1], [1, 2, 3, 4])
    plot_uniform(axes[1], "curvature_a", -0.03, 0.03)
    plot_uniform(axes[2], "curvature_b", -0.005, 0.005)
    plot_uniform(axes[3], "curvature_c", -0.01, 0.01)
    plot_uniform(axes[4], "curvature_exit", -0.03, 0.03)
    fig.tight_layout()
    plt.savefig("highway_exit_dist.pdf")
    plt.show()


def plot_highway_drive():
    fig, axes = plt.subplots(1, 5, figsize=(5 * 3.5, 3.5))
    plot_categorical(axes[0], "num_lanes", [0.1, 0.4, 0.4, 0.1], [1, 2, 3, 4])
    plot_uniform(axes[1], "curvature_a", -0.015, 0.015)
    plot_uniform(axes[2], "curvature_b", -0.015, 0.015)
    plot_uniform(axes[3], "curvature_c", -0.015, 0.015)
    plot_uniform(axes[4], "curvature_d", -0.015, 0.015)
    fig.tight_layout()
    plt.savefig("highway_drive_dist.pdf")
    plt.show()


def plot_roundabout():
    fig, axes = plt.subplots(1, 8, figsize=(8 * 3.5, 3.5))
    plot_uniform(axes[0], "radius", 20.0, 40.0)
    plot_categorical(axes[1], "num_roads", [0.25, 0.25, 0.25, 0.25], [2, 3, 4, 5])
    plot_categorical(axes[2], "num_lanes", [0.5, 0.5], [1, 2])
    plot_normal(axes[3], "angle_offset", 0.0, 11.5, None)
    plot_normal(axes[4], "angle_rot", 0.0, 8.5, None)
    plot_uniform(axes[5], "road_length", 60.0, 100.0)
    plot_uniform(axes[6], "curvature", -0.0002, 0.0002)
    plot_uniform(axes[7], "squeeze", 0.8, 1.2)
    fig.tight_layout()
    plt.savefig("roundabout_dist.pdf")
    plt.show()


if __name__ == '__main__':
    plot_intersection()
    plot_highway_drive()
    plot_highway_entry()
    plot_highway_exit()
    plot_roundabout()
