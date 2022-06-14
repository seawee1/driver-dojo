import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
import math
import argparse

from driver_dojo.examples.utils import plot_normal, plot_uniform, plot_const


def check_dist_string(dist):
    import string
    is_string = False
    for char in string.ascii_letters:
        if char in dist:
            is_string = True

    if 'normal' in dist:
        return 'normal'
    elif 'uniform' in dist:
        return 'uniform'
    elif is_string:
        return 'string'
    else:
        return 'const'


if __name__ == "__main__":
    """For this to work correctly, the specifications inside the distribution file has to be grouped 
    into blocks general/car-follow/lane-change/junction models, separated by an empty line.
    """
    parser = argparse.ArgumentParser(description="Plot the vType distributions.")
    parser.add_argument(
        "dist_path",
        type=str,
        help="Path to the .txt file for createVehTypeDistributions.py.",
    )
    parser.add_argument(
        "--cols",
        type=int,
        default=6,
        help="Number of columns to plot.",
    )
    parser.add_argument(
        "--subplot_size",
        type=int,
        default=3,
    )
    args = parser.parse_args()

    sns.set_style('white')

    category_names = ["General", "Car-following Model", "Lane-change Model", "Junction Model"]
    # Open the file
    with open(args.dist_path) as f:
        lines = f.read()
        categories = []
        for category in lines.split("\n\n"):
            entry = category.split('\n')
            entry = [e for e in entry if e.strip() != '']
            categories.append(entry)

    for c, category in enumerate(categories):
        plot_types = [check_dist_string(entry.split(';')[1].strip()) for entry in category if entry != '']
        num_string_plots = 0
        for t in plot_types:
            if t == 'string':
                num_string_plots += 1
        rows = math.ceil((len(category) - num_string_plots) / args.cols)

        fig, axes = plt.subplots(rows, args.cols, figsize=(args.cols * args.subplot_size, rows * args.subplot_size))
        # fig.suptitle(category_names[c], size='xx-large')
        col_i = 0
        row_i = 0
        for entry in category:
            if entry == '':
                continue
            split = entry.split(';')
            name = split[0].strip()
            dist = split[1].strip()
            clip = None

            if check_dist_string(dist) == 'string':
                print(category_names[c], name, dist)
                continue

            if len(split) == 3:
                clip = split[2].strip()
                clip = clip.replace('[', '')
                clip = clip.replace(']', '')
                clip = clip.split(',')
                clip_min = float(clip[0].strip())
                clip_max = float(clip[1].strip())
                clip = [clip_min, clip_max]

            if len(axes.shape) == 1:
                ax = axes[col_i]
            else:
                ax = axes[row_i, col_i]
            col_i = (col_i + 1) % args.cols
            if col_i == 0:
                row_i += 1

            if check_dist_string(dist) == 'normal':
                dist = dist[7:]
                dist = dist.replace(")", "")
                dist = dist.split(',')
                mu = float(dist[0].strip())
                std = float(dist[1].strip())
                plot_normal(ax, name, mu, std, clip)
            elif check_dist_string(dist) == 'uniform':
                dist = dist[8:]
                dist = dist.replace(")", "")
                dist = dist.split(',')
                mi = float(dist[0].strip())
                ma = float(dist[1].strip())
                plot_uniform(ax, name, mi, ma)
            else:
                val = float(dist.strip())
                plot_const(ax, name, val)

        while col_i < args.cols and row_i < rows:
            if len(axes.shape) == 1:
                ax = axes[col_i]
            else:
                ax = axes[row_i, col_i]
            fig.delaxes(ax)
            col_i += 1
        fig.tight_layout()
        plt.savefig(category_names[c].lower().split(' ')[0] + '.pdf')
        plt.show()
