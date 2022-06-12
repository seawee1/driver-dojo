import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
import argparse


if __name__ == "__main__":
    """For this to work correctly, the specifications inside the distribution file has to be groupped into blocks general/carfollow/lanechange/[
    """
    parser = argparse.ArgumentParser(description="Plot the vType distributions.")
    parser.add_argument(
        "dist_path",
        type=str,
        help="Path to the .txt file for createvTypeDistributions.py.",
    )
    plt.style.use("classic")
    sns.set()

    # fig, axes = plt.subplots(4, 4)

    data = pd.read_csv("general.txt", sep=";", header=None)
    data.columns = ["name", "dist", "clip"]

    for index, row in data.iterrows():
        name = row["name"]
        normal = row["dist"].find("normal")
        if normal == 1:
            dist = row["dist"]
            dist = dist.replace("(", "")
            dist = dist.replace(")", "")
            dist = dist.replace("normal", "")
            dist_list = dist.partition(",")
            mu = float(dist_list[0])
            std = float(dist_list[2])

            clip = row["clip"]
            clip = clip.replace("[", "")
            clip = clip.replace("]", "")
            clip_list = clip.partition(",")
            lower = float(clip_list[0])
            upper = float(clip_list[2])

            s = np.random.normal(mu, std, 100000)

            # for index, value in enumerate(s):
            #     if value < lower:
            #         s[index] = lower
            #     elif value > upper:
            #         s[index] = upper

            x = pd.Series(s, name=name)
            ax = sns.distplot(x, hist=True)
            plt.text(x=mu + 0.01, y=0.3, s=f"μ={mu}\nσ={std}")
            plt.text(x=upper + 0.01, y=0.05, s=f"max={upper}")
            plt.text(x=lower + 0.01, y=0.05, s=f"min={lower}")

            plt.axvline(x=upper, color="gray", ls="--", lw=2, label=f"max={upper}")

            plt.axvline(x=lower, color="gray", ls="--", lw=2, label=f"min={lower}")

            plt.axvline(x=mu, color="gray", ls="--", lw=2, label=f"μ={mu}")

            plt.axvline(x=std, color="gray", ls="--", lw=0, label=f"σ={std}")

            plt.show()

        uniform = row["dist"].find("uniform")
        if uniform == 1:
            dist = row["dist"]
            dist = dist.replace("(", "")
            dist = dist.replace(")", "")
            dist = dist.replace("uniform", "")
            dist_list = dist.partition(",")
            low = float(dist_list[0])
            high = float(dist_list[2])

            s = np.random.uniform(low, high, 100000)

            x = pd.Series(s, name=name)
            ax = sns.distplot(x, hist=True)
            # plt.text(x=s.mean(), y=0.3, s=f"low={low}\nhigh={high}")
            plt.text(x=high + 0.005, y=0.1, s=f"high={high}")
            plt.text(x=low + 0.005, y=0.1, s=f"low={low}")

            plt.axvline(x=high, color="gray", ls="--", lw=2, label=f"high={high}")

            plt.axvline(x=low, color="gray", ls="--", lw=2, label=f"low={low}")

            plt.axvline(x=s.mean(), color="gray", ls="--", lw=2, label=f"μ={s.mean()}")

            plt.show()
