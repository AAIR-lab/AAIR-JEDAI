import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import statistics;

plt.rcParams.update({
    # "font.family": "sans-serif",
    # "font.sans-serif": "Helvetica",
    "font.size": 14,
})

if __name__ == "__main__":

    BASE_DIR = "/Users/Daksh/tmp/ijcai24_results"
    COLLATED_SHEET = "%s/collated.csv" % (BASE_DIR)

    df = pd.read_csv(COLLATED_SHEET)

    jedai_data = df.loc[df["start_system"] == "jedai"]["time"].values
    jedai_ed_data = df.loc[df["start_system"] == "jedai_ed"]["time"].values

    # https://stackoverflow.com/questions/11620914/how-do-i-remove-nan-values-from-a-numpy-array
    jedai_data = jedai_data[~np.isnan(jedai_data)]
    jedai_ed_data = jedai_ed_data[~np.isnan(jedai_ed_data)]

    # https://matplotlib.org/stable/gallery/statistics/violinplot.html#sphx-glr-gallery-statistics-violinplot-py
    fig = plt.figure(figsize=(6.4, 1.8))
    ax = fig.add_subplot(1, 1, 1)

    pos = [1, 2]
    ax.violinplot([jedai_data, jedai_ed_data], pos,
                  showmeans=False, showextrema=True, showmedians=False,
                  vert=False)
    medians = [statistics.median(jedai_data),
               statistics.median(jedai_ed_data)]
    means = [statistics.mean(jedai_data),
               statistics.mean(jedai_ed_data)]
    ax.set_yticks([1, 2])
    ax.set_yticklabels(["JEDAI", "JEDAI.Ed"],usetex=True)

    ax.scatter(means, pos, color="C0")
    ax.scatter(medians, pos, marker="x", color="C0")

    ax.set_xlabel("Time (s) Taken to Solve Test Problem",usetex=True)
    ax.spines['top'].set_visible(False)
    ax.spines['left'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.tick_params(axis="y" , length=0)

    ax.plot([means[0] - 5, means[1] + 5],
            [pos[0] + 0.1, pos[1] - 0.1],
            color="green",
            linestyle=":")

    ax.set_xticklabels(ax.get_xticklabels(), fontsize=12)
    ax.annotate("20\\% Improvement\nfor JEDAI.Ed",
                color="green",
                xy=(280, 1.5), xycoords='data',
                xytext=(500, 1.8), textcoords='data',
                size=14, va="center", ha="center",
                bbox=dict(boxstyle=None, fc="w", ec="w"),
                arrowprops=dict(arrowstyle="-|>",
                                connectionstyle="arc3,rad=-0.2",
                                fc="w", ec="green"),usetex=True)

    fig.savefig("%s/violinplot.png" % (BASE_DIR), bbox_inches="tight")
    fig.savefig("%s/violinplot.pdf" % (BASE_DIR), bbox_inches="tight")
