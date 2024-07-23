import pandas as pd

import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.lines import Line2D
from collections import namedtuple
import math
import numpy as np

Sentiment = namedtuple("Sentiment",
                       ["positive", "neutral", "negative"])

# plt.rcParams.update({
#     "text.usetex": True,
#     "font.family": "sans-serif",
#     "font.sans-serif": "Helvetica",
# })

# From https://stackoverflow.com/questions/29765357/how-to-annotate-text-along-curved-lines
offset = [0,1]
def angle(rng,point,x_fig,y_fig):
    '''
    This will return the alignment angle
    '''
    # print("rng = ", rng)
    x_scale = x_fig /(rng[1] - rng[0])
    y_scale = y_fig /(rng[3] - rng[2])
    assert len(point) == 2, 'point is not properly defined'
    x = point[0]
    y = point[1]
    rot = ((math.atan2( y*y_scale , x*x_scale ))*180)/math.pi
    print(rot,point)
    return rot

def center_point(start,end,height_offset):
    '''
    This will return the location of text placement
    '''
    x_mid = (end[0]-start[0])/2
    y_mid = (end[1]-start[1])/2
    return[x_mid+height_offset[0],y_mid+height_offset[1]]


def compute_sentiment(df):

    df["positive"] = df["3"] + df["4"]
    df["neutral"] = df["2"]
    df["negative"] = df["0"] + df["1"]

def get_sentiment(question, df):

    question_df = df.loc[df["question"] == question]
    assert len(question_df) == 1

    positive = question_df["positive"].values[0]
    neutral = question_df["neutral"].values[0]
    negative = question_df["negative"].values[0]

    positive = round(positive, 1)
    neutral = round(neutral, 1)
    negative = round(negative, 1)

    return Sentiment(positive, neutral, negative)

def get_sentiment_colors(s1_data, s2_data):

    if s2_data.positive > s1_data.positive:
        positive_color = "green"
    elif s2_data.positive == s1_data.positive:
        positive_color = "black"
    else:
        positive_color = "red"

    if s2_data.negative < s1_data.negative:
        negative_color = "green"
    elif s2_data.negative == s1_data.negative:
        negative_color = "black"
    else:
        negative_color = "red"

    return positive_color, negative_color

def annotate_percent_improvement(ax, x1, x2, y1, y2, color,
                                 yshift=2):

    if y1 == 0:
        string = "$\\varnothing$"
    else:
        pct_change = round((y2 * 100.0) / y1, 2) - 100

        if math.floor(pct_change) == math.ceil(pct_change):
            string = "%d%%" % (pct_change)
        else:
            string = "%.1f%%" % (pct_change)

    dx = x2 - x1
    dy = y2 - y1
    rotn = np.degrees(np.arctan2(dy, dx))

    midx = (x1 + x2) / 2
    midy = (y1 + y2) / 2 + yshift

    ax.text(midx, midy, string, color=color, ha='center',
            va='bottom', fontsize=9,
            rotation=rotn, rotation_mode='anchor',
            transform_rotates_text=True)

def plot_slope_chart(ax, s1_df, questions,
                     line_spacing=3,
                     cross_spacing=3,
                     plot_system_text=True,
                     secondary_x=None):

    xticks = []
    start = 0
    for question in questions:

        end = start + line_spacing
        xticks.append((start + end) / 2)
        ax.axvline(x=start, color="grey", alpha=0.2)
        ax.axvline(x=end, color="grey", alpha=0.2)

        s1_data = get_sentiment("S1_%s" % (question), s1_df)
        s2_data = get_sentiment("S2_%s" % (question), s1_df)

        positive_color, negative_color = get_sentiment_colors(s1_data,
                                                              s2_data)

        ax.set_ylim([-5, 110])

        if plot_system_text:
            ax.text(start - 0.5, 112, "$S_1$")
            ax.text(start - 0.5 + line_spacing, 112, "$S_2$")

        ax.plot([start, end], [s1_data.positive, s2_data.positive],
                color=positive_color)
        annotate_percent_improvement(ax, start, end, s1_data.positive,
                                     s2_data.positive,
                                     positive_color)

        ax.plot([start, end], [s1_data.negative, s2_data.negative],
                color=negative_color, linestyle="dashed")
        annotate_percent_improvement(ax, start, end, s1_data.negative,
                                     s2_data.negative,
                                     negative_color)

        ax.set_xticks(xticks)
        ax.set_xticklabels([])

        if secondary_x is not None:
            secondary_x.set_xticks(xticks)
            secondary_x.set_xticklabels([])

        start = end + cross_spacing

    pass

def main_slope_chart(base_dir, jedai_df, jedai_ed_df):

    fig = plt.figure(figsize=(6.4, 3.7))
    gs = GridSpec(nrows=2, ncols=1)

    questions = ["PostQ7", "PostQ14", "PostQ15",
                 "PostQ4", "PostQ6", "PostQ11"]
    
    post_q_list = ['Q5','Q7','Q8','Q2','Q4','Q6']
    labels = post_q_list
    h_labels = ["H1", "H2", "H3", "H4", "H5", "H6"]

    ax1 = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[1, 0])
    secondary_x = ax2.secondary_xaxis(-0.4,
                                      functions=(lambda x:x, lambda x: x))

    plot_slope_chart(ax1, jedai_df, questions,
                     plot_system_text=True)
    plot_slope_chart(ax2, jedai_ed_df, questions,
                     plot_system_text=False,
                     secondary_x=secondary_x)

    ax1.set_title("Sentiment Change from $S_1=$JEDAI to $S_2=$JEDAI.Ed",
                  pad=20,x=0.5,y=0.95, usetex=True, fontsize=15)
    ax2.set_title("Sentiment Change from $S_1=$JEDAI.Ed to $S_2=$JEDAI",x=0.5,y=0.98, usetex=True, fontsize=15)
    ax2.set_xticklabels(h_labels, fontsize=15)
    secondary_x.set_xticklabels(labels, fontsize=12)
    ax2.set_xlabel("$\\uparrow$ Hypotheses/Post Survey Questions $\\downarrow$", fontsize=14, usetex=True)
    fig.text(0.03, 0.15, "User Sentiment \\% $(n=42)$", rotation=90,
             fontsize=14, usetex=True)

    pos_handles = [Line2D([], [], color="green"),
                   Line2D([], [], color="black"),
                   Line2D([], [], color="red")]
    fig.text(0.1, 1.07, " Positive:", fontsize=12,usetex=True)
    fig.legend(bbox_to_anchor=(0.86, 1.13),
               ncols=3,
               handles=pos_handles,
               labels=["Increase", "No Change", "Decrease"],
               frameon=False,
               fontsize=10)

    neg_handles = [Line2D([], [], color="red", linestyle="dashed"),
                   Line2D([], [], color="black", linestyle="dashed"),
                   Line2D([], [], color="green", linestyle="dashed"),]
    fig.text(0.1, 1.01, "Negative:", fontsize=12,usetex=True)
    fig.legend(bbox_to_anchor=(0.86, 1.08),
               ncols=3,
               handles=neg_handles,
               labels=["Increase", "No Change", "Decrease"],
               frameon=False,
               fontsize=10)

    fig.savefig("/%s/slope_chart.png" % (base_dir), bbox_inches="tight")
    fig.savefig("/%s/slope_chart.pdf" % (base_dir), bbox_inches="tight")

# Script that will plot changes from S1 -> S2.
if __name__ == "__main__":

    BASE_DIR = "/Users/daksh/tmp/ijcai24_results"

    JEDAI_SHEET = "%s/jedai_data.csv" % (BASE_DIR)
    JEDAI_ED_SHEET = "%s/jedai_ed_data.csv" % (BASE_DIR)

    jedai_df = pd.read_csv(JEDAI_SHEET)
    jedai_ed_df = pd.read_csv(JEDAI_ED_SHEET)

    compute_sentiment(jedai_df)
    compute_sentiment(jedai_ed_df)

    main_slope_chart(BASE_DIR, jedai_df, jedai_ed_df)