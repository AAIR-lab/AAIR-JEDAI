import matplotlib.pyplot as plt
import pandas as pd
import numpy as np


# Sample DataFrames with the same index and column names
BASE_DIR = "/Users/Daksh/tmp/ijcai24_results/"
ANNOTATION_FONTSIZE=10
jedai = pd.read_csv("/Users/Daksh/tmp/ijcai24_results/jedai_data.csv")
jedai_ed = pd.read_csv("/Users/Daksh/tmp/ijcai24_results/jedai_ed_data.csv")

questions = [7, 14, 15, 4, 6, 11]
questions = ["S1_PostQ%s" % (x) for x in questions]
hypotheses = {question: "H%u" % (i+1)
              for i, question in enumerate(questions)}

df2 = jedai.loc[jedai["question"].isin(questions)]
df1 = jedai_ed.loc[jedai["question"].isin(questions)]

df1["Hypothesis"] = df1["question"].apply(lambda x: hypotheses[x])
df2["Hypothesis"] = df2["question"].apply(lambda x: hypotheses[x])

df1 = df1.sort_values(by="Hypothesis")
df2 = df2.sort_values(by="Hypothesis")

df1 = df1.reset_index()
df2 = df2.reset_index()

from matplotlib.gridspec import GridSpec
plt.rcParams.update({
    "text.usetex": False,
    # "font.family": "sans-serif",
    # "font.sans-serif": "Helvetica",
    "font.size": 14,
})



space = 3
# Function to adjust spacing for every pair of lines
def adjust_spacing1(index):
    # Adding extra space after every pair of lines
    return space * index

def adjust_spacing2(index):
    # Adding extra space after every pair of lines
    return space * index +1

def preprocess(df):
  df['n1'] = -1*df['1']
  df['n2'] = -1*df['0']
  df['p1'] = df['3']
  df['p2'] = df['4']

preprocess(df1)
preprocess(df2)

# Applying the function to get adjusted positions
df1['Adjusted Position'] = df1.index.to_series().apply(adjust_spacing1)
df2['Adjusted Position'] = df2.index.to_series().apply(adjust_spacing2)

# Create the divergent lollipop chart
fig = plt.figure(figsize=(15, 4))
gs = GridSpec(nrows=3, ncols=4, wspace=0.4)

ax = fig.add_subplot(gs[0:3, 1:4])
plt.title('Percentage Responses to Survey Questions for JEDAI and JEDAI.Ed',
          fontsize=16, x=0.5, y=0.95,usetex=True)

# ======================== Begin creating donuts
ax2 = fig.add_subplot(gs[1, 0])
category_values = [3, 20, 6, 10, 3]
colors=['#820000', '#E00000', '#A3A3A3', "#00B856", "#006630"]

wedges, label_texts, autotexts = ax2.pie(category_values,
        radius=3,
        colors=colors,
        wedgeprops={'alpha': 0.8},
        autopct=lambda pct: str(int(round(pct))) + "%",
        pctdistance=0.65,
        labels=["", "", "Neutral\nResponses", "", ""],
        labeldistance=1.1,
        textprops=dict(color="black", fontsize=11,
                       weight="bold"))

plt.rc({"usetex":True})
# Set the text colors of all the percentages to white.
for text in autotexts:
    text.set_color("white")

# Set the label text for neutral responses to normal
# and reduce the font size.
label_texts[2].set_weight("normal")
label_texts[2].set_fontsize(11)

# Setup custom pct distances
autotexts[1].set_position((autotexts[1].get_position()[0] + 0.4,
    autotexts[1].get_position()[1] - 0.5))
autotexts[3].set_position((autotexts[3].get_position()[0] - 0.1,
    autotexts[3].get_position()[1] + 0.3))

ax2.text(0, 3.5, "Pre-survey Q5\nHow easy is robot programming?",
         ha="center", fontsize=14.5,usetex=True)

# my_circle=plt.Circle( (0,0), 0.7, color='white')
# ax2.add_artist(my_circle)



# ======================== Begin creating percentage plot
darkgreen = '#006630'  # Darker green
lightgreen = '#00B856'  # Lighter green

darkred = '#820000'  # Darker red
lightred = '#E00000'  # Lighter red


# Plotting lines from df1
for i, row in df1.iterrows():
    # linestyle = '-' if i % 2 == 0 else '--'
    linestyle = "-"

    ax.hlines(y=row['Adjusted Position'], xmin=row['n1'], xmax=0, color=lightred, alpha=0.8, linewidth=2, linestyles=linestyle)
    ax.hlines(y=row['Adjusted Position'], xmin=row['n1'] + row['n2'], xmax=row['n1'], color=darkred, alpha=0.8, linewidth=2, linestyles=linestyle)

    # Scatter point at the end of each pair of lines
    ax.scatter(row['n1'] + row['n2'], row['Adjusted Position'], color=darkred, alpha=1, s=50, zorder=5)

    ax.hlines(y=row['Adjusted Position'], xmin=0, xmax=row['p1'], color=lightgreen, alpha=0.8, linewidth=2, linestyles=linestyle)
    ax.hlines(y=row['Adjusted Position'], xmin=row['p1'], xmax=row['p1'] + row['p2'], color=darkgreen, alpha=0.8, linewidth=2, linestyles=linestyle)

    # Scatter point at the end of each pair of lines
    ax.scatter(row['p1'] + row['p2'], row['Adjusted Position'], color=darkgreen, alpha=1, s=50, zorder=5)
    annotation_text = f"{abs(row['p1'] + row['p2']):.1f}%"
    ax.annotate(annotation_text, xy=(row['p1'] + row['p2'],
                                     row['Adjusted Position']), xytext=(8, 0),
                                    textcoords='offset points', ha='left', va='center', color='#006630', fontsize=ANNOTATION_FONTSIZE)

    annotation_text = f"{abs(row['n1'] + row['n2']):.1f}%"
    ax.annotate(annotation_text, xy=(row['n1'] + row['n2'],
                                     row['Adjusted Position']),xytext=(-8, 0),
                                    textcoords='offset points', ha='right', va='center', color='#820000', fontsize=ANNOTATION_FONTSIZE)

# Plotting lines from df2 with extra spacing
for i, row in df2.iterrows():
    ax.add_patch(plt.Rectangle((-100, row['Adjusted Position'] - 1.5), 100 + ax.get_xlim()[1] - ax.get_xlim()[0], 2,
                               color='#EBEBEB', alpha=0.4))

    # linestyle = '-' if i % 2 == 0 else '--'
    linestyle = "--"
    ax.hlines(y=row['Adjusted Position'] , xmin=row['n1'], xmax=0, color=lightred, alpha=0.8, linewidth=2, linestyles=linestyle)
    ax.hlines(y=row['Adjusted Position'] , xmin=row['n1'] + row['n2'], xmax=row['n1'], color=darkred, alpha=0.8, linewidth=2, linestyles=linestyle)

    # Scatter point at the end of each pair of lines
    ax.scatter(row['n1'] + row['n2'], row['Adjusted Position'] , color=darkred, alpha=1, s=50, zorder=5)

    ax.hlines(y=row['Adjusted Position'] , xmin=0, xmax=row['p1'], color=lightgreen, alpha=0.8, linewidth=2, linestyles=linestyle)
    ax.hlines(y=row['Adjusted Position'] , xmin=row['p1'], xmax=row['p1'] + row['p2'], color=darkgreen, alpha=0.8, linewidth=2, linestyles=linestyle)

    # Scatter point at the end of each pair of lines

    ax.scatter(row['p1'] + row['p2'], row['Adjusted Position'], color=darkgreen, alpha=1, s=50, zorder=5)
    annotation_text = f"{abs(row['p1'] + row['p2']):.1f}%"
    ax.annotate(annotation_text, xy=(row['p1'] + row['p2'],
                                     row['Adjusted Position'] - 0.1),xytext=(8, 0),
                                    textcoords='offset points', ha='left', va='center', color='#006630', fontsize=ANNOTATION_FONTSIZE)

    annotation_text = f"{abs(row['n1'] + row['n2']):.1f}%"
    ax.annotate(annotation_text, xy=(row['n1'] + row['n2'],
                                     row['Adjusted Position'] - 0.1),xytext=(-8, 0),
                                    textcoords='offset points', ha='right', va='center', color='#820000', fontsize=ANNOTATION_FONTSIZE)


# Adjusting y-ticks to match the adjusted positions
# plt.yticks(np.concatenate((df1['Adjusted Position'], df2['Adjusted Position'] + 1)),
#            np.concatenate((df1.index, df2.index)))

# Middle line
ax.vlines(x=0, ymin=-1, ymax=df2['Adjusted Position'].max() + 1, color='grey', linewidth=1, linestyles='dashdot')
# ax.legend()

# Add labels and title
# plt.xlabel('Deviation')
# plt.ylabel('Question')
# plt.grid(linestyle='--', alpha=0.6)

# Show plot
# plt.axis('off')
# plt.xaxis.set_lim()
import matplotlib.lines as mlines
import matplotlib.patches as mpatches

plt.rc('text', usetex=True)
lightred_patch = mpatches.Patch(color=lightred, label='Slightly')
darkred_patch = mpatches.Patch(color=darkred, label='Not At All')
lightgreen_patch = mpatches.Patch(color=lightgreen, label='Very')
darkgreen_patch = mpatches.Patch(color=darkgreen, label='Extremely')


# ["Not at all", "Slightly","Moderately","Very", "Extremely"]

line1 = mlines.Line2D([], [], color='black', linestyle='--', label='JEDAI')
line2 = mlines.Line2D([], [], color='black', linestyle='-', label='JEDAI.Ed')

ax.legend(handles=[darkred_patch, lightred_patch,lightgreen_patch,darkgreen_patch,line1, line2],
          ncol=6, bbox_to_anchor=(0.9, 1.23),
          frameon=False)

ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)
ax.spines['bottom'].set_visible(False)
ax.spines['left'].set_visible(True)
ax.set_xlim([-40, 110])
ax.set_xticks([])

hypothesis_list = [hypotheses[question] for question in questions]
ax.set_yticks(df2['Adjusted Position'] - 0.5, df1['question'])
ax.set_yticklabels(hypothesis_list,usetex=True)
ax.set_ylabel("Hypotheses",usetex=True)

ax_secondary_y = ax.secondary_yaxis("right")
ax_secondary_y.set_yticks(df2['Adjusted Position'] - 0.5, df1['question'])
yticklabels = ax_secondary_y.get_yticklabels()

post_q_list = ['Q5','Q7','Q8','Q2','Q4','Q6']

for i, yticklabel in enumerate(yticklabels):
    # yticklabels[i] = yticklabel.get_text().replace("S1_Post", "")
    yticklabels[i] = post_q_list[i]
    
ax_secondary_y.set_yticklabels(yticklabels,usetex=True)
# ax.tick_params(axis=u'both', which=u'both',length=0)
ax_secondary_y.set_ylabel("Post Survey Questions",usetex=True)
# ax.axis('off')




# ======================== Begin creating timing plots.


# Save the plots.
plt.savefig("%s/responses.png" % (BASE_DIR), bbox_inches="tight")
plt.savefig("%s/responses.pdf" % (BASE_DIR), bbox_inches="tight")



