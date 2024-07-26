
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


# Create a DataFrame
df = pd.read_csv("jedai_survey.csv")
questions = df.columns
likert_data = {}
question_options = {'Q1pre': {0:'Not at all', 1:'Slightly well', 2:'Moderately well', 3:'Very well', 4:'Extremely well'}, 
                    'Q2pre': {4:'Daily', 3:'4-6 times a week', 2:'2-3 times a week', 1:'Once a week', 0:'Never'}, 
                    'Q4pre': {4:'Daily', 3:'4-6 times a week', 2:'2-3 times a week', 1:'Once a week', 0:'Never'}, 
                    'Q5pre': {0:'Not curious at all', 1:'Slightly curious', 2:'Moderately curious', 3:'Very curious', 4:'Extremely curious'}, 
                    'Q1': {0:'Not inclined at all', 1:'Slightly inclined', 2:'Moderately inclined', 3:'Very inclined', 4:'Extremely inclined'}, 
                    'Q2': {0:'Not helpful', 1:'Slightly helpful', 2:'Moderately helpful', 3:'Very helpful', 4:'Extremely helpful'}, 
                    'Q3': {0:'Not challenging at all', 1:'Slightly challenging', 2:'Moderately challenging', 3:'Very challenging', 4:'Extremely challenging'}, 
                    'Q4': {0:'Not intuitive at all', 1:'Slightly intuitive', 2:'Moderately Intuitive', 3:'Very Intuitive', 4:'Extremely Intuitive'}, 
                    'Q5': {0:'Not curious at all', 1:'Slightly curious', 2:'Moderately curious', 3:'Very curious', 4:'Extremely curious'},
                    'Q6': {0:'Not well at all', 1:'Slightly well', 2:'Moderately well', 3:'Very well', 4:'Extremely well'}}

legend_labels = {
    0: 'Not at all',
    1: 'Slight',
    2: 'Moderate',
    3: 'Very',
    4: 'Extremely'
}


for heading in questions:
    if heading in question_options.keys():
    # if heading in ["Q1"]:
        options = question_options[heading]
        options = {value:key for key,value in options.items()}
        data = list(df[heading].replace(options).reset_index(drop=True))
        new_data = []
        for i in legend_labels.keys():
            new_data.append(data.count(i))
        likert_data[heading] = pd.Series(list(new_data))

print(likert_data)

# Convert the dictionary to a DataFrame
# columns=['Strongly Disagree', 'Disagree', 'Neutral', 'Agree', 'Strongly Agree']
df = pd.DataFrame.from_dict(likert_data, orient='index', columns=[0,1,2,3,4])

# Transpose the DataFrame for plotting
df = df.T

# Create a stacked bar plot
fig, ax = plt.subplots(figsize=(10, 6))

categories = likert_data.keys()
bar_positions = np.arange(len(categories))
width = 0.7

bottom = np.zeros(len(categories))

likert_colors = ['firebrick','lightcoral','gainsboro','cornflowerblue', 'darkblue']
for i, question in enumerate(df.index):
    responses = df.loc[question]
    ax.bar(
        bar_positions,
        responses,
        width=width,
        label=legend_labels[i],
        bottom=bottom,
        color=likert_colors[i % len(likert_colors)] 
    )
    bottom += responses

# Customize the plot
# ax.set_title('Likert Scale Stacked Bar Plot')
ax.set_xlabel('Response')
ax.set_ylabel('Count')
ax.set_xticks(bar_positions)
ax.set_xticklabels(categories)
ax.set_ylim(0, 32)
ax.legend(title='Responses',loc='upper center', bbox_to_anchor=(0.5, 1.12), ncol=6)

# Show the plot
plt.tight_layout()
plt.savefig("plot.png")
