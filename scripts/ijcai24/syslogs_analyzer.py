import argparse
import csv
import shutil
import pandas as pd
import re
from datetime import datetime

"""
Installation instructions
==========================
python3 -m venv ijcai24_env
source ijcai24_env/bin/activate
pip3 install pip --upgrade
pip3 install pandas
"""

scale_1 = {
    "0": "Not inclined at all",
    "1": "Slightly inclined",
    "2": "Moderately inclined",
    "3": "Very inclined",
    "4": "Extremely inclined"
}

scale_2 = {
    "0": "Not helpful",
    "1": "Slightly helpful",
    "2": "Moderately helpful",
    "3": "Very helpful",
    "4": "Extremely helpful"
}

scale_3 = {
    "0": "Not challenging at all",
    "1": "Slightly challenging",
    "2": "Moderately challenging",
    "3": "Very challenging",
    "4": "Extremely challenging"
}

scale_4 = {
    "0": "Not intuitive at all",
    "1": "Slightly intuitive",
    "2": "Moderately intuitive",
    "3": "Very intuitive",
    "4": "Extremely intuitive"
}

scale_5 = {
    "0": "Highly decreased",
    "1": "Slightly decreased",
    "2": "Neither increased or decreased",
    "3": "Slightly increased",
    "4": "Highly increased"
}

scale_6 = {
    "0": "Not well at all",
    "1": "Slightly well",
    "2": "Moderately well",
    "3": "Very well",
    "4": "Extremely well"
}

scale_7 = {
    "0": "Strongly disagree",
    "1": "Disagree",
    "2": "Neither agree or disagree",
    "3": "Agree",
    "4": "Strongly agree"
}

def likert_to_text(text, scale):

    return scale.get(text, text)

def text_to_likert(text, likert_converter):

    text = text.lower()
    for value, text_options in likert_converter.items():

        if text in text_options:
            return value

    return text

def create_likert_converter(scales):

    likert_converter = {}

    for scale in scales:
        for key, value in scale.items():
            text_options = likert_converter.setdefault(key, [])
            text_options.append(value.lower())

    return likert_converter

def get_survey_log_info(user_id, jedai_logs_dir,
                       jedai_ed_logs_dir):

    if user_id.startswith("j_"):

        return "%s/%s" % (jedai_logs_dir, user_id), "jedai", "jedai_ed"
    else:

        assert user_id.startswith("d_")
        return "%s/%s" % (jedai_ed_logs_dir, user_id), "jedai_ed", "jedai"

def get_signup_sheet(base_dir):

    return "%s/signups.csv" % (base_dir)

def parse_log_file(log_filename):

    fh = open(log_filename, "r")

    TIME_PATTERN = "(\w|\W)* "\
        "(?P<time>\d{2}:\d{2}:\d{2} (AM|PM))" \
        " (\w|\W)*"

    start_time = None
    end_time = None
    total_bad_actions = 0
    total_hints = 0

    test_problem_found = False
    for line in fh:

        line = line.strip()
        if "training_area_problem_Test" in line:

            assert "modules/Cafe World" in line
            matcher = re.match(TIME_PATTERN, line)
            start_time = datetime.strptime(matcher.group("time"),
                                                    "%I:%M:%S %p")
            test_problem_found = True
            break
    assert test_problem_found

    for line in fh:

        line = line.strip()

        if "hinting" in line:
            total_hints += 1
        elif "ACTION_BAD" in line:
            total_bad_actions += 1
        elif "plan_evaluation_True" in line:

            matcher = re.match(TIME_PATTERN, line)
            end_time = datetime.strptime(matcher.group("time"),
                                     "%I:%M:%S %p")
            break

    try:
        elapsed_time = end_time - start_time
        return elapsed_time.total_seconds(), total_hints, total_bad_actions
    except TypeError as e:

        assert end_time is None
        return float("nan"), total_hints, total_bad_actions

def get_pre_data(user_id, pre_df):

    pre_id = re.sub("(d_|j_)", "", user_id)
    pre_data = pre_df.loc[pre_df["Random Number"] == pre_id]
    assert len(pre_data) == 1

    return pre_data

def get_system_data(user_id, jedai_df, jedai_ed_df, s1, s2):

    post_jedai_data = jedai_df.loc[jedai_df["Q16"] == user_id]
    post_jedai_ed_data = jedai_ed_df.loc[jedai_ed_df["Q16"] == user_id]

    if s1 == "jedai":

        s1_df = post_jedai_data
        s2_df = post_jedai_ed_data
    else:
        s1_df = post_jedai_ed_data
        s2_df = post_jedai_data

    if len(s1_df) == 0:

        assert len(s2_df) == 2
        s1_df = pd.DataFrame(data=[s2_df.values[0]], columns=s2_df.columns)
        s2_df = pd.DataFrame(data=[s2_df.values[1]], columns=s2_df.columns)
    elif len(s2_df) == 0:

        assert len(s1_df) == 2
        s2_df = pd.DataFrame(data=[s1_df.values[1]], columns=s1_df.columns)
        s1_df = pd.DataFrame(data=[s1_df.values[0]], columns=s1_df.columns)

    return s1_df, s2_df

def get_header(pre_questions, post_questions):

    header = ["user_id", "major", "expertise", "start_system", "time",
          "total_hints", "total_bad_actions"]
    for pre_question in pre_questions:

        header.append("pre_%s" % (pre_question))

    for post_question in post_questions:
        header.append("s1_post_%s" % (post_question))
        header.append("s2_post_%s" % (post_question))

    return header

def get_value(df, key, default="", converter_func=lambda x: x):
    
    try:
        value = converter_func(df[key].values[0])
    except Exception:
        value = default

    return value

def create_data(user_id, signup_df, s1, pre_df, s1_df, s2_df,
                solve_time, num_hints, num_bad_actions,
                pre_questions, post_questions, likert_converter):

    user_df = signup_df.loc[signup_df["User ID"] == user_id]
    assert len(user_df) == 1

    pre_df = get_pre_data(user_id, pre_df)

    data = {
        "user_id": user_id,
        "major": user_df["Major"].values[0],
        "expertise": user_df["Programming Expertise Level"].values[0],
        "start_system": s1,
        "time": solve_time,
        "total_hints": num_hints,
        "total_bad_actions": num_bad_actions
    }

    for pre_question in pre_questions:

        data["pre_%s" % (pre_question)] = get_value(pre_df, pre_question,
            lambda x: text_to_likert(x, likert_converter))

    for post_question in post_questions:

        converter_func = lambda x: x
        # converter_func = lambda x: text_to_likert(x, likert_converter)

        data["s1_post_%s" % (post_question)] = get_value(s1_df, post_question,
            default="",
            converter_func=converter_func)
        data["s2_post_%s" % (post_question)] = get_value(s2_df, post_question,
             default="",
             converter_func=converter_func)

    return data

if __name__ == "__main__":

    BASE_DIR = "/tmp/ijcai24_results"
    LOGS_DIR = "%s/logs" % (BASE_DIR)

    JEDAI_LOGS_DIR = "%s/jedai" % (LOGS_DIR)
    JEDAI_ED_LOGS_DIR = "%s/jedai_ed" % (LOGS_DIR)

    SIGNUP_SHEET = "%s/signups.csv" % (BASE_DIR)
    PRE_SHEET = "%s/pre.csv" % (BASE_DIR)
    POST_JEDAI_SHEET = "%s/post_jedai.csv" % (BASE_DIR)
    POST_JEDAI_ED_SHEET = "%s/post_jedai.ed.csv" % (BASE_DIR)

    PRE_QUESTIONS = ["Q1", "Q2", "Q4", "Q5", "Q8", "Q9"]
    POST_QUESTIONS = ["Q2", "Q4", "Q5", "Q6", "Q7", "Q11",
                      "Q14", "Q15", "Q3", "Q12"]

    signup_df = pd.read_csv(SIGNUP_SHEET, dtype=str)
    pre_df = pd.read_csv(PRE_SHEET, dtype=str)
    jedai_df = pd.read_csv(POST_JEDAI_SHEET, dtype=str)
    jedai_ed_df = pd.read_csv(POST_JEDAI_ED_SHEET, dtype=str)

    likert_converter = create_likert_converter([scale_1, scale_2, scale_3,
                                              scale_4, scale_5, scale_6,
                                              scale_7])

    header = get_header(PRE_QUESTIONS, POST_QUESTIONS)
    output_fh = open("%s/collated.csv" % (BASE_DIR), "w")
    dict_writer = csv.DictWriter(output_fh, header)
    dict_writer.writeheader()

    for user_id in signup_df["User ID"]:

        # No more processing once the signup sheet ends.
        if str(user_id) == "nan":
            break

        try:
            log_filename, s1, s2 = get_survey_log_info(user_id,
                                                        JEDAI_LOGS_DIR,
                                                        JEDAI_ED_LOGS_DIR)
            solve_time, num_hints, num_bad_actions = \
                parse_log_file(log_filename)

            pre_data = get_pre_data(user_id, pre_df)
            s1_df, s2_df = get_system_data(user_id, jedai_df, jedai_ed_df,
                                               s1, s2)

            row_data = create_data(user_id, signup_df, s1, pre_df, s1_df, s2_df,
                        solve_time, num_hints, num_bad_actions,
                        PRE_QUESTIONS, POST_QUESTIONS, likert_converter)
            dict_writer.writerow(row_data)

        except Exception as e:
            print("Warning: Skipping user %s due to exception %s" % (user_id,
                                                                     e))
    pass