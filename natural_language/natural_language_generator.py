from collections import defaultdict


def get_natural_language_action(semantics, action_str):
    # assuming action_str includes grounded objects as parameters
    
    print("action being nlged = ",action_str)

    if semantics is None:
        return action_str

    assert action_str.startswith("(") and action_str.endswith(")")
    action_str = action_str[1:-1]
    action_parts = action_str.split(" ")
    action_name = action_parts[0]
    object_names = action_parts[1:]
    action_semantics = _get_action_semantics(semantics, action_name)
    action_display = action_semantics["display"]
    for param_details in action_semantics["parameters"]:
        action_display += " " + param_details["sentenceDisplay"]
        object_name = object_names[param_details["index"]]
        action_display += " '" + object_name + "'"

    return action_display


def get_natural_language_single_predicate(semantics, predicate_str, negated):
    if semantics is None:
        return predicate_str if not negated else f"not {predicate_str}"

    assert predicate_str.startswith("(") and predicate_str.endswith(")")
    naked_predicate_str = predicate_str[1:-1].strip()
    predicate_parts = naked_predicate_str.split(" ")
    predicate_name = predicate_parts[0]
    objects = predicate_parts[1:]
    predicate_semantics = _get_predicate_semantics(semantics, predicate_name)
    display = predicate_semantics["display"]
    subject_index = predicate_semantics["subjectIndex"]
    for i in range(len(objects)):
        if i != subject_index:
            display = display.replace(f"%%{i}%%", f"'{objects[i]}'")

    if negated:
        return f"'{objects[subject_index]}' is not {display}"

    return f"'{objects[subject_index]}' is {display}"


def get_natural_language_multiple_predicates(semantics, predicate_strs, predicate_negations):
    assert len(predicate_strs) == len(predicate_negations)
    if len(predicate_strs) == 0:
        return None
    if len(predicate_strs) == 1:
        return get_natural_language_single_predicate(semantics, predicate_strs[0], predicate_negations[0]) + "."
    if semantics is None:
        return ",".join(predicate_strs)
    for grounded_predicate in predicate_strs:
        assert grounded_predicate.startswith("(") and grounded_predicate.endswith(")")

    naked_predicate_strs = [p[1:-1].strip() for p in predicate_strs]
    predicate_negation_dict = dict(zip(naked_predicate_strs, predicate_negations))

    # extract all objects from predicates
    predicates_to_object_lists = _get_predicate_to_object_list_dict(naked_predicate_strs)
    # get sets of predicates that share subjects (called 'groups' for disambiguatory purposes)
    subjects_to_groups = _get_subject_to_group_dict(semantics, naked_predicate_strs, predicates_to_object_lists)

    # TODO order between and within groups by configuration
    # just sorting subjects alphanumerically for now (although subgroups are sorted by number of objects involved)
    ordered_subjects = sorted(subjects_to_groups.keys())

    # now the string will be stitched together
    nlp = ""
    for subject in ordered_subjects:
        nlp += _get_natural_language_predicates_for_single_subject_group(
            semantics,
            subject,
            subjects_to_groups[subject],
            predicates_to_object_lists,
            predicate_negation_dict
        ) + " "

    return nlp.strip()


def _get_natural_language_predicates_for_single_subject_group(
        semantics,
        subject,
        group,
        predicates_to_object_lists,
        predicate_negation_dict):
    # split the group into disjoint subgroups by involved objects (sometimes there will only be one subgroup)
    object_sets_to_subgroups = _get_object_set_to_subgroup_dict(group, predicates_to_object_lists)
    # sort subgroups by the number of objects involved (ascending)
    ordered_object_sets = sorted(object_sets_to_subgroups.keys(), key=len)
    ordered_group = []
    for object_set in ordered_object_sets:
        ordered_group += object_sets_to_subgroups[object_set]

    nlp = f"'{subject}' is"
    for predicate_index in range(len(ordered_group)):
        predicate = ordered_group[predicate_index]
        objects = predicates_to_object_lists[predicate]
        predicate_semantics = _get_predicate_semantics(semantics, predicate.split(" ")[0])
        subject_index = predicate_semantics["subjectIndex"]
        display = predicate_semantics["display"]
        for object_index in range(len(objects)):
            if object_index != subject_index:
                display = display.replace(f"%%{object_index}%%", f"'{objects[object_index]}'")
        if predicate_negation_dict[predicate]:
            display = f"not {display}"
        if predicate_index < len(ordered_group) - 1:
            nlp += f" {display},"
        elif predicate_index != 0:
            nlp += f" and {display}."
        else:
            nlp += f" {display}."

    return nlp


def _get_predicate_semantics(semantics, predicate_name):
    return [p for p in semantics["predicates"] if p["name"].lower() == predicate_name.lower()][0]


def _get_action_semantics(semantics, action_name):
    print("action_name = ",action_name)
    return [a for a in semantics["actions"] if a["name"].lower() == action_name.lower()][0]


def _get_subject_to_group_dict(semantics, predicates, predicates_to_object_lists):
    subjects_to_groups = defaultdict(lambda: [])
    for predicate in predicates:
        predicate_semantics = _get_predicate_semantics(semantics, predicate.split(" ")[0])
        subject_index = predicate_semantics["subjectIndex"]
        subject = predicates_to_object_lists[predicate][subject_index]
        subjects_to_groups[subject].append(predicate)

    return subjects_to_groups


def _get_object_set_to_subgroup_dict(predicates, predicates_to_object_lists):
    object_sets_to_groups = defaultdict(lambda: [])
    for predicate in predicates:
        object_combo = frozenset(predicates_to_object_lists[predicate])
        # add to the entry in the dict for that *combination* (set) of object parameters
        object_sets_to_groups[object_combo].append(predicate)
        object_sets_to_groups[object_combo].sort()

    return object_sets_to_groups


def _get_predicate_to_object_list_dict(naked_predicate_strs):
    predicates_to_object_lists = {}
    for predicate in naked_predicate_strs:
        predicate_parts = predicate.split(" ")
        object_combo = predicate_parts[1:]
        # create an entry in the dict for that *permutation* (list) of object parameters
        # there's probably a way to do all this without using that space, but I want to avoid working with substrings
        predicates_to_object_lists[predicate] = object_combo[:]
    
    return predicates_to_object_lists
