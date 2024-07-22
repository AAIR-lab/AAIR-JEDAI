IS_VM = False

PYTHON_2_PATH = "/usr/bin/python2.7"
VAL_PATH = "~/VAL/build/linux64/Release/bin/Validate"
DOCUMENTS_PATH = "./media/documents"
STATIC_FILES_PATH = "./roblocks/static/roblocks"
STATIC_IMAGES_PATH = STATIC_FILES_PATH + "/images"
GOAL_IMAGE_STATIC_FILE = STATIC_IMAGES_PATH + "/goal.png"
GOAL_IMAGE_NAME_IN_MODULES = "img.png"
INIT_IMAGE_STATIC_FILE = STATIC_IMAGES_PATH + "/init.png"
INIT_IMAGE_NAME_IN_MODULES = "init_img.png"
PREBUILT_DOMAIN_FOLDERS = "./modules"
SEMANTICS_FILE = "/semantics.json"
DOMAIN_DOCUMENT_FILE = DOCUMENTS_PATH + "/domainD.pddl"
PROBLEM_DOCUMENT_FILE = DOCUMENTS_PATH + "/problemP.pddl"
LATTICE_DOCUMENT_FILE = DOCUMENTS_PATH + "/lattice.yaml"
ENV_DOCUMENT_FILE = DOCUMENTS_PATH + "/env.dae"
DOMAIN_TEMPL_DOCUMENT_FILE = DOCUMENTS_PATH + "/domain_templ.pddl"
PROBLEM_TEMPL_DOCUMENT_FILE = DOCUMENTS_PATH + "/prob_templ.pddl"
FOIL_DOCUMENT_PATH = DOCUMENTS_PATH + "/foil"
SOLUTION_DOCUMENT_FILE = DOCUMENTS_PATH + "/solution.txt"
TMP_PATH = "./TMP_Merged/TMP.py"
PORT_NUMBER = 1233
PROBLEM_RANDOM_TEMPL_DOCUMENT_FILE = DOCUMENTS_PATH + "/prob_random_templ.pddl"
PROBLEM_RANDOM_GOAL_FILE = '/problem_random_created.pddl'
PROBLEM_CUSTOM_TEMPL_DOCUMENT_FILE = DOCUMENTS_PATH + "/prob_custom_templ.pddl"
PROBLEM_CUSTOM_GOAL_FILE = '/problem_custom_created.pddl'


if IS_VM:
    FF_PATH = '/home/jedai/JEDAI/TMP_Merged/planners/FF-v2.3modified/ff'
    LOG_FILE = '/home/jedai/JEDAI/logs_new.txt'
else:
    FF_PATH = '/root/git/JEDAI/TMP_Merged/planners/FF-v2.3modified/ff'
    LOG_FILE = '/root/git/JEDAI/logs_new.txt'


