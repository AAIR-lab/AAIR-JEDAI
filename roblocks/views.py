# -*- coding: utf-8 -*-
from __future__ import unicode_literals

# hello

import re
import pprint
import json
import logging
import shutil
import signal
import tempfile
from random import random
from .gpt import *
from . import global_vars
from natural_language import natural_language_generator as nlg

import config
# import pddl_parser.pddl_parser.planner as pl
# import pddl_parser.pddl_parser.PDDL as ps
# from state_space import StateSpace

from django.shortcuts import render

from Explainer import explainer_server, create_lattice, testRandomActions
from .forms import UploadBookForm
from django.views.decorators.csrf import csrf_exempt
from .models import EBooksModel
from .problem_info_generator import get_problem_info
import os
import subprocess
from django.http import JsonResponse
import socket
from PIL import Image
from .state_space import StateSpace
from django.shortcuts import render, redirect

import requests

file_dir = os.path.dirname(os.path.abspath(__file__))[:-8]
print("File Dir = ",file_dir)
# print('planner = ',dir(pl.Planner))

tmp_process = None
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('127.0.0.1', config.PORT_NUMBER))
server_socket.listen(1)

semantics = None
cs = None
log = logging.getLogger(__name__)
print("ABCD")


global global_domain 
global global_problem
global_domain = ""
global_problem = ""
global action_costs
global user_id
global dom 
action_costs = {}
user_id = ''
dom = ''


def see_action_costs(request):
    return JsonResponse(action_costs)


def create_log_file(filename):
    if not os.path.exists(filename):
        with open(filename,'a+') as file:
            pass

def setup_logger(logger_name, log_file, level=logging.INFO):

    create_log_file(log_file)


    log_setup = logging.getLogger(logger_name)
    formatter = logging.Formatter('%(levelname)s: %(asctime)s %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p')
    fileHandler = logging.FileHandler(log_file, mode='a')
    fileHandler.setFormatter(formatter)
    streamHandler = logging.StreamHandler()
    streamHandler.setFormatter(formatter)
    log_setup.setLevel(level)
    log_setup.addHandler(fileHandler)
    log_setup.addHandler(streamHandler)

# create_log_file()

# x = 
# print(datetime.datetime.now())
if config.IS_VM:
    name = "/tmp/jedai_logs.txt"
else:
    name = "/root/git/JEDAI/logs_dir/log_file_new_"+os.getenv('USER_HASH',"")+".txt"
print("NAME  = ",name)

setup_logger('log_new', '' + name)

log_new = logging.getLogger('log_new')

# logger = setup_logger('first_logger', config.LOG_FILE)





def get_full_hint(hint,semantics):
    hint = hint[0]
    params = hint.split(":")[1].split()[1:]
    hint = hint.split(":")[1].split()[0].lower() 
    action_det = [i for i in semantics['actions'] if i['name'].lower()==hint]
    action = [i['display'] for i in semantics['actions'] if i['name'].lower()==hint][0]

    det = (action_det)[0]['parameters']
    n = len(det)

    hint_string = [""]*n

    for i in range(n):
        idx = (det[i]['index'])

        param = "?"
        if random()<0.5:
            param = params[idx].lower()

        hint_string[i] = det[i]['display'] + " " + param

    ans = action + "<br>" + "<br>".join(hint_string)
    return ans   


def create_state_seq_str(plan,seq):
  state_str = ""
  actions = ["("+x.strip()+")" for x in plan.strip().split(",")]
  for idx, state in enumerate(seq):
      if(idx==0):
        state_str += f"Initial State"
      else:
        state_str += f"After applying Action {idx} "
        # state_str += '\n'
        state_str += "<strong>"+ nlg.get_natural_language_action(semantics, actions[idx-1]) + "</strong> :"
      state_str += '\n'
      
      for _ in state:
        state_str += "\t"+nlg.get_natural_language_single_predicate(semantics,str(_).replace(",","").replace("'",''),False).replace("'","")+'\n'
      state_str += '\n'
  return state_str

def create_state_seq_str2(plan,seq):
  actions = ["("+x.strip()+")" for x in plan.strip().split(",")]
  
  action_headings = []
  state_contents = []
  
  for idx, state in enumerate(seq):
      content = ""
      if(idx==0):
        action_headings.append("Initial State")
      else:
        heading = f"After applying Action {idx} : "
        heading += "<strong>"+ nlg.get_natural_language_action(semantics, actions[idx-1]) + "</strong> :"
        action_headings.append(heading)
          
      
      for _ in state:
        content += "\t"+nlg.get_natural_language_single_predicate(semantics,str(_).replace(",","").replace("'",''),False).replace("'","")+'\n'
      state_contents.append(content)
    
  return [action_headings, state_contents]



# TODO check csrf
@csrf_exempt
# /plan_submit
def submit_plan(request):
    
    submitPlanHitOrder = request.GET.get('submitPlanHitOrder')
    # print("semantics = ",semantics) 

    submitted_plan = request.GET.get('plan')
    get_hint = request.GET.get('get_hint') == 'true'
    print("IN SUBMIT PLAN!! ",(get_hint))
    log.debug(submitted_plan)

    if get_hint:
        log_new.info("hinting")
    else:
        log_new.info("plan_Evaluation")
    # type_of_plan = type(submitted_plan)
    # log.debug(f"Entering /plan_submit endpoint with plan: {submitted_plan}")
    # log.debug(f"Entering /plan_submit endpoint with type of plan: {type_of_plan}")


    # type_of_semantics = type(semantics)
    # log.debug(f"Entering /plan_submit endpoint with semantics: {semantics}")
    # log.debug(f"Entering /plan_submit endpoint with type of semantics: {type_of_semantics}")

    ss = StateSpace(config.DOMAIN_DOCUMENT_FILE,config.PROBLEM_DOCUMENT_FILE,True)
    final_state, state_sequence = ss.implementPlan(submitted_plan,action_costs,user_id,dom)
    print("=================================")
    print("=================================")
    print("=================================")
    print("=================================")
    print(state_sequence)
    state_sequence = create_state_seq_str2(submitted_plan,state_sequence)
    
    
    print(state_sequence)    
    # for _ in state_sequence:
    #     print(str(_))
    #     print(nlg.get_natural_language_single_predicate(semantics,str(_).replace(",","").replace("'",''),False).replace("'",""))
    # print()
    print("=================================")
    print("=================================")
    print("=================================")
    print("=================================")
    
    log.debug("Returned from SS")
    init_state = " ".join(["("+" ".join(atom)+")" for atom in final_state])
    with open(config.DOMAIN_DOCUMENT_FILE, 'r') as file:
        domain_string  = file.read()
    with open(config.PROBLEM_DOCUMENT_FILE, 'r') as file:
        problem_string  = file.read()

    hint_gpt = ""

    if(submitted_plan=="" and get_hint!=True):
        return JsonResponse({"hint":"", "hint_gpt":hint_gpt, "explanation_gpt":"","state_sequence":state_sequence,"submitPlanHitOrder":submitPlanHitOrder})


    if(submitted_plan=="" and get_hint==True):
        log.debug("Inside the empty plan if")
        ss = StateSpace(config.DOMAIN_DOCUMENT_FILE,config.PROBLEM_DOCUMENT_FILE,True)
        final_state, final_sequence = ss.implementPlan(submitted_plan,action_costs,user_id,dom)
        new_data = ''
        init_state = " ".join(["("+" ".join(atom)+")" for atom in final_state])

        log.debug("Returned from SS")

        with open(config.PROBLEM_DOCUMENT_FILE,'r') as file:
            filedata = file.read()
            new_data = "\n".join([filedata.split("(:init")[0],
                                "(:init",init_state,")","(:goal",filedata.split("(:goal")[1]])
            print("NEW PROBLEM FILE FOR GETTING HINTS = \n",new_data)

        temp_dir = tempfile.gettempdir()
        problem_temp_path = os.path.join(temp_dir, 'problem_copy.pddl')
  
        with open(problem_temp_path, 'w+') as file:
            file.write(new_data)

        run_ff_command = str(config.FF_PATH+" -o "+config.DOMAIN_DOCUMENT_FILE+" -f "+problem_temp_path)

        print("PRINT RUN FF COMMAND = ",run_ff_command)
        # log.debug("RUN FF COMMAND = ",run_ff_command)
        # output = [i.strip() for i in
        #       os.popen(run_ff_command).read().strip().split('\n')]
        output = os.popen(run_ff_command).read().strip()
        # output = os.popen(run_ff_command).read().strip()
        output = re.sub(' +', ' ', output)

        print("FF OUTPUT = ",output)
        hint = [i for i in output.split("\n") if "step 0" in i]

        full_hint = get_full_hint(hint,semantics)
        log_new.info(f"Full hint = {full_hint}")


        if(len(hint)>0):
            hint = hint[0].split(":")[1].split()[0].lower() 
            hint = [i['display'] for i in semantics['actions'] if i['name'].lower()==hint]
            # hint = [i['display'] for i in semantics['actions'] if i['name']==hint.split()[0].lower()][0]            
        print("HINT = ",hint)
        # hint = [i.split(":")[1].strip() for i in output.split("\n") if "step 0" in "".join(i.split())][0]
        # hint = [i['display'] for i in semantics['actions'] if i['name']==hint.split()[0].lower()][0]
        # print("OUTPUT FROM FF = ",output)
        hint = full_hint
        os.remove(problem_temp_path)

        global global_domain
        hint_gpt_inputs = (global_domain, domain_string,problem_string, init_state, full_hint)

        # log.debug(f"Hint GPT Inputs = {global_domain}, {domain_string}, {init_state}, {full_hint}")

        # hint_gpt = get_hint_gpt(hint_gpt_inputs)
        hint_gpt = get_hint_gpt(hint_gpt_inputs)          

        # log.debug(f"Hint GPT Inputs = {hint_gpt_inputs}")
        log.debug(f"Hint GPT output = {hint_gpt}")

        return JsonResponse({"hint":hint, "hint_gpt":hint_gpt, "explanation_gpt":"","state_sequence":state_sequence,"submitPlanHitOrder":submitPlanHitOrder})






    try:
        log.info("Calling explanation code")
        explanation_map, explained_plan, explanation_map_non_helm, explained_plan_non_helm  = explainer_server.call_server(submitted_plan, semantics)
        log.info(f"Explanation code reports plan to be {'unsuccessful' if explanation_map['failed'] else 'successful'}")
        success = "False" if explanation_map["failed"] else "True"
    except:
        return JsonResponse({"helm_failure":"true"})

    print("----------------------__------------__-----__-----")
    print("MOVING PAST EXPLAINER!!")
    print("----------------------__------------__-----__-----")        
    
    final_state = None
    new_data = ""
    hint = ""
    output = ""
    full_hint = ""
    
    
    log.debug(f"ERR CODE = {explanation_map.get('err_code')}")

    log.debug(f"get_hint = {(get_hint)}")
    log.debug(f"success = {(success)}")
    log.debug(f"ERR CODE = {(explanation_map.get('err_code'))}")
    c = get_hint and (success=="False") and explanation_map.get('err_code',"")=='GOAL ERROR'
    #  and (not success))
    #  and explanation_map.get('err_code',"")=='GOAL ERROR')

    log.debug(f"Total condition {c}")

    log_new.info(f"plan_evaluation_{success}")
    e = explanation_map.get('err_code',"")
    e2 = explanation_map_non_helm.get('err_code',"")    
    log_new.info(f"err_code_{e}_{e2}")

    if get_hint and (success=="False") and explanation_map.get('err_code',"")=='GOAL ERROR' and explanation_map_non_helm.get('err_code',"")=='GOAL ERROR':
                
        print("----------------------__------------__-----__-----")
        print("IN STATESPACE FF CALLER!!")
        print("----------------------__------------__-----__-----")        
        ss = StateSpace(config.DOMAIN_DOCUMENT_FILE,config.PROBLEM_DOCUMENT_FILE,True)
        final_state, state_sequence = ss.implementPlan(submitted_plan,action_costs,user_id,dom)
        state_sequence = create_state_seq_str2(submitted_plan,state_sequence)
        new_data = ''
        init_state = " ".join(["("+" ".join(atom)+")" for atom in final_state])

       
        with open(config.PROBLEM_DOCUMENT_FILE,'r') as file:
            filedata = file.read()
            new_data = "\n".join([filedata.split("(:init")[0],
                                "(:init",init_state,")","(:goal",filedata.split("(:goal")[1]])
            print("NEW PROBLEM FILE FOR GETTING HINTS = \n",new_data)

        temp_dir = tempfile.gettempdir()
        problem_temp_path = os.path.join(temp_dir, 'problem_copy.pddl')

        with open(problem_temp_path, 'w+') as file:
            file.write(new_data)

        run_ff_command = str(config.FF_PATH+" -o "+config.DOMAIN_DOCUMENT_FILE+" -f "+problem_temp_path)

        print("PRINT RUN FF COMMAND = ",run_ff_command)
        # log.debug("RUN FF COMMAND = ",run_ff_command)
        # output = [i.strip() for i in
        #       os.popen(run_ff_command).read().strip().split('\n')]
        output = os.popen(run_ff_command).read().strip()
        # output = os.popen(run_ff_command).read().strip()
        output = re.sub(' +', ' ', output)

        print("FF OUTPUT = ",output)
        hint = [i for i in output.split("\n") if "step 0" in i]

        full_hint = get_full_hint(hint,semantics)
        log_new.info(f"Full hint = {full_hint}")

        print("HINT = ",hint)
        if(len(hint)>0):
            hint = hint[0].split(":")[1].split()[0].lower() 
            print("HINT2 = ",hint)
            hint = [i['display'] for i in semantics['actions'] if i['name'].lower()==hint]
            # hint = [i['display'] for i in semantics['actions'] if i['name']==hint.split()[0].lower()][0]            

        hint = full_hint
        print("HINT = ",hint)
        # hint = [i.split(":")[1].strip() for i in output.split("\n") if "step 0" in "".join(i.split())][0]
        # hint = [i['display'] for i in semantics['actions'] if i['name']==hint.split()[0].lower()][0]
        # print("OUTPUT FROM FF = ",output)
        os.remove(problem_temp_path)


        hint_gpt_inputs = (global_domain, domain_string, problem_string,init_state, full_hint)
        hint_gpt = get_hint_gpt(hint_gpt_inputs)                  



    explanation_non_helm_gpt = get_gpt_human_description_for_explanation(global_domain, domain_string,problem_string,explanation_map_non_helm,init_state)
    explanation_helm_gpt = explanation_non_helm_gpt
    
    if explanation_map_non_helm!=explanation_map:
        explanation_helm_gpt = get_gpt_human_description_for_explanation(global_domain, domain_string,problem_string,explanation_map,init_state)
        
    
    log.debug(f"explanation_helm_gpt GPT = {explanation_helm_gpt}")
    log.debug(f"explanation_non_helm_gpt GPT = {explanation_non_helm_gpt}")

    data = {
        "helm_failure": "false",
        "explanation_map": explanation_map,
        "explanation_map_non_helm": explanation_map_non_helm,
        "success": success,
        "execution_plan": explained_plan,
        "execution_plan_non_helm": explained_plan_non_helm,
        "final_state": str(final_state),
        "new_problem_file": new_data,
        "err_code_helm" : explanation_map.get('err_code',""),
        "err_code_non_helm" : explanation_map_non_helm.get('err_code',""),
        "get_hint" : get_hint,
        "output":output,
        "hint" : hint,
        "hint_gpt":hint_gpt,
        "explanation_non_helm_gpt":explanation_non_helm_gpt,
        "explanation_helm_gpt":explanation_helm_gpt,
        "state_sequence":state_sequence,
        "submitPlanHitOrder":submitPlanHitOrder
    }
    # this updates TMP to execute the plan on robot

    # log.debug(f"Data returned from views.py = {data}")


    hint_gpt_inputs = (global_domain, domain_string, init_state, full_hint)
    # log.debug(f"Hint GPT Inputs = {global_domain}, {domain_string}, {init_state}, {full_hint}")

    # hint_gpt = get_hint_gpt(hint_gpt_inputs)        

    # log.debug(f"Hint GPT Inputs = {hint_gpt_inputs}")

    return JsonResponse(data)

 
def log_workspace(request):
    log_new.info(request.GET['event_code'])
    return JsonResponse({"log_workspace":"TRUE"})

def inactivity(request):
    server = "http://"+os.environ["ASSIGNED_USER_IP"] + ":8000/inactivity"
    print(server)
    x = requests.get(server,params={"jedai_port":os.environ['JEDAI_PORT']})
    print("VALUE RETURNED FROM SERVER = ",x.json())
    return JsonResponse({"INACTIVITY SERVER":"TRUE"})


def save_plan(request):
    blob = request.GET.get('blob')
    file_name = request.GET.get('file_name')
    # log.debug(file_name)
    with open(file_name,'w+') as file:
        file.write(blob)
    return JsonResponse({})


# /start_TMP
def start_tmp(request):
    log.debug("Entering /start_TMP endpoint")
    domain = _get_domain_name_from_documents()
    problem = _get_problem_name_from_documents()
    _spawn_tmp_for_domain_and_problem(domain, problem)
    return JsonResponse({})


# /call_TMP
def call_tmp(request):
    global cs
    submitted_plan = request.GET.get('plan', None)
    log.debug(f"Entering /call_TMP endpoint with plan: {submitted_plan}")
    # TODO actually check for success
    success = True
    data_val = {}
    if success:
        data_val["success"] = "True"

    (cs, address) = server_socket.accept()
    log.debug("Accepted connection")
    log.debug(f"cs:{cs}")
    log.debug(f"address:{address}")
    log.debug("Waiting for 'ready' message")
    msg = cs.recv(1024).decode("utf-8")
    # TODO
    while msg != "ready":
        log.debug(f"Message: {msg}")
        msg = cs.recv(1024).decode("utf-8")

    log.debug("Received 'ready' message, now sending plan over socket")
    cs.sendall(bytes(submitted_plan, "utf-8"))
    log.info("User's plan sent to TMP")
    # TODO
    while True:
        msg = cs.recv(1024).decode("utf-8")
        if msg[0] == "u":
            '''
            update the bar here.. format will be u|#refined_actions|#total_actions 
            '''
            log.debug(f"Received update from TMP: {msg}")
        elif msg == "refined":
            '''
            TODO: Use this elif to report success.
            refinement done. ALL actions passed. report success
            '''
            log.info("TMP reported successful refinement")
            break
        elif msg == "failure":
            '''
            TODO: Use this elif to report failure to the users. right now it just report success. 
            refinement reported failure. report failure
            '''
            log.warning("TMP reported failed refinement")
            break
    return JsonResponse(data_val)


# /run_plan
def run_plan(request):
    log.debug("Entering /run_plan endpoint")
    log.debug("Sending 'run' message to TMP")
    global cs
    cs.sendall(bytes("run", "utf-8"))
    log.debug("Waiting for 'run_complete' message from TMP")
    msg = cs.recv(1024).decode("utf-8")
    while msg != "run_complete":
        log.debug(f"Message from TMP: {msg}")
        msg = cs.recv(1024).decode("utf-8")
    log.debug("Received 'run_complete' message from TMP")
    return JsonResponse({})


# /kill_tmp
def kill_tmp(request):
    log.debug("Entering /kill_tmp endpoint")
    log.debug("Sending 'terminate' message to TMP")
    global cs
    cs.sendall(bytes("terminate", "utf-8"))
    log.debug("Closing socket connection")
    cs.shutdown(socket.SHUT_RDWR)
    cs.close()
    return JsonResponse({})

# /terminate_tmp
def terminate_tmp(request):
    log.debug("Entering /terminate_tmp endpoint")
    log.debug("Force killing tmp process")
    _try_to_kill_tmp_and_close_socket()
    return JsonResponse({})


# /get_problem_names
def get_problem_file_names(request):
    
    domain = request.GET.get('domain')
    log.debug(f"Entering /get_problem_names endpoint with domain: {domain}")

    files = os.listdir(domain)

    final = []
    # TODO access the directories directly
    for f in files:
        if "." not in f:
            final.append(f)

    result = {"problem_names": final}
    return JsonResponse(result)




def cafe_world_tutorial(request):
    return render(request, '/root/git/JEDAI/roblocks/templates/roblocks/CafeWorld.html', {
    })
    
def keva_tutorial(request):
    return render(request, '/root/git/JEDAI/roblocks/templates/roblocks/Keva.html', {
    })
    

# /
def upload_view(request,random_goal=False):

    log_new.info("home_page_loaded")

    print("Request = ",request.POST)
    log.debug("Entering root endpoint")


    # request_ip = get_client_ip(request)
    # assigned_user_ip = os.environ['ASSIGNED_USER_IP']

    # print("Request IP = ",request_ip)
    # print("Assigned User IP = ", assigned_user_ip)


    # if(request_ip!=assigned_user_ip):
    #     return render(request, 'roblocks/unauthorized_access.html', {
    #         'error': "You dont have access to view this page",
    #         'code': 403,
    #     })
    




    _cleanup_documents()
    if request.method == 'POST':
        print("POST HI HAI ")
        # print(request.POST)
        return _handle_post_request_to_upload_view(request)

    if tmp_process is not None:
        _try_to_kill_tmp_and_close_socket()

    form = UploadBookForm()
    name = config.PREBUILT_DOMAIN_FOLDERS
    return render(request, 'roblocks/home.html', {
        'form': form,
        'name': name
    })



def handle_custom_goals(data):
    data = data.GET
    _cleanup_documents()
    form = UploadBookForm(data)

    print("DATA = ",data)

    files = [data['domain'] + '/domain.pddl',
             data['domain'] + "/" + data['problem'] + '/problem.pddl',
             data['domain'] + "/" + data['problem'] + '/env.dae',
             data['domain'] + config.SEMANTICS_FILE]

    _save_images_to_static_files(data['domain'], data['problem'])
    _set_file_instance_names(form, files)
    _write_template_files()

    with open(config.PROBLEM_CUSTOM_TEMPL_DOCUMENT_FILE) as p_fd:
        prob_template_str = p_fd.read().strip()

        goals = json.loads(data['goal_str'])

        custom_goal_string = ""
        for goal in goals:
            custom_goal_string += "("
            custom_goal_string += goal['pred_name']
            for param in goal['params']:
                custom_goal_string += " "
                custom_goal_string += param
            custom_goal_string += ")"


        prob_str = prob_template_str.format(custom_goal_string)

        final_path = os.path.join(file_dir,data['domain'],
                                  data['problem'],'problem_custom_created.pddl')
        
        print("Final Path = ",final_path)
        text_file = open(final_path, "w+")
        text_file.write(prob_str)
        text_file.close()
        print("Written to the custom created problem file ")

    files = [data['domain'] + '/domain.pddl',
    data['domain'] + "/" + data['problem'] + config.PROBLEM_CUSTOM_GOAL_FILE,
    data['domain'] + "/" + data['problem'] + '/env.dae',
    data['domain'] + config.SEMANTICS_FILE]



    run_ff_command = str(config.FF_PATH+" -o '"+files[0]+"' -f '"+files[1]+"'")
    # run_ff_command = str(config.FF_PATH+" -o "+config.DOMAIN_DOCUMENT_FILE+" -f "+problem_temp_path)
    
    print("PRINT RUN FF COMMAND = ",run_ff_command)
    output = os.popen(run_ff_command).read().strip()
    print("FF OUTPUT = ",output)

    if("unsolvable" in output):
        return JsonResponse({"status":"fail"})

    # planner = pl.Planner()
    # plan = planner.solve(files[0],files[1])

    # if(plan==None):
    #     return JsonResponse({"status":"fail"})

    _set_file_instance_names(form, files)

    domain_name = _get_domain_name_from_documents()
    problem_name = _get_problem_name_from_documents()

    print("DOmain and problem name = ",domain_name," and ",problem_name)
    # TODO this will throw an error if there are no semantics, change this once they are optional
    with open(config.DOCUMENTS_PATH + config.SEMANTICS_FILE, "r") as file:
        semantics_json = file.read()
    global semantics
    semantics = json.loads(semantics_json)

    
    create_lattice()
    _spawn_tmp_for_domain_and_problem(domain_name, problem_name)

    problem_info = get_problem_info(semantics)


    return JsonResponse({"problem_info":problem_info,"status":"success"})


def handle_random_goals(request):
    
    log_new.info(f"curriculum_problem_generated")
    
    form = UploadBookForm(request.GET)
    files = [request.GET['domain'] + '/domain.pddl',
             request.GET['domain'] + "/" + request.GET['problem'] + '/problem.pddl',
             request.GET['domain'] + "/" + request.GET['problem'] + '/env.dae',
             request.GET['domain'] + config.SEMANTICS_FILE]
    
    # parser = ps.PDDL_Parser()
    # parser.parse_domain(files[0])
    # parser.parse_problem(files[1])


    # for atom in parser.state:
    #     print(atom)

        # TODO

    log.debug(f"Request.get = {request.GET}")

    if len(files) < 3:
        return render(request, 'roblocks/home.html', {
            'error': "please upload all files",
            'code': 404,
            'form': form,
        })
    

    _save_images_to_static_files(request.GET['domain'], request.GET['problem'])
    _set_file_instance_names(form, files)
    _write_template_files()

    if(True):

        diff = request.GET['diff']
        diff_file = (request.GET['domain']+"/difficulty_config.json")

        with open(diff_file) as json_file:
            domain_config = json.load(json_file)
        
        plan_len = 3
        if diff=='easy':
            plan_len = domain_config['EASY']
            # plan_len = 3
        if diff == 'medium':
            plan_len = domain_config['MEDIUM']
            # plan_len = 5
        if diff == 'hard':
            plan_len = domain_config['DIFFICULT']
            # plan_len = 7
        
        log.debug(f"DIFF = {diff}")
        log.debug(f"plan_len = {plan_len}")

        # planner = pl.Planner()
        # state, plan = planner.bfs(files[0],files[1],plan_len)
        
        #testing gbfs
        plan_len = 6

        ss = StateSpace(files[0],files[1])
        init_state = ss.state
        
        state, plan, level = ss.greedy_best_first(plan_len,action_costs, user_id, dom)

        print("state = ",state),
        print("plan = ")
        for x in plan:
            print(str(x)+" ")

        print("Final Goal = ")
        for atom in state:
            print(atom)
        

        with open(config.PROBLEM_RANDOM_TEMPL_DOCUMENT_FILE) as p_fd:
            prob_template_str = p_fd.read().strip()
            goal_state_str_list = ""

            # for atom in state:
            #     x = str(atom).replace("," ,"").replace("'","") 
            #     if("=" not in x):
            #         goal_state_str_list += x

            for atom in state:
                if atom not in init_state and ('equals' not in atom):
                    goal_state_str_list += ("".join(["("," ".join(atom),")"]))     
                    goal_state_str_list += " "               


            prob_str = prob_template_str.format(goal_state_str_list)
            print("PROB_RANDOM_STR =",prob_str)
        
        # final_path = os.path.join(file_dir,request.POST['domain'],
        #                           request.POST['problem'],'problem_random_created.pddl')
        # print("Final Path = ",final_path)

        final_path = os.path.join(file_dir,request.GET['domain'],
                                  request.GET['problem'],'problem_random_created.pddl')
        print("Final Path = ",final_path)
        text_file = open(final_path, "w+")
        text_file.write(prob_str)
        text_file.close()
        print("Written to the random created problem file ")

        files = [request.GET['domain'] + '/domain.pddl',
        request.GET['domain'] + "/" + request.GET['problem'] + config.PROBLEM_RANDOM_GOAL_FILE,
        request.GET['domain'] + "/" + request.GET['problem'] + '/env.dae',
        request.GET['domain'] + config.SEMANTICS_FILE]

        _set_file_instance_names(form, files)

        print("all good in random ")


    domain_name = _get_domain_name_from_documents()
    problem_name = _get_problem_name_from_documents()

    print("Domain and problem name = ",domain_name," and ",problem_name)
    # TODO this will throw an error if there are no semantics, change this once they are optional
    with open(config.DOCUMENTS_PATH + config.SEMANTICS_FILE, "r") as file:
        semantics_json = file.read()
    global semantics
    semantics = json.loads(semantics_json)

    
    create_lattice()
    _spawn_tmp_for_domain_and_problem(domain_name, problem_name)

    problem_info = get_problem_info(semantics)

    return JsonResponse({"problem_info":problem_info})



def extract_predicates(domain):
  data = ""
  with open(domain,'r') as file:
    data = file.read()
  k = data.split(":predicates")[1].split("(:")[0]
  d = re.findall('\(.*?\)',k.replace("-",""))
  preds = {}
  for pred in d:
      objects = pred.replace("(","").replace(")","").split()
      preds[objects[0]] = {}
      for i in range(len(objects)//2):
        preds[objects[0]][objects[2*i+1]]=objects[2*i+2]  

  return preds

def get_client_ip(request):
    x_forwarded_for = request.META.get('HTTP_X_FORWARDED_FOR')
    if x_forwarded_for:
        ip = x_forwarded_for.split(',')[0]
    else:
        ip = request.META.get('REMOTE_ADDR')
    return ip



def get_llm(request):
    dom_prob = get_domain_problem()
    ans = dom_prob[0]+" "+dom_prob[1]
    log.debug(f"getllm = {ans}")
    return JsonResponse({"text":ans})


def get_domain_problem():
    print("called get_domain_problem ",global_domain)
    return (global_domain, global_problem)


def handle_action_costs(domain):

    global dom
    dom = domain
    
    print("user id = ",user_id)
    print("dom = ",dom)

    if action_costs == {} or user_id not in action_costs:
        
        print("****************")
        print("****************")
        print("****************")

        print("action costs = ",action_costs)
        print("user id = ",user_id)

        if user_id not in action_costs:
            action_costs[user_id] = {}
            action_costs[user_id][dom] = {}
            # action_costs[dom] = {}
            
        ss = StateSpace(config.DOMAIN_DOCUMENT_FILE,config.PROBLEM_DOCUMENT_FILE,True)        
        for action in ss.parser.actions:
            action_costs[user_id][dom][action.name] = 0
            # action_costs[dom][action.name] = 0

        print("action-costs = ",action_costs)
    
        print("****************")
        print("****************")
        print("****************")
        
    elif user_id in action_costs and dom not in action_costs[user_id]:
            action_costs[user_id][dom] = {}
            ss = StateSpace(config.DOMAIN_DOCUMENT_FILE,config.PROBLEM_DOCUMENT_FILE,True)        
            for action in ss.parser.actions:
                action_costs[user_id][dom][action.name] = 0
        



def _handle_post_request_to_upload_view(request):


    print(request.POST)

    problem = request.POST['problem']
    dom = request.POST['domain']

    global global_domain 
    global_domain = dom        
    
    


    global global_problem 
    global_problem = problem

    log.debug(f"Global domain = {global_domain} and global problem = {global_problem}")

    log_new.info(f"training_area_problem_{problem}_domain_{dom}")

    _mutable = request.POST._mutable

    request.POST._mutable = True

    # сhange the values you want
    request.POST['setting'] = 'not_random'
    # set mutable flag back
    request.POST._mutable = _mutable

    if 'Random' in request.POST['problem'] or 'Curriculum' in request.POST['problem']:

        domain = request.POST['domain']
        files = os.listdir(domain)

        final = []
        # TODO access the directories directly
        for f in files:
            if "." not in f:
                final.append(f)        
        print("PROBLEM SETTING for RANDOM PROBLEM = ",final[0])
        _mutable = request.POST._mutable

        # set to mutable
        request.POST._mutable = True

        # сhange the values you want
        request.POST['problem'] = final[0]
        request.POST['setting'] = 'random'
        # set mutable flag back
        request.POST._mutable = _mutable
        problem = final[0]

    form = UploadBookForm(request.POST)


    

    files = [request.POST['domain'] + '/domain.pddl',
             request.POST['domain'] + "/" + request.POST['problem'] + '/problem.pddl',
             request.POST['domain'] + "/" + request.POST['problem'] + '/env.dae',
             request.POST['domain'] + config.SEMANTICS_FILE]
    
    # parser = ps.PDDL_Parser()
    # parser.parse_domain(files[0])
    # parser.parse_problem(files[1])


    # for atom in parser.state:
    #     print(atom)

        # TODO
    if len(files) < 3:
        return render(request, 'roblocks/home.html', {
            'error': "please upload all files",
            'code': 404,
            'form': form,
        })
    

    _save_images_to_static_files(request.POST['domain'], request.POST['problem'])
    _set_file_instance_names(form, files)
    _write_template_files()

    # if(request.POST['goal_setting']=='random'):

    #     diff = request.POST['diff']

    #     if diff=='easy':
    #         plan_len = 3
    #     if diff == 'medium':
    #         plan_len = 5
    #     if diff == 'hard':
    #         plan_len = 7
        

    #     planner = pl.Planner()
    #     state, plan = planner.bfs(files[0],files[1],plan_len)

    #     print("state = ",state),
    #     print("plan = ",plan)

    #     print("Final Goal = ")
    #     for atom in state:
    #         print(atom)
        

    #     with open(config.PROBLEM_RANDOM_TEMPL_DOCUMENT_FILE) as p_fd:
    #         prob_template_str = p_fd.read().strip()
    #         goal_state_str_list = ""
    #         for atom in state:
    #             x = str(atom).replace("," ,"").replace("'","") 
    #             if("=" not in x):
    #                 goal_state_str_list += x

    #         prob_str = prob_template_str.format(goal_state_str_list)
    #         print("PROB_RANDOM_STR =",prob_str)
        
    #     # final_path = os.path.join(file_dir,request.POST['domain'],
    #     #                           request.POST['problem'],'problem_random_created.pddl')
    #     # print("Final Path = ",final_path)

    #     final_path = os.path.join(file_dir,request.POST['domain'],
    #                               request.POST['problem'],'problem_random_created.pddl')
    #     print("Final Path = ",final_path)
    #     text_file = open(final_path, "w+")
    #     text_file.write(prob_str)
    #     text_file.close()
    #     print("Written to the random created problem file ")

    #     files = [request.POST['domain'] + '/domain.pddl',
    #     request.POST['domain'] + "/" + request.POST['problem'] + config.PROBLEM_RANDOM_GOAL_FILE,
    #     request.POST['domain'] + "/" + request.POST['problem'] + '/env.dae',
    #     request.POST['domain'] + config.SEMANTICS_FILE]

    #     _set_file_instance_names(form, files)

    #     print("all good in random ")


    domain_name = _get_domain_name_from_documents()
    problem_name = _get_problem_name_from_documents()

    print("DOmain and problem name = ",domain_name," and ",problem_name)
    # TODO this will throw an error if there are no semantics, change this once they are optional
    with open(config.DOCUMENTS_PATH + config.SEMANTICS_FILE, "r") as file:
        semantics_json = file.read()
    global semantics
    semantics = json.loads(semantics_json)
    # print("semantics = ",semantics)
    
    create_lattice()
    _spawn_tmp_for_domain_and_problem(domain_name, problem_name)

    # randomPlan = testRandomActions.run(semantics,get_problem_info(semantics),5)
    # print("=============")
    # print("=============")
    # print("=============")
    # print("RANDOM PLAN = ", randomPlan)
    # print("=============")
    # print("=============")
    # print("=============")



    handle_action_costs(global_domain)


    preds = extract_predicates(config.DOMAIN_DOCUMENT_FILE)
    return _get_blockly_render(request, form, semantics_json, preds)    
    # return _get_blockly_render(request, form, semantics_json, parser.objects,parser.predicates)



def _save_images_to_static_files(domain_path, problem_name):
    init_image_for_this_domain = domain_path + "/" + problem_name + "/" + config.INIT_IMAGE_NAME_IN_MODULES
    goal_image_for_this_domain = domain_path + "/" + problem_name + "/" + config.GOAL_IMAGE_NAME_IN_MODULES
    if os.path.exists(config.INIT_IMAGE_STATIC_FILE):
        os.remove(config.INIT_IMAGE_STATIC_FILE)
    if os.path.exists(init_image_for_this_domain):
        image = Image.open(init_image_for_this_domain)
        image_copy = image.copy()
        image_copy.save(config.INIT_IMAGE_STATIC_FILE)
    
    if os.path.exists(config.GOAL_IMAGE_STATIC_FILE):
        os.remove(config.GOAL_IMAGE_STATIC_FILE)
    if os.path.exists(goal_image_for_this_domain):
        image = Image.open(goal_image_for_this_domain)
        image_copy = image.copy()
        image_copy.save(config.GOAL_IMAGE_STATIC_FILE)


def _try_to_kill_tmp_and_close_socket():
    log.debug("Trying to kill TMP and close socket connection")
    global cs
    try:
        if cs is not None:
            log.debug("cs is not None")
            if "closed" not in cs.__repr__():
                log.debug("Closing cs without accept()")
                cs.shutdown(socket.SHUT_RDWR)
                cs.close()
            else:
                log.debug("Closing cs with accept() and one recv() with timeout=1s")

                # Only allow 1 second timeout since we are wrapping up.
                server_socket.settimeout(1.0)

                try:
                    (cs, address) = server_socket.accept()
                    cs.settimeout(1.0)

                    data = cs.recv(1024)
                except Exception:

                    print("Socket timedout")
                    pass

                # Restore the timeout back.
                server_socket.settimeout(None)
                cs.shutdown(socket.SHUT_RDWR)
                cs.close()
        else:
            log.debug("cs is None, closing with accept()")
            (cs, address) = server_socket.accept()
            cs.shutdown(socket.SHUT_RDWR)
            cs.close()
        log.debug("Sending kill signal to TMP process")
        os.killpg(os.getpgid(tmp_process.pid), signal.SIGTERM)
    except Exception as e:
        # log.exception automatically adds exception trace from handler
        log.exception("Exception while killing TMP")


def _get_blockly_render(request, form, semantics_json,predicates):
    problem_info = get_problem_info(semantics)

    print("From views._get_blockly_render predicates = ",predicates)
    
    for pred in predicates:
      predicates[pred] = [obj_type for obj_type in predicates[pred].values()]

    return render(request, 'roblocks/training-area.html', {
        'setting': request.POST['setting'],
        'form': form,
        'problem_actions': json.dumps(problem_info["actions"]),
        'problem_objects': json.dumps(problem_info["objects"]),
        'types_to_supertypes': json.dumps(problem_info["types_to_supertypes"]),
        'goal_str': problem_info["goal_str"],
        'semantics': semantics_json,
        'domain': request.POST['domain'],
        'problem': request.POST['problem'],
        'predicates' : predicates
    })


def _cleanup_documents():
    global semantics
    semantics = None

    # TODO get rid of/rename all the EBooks stuff
    files = [EBooks_Model.domain for EBooks_Model in EBooksModel.objects.all() if
             EBooks_Model.domain is not None and EBooks_Model.domain != ""]
    # Deleting all created files in media/documents
    for f in files:
        if os.path.exists(f):
            os.remove(f)

    for filename in os.listdir(config.DOCUMENTS_PATH):
        file_path = os.path.join(config.DOCUMENTS_PATH, filename)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
        except Exception as e:
            log.exception(f"Failed to delete {file_path}")


def _write_template_files():
    with open(config.DOMAIN_DOCUMENT_FILE, 'r') as file:
        data = file.read().replace('\n', '')

    text_file = open(config.DOMAIN_TEMPL_DOCUMENT_FILE, "w")
    text_file.write(data.split("(:action")[0])
    text_file.write("{})")
    text_file.close()

    with open(config.PROBLEM_DOCUMENT_FILE, 'r') as file:
        data = file.read().replace('\n', '')

    text_file = open(config.PROBLEM_TEMPL_DOCUMENT_FILE, "w")
    text_file.write(data.split("(:init")[0])
    text_file.write("(:init {})(:goal (and {})))")
    text_file.close()

    text_file = open(config.PROBLEM_RANDOM_TEMPL_DOCUMENT_FILE, "w")
    text_file.write(data.split("(:goal")[0])
    text_file.write("(:goal (and {})))")
    text_file.close()

    text_file = open(config.PROBLEM_CUSTOM_TEMPL_DOCUMENT_FILE, "w")
    text_file.write(data.split("(:goal")[0])
    text_file.write("(:goal (and {})))")
    text_file.close()


def write_random_problem_template_file():
    with open(config.PROBLEM_DOCUMENT_FILE, 'r') as file:
        data = file.read().replace('\n', '')

    text_file = open(config.PROBLEM_RANDOM_TEMPL_DOCUMENT_FILE, "w")
    text_file.write(data.split("(:goal")[0])
    text_file.write("(:goal (and {})))")
    text_file.close()




def _get_domain_name_from_documents():
    with open(config.PROBLEM_DOCUMENT_FILE, 'r') as file:
        data = file.read().replace('\n', '')
    return data.split("domain ")[1].split(")")[0].replace(' ', '')

def hello_msg(request):
    console.log("INSIDE HELLO MSG")
    return True

def _get_problem_name_from_documents():
    with open(config.PROBLEM_DOCUMENT_FILE, 'r') as file:
        data = file.read().replace('\n', '')
    return data.split("problem ")[1].split(")")[0].replace(' ', '')


def _spawn_tmp_for_domain_and_problem(domain_name, problem_name):
    log.info(f"Spawning TMP process with domain name {domain_name} and problem name {problem_name}")
    python2_command = config.PYTHON_2_PATH + ' "' + file_dir + f'{config.TMP_PATH}" ' + domain_name + ' ' + problem_name
    print("SPAWN TMP PYTHON CMD = ",python2_command)
    my_env = os.environ.copy()
    global tmp_process
    tmp_process = subprocess.Popen(python2_command,
                                   shell=True,
                                   env=my_env,
                                   preexec_fn=os.setsid)


def _set_file_instance_names(form, files):
    if form.is_valid():
        with open(files[0], 'r') as domain, open(config.DOMAIN_DOCUMENT_FILE, 'w') as doc_media:
            print("_set-file+instances ",files[0])
            for line in domain:
                doc_media.write(line)
            doc_media.close()
        with open(files[1], 'r') as problem, open(config.PROBLEM_DOCUMENT_FILE, 'w') as doc_media:
            for line in problem:
                doc_media.write(line)
            doc_media.close()
        with open(files[2], 'r') as env, open(config.ENV_DOCUMENT_FILE, 'w') as doc_media:
            for line in env:
                doc_media.write(line)
            doc_media.close()
        # TODO this will throw an error if there is no semantics file
        #  update this when the semantics file is made optional
        with open(files[3], 'r') as semantics_document_file, open(
                                                        config.DOCUMENTS_PATH + config.SEMANTICS_FILE,
                                                        'w'
                                                    ) as doc_media:
            for line in semantics_document_file:
                doc_media.write(line)
            doc_media.close()


def get_random(request):
    print("Setting Random GOALS!!")
    # print(request.GET.get('domain'),",",request.GET.get('problem'))
    return upload_view(request,True)
    # data = {"status":"successful"}
    # return JsonResponse(data)

def goal_submit(request):
    
    goals = json.loads(request.GET.get('goal'))
    # print("CUSTOM GOAL = ",goal)

    custom_goal_string = ""

    for goal in goals:
        custom_goal_string += "("
        custom_goal_string += goal['pred_name']
        for param in goal['params']:
            custom_goal_string += " "
            custom_goal_string += param
        custom_goal_string += ")"


    data = {"from_goal_submit in views.py":custom_goal_string}


    

    return JsonResponse({"ans":custom_goal_string})


def handle_close(request):
    print("=========================================================")
    print("HANDLING THE CLOSE ACTION")
    print("=========================================================")
    return JsonResponse({})


def input_id(request):
    if request.method == 'POST':
        global user_id
        user_id = request.POST.get('user_id')
        global action_costs
        action_costs = {}
        print("Received user id = ",user_id)
        setup_logger('log_new', '/root/git/JEDAI/logs_dir/' + user_id)
        
        return redirect('/intro_video')
    return render(request, 'roblocks/input_id.html')

def intro_video(request):
    if request.method == 'POST':        
        return redirect('/')
    
    return render(request, 'roblocks/intro_video.html')

def dropdown(request):    
    return render(request, 'roblocks/dropdown.html')


# def get_hint():
#     filepath = 'out.txt'
#     with open(filepath) as fp:
#         line = fp.readline()
#         cnt = 1
#         while line:       
#             if "step" in line:
#                 print(line.split(":")[1].strip())
#             line = fp.readline()
#             cnt += 1
