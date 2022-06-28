# -*- coding: utf-8 -*-
from __future__ import unicode_literals

import json
import logging
import shutil
import signal

import config

from django.shortcuts import render

from Explainer import explainer_server, create_lattice
from .forms import UploadBookForm
from django.views.decorators.csrf import csrf_exempt
from .models import EBooksModel
from .problem_info_generator import get_problem_info
import os
import subprocess
from django.http import JsonResponse
import socket
from PIL import Image

file_dir = os.path.dirname(os.path.abspath(__file__))[:-8]

tmp_process = None
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('127.0.0.1', config.PORT_NUMBER))
server_socket.listen(1)
semantics = None
cs = None
log = logging.getLogger(__name__)


# TODO check csrf
@csrf_exempt
# /plan_submit
def submit_plan(request):
    submitted_plan = request.GET.get('plan')
    log.debug(f"Entering /plan_submit endpoint with plan: {submitted_plan}")
    log.info("Calling explanation code")
    explanation_map, explained_plan = explainer_server.call_server(submitted_plan, semantics)
    log.info(f"Explanation code reports plan to be {'unsuccessful' if explanation_map['failed'] else 'successful'}")
    success = "False" if explanation_map["failed"] else "True"
    data = {
        "explanation_map": explanation_map,
        "success": success,
        "execution_plan": explained_plan
    }
    # this updates TMP to execute the plan on robot
    return JsonResponse(data)


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
    cs.close()
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


# /
def upload_view(request):
    log.debug("Entering root endpoint")
    _cleanup_documents()
    if request.method == 'POST':
        return _handle_post_request_to_upload_view(request)

    if tmp_process is not None:
        _try_to_kill_tmp_and_close_socket()

    form = UploadBookForm()
    name = config.PREBUILT_DOMAIN_FOLDERS
    return render(request, 'roblocks/home.html', {
        'form': form,
        'name': name
    })


def _handle_post_request_to_upload_view(request):
    form = UploadBookForm(request.POST)
    files = [request.POST['domain'] + '/domain.pddl',
             request.POST['domain'] + "/" + request.POST['problem'] + '/problem.pddl',
             request.POST['domain'] + "/" + request.POST['problem'] + '/env.dae',
             request.POST['domain'] + config.SEMANTICS_FILE]
    # TODO
    if len(files) < 3:
        return render(request, 'roblocks/home.html', {
            'error': "please upload all files",
            'code': 404,
            'form': form,
        })
    _save_goal_image_to_static_files(request.POST['domain'], request.POST['problem'])
    _set_file_instance_names(form, files)
    _write_template_files()
    domain_name = _get_domain_name_from_documents()
    problem_name = _get_problem_name_from_documents()
    # TODO this will throw an error if there are no semantics, change this once they are optional
    with open(config.DOCUMENTS_PATH + config.SEMANTICS_FILE, "r") as file:
        semantics_json = file.read()
    global semantics
    semantics = json.loads(semantics_json)
    create_lattice()
    _spawn_tmp_for_domain_and_problem(domain_name, problem_name)
    return _get_blockly_render(request, form, semantics_json)


def _save_goal_image_to_static_files(domain_path, problem_name):
    goal_image_for_this_domain = domain_path + "/" + problem_name + "/" + config.GOAL_IMAGE_NAME_IN_MODULES
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
                cs.close()
            else:
                log.debug("Closing cs with accept() and one recv()")
                (cs, address) = server_socket.accept()
                data = cs.recv(1024)
                cs.close()
        else:
            log.debug("cs is None, closing with accept()")
            (cs, address) = server_socket.accept()
            cs.close()
        log.debug("Sending kill signal to TMP process")
        os.killpg(os.getpgid(tmp_process.pid), signal.SIGTERM)
    except Exception as e:
        # log.exception automatically adds exception trace from handler
        log.exception("Exception while killing TMP")


def _get_blockly_render(request, form, semantics_json):
    problem_info = get_problem_info(semantics)
    return render(request, 'roblocks/training-area.html', {
        'form': form,
        'problem_actions': json.dumps(problem_info["actions"]),
        'problem_objects': json.dumps(problem_info["objects"]),
        'types_to_supertypes': json.dumps(problem_info["types_to_supertypes"]),
        'goal_str': problem_info["goal_str"],
        'semantics': semantics_json
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


def _get_domain_name_from_documents():
    with open(config.PROBLEM_DOCUMENT_FILE, 'r') as file:
        data = file.read().replace('\n', '')
    return data.split("domain ")[1].split(")")[0].replace(' ', '')


def _get_problem_name_from_documents():
    with open(config.PROBLEM_DOCUMENT_FILE, 'r') as file:
        data = file.read().replace('\n', '')
    return data.split("problem ")[1].split(")")[0].replace(' ', '')


def _spawn_tmp_for_domain_and_problem(domain_name, problem_name):
    log.info(f"Spawning TMP process with domain name {domain_name} and problem name {problem_name}")
    python2_command = config.PYTHON_2_PATH + ' "' + file_dir + f'{config.TMP_PATH}" ' + domain_name + ' ' + problem_name
    my_env = os.environ.copy()
    global tmp_process
    tmp_process = subprocess.Popen(python2_command,
                                   shell=True,
                                   env=my_env,
                                   preexec_fn=os.setsid)


def _set_file_instance_names(form, files):
    if form.is_valid():
        with open(files[0], 'r') as domain, open(config.DOMAIN_DOCUMENT_FILE, 'w') as doc_media:
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
