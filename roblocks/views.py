# -*- coding: utf-8 -*-
from __future__ import unicode_literals

import json
import os
import shutil
import signal
import sys

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

# TODO why is this being set to a boolean?
process = True
server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
server_socket.bind(('127.0.0.1',1233))
server_socket.listen(1)
semantics = None
cs = None

# TODO check csrf
@csrf_exempt
def submit_plan(request):
    plan_submitted = request.GET.get('plan', None)
    # TODO display something that lets the user know the plan is being examined
    explanation_map = explainer_server.call_server(plan_submitted, semantics)
    success = "False" if explanation_map["failed"] else "True"
    data = {
        "explanation_map": explanation_map,
        "success": success
    }
    # this updates TMP to execute the plan on robot
    return JsonResponse(data)


def start_TMP(request):
    domain = get_domain_name()
    problem = get_problem_name()
    call_TMP_for_domain(domain,problem)
    return JsonResponse({})

def call_TMP(request):
    global cs
    plan_submitted = request.GET.get('plan', None)
    # TODO actually check for success
    success = True
    data_val = {}
    if success:
        data_val["success"] = "True"

    text_file = open(config.SOLUTION_DOCUMENT_FILE, "w")
    text_file.write(plan_submitted)
    text_file.close()

    # global process
    # res = process.communicate()[0]

    os.remove(config.SOLUTION_DOCUMENT_FILE)
    domain_name = get_domain_name()
    # call_TMP_for_domain(domain_name)
    (cs, address) = server_socket.accept()
    print(cs,address)
    print("connection accepted.")
    while True:
        msg = cs.recv(1024).decode("utf-8")
        print(msg)
        if msg == "ready":
            print("now sending msg")
            break
    cs.sendall(bytes(plan_submitted,"utf-8"))
    while True:
        msg = cs.recv(1024).decode("utf-8")
        if msg[0] == "u":
            '''
            update the bar here.. format will be u|#refined_actions|#total_actions 
            '''
            pass
        elif msg== "refined":
            '''
            TODO: Use this elif to report success.
            refinement done. ALL actions passed. report success
            '''
            break
        elif msg == "failure":
            '''
            TODO: Use this elif to report failure to the users. right now it just report success. 
            refinement reported failure. report failure
            '''
            break
    return JsonResponse(data_val)

def run_plan(request):
    global cs
    cs.sendall(bytes("run","utf-8"))
    while True:
        msg = cs.recv(1024).decode("utf-8")
        if msg == "run_complete":
            break
    return JsonResponse({})

def kill_tmp(request):
    global cs
    cs.sendall(bytes("terminate","utf-8"))
    cs.close()
    return JsonResponse({})


# Storing files uploaded by user and creating new files from it
def upload_view(request):
    global cs
    cleanup_documents()
    if not type(process) == bool and request.method != "POST":

        try:
            if cs is not None:
                if "closed" not in cs.__repr__():
                    cs.close()
                else:
                    (cs, address) = server_socket.accept()
                    data = cs.recv(1024)
                    cs.close()
            else:
                (cs, address) = server_socket.accept()
                cs.close()
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        except Exception as e:
            # TODO real loggings
            print(f"WARNING Error while killing process:", e)
    if request.method == 'POST':
        #print(request.POST)
        form = UploadBookForm(request.POST)
        name = config.PREBUILT_DOMAIN_FOLDERS
        #files = request.FILES.getlist('domain')
        #print(form.is_valid())
        """
        the_main_folder = request.POST['domain']
        the_base = ''
        while(len(the_base) < len(the_main_folder))
        """
        files = [request.POST['domain'] + '/domain.pddl', request.POST['domain'] + "/" + request.POST['problem'] + '/problem.pddl', request.POST['domain'] + "/"  + request.POST['problem'] + '/env.dae', request.POST['domain'] + config.SEMANTICS_FILE]
        if len(files) < 3:
            return render(request, 'roblocks/home.html', {
                'error': "please upload all files",
                'code': 404,
                'form': form,
            })
        image_address0 = request.POST['domain'] + "/" + request.POST['problem'] + '/img.png'
        image_address1 = request.POST['domain'] + "/" + request.POST['problem'] + '/img.jpg'
        image_address2 = request.POST['domain'] + "/" + request.POST['problem'] + '/img.jpeg'
        if(os.path.exists(config.STATIC_FOLDER + 'img.png')):
            os.remove(config.STATIC_FOLDER + 'img.png')
        if(os.path.exists(image_address0)):
            im0 = Image.open(image_address0)
            im1 = im0.copy()
            im1.save(config.STATIC_FOLDER + 'img.png')
        if(os.path.exists(image_address1)):
            print(image_address1)
            im0 = Image.open(image_address1)
            im1 = im0.copy()
            im1.save(config.STATIC_FOLDER + 'img.png')
        if(os.path.exists(image_address2)):
            im0 = Image.open(image_address2)
            im1 = im0.copy()
            im1.save(config.STATIC_FOLDER + 'img.png')
        set_file_instance_names(form, files)
        write_template_files()
        domain_name = get_domain_name()
        problem_name = get_problem_name()
        # TODO this will throw an error if there are no semantics, change this once they are optional
        with open(config.DOCUMENTS_PATH + config.SEMANTICS_FILE, "r") as file:
            semantics_json = file.read()
        global semantics
        semantics = json.loads(semantics_json)
        create_lattice()
        call_TMP_for_domain(domain_name, problem_name)
        return get_blockly_render(request, form, semantics_json)
    else:
        cleanup_documents()
        form = UploadBookForm()
        name = config.PREBUILT_DOMAIN_FOLDERS
        return render(request, 'roblocks/home.html', {
            'form': form,
            'name': name
        })


def get_blockly_render(request, form, semantics_json):
    problem_info = get_problem_info(semantics)
    return render(request, 'roblocks/blockly.html', {
        'form': form,
        'planning_elements': problem_info["planning_elements"],
        'goal': problem_info["goal_str"],
        'semantics': semantics_json
    })

def get_problem_file_names(request):
    domain = request.GET.get('domain')
    file_path = config.PREBUILT_DOMAIN_FOLDERS + '/' + domain
    files = os.listdir(domain)

    final = []
    for f in files:
        if "." not in f:
            final.append(f)
    
    result = {"problem_names":final}
    return JsonResponse(result)


def cleanup_documents():
    global semantics
    semantics = None

    # TODO get rid of/rename all the EBooks stuff
    files = [EBooks_Model.domain for EBooks_Model in EBooksModel.objects.all() if EBooks_Model.domain is not None and EBooks_Model.domain != ""]
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
            print('Failed to delete %s. Reason: %s' % (file_path, e))


def write_template_files():
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


def get_domain_name():
    with open(config.PROBLEM_DOCUMENT_FILE, 'r') as file:
        data = file.read().replace('\n', '')
    return data.split("domain ")[1].split(")")[0].replace(' ', '')

def get_problem_name():
    with open(config.PROBLEM_DOCUMENT_FILE, 'r') as file:
        data = file.read().replace('\n', '')
    return data.split("problem ")[1].split(")")[0].replace(' ', '')


def call_TMP_for_domain(domain_name,problem_name):
    python2_command = config.PYTHON_2_PATH + ' "' + file_dir + f'{config.TMP_PATH}" ' + domain_name + ' ' + problem_name
    my_env = os.environ.copy()
    global process
    print("call TMP now.............................")
    process = subprocess.Popen(python2_command,
                               shell=True,
                               env=my_env,
                               preexec_fn=os.setsid)


def set_file_instance_names(form, files):
    if form.is_valid():
        """
        for f in files:
            break
            file_instance = EBooksModel(domain=f)
            # TODO don't make assumptions on the names of the files
            if "domain" in file_instance.domain.name:
                file_instance.domain.name = "domainD.pddl"
            if "problem" in file_instance.domain.name:
                file_instance.domain.name = "problemP.pddl"
            if "dae" in file_instance.domain.name:
                file_instance.domain.name = "env.dae"
            file_instance.save()
        """
        with open(files[0], 'r') as domain, open(config.DOCUMENTS_PATH + '/domainD.pddl', 'w') as doc_media:
            for line in domain:
                doc_media.write(line)
            doc_media.close()
        with open(files[1], 'r') as problem, open(config.DOCUMENTS_PATH + '/problemP.pddl', 'w') as doc_media:
            for line in problem:
                doc_media.write(line)
            doc_media.close()
        with open(files[2], 'r') as env, open(config.DOCUMENTS_PATH + '/env.dae', 'w') as doc_media:
            for line in env:
                doc_media.write(line)
            doc_media.close()
        # TODO this will throw an error if there is no semantics file
        #  update this when the semantics file is made optional
        with open(files[3], 'r') as semantics_file, open(config.DOCUMENTS_PATH + config.SEMANTICS_FILE, 'w') as doc_media:
            for line in semantics_file:
                doc_media.write(line)
            doc_media.close()
