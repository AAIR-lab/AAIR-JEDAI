from django.urls import path
from django.views.generic.base import TemplateView

from django.urls import re_path
from django.views.static import serve
from django.conf import settings

from roblocks import views

urlpatterns = [
    path("input_id/", views.input_id, name='input_id'),
    path("keva_tutorial/",views.keva_tutorial,name='keva_tutorial'),
    path("dropdown/", views.dropdown, name='dropdown'),
    path("intro_video/", views.intro_video, name='intro_video'),
    path("", views.upload_view, name="upload_view"),
    path("plan_submit/", views.submit_plan, name="submit_plan"),
    path("call_TMP/", views.call_tmp, name="call_TMP"),
    path("start_TMP/", views.start_tmp, name="start_TMP"),
    path("run_plan/", views.run_plan, name="run_plan"),
    path("kill_tmp/", views.kill_tmp, name="kill_tmp"),
    path("terminate_tmp/", views.terminate_tmp, name="terminate_tmp"),
    path("get_problem_names/", views.get_problem_file_names, name="get_problem_names"),
    path("get_random/", views.get_random, name="get_random"),
    path("goal_submit/",views.goal_submit,name="goal_submit"),
    path("handle_custom_goals/",views.handle_custom_goals,name="handle_custom_goals"),
    path("handle_random_goals/",views.handle_random_goals,name="handle_random_goals"),
    path("save_plan/",views.save_plan,name="save_plan"),
    path("cafe_world_tutorial/",views.cafe_world_tutorial,name="cafe_world_tutorial"),
    path("hanoi_tutorial/",views.hanoi_tutorial,name="hanoi_tutorial"),
    path("handle_close/",views.handle_close,name="handle_close"),
    path("hello_msg/,",views.hello_msg,name='hello_msg'),
    path("inactivity/",views.inactivity,name='inactivity'),
    path("log_workspace/",views.log_workspace,name='log_workspace'),
    path("get_llm/",views.get_llm,name='get_llm'),
    path("see_action_costs/",views.see_action_costs,name='see_action_costs')

    
]

urlpatterns += [
    re_path(r'^docs/(?P<path>.*)', serve, {'document_root': settings.DOCS_ROOT})
]