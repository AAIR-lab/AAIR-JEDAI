from django.urls import path
from django.views.generic.base import TemplateView

from django.urls import re_path
from django.views.static import serve
from django.conf import settings

from roblocks import views

urlpatterns = [
    path("", views.upload_view, name="upload_view"),
    path("plan_submit/", views.submit_plan, name="submit_plan"),
    path("call_TMP/", views.call_tmp, name="call_TMP"),
    path("start_TMP/", views.start_tmp, name="start_TMP"),
    path("run_plan/", views.run_plan, name="run_plan"),
    path("kill_tmp/", views.kill_tmp, name="kill_tmp"),
    path("get_problem_names/", views.get_problem_file_names, name="get_problem_names")

]

urlpatterns += [
    re_path(r'^docs/(?P<path>.*)', serve, {'document_root': settings.DOCS_ROOT})
]