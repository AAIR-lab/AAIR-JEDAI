# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models
import config
import os


# Create your models here.

class EBooksModel(models.Model):
    domain = models.FilePathField(path=config.PREBUILT_DOMAIN_FOLDERS, allow_folders=True, allow_files=False, blank=True, null=True)
    # problem = models.FilePathField(path=config.PREBUILT_DOMAIN_FOLDERS, match="problem", recursive=True, allow_folders=True, allow_files=False, blank=True, null=True)
    #env = models.FileField(upload_to='documents/', blank=False, null=True)
