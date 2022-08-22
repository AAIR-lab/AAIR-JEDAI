from django import forms
from .models import *
from django.forms import ClearableFileInput
from django.forms import Select


class UploadBookForm(forms.ModelForm):
    class Meta:
        model = EBooksModel
        fields = ['domain']
        widgets = {
            # TODO what is this? why can't we just use html?
            'domain': Select(
                attrs={'multiple': False, 'id': 'domain_select', 'onclick': 'getMenu()', 'required': True}),
        }
