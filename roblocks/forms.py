from django import forms
from .models import *
from django.forms import ClearableFileInput
from django.forms import Select


class UploadBookForm(forms.ModelForm):
    class Meta:
        model = EBooksModel
        fields = ['domain']
        widgets = {
            'domain': Select(
                attrs={'multiple': False, 'id': 'first_menu', 'onclick': 'getMenu()', 'required': True}),
            # 'problem': Select(
            #     attrs={'multiple': False, 'id': 'second_menu', 'required': True}
            # )
        }
