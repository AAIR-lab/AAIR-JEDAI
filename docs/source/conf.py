# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys
sys.path.insert(0, os.path.abspath('.'))
import sphinx_bootstrap_theme

# -- Project information -----------------------------------------------------

project = 'JEDAI'
copyright = '2022, AAIR Lab, ASU'
author = 'AAIR Lab, ASU'


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
# html_theme = 'alabaster'
# html_theme = 'sphinx_rtd_theme'
#html_theme = "sphinx_documatt_theme"
html_theme = 'bootstrap'
html_theme_path = sphinx_bootstrap_theme.get_html_theme_path()
html_show_sourcelink = False
html_theme_options = {
    # Disable showing the sidebar. Defaults to 'false'
    'nosidebar': True,
    # Navigation bar title. (Default: ``project`` value)
    'navbar_title': " ",
    # 'header_searchbox': False,
    # Tab name for entire site. (Default: "Site")
    'navbar_site_name': "JEDAI Documentation",
    'navbar_links': [
        ("JEDAI", "/", True),
        ("AAIR Lab", "https://aair-lab.github.io", True),
    ],
    'navbar_class': "navbar navbar-inverse",

    # Fix navigation bar to top of page?
    # Values: "true" (default) or "false"
    'navbar_fixed_top': "true",

    # Location of link to source.
    # Options are "nav" (default), "footer".
    'source_link_position': "nav",
}
html_sidebars = {
    '**': [
        'about.html',
        'navigation.html',
    ]
}
# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']
html_css_files = [
    'style.css',
]
master_doc = 'index'
extensions = [
    'sphinx_copybutton',
]

# html_logo = "images/aair_logo.png"
# html_theme_options = {
#     'logo_only': True,
#     'display_version': False,
# }
