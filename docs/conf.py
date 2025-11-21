import os
import sys
sys.path.insert(0, os.path.abspath('..'))

project = 'Robot Control Stack'
author = 'Tobias Jülg'
release = '0.4'

extensions = [
    'myst_parser',
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
]

templates_path = ['_templates']
exclude_patterns = []

html_theme = 'pydata_sphinx_theme'
html_static_path = ['_static']
html_logo = "images/rcs_logo_multiline.svg"
html_favicon = "images/favicon.ico"

html_theme_options = {
    "github_url": "https://github.com/RobotControlStack/robot-control-stack",
    "use_edit_page_button": True,
    "show_prev_next": False,
    "navbar_start": ["navbar-logo"],  # ensures the logo is shown
    "logo": {
        "image_light": "images/rcs_logo_multiline.svg",  # your PNG
        "image_dark": "images/rcs_logo_multiline.svg",   # can be same or a dark-mode version
    },

}

html_context = {
    "github_user": "RobotControlStack",
    "github_repo": "robot-control-stack",
    "github_version": "main",  # branch name
    "doc_path": "docs",        # relative path in the repo where your docs live
}


myst_enable_extensions = [
    "colon_fence",
    "deflist",
    "linkify",
]
