import os
import sys

# inject path to rcs package to enable autodoc/autoapi to find packages
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../python")))

project = "Robot Control Stack"
copyright = "2025, RCS Contributors"
author = "Tobias Jülg"
release = "0.5.2"
version = "0.5.2"

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "sphinx.ext.mathjax",
    "sphinx.ext.viewcode",
    "sphinx.ext.napoleon",
    "sphinx.ext.intersphinx",
    "sphinx_copybutton",
    "myst_parser",
    "sphinx_design",
]

# https://myst-parser.readthedocs.io/en/latest/syntax/optional.html
myst_enable_extensions = ["colon_fence", "dollarmath"]
myst_heading_anchors = 4

exclude_patterns = ["README.md"]

templates_path = ["_templates"]

html_theme = "pydata_sphinx_theme"
html_logo = "_static/rcs_logo_multiline.svg"
html_favicon = "_static/favicon.ico"

html_theme_options = {
    "use_edit_page_button": True,
    "icon_links": [
        {
            "name": "GitHub",
            "url": "https://github.com/RobotControlStack/robot-control-stack",
            "icon": "fa-brands fa-github",
        },
    ],
    "logo": {
        "image_dark": "_static/rcs_logo_multiline.svg",
    },
    "navbar_center": ["version-switcher", "navbar-nav"],
    "show_version_warning_banner": False,
    "switcher": {
        "json_url": "/_static/version_switcher.json",
        "version_match": "latest",
    },
}

html_context = {
    "display_github": True,
    "github_user": "RobotControlStack",
    "github_repo": "robot-control-stack",
    "github_version": "main",
    "conf_py_path": "/docs/",
}

html_static_path = ['_static']

# autodoc configs
autosummary_generate = True
autodoc_typehints = "description"
autodoc_member_order = "groupwise"

# Intersphinx mapping
intersphinx_mapping = {'gymnasium': ('https://gymnasium.farama.org/', None)}
