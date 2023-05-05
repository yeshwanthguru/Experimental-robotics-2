# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html
import os
import sys
import sphinx_rtd_theme

# Add the scripts directory to the Python path
sys.path.insert(0, os.path.abspath('~/ros_ws/src/assignment2/scripts'))
# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'assignment2'
copyright = '2023, yeshwanth guru krishnakumar'
author = 'yeshwanth guru krishnakumar'
release = '0.1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
'sphinx.ext.doctest',
'sphinx.ext.intersphinx',
'sphinx.ext.todo',
'sphinx.ext.coverage',
'sphinx.ext.mathjax',
'sphinx.ext.ifconfig',
'sphinx.ext.viewcode',
'sphinx.ext.githubpages',
"sphinx.ext.napoleon",
'sphinx.ext.inheritance_diagram',
'breathe'
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

source_suffix = '.rst'
master_doc = 'index'
html_theme = 'sphinx_rtd_theme'

html_static_path = ['_static']

todo_include_todos = True

# -- Options for breathe ------------------------------------------------------
breathe_projects = {
    "assignment2": "_build/xml/"
}
breathe_default_project = "assignment2"
breathe_default_members = ('yeshwanth guru ', 'undoc-members')

