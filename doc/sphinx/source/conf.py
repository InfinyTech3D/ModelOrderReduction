# -*- coding: utf-8 -*-
#
# Configuration file for the Sphinx documentation builder.
#
# This file does only contain a selection of the most common options. For a
# full list see the documentation:
# http://www.sphinx-doc.org/en/master/config

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys

sys.path.append(os.getcwd()+"/../../../python")

project = u'ModelOrderReduction'
copyright = u'2018, Defrost Team'
author = u'Defrost Team'

# The short X.Y version
version = u'1.0'
# The full version, including alpha/beta/rc tags
release = u'1.0'


# -- General configuration ---------------------------------------------------

# If your documentation needs a minimal Sphinx version, state it here.
#
# needs_sphinx = '1.0'

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.doctest',
    'sphinx.ext.autosummary',
    'sphinx.ext.intersphinx',
    #'sphinx.ext.ifconfig',
    #'sphinx.ext.mathjax',
    'sphinx.ext.viewcode',
    'sphinx.ext.githubpages',

    # C++ / Breathe
    # 'sphinx.ext.ifconfig',
    # 'sphinx.ext.todo',
    'breathe',
    'exhale'

    # Generate pdf
    # 'rst2pdf.pdfbuilder'
]


pdf_documents = [('index', u'morDoc', u'Model Order Reduction Documentation', u'Olivier Goury & Félix Vanneste')]

# index - master document
# rst2pdf - name of the generated pdf
# Sample rst2pdf doc - title of the pdf
# Your Name - author name in the pdf

## Include Python objects as they appear in source files
## Default: alphabetically ('alphabetical')
autodoc_member_order = 'bysource'
## Default flags used by autodoc directives
autodoc_default_flags = []
## Generate autodoc stubs with summaries from code
autosummary_generate = True

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# The suffix(es) of source filenames.
# You can specify multiple suffix as a list of string:
from recommonmark.parser import CommonMarkParser
source_suffix = ['.rst', '.md']
source_parsers = {'.md': CommonMarkParser}

# The master toctree document.
master_doc = 'index'

# The language for content autogenerated by Sphinx. Refer to documentation
# for a list of supported languages.
#
# This is also used if you do content translation via gettext catalogs.
# Usually you set "language" from the command line for these cases.
language = None

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path .
exclude_patterns = [u'_build', 'Thumbs.db', '.DS_Store']

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = 'sphinx'


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
# html_theme = 'alabaster'
# html_theme = "sphinx_rtd_theme"
# html_theme_path = ["_themes", ]

# on_rtd is whether we are on readthedocs.org, this line of code grabbed from docs.readthedocs.org
on_rtd = os.environ.get('READTHEDOCS', None) == 'True'

if not on_rtd:  # only import and set the theme if we're building docs locally
    import sphinx_rtd_theme
    html_theme = 'sphinx_rtd_theme'
    html_theme_path = [sphinx_rtd_theme.get_html_theme_path()]

# Theme options are theme-specific and customize the look and feel of a theme
# further.  For a list of options available for each theme, see the
# documentation.
#
# html_theme_options = {}

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = [] # '_static'

# Custom sidebar templates, must be a dictionary that maps document names
# to template names.
#
# The default sidebars (for documents that don't match any pattern) are
# defined by theme itself.  Builtin themes are using these templates by
# default: ``['localtoc.html', 'relations.html', 'sourcelink.html',
# 'searchbox.html']``
html_sidebars = {
    '**': [
            'about.html',
            'navigation.html',
            'searchbox.html',
    ]
}


# -- Options for HTMLHelp output ---------------------------------------------

# Output file base name for HTML help builder.
htmlhelp_basename = 'ModelOrderReductiondoc'


# -- Options for LaTeX output ------------------------------------------------

latex_elements = {
    # The paper size ('letterpaper' or 'a4paper').
    #
    # 'papersize': 'letterpaper',

    # The font size ('10pt', '11pt' or '12pt').
    #
    # 'pointsize': '10pt',

    # Additional stuff for the LaTeX preamble.
    #
    # 'preamble': '',

    # Latex figure (float) alignment
    #
    # 'figure_align': 'htbp',
}

# Grouping the document tree into LaTeX files. List of tuples
# (source start file, target name, title,
#  author, documentclass [howto, manual, or own class]).
latex_documents = [
    (master_doc, 'ModelOrderReduction.tex', u'ModelOrderReduction Documentation',
     u'Defrost Team', 'manual'),
]


# -- Options for manual page output ------------------------------------------

# One entry per manual page. List of tuples
# (source start file, name, description, authors, manual section).
man_pages = [
    (master_doc, 'modelorderreduction', u'ModelOrderReduction Documentation',
     [author], 1)
]


# -- Options for Texinfo output ----------------------------------------------

# Grouping the document tree into Texinfo files. List of tuples
# (source start file, target name, title, author,
#  dir menu entry, description, category)
texinfo_documents = [
    (master_doc, 'ModelOrderReduction', u'ModelOrderReduction Documentation',
     author, 'ModelOrderReduction', 'One line description of project.',
     'Miscellaneous'),
]


# -- Extension configuration -------------------------------------------------
from unittest import *
from mock import MagicMock

# class Mock(MagicMock):
#     __all__ = ['QApplication','pyqtSignal','pyqtSlot','QObject','QAbstractItemModel','QModelIndex','QTabWidget',
#         'QWebPage','QTableView','QWebView','QAbstractTableModel','Qt','QWidget','QPushButton','QDoubleSpinBox',
#         'QListWidget','QDialog','QSize','QTableWidget','QMainWindow','QTreeWidget',
#         'QAbstractItemDelegate','QColor','QGraphicsItemGroup','QGraphicsItem','QGraphicsPathItem',
#         'QGraphicsTextItem','QGraphicsRectItem','QGraphicsScene','QGraphicsView',]

#     def __init__(self, *args, **kwargs):
#         super(Mock, self).__init__()


#     @classmethod
#     def __getattr__(cls, name):
#         if name in ('__file__', '__path__'):
#             return os.devnull
#         else:
#             return Mock

#     @classmethod
#     def __setattr__(*args, **kwargs):
#         pass

#     def __setitem__(self, *args, **kwargs):
#         return

#     def __getitem__(self, *args, **kwargs):
#         return Mock

# class Mock(MagicMock):
#     @classmethod
#     def __getattr__(cls, name):
#             return MagicMock()

MOCK_MODULES = ['Sofa',
                'stlib','splib',
                'SofaPython','Quaternion','SofaPython.Quaternion',  # Needed for numerics
                'PythonScriptController', 'Sofa.PythonScriptController',
                'launcher',
                'yaml',
                'numpy',
                'PyQt4',"PyQt4.QtCore","PyQt4.QtGui"] # for AnimationManagerController but doesn't work...

MOCK_CLASSES = [
    # classes you are inheriting from
    "QAbstractItemModel",
    "QDialog",
    "QCompleter",
    "QWidget",
    "QLineEdit",
    "QMainWindow"
]

MockingClass = type('MockingClass', (), {}) 

class Mock(MagicMock):
    @classmethod
    def __getattr__(cls, name):
        if name in MOCK_CLASSES:
            # print("---------------------------->  "+name)
            return object #MockingClass
        return MagicMock()

sys.modules.update((mod_name, Mock()) for mod_name in MOCK_MODULES)
# sys.modules["QtCore.QAbstractItemModel"] = mock.Mock(TreeModel=object)
# autodoc_mock_imports= [ "math", # Standard import
#                         "Sofa",
#                         "stlib","wrapper","scene","splib","QtCore","QtGui"]
                        # "QtCore","QAbstractItemModel","QtCore.QAbstractItemModel"]

autoclass_content = 'both' # When auto doc a class it will automatically add the special method __init__ doc
add_function_parentheses = False

# Add mappings
intersphinx_mapping = {
    'stlib': ('https://stlib.readthedocs.io/en/latest/', None),
    'python': ('http://docs.python.org/3', None),
}

###########################################################
# -- To Build EXHALE --------------------------------------

# Setup the breathe extension
breathe_projects = {
    "ExhaleTest": "./doxyoutput/xml"
}

breathe_default_project = "ExhaleTest"
import textwrap
# Setup the exhale extension
exhale_args = {
    # These arguments are required
    "containmentFolder":     "./api",
    "rootFileName":          "library_root.rst",
    "rootFileTitle":         "Library API",
    "doxygenStripFromPath":  "../../../src/component",
    # Suggested optional arguments
    "createTreeView":        True,
    # TIP: if using the sphinx-bootstrap-theme, you need
    # "treeViewIsBootstrap": True,
    # "verboseBuild":True
    "exhaleExecutesDoxygen": True,
    "exhaleDoxygenStdin":    textwrap.dedent('''
        INPUT = ../../../src/component
        FILE_PATTERNS = *.h
        ''')
}

# # Tell sphinx what the primary language being documented is.
# primary_domain = 'cpp'

# # Tell sphinx what the pygments highlight language should be.
# highlight_language = 'cpp'

###############################################################################
# -- To Build Breath doc with ReadTheDocs--------------------------------------


# # Breathe 
# import glob


# listProject = [("loader",os.getcwd()+"/../../../src/component/loader"),
#                ("forcefield",os.getcwd()+"/../../../src/component/forcefield"),
#                ("mapping",os.getcwd()+"/../../../src/component/mapping")]

# breathe_projects_source = {}

# def addDoxiProject(projectName,pathToProject,filesType = "/*.h"):
#     filesPath = glob.glob(pathToProject+filesType)
#     filesNames = [os.path.basename(x) for x in filesPath]
#     print(filesNames)
#     breathe_projects_source[projectName] = (pathToProject,filesNames) 
 
# for projectName,pathToProject in listProject:
#     addDoxiProject(projectName,pathToProject)

# print (breathe_projects_source)

# breathe_default_members = ('members', 'undoc-members')

# breathe_doxygen_config_options = {
#     'ENABLE_PREPROCESSING' : 'YES',
#     'PREDEFINED' : 'DOXYGEN_SHOULD_SKIP_THIS',
#     'EXTRACT_LOCAL_METHODS' : 'YES',
#     'HIDE_UNDOC_MEMBERS' : 'YES',
#     'HIDE_UNDOC_CLASSES' : 'YES',
#     'HIDE_SCOPE_NAMES' : 'YES'}

# import subprocess

# # read_the_docs_build = os.environ.get('READTHEDOCS', None) == 'True'

# # if read_the_docs_build:

# #     subprocess.call('cd .. ; doxygen', shell=True)
# #     html_extra_path = ['../build/html']

# def run_doxygen(folder):
#     """Run the doxygen make command in the designated folder"""

#     try:
#         retcode = subprocess.call("cd %s; make" % folder, shell=True)
#         if retcode < 0:
#             sys.stderr.write("doxygen terminated by signal %s" % (-retcode))
#     except OSError as e:
#         sys.stderr.write("doxygen execution failed: %s" % e)


# def generate_doxygen_xml(app):
#     """Run the doxygen make commands if we're on the ReadTheDocs server"""

#     read_the_docs_build = os.environ.get('READTHEDOCS', None) == 'True'

#     if read_the_docs_build:

#         run_doxygen("../../../src/component/loader")
#         run_doxygen("../../../src/component/forcefield")
#         run_doxygen("../../../src/component/mapping")

# def setup(app):

#     # Add hook for building doxygen xml when needed
#     app.connect("builder-inited", generate_doxygen_xml)