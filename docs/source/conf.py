# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------
import os
import sys
sys.path.insert(0, os.path.abspath('../../src')) # Point Sphinx to the src directory

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'blimp-utils'
copyright = '2025, Eli Ferrara'
author = 'Eli Ferrara'
release = '1.0.0'

# :license: The MIT License (MIT)
# :author: Eli Ferrara 
# :email: eli.ferrara256@gmail.com
# :version: V1.0.0
# :date: 2025-05-28
# :url: https://github.com/Ballistyxx/blimp-utils


# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',  # For Google and NumPy style docstrings
    'sphinx_rtd_theme',
    'sphinx.ext.githubpages',  # For GitHub Pages support
    'myst_parser', # For Markdown support
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store'] # Exclude build directory


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_favicon = '_static/favicon.ico'
html_title = 'blimp-utils Documentation'

# -- Options for autodoc -----------------------------------------------------
autodoc_mock_imports = [
    "RPi.GPIO", 
    "RPi", 
    "smbus2", 
    "smbus",
    "serial",
    "pigpio",
    "i2c_mp_usb"
] # Mock hardware-specific libraries
autodoc_member_order = 'bysource' # Optional: Order members by source order

# -- Options for MyST Parser -------------------------------------------------
source_suffix = {
    '.rst': 'restructuredtext',
    '.txt': 'markdown',
    '.md': 'markdown',
}
