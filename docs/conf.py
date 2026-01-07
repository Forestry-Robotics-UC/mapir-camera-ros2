#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]

# Make local python packages importable for autodoc when building from source.
sys.path.insert(0, str(PROJECT_ROOT))

project = 'MAPIR Survey3 Camera'
author = 'Duda Andrada'
copyright = f'{author}'  # noqa: A001

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'sphinx.ext.autosummary',
    'myst_parser',
]

templates_path = ['_templates']
exclude_patterns = ['_build']

autosummary_generate = True
napoleon_google_docstring = True
napoleon_numpy_docstring = False

# Docs should be buildable without ROS installed.
autodoc_mock_imports = [
    'ament_index_python',
    'camera_info_manager',
    'cv2',
    'cv_bridge',
    'launch',
    'launch_ros',
    'numpy',
    'rclpy',
    'sensor_msgs',
    'tf2_ros',
]

myst_enable_extensions = [
    'colon_fence',
    'deflist',
]

html_static_path = ['_static']

html_theme = 'furo'
html_logo = '_static/fruc_logo.png'
html_theme_options = {
    'sidebar_hide_name': True,
    'light_logo': 'fruc_logo.png',
    'dark_logo': 'fruc_logo.png',
}

master_doc = 'index'
root_doc = 'index'
