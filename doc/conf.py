#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright 2025 Duda Andrada
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#

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
