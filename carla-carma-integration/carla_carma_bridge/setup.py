# Copyright (c) 2021 Intel Corporation
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# -*- coding: utf-8 -*-
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['carla_carma_bridge'],
    package_dir={'': 'src'},
)

setup(**d)
