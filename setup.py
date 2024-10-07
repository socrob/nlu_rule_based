#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['nlu_rule_based_ros'],
 package_dir={'nlu_rule_based_ros': 'src/nlu_rule_based_ros'}
)

setup(**d)
