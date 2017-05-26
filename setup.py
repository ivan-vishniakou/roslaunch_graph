#!/usr/bin/env python
import distutils.core
import catkin_pkg.python_setup
d = catkin_pkg.python_setup.generate_distutils_setup(
   packages=['ros-tools'],
   package_dir={'ros-tools':
                'src/ros-tools'}
)
distutils.core.setup(**d)
