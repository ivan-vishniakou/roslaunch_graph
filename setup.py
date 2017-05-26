#!/usr/bin/env python
import distutils.core
import catkin_pkg.python_setup
d = catkin_pkg.python_setup.generate_distutils_setup(
   packages=['ros_tools'],
   package_dir={'':
                'src/'}
)
distutils.core.setup(**d)
