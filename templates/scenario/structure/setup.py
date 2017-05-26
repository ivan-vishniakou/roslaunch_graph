#!/usr/bin/env python
import distutils.core
import catkin_pkg.python_setup
d = catkin_pkg.python_setup.generate_distutils_setup(
   packages=["project_name"],
   package_dir={"project_name":
                "ros/src/project_name"}
)
distutils.core.setup(**d)
