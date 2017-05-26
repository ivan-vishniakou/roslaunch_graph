# ros_tools
tools to make life with ROS easier

## Tracing launch files
```
rosrun ros_tools trace_launch_files <package_name> <launch_file_name>
```

## Package creator
Based on catkin-tools, creates packages based on the team's guidelines.

This is a work in progress, please feel free to contribute.

Packages that can be created:
* metapackage
* package
* scenarios
* skills

### Usage
In the directory where you want to create a new package run:
```bash
rosrun ros_tools create_package <package_name> --type <pkg_type>
```
Where `<pkg_type>` can be:
* meta : metapackage
* pkg : package
* scenario
* skill
