#!/bin/env python
import os
import re
import sys
import xml.etree.ElementTree as ET
from roslib.packages import find_resource, get_pkg_dir, InvalidROSPkgException
from rospkg import ResourceNotFound


class LaunchRootParser(object):
    def __init__(self, launch_root):
        self.includes = []
        self.groups = {}
        self.arg_dict = {}
        self.conditions = {}
        self.meaningless_conditions = []
        self.parse_launch_root(launch_root)
        pass

    def parse_launch_root(self, launch_root):
        arguments = launch_root.findall('arg')
        for arg in arguments:
            name, value = self.parse_arg(arg)
            self.arg_dict[name] = value
            pass

        include_elements = launch_root.findall('include')
        for include_element in include_elements:
            self.includes.append(self.parse_include(include_element))
            pass

        group_elements = launch_root.findall('group')
        for group_element in group_elements:
            self.parse_group(group_element)
            pass
        return

    def parse_include(self, include_element):
        include_file = self.parse_xml_value(include_element.get('file'))
        return LaunchFileParser(include_file)

    def parse_group(self, group_element):
        ns = group_element.get('ns')
        group_key = len(self.groups.keys())
        if ns is not None:
            group_key = ns
        self.groups[group_key] = self.parse_launch_root(group_element)
        pass

    def parse_xml_value(self, value_string):
        match = re.match(r'.*\$\(optenv (\S+).*\).*', value_string)
        if match is not None:
            env_variable = match.group(1)
            if env_variable not in os.environ:
                print('environment variable %s is not set' % env_variable)
                return None
            value_string = re.sub(r'\$\(optenv \S+.*\)',
                                  os.environ[env_variable], value_string)
            pass

        match = re.match(r'.*\$\(env (\S+).*\).*', value_string)
        if match is not None:
            env_variable = match.group(1)
            if env_variable not in os.environ:
                print('environment variable %s is not set' % env_variable)
                return None
            value_string = re.sub(r'\$\(env \S+.*\)',
                                  os.environ[env_variable], value_string)
            pass

        match = re.match(r'.*\$\(find (\S+)\).*', value_string)
        if match is not None:
            package_name = match.group(1)
            try:
                package_path = get_pkg_dir(package_name)
            except InvalidROSPkgException as e:
                print('package %s is invalid: %s' % (package_name, e))
                return None

            value_string = re.sub(r'\$\(find \S+\)', package_path,
                                  value_string)

        match = re.match(r'.*\$\(arg (\S+)\).*', value_string)
        if match is not None:
            arg_name = match.group(1)
            if arg_name not in self.arg_dict:
                print('arg not defined: %s' % (arg_name))
                return None
            value_string =  re.sub(r'\$\(arg \S+\)', self.arg_dict[arg_name],
                                   value_string)

        return value_string

    def parse_arg(self, argument):
        name = argument.get('name')
        assert name is not None, 'invalid launch: arg tag must have name attribute'
        if argument.get('default') is None:
            print('argument %s does not have a default value' % name)
            default_value = None
        else:
            default_value = self.parse_xml_value(argument.get('default'))
            pass
        return name, default_value


class LaunchFileParser(LaunchRootParser):
    def __init__(self, launch_file_path):
        try:
            launch_root = ET.parse(launch_file_path).getroot()
        except IOError as e:
            print('IOError parsing %s: %s' % (launch_file_path, e))
            sys.exit(1)
        print('parsing launch file %s' % launch_file_path)
        super(LaunchFileParser, self).__init__(launch_root)
        pass
    pass


def main(argv):
    print('package: ' + argv[1])
    print('launch file: ' + argv[2])
    package = argv[1]
    launch_file = argv[2]
    try:
        launch_file_paths = find_resource(package, launch_file)
    except ResourceNotFound as e:
        print('Unable to find package %s: %s' % (package, e))
        return
    if len(launch_file_paths) == 0:
        print('Unable to find launch file %s' % launch_file)
        return
    print('Found launch files:\n%s' % (launch_file_paths))
    launch_records = []
    for path in launch_file_paths:
        launch_records.append(LaunchFileParser(path))
        pass
    return


if __name__ == '__main__':
    assert len(sys.argv) == 3, 'script accept exactly 2 arguments: ros package name and launch file'
    main(sys.argv)
