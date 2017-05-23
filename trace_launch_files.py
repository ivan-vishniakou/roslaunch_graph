#!/bin/env python
import os
import re
import sys
import xml.etree.ElementTree as ET
from roslib.packages import find_resource, get_pkg_dir, get_dir_pkg, \
                            InvalidROSPkgException
from rospkg import ResourceNotFound


INDENT = '  '


class LaunchRootParser(object):
    def __init__(self, launch_root, arg_dict={}):
        self.includes = []
        self.groups = {}
        self.nodes = {}                     # TODO
        self.arg_dict = arg_dict
        self.input_arg_dict = arg_dict.copy()
        self.conditions = {}                # TODO
        self.meaningless_conditions = []    # TODO
        self.issues = {}
        self.parse_launch_root(launch_root)
        pass

    def add_issue(self, issue):
        if issue in self.issues:
            self.issues[issue] += 1
        else:
            self.issues[issue] = 1
            pass
        pass

    def parse_launch_root(self, launch_root):
        arguments = launch_root.findall('arg')
        for arg in arguments:
            name, default_value, value = self.parse_argument(arg)
            if default_value is None:
                self.add_issue('launch root argument \'%s\' does not have a' \
                               ' default value' % name)
                continue
            if name not in self.arg_dict:
                self.arg_dict[name] = default_value
                pass
            pass

        node_elements = launch_root.findall('node')
        for node_element in node_elements:
            self.parse_node(node_element)
            pass

        group_elements = launch_root.findall('group')
        for group_element in group_elements:
            self.parse_group(group_element)
            pass

        include_elements = launch_root.findall('include')
        for include_element in include_elements:
            self.parse_include(include_element)
            pass
        return

    def parse_node(self, node_element):
        node_name = self.parse_xml_value(node_element.get('name'))
        if node_name is None:
            sys.stderr.write('node element missing a name argument')
            return

        package = self.parse_xml_value(node_element.get('pkg'))
        if package is None:
            self.add_issue('error: node %s has no pkg tag' % node_name)
            return

        type_name = self.parse_xml_value(node_element.get('type'))
        if type_name is None:
            self.add_issue('error: node %s has no type tag' % type_name)
            return

        self.nodes[node_name] = {'package' : package, 'type' : type_name}
        pass

    def parse_include(self, include_element):
        include_file = self.parse_xml_value(include_element.get('file'))
        if include_file is not None:
            include_arguments = include_element.findall('arg')
            arg_dict = {}
            for arg in include_arguments:
                name, default_value, value = self.parse_argument(arg)
                if default_value is not None:
                    self.add_issue('include argument \'%s\' use \'default\' key' % name)
                    arg_dict[name] = default_value
                    pass
                if value is not None:
                    arg_dict[name] = value
                    pass
                pass
            self.includes.append(LaunchFileParser(include_file, arg_dict))
        return

    def parse_group(self, group_element):
        ns = group_element.get('ns')
        group_key = len(self.groups)
        if ns is not None:
            group_key = ns
            pass
        self.groups[group_key] = LaunchRootParser(group_element, self.arg_dict)
        pass

    def parse_xml_value(self, value_string):
        if value_string is None:
            return None
        value_string = self.parse_env_keyword(value_string)
        value_string = self.parse_find_keyword(value_string)
        value_string = self.parse_arg_keyword(value_string)
        return value_string

    def parse_arg_keyword(self, string):
        match = re.match(r'.*\$\(arg (\S+)\).*', string)
        while match is not None:
            arg_name = match.group(1)
            if arg_name not in self.arg_dict:
                self.add_issue('argument \'%s\' not defined' % (arg_name))
                return None
            sub_string = r'\$\(arg %s\)' % arg_name
            string = re.sub(sub_string, self.arg_dict[arg_name], string)
            match = re.match(r'.*\$\(arg (\S+)\).*', string)
            pass
        return string

    def parse_find_keyword(self, string):
        match = re.match(r'.*\$\(find (\S+)\).*', string)
        if match is not None:
            package_name = match.group(1)
            try:
                package_path = get_pkg_dir(package_name)
            except InvalidROSPkgException:
                self.add_issue('cannot find package %s' % (package_name))
                return None

            string = re.sub(r'\$\(find \S+\)', package_path, string)
        return string

    def parse_env_keyword(self, string):
        match = re.match(r'.*\$\(optenv (\S+).*\).*', string)
        if match is not None:
            env_variable = match.group(1)
            if env_variable not in os.environ:
                self.add_issue('environment variable %s is not set' % env_variable)
                return None
            string = re.sub(r'\$\(optenv \S+.*\)',
                            os.environ[env_variable], string)
            pass

        match = re.match(r'.*\$\(env (\S+).*\).*', string)
        if match is not None:
            env_variable = match.group(1)
            if env_variable not in os.environ:
                self.add_issue('environment variable %s is not set' % env_variable)
                return None
            string = re.sub(r'\$\(env \S+.*\)',
                            os.environ[env_variable], string)
            pass
        return string

    def parse_argument(self, argument):
        name = argument.get('name')
        assert name is not None, 'invalid launch file: arg tag must have name attribute'
        default_value = self.parse_xml_value(argument.get('default'))
        value = self.parse_xml_value(argument.get('value'))
        return name, default_value, value

    def print_launch_components_recursive(self, prefix=''):
        if len(self.issues) > 0:
            print(prefix + 'Issues:')
            pass
        for issue, count in self.issues.iteritems():
            print(prefix + INDENT + '%s, count: %2d' % (issue, count))
            pass

        if len(self.nodes) > 0:
            print(prefix + 'Nodes:')
            pass
        for node_name, info in self.nodes.iteritems():
            print(prefix + INDENT + 'Name: %s, package: %s, type: %s'
                  % (node_name, info['package'], info['type']))
            pass

        if len(self.includes) > 0:
            print(prefix + 'Includes:')
            pass
        for include in self.includes:
            print(prefix + INDENT + 'File: %s, package: %s'
                  % (os.path.basename(include.path), include.package_name))
            pass

        if len(self.groups) > 0:
            print(prefix + 'Groups:')
            pass
        for ns, group_object in self.groups.iteritems():
            print(prefix + INDENT + 'ns: %s' % (ns))
            group_object.print_launch_components_recursive(prefix + INDENT*2)
            pass
        return

    def print_includes_recursive(self, prefix=''):
        for include_object in self.includes:
            include_object.print_launch_components_recursive(prefix + INDENT)
            pass
        for group_object in self.groups.itervalues():
            group_object.print_includes_recursive(prefix + INDENT)
            pass
        return

    pass


class LaunchFileParser(LaunchRootParser):
    def __init__(self, launch_file_path, arg_dict={}):
        try:
            launch_root = ET.parse(launch_file_path).getroot()
        except IOError as e:
            sys.stderr.write('IOError parsing %s: %s' % (launch_file_path, e))
            #sys.exit(1)
            launch_root = ET.Element('launch')
        self.path = launch_file_path
        self.file_name = os.path.basename(self.path)
        self.package_name = get_dir_pkg(launch_file_path)[1]
        if self.package_name is None:
            sys.stderr.write('no package found for file %s' % (self.path))
            self.package_name = ''
            pass
        print('\nparsing launch file %s' % launch_file_path)
        super(LaunchFileParser, self).__init__(launch_root, arg_dict=arg_dict)
        pass

    def print_launch_components_recursive(self, prefix=''):
        print('\n' + prefix + '-' * 50)
        print(prefix + 'File:    ' + self.file_name)
        print(prefix + 'Package: ' + self.package_name)
        print(prefix + '-' * 50)
        super(LaunchFileParser, self).print_launch_components_recursive(prefix)
        super(LaunchFileParser, self).print_includes_recursive(prefix)
        return
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
        launch_file_obj = LaunchFileParser(path)
        launch_records.append(launch_file_obj)
        launch_file_obj.print_launch_components_recursive()
        pass
    return


if __name__ == '__main__':
    assert len(sys.argv) == 3, 'script accept exactly 2 arguments: ros package name and launch file'
    main(sys.argv)
