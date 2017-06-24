#!/bin/env python
import os
import re
import sys
import xml.etree.ElementTree as ET
from roslib.packages import find_resource, get_pkg_dir, get_dir_pkg, \
                            InvalidROSPkgException
from rospkg import ResourceNotFound
from collections import defaultdict

class RoslaunchElement(object):

    LAUNCH = 'launch'
    ARG = 'arg'
    NODE = 'node'
    MACHINE = 'machine'
    INCLUDE = 'include'
    REMAP = 'remap'
    ENV = 'env'
    PARAM = 'param'
    ROSPARAM = 'rosparam'
    GROUP = 'group'

    TRUE = 'true'
    FALSE = 'false'

    @staticmethod
    def load_xml(launch_file_path):
        launch_root = ET.parse(launch_file_path).getroot()
        launch_root.attrib['file'] = {'resolved':launch_file_path}
        return launch_root

    def __init__(self, xml, input_agr_dict={}, path=None):
        self.children = []
        if path is not None:
            self.path = path
        else:
            self.path = ''

        if isinstance(xml, ET.ElementTree):
            xml = xml.getroot()
        self.type = xml.tag if hasattr(xml, 'tag') else None
        self.attributes = defaultdict(lambda: {'resolved':'', 'variable':''}, xml.attrib)
        self.input_arg_dict = input_agr_dict.copy()

        self.text = xml.text
        for el in list(xml):
            self.parse_element(el, self.input_arg_dict)

        self.arg_dict = self.input_arg_dict.copy()

        if self.type == self.INCLUDE:
            try:
                included_xml = RoslaunchElement.load_xml(self.attributes['file']['resolved'])
                self.path = self.attributes['file']['resolved']
            except Exception as e:
                print 'Exception while parsing launch file: {}'.format(e.message)
                included_xml = ET.Element(self.LAUNCH) # Empty dummy xml
            else:
                for el in list(included_xml):
                    self.parse_element(el, self.arg_dict)

    def parse_element(self, xml, arg_dict):
        for k, v in xml.attrib.iteritems():
            xml.attrib[k] = self.resolve(v, arg_dict)
        if xml.tag == self.ARG:
            a_name = xml.attrib['name']['resolved']
            a_value = None
            if xml.attrib.has_key('default'):
                a_value = xml.attrib['default']['resolved']
            if arg_dict.has_key(a_name):
                a_value = arg_dict[a_name]
            if xml.attrib.has_key('value'):
                a_value = xml.attrib['value']['resolved']
            if a_value is None:
                print 'Unassigned value %s' % a_name
            else:
                arg_dict[a_name] = a_value
        if (xml.tag == self.INCLUDE or
            xml.tag == self.NODE    or
            xml.tag == self.GROUP   or
            xml.tag == self.MACHINE or
            xml.tag == self.ROSPARAM or
            xml.tag == self.PARAM or
            xml.tag == self.ARG or
            xml.tag == self.REMAP):
            roslaunch_element = RoslaunchElement(xml, arg_dict)
            self.children.append(roslaunch_element)
        pass

    def resolve(self, string, arg_dict):
        """Resolves $() substitutions in the roslaunch 
        """
        original = string
        regex = r'\$\(([^)]+)\)'
        match = re.search(regex, string)
        while match is not None:
            full_match = match.group(0)
            m = match.group(1)
            keyword = m.split(' ')[0]
            substitution = full_match.replace('$(', '$_(') #default substitution
            if keyword == 'arg':
                arg_name = m.split(' ')[1]
                if arg_name in arg_dict:
                    substitution = arg_dict[arg_name]
                else:
                    print 'Argument %s not set' % arg_name
            elif keyword == 'env':
                env_variable = m.split(' ')[1]
                if env_variable in os.environ:
                    substitution = os.environ[env_variable]
                else:
                    print 'Environment variable %s not set' % env_variable
            elif keyword == 'find':
                package_name = m.split(' ')[1]
                try:
                    package_path = get_pkg_dir(package_name)
                    substitution = package_path
                except InvalidROSPkgException:
                    print 'Could not find pacakage %s' % package_name 
            string = string.replace(full_match, substitution)
            match = re.search(regex, string)            
        return {'resolved':string, 'variable':original}

    def __str__(self, prefix = ''):
        lines = []
        lines.append("{}{}: {}".format(prefix, self.type, zip(self.attributes.keys(), self.attributes.values())))
        for c in self.children:
            lines.append(c.__str__(prefix=prefix+'\t'))
        return '\n'.join(lines)


if __name__ == '__main__':
    package = 'mdr_moveit_cob'
    launch_file = 'demo.launch'

    launch_file_path = find_resource(package, launch_file)[0]
    launch_root = ET.parse(launch_file_path)
    r = launch_root.getroot()

    m = RoslaunchElement(RoslaunchElement.load_xml(launch_file_path), launch_file_path)
    print m
    #print m.resolve('$(arg asd)')