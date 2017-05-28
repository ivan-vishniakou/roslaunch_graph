# -*- coding: utf-8 -*-
import sys
import wxversion
wxversion.select("2.8")
import wx
import xdot

from roslib.packages import find_resource, get_pkg_dir, get_dir_pkg, \
                            InvalidROSPkgException
from ros_tools.trace_launch_files import LaunchFileParser


class RIElement(object):
    """
    
    """
    #Unique id for the node
    uid_counter = 0

    @staticmethod
    def wrap_in_html_table():
        #TODO
        pass

    def __init__(self):
        self.children = []
        self.input_args = []
        self.uid = RINode.uid_counter
        RIElement.uid_counter += 1

    def find_by_uid(self, uid):
        if self.uid == uid:
            return self
        for c in self.children:
            ri_el = c.find_by_uid(uid)
            if not ri_el is None:
                return ri_el
        return None

    def get_dot_lines(self):
        """Returns the lines of the dot representation of the element
        """
        s = '''%s [URL=%s, label="node %s" shape=record]''' % (self.uid,
                                                               self.uid,
                                                               self.uid)
        lines = [s]
        for c in self.children:
            lines.extend(c.get_dot_lines())
        return lines


class RILaunch(RIElement):

    def __init__(self, parsed_launch):
        super(RILaunch, self).__init__()
        
        self.parsed_launch = parsed_launch
        
        for l in parsed_launch.includes:
            self.children.append(RILaunch(l))
        
        self.input_args = self.parsed_launch.input_arg_dict.keys()
            
        for n in self.parsed_launch.nodes.keys():
            self.children.append(RINode(n, self.parsed_launch.nodes[n]))

    def get_dot_lines(self):
        header_dot_str = '{%s|%s}|%s' % (self.parsed_launch.package_name,
                                         self.parsed_launch.file_name,
                                         self.parsed_launch.path)
        
        output_args = sorted(self.parsed_launch.arg_dict.keys())
        input_vals = ['<in_%s> ' % a + self.parsed_launch.input_arg_dict[a]
                      if a in self.parsed_launch.input_arg_dict.keys() else ' '
                      for a in output_args]
        output_vals = ['<out_%s> ' % a + self.parsed_launch.arg_dict[a]
                      if a in self.parsed_launch.arg_dict.keys() else ' '
                      for a in output_args]
        args_dot_str = '{{'+'|'.join(input_vals)+'}|{'+\
                          '|'.join(output_args)+\
                          '}|{'+'|'.join(output_vals)+'}}'
        label='|'.join([header_dot_str, args_dot_str])

        s = '''%s [URL=%s, label="%s" shape=record];''' % (self.uid,
                                                          self.uid,
                                                          label)
        lines = [s]
        for c in self.children:
            lines.extend(c.get_dot_lines())
            edges = [('%s:out_%s' % (self.uid,a),
                      ('%s:in_%s' % (c.uid,a))) for a in output_args
                      if a in c.input_args]
            for u, v in edges:
                lines.append('%s -> %s;' % (u, v))
        return lines


class RINode(RIElement):

    def __init__(self, nodename, nodeparams):
        super(RINode, self).__init__()
        self.name = nodename
        self.params = nodeparams
        print nodeparams
        #self.input_args = input_args


class RoslaunchInspector(wx.Frame):
    """
    This class provides a GUI application for viewing roslaunch files.
    """
    def __init__(self, dot_filename=None):
        wx.Frame.__init__(self, None, -1, "Roslaunch inspector")
        self.graph_view = xdot.wxxdot.WxDotWindow(self, -1)
        #self.graph_view.set_filter('neato')
        
        self.graph_view.register_select_callback(self.select_cb)
        
        
        if not dot_filename is None:
            graph = gv.AGraph(filename=dot_filename, directed=True)
            graph.node = {'shape':'record'}
        else:
            package = 'openni2_launch'
            launch_file = 'openni2.launch'

            #package = 'cob_people_detection'
            #launch_file = 'people_detection_with_viewer.launch'
            
            launch_file_paths = find_resource(package, launch_file)
            #print launch_file_paths
    
            for path in launch_file_paths:
                launch_file_obj = LaunchFileParser(path)
                self.root = RILaunch(launch_file_obj)
            pass
            
        t = '''digraph {
               graph [rankdir=LR];
               node [label="\N"];
            '''
        t += '\n'.join(self.root.get_dot_lines())
        t += '}'
        self.graph_view.set_dotcode(t)
        self.graph_view.zoom_to_fit()


    def select_cb(self, item, event):
        """Event: Click to select a graph node."""
        if event.ButtonUp(wx.MOUSE_BTN_LEFT):
            el = self.root.find_by_uid(int(item.url))
            if isinstance(el, RILaunch):
                print el.parsed_launch.path

def main(argv):
    app = wx.App()
    frame = RoslaunchInspector(dot_filename=argv[1] if len(argv)>=2 else None)
    frame.Show()
    app.MainLoop()


if __name__ == '__main__':
    #assert len(sys.argv) == 2, 'dot filename input needed'
    main(sys.argv)