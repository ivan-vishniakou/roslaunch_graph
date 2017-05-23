# -*- coding: utf-8 -*-
import sys
import wxversion
wxversion.select("2.8")
import wx
import xdot

from roslib.packages import find_resource, get_pkg_dir, get_dir_pkg, \
                            InvalidROSPkgException
from trace_launch_files import LaunchFileParser

import pygraphviz as gv


class RIElement(object):
    """
    
    """
    #Unique id for the node
    uid_counter = 0

    def __init__(self):
        self.children = []
        self.uid = RINode.uid_counter
        RIElement.uid_counter += 1

    def add_to_graph(self, gv_graph):
        gv_graph.add_node(self.uid,
                          label=str(self.uid)
                          )

    def find_by_uid(self, uid):
        if self.uid == uid:
            return self
        for c in self.children:
            ri_el = c.find_by_uid(uid)
            if not ri_el is None:
                return ri_el
        return None

class RILaunch(RIElement):
    
    def __init__(self, parsed_launch):
        super(RILaunch, self).__init__()
        
        self.parsed_launch = parsed_launch
        
        for l in parsed_launch.includes:
            self.children.append(RILaunch(l))
    
        for n in self.parsed_launch.nodes.keys():
            self.children.append(RINode(n, self.parsed_launch.nodes[n]))


    def add_to_graph(self, gv_graph):
        
        header_dot_str = '{%s|%s}|%s' % (self.parsed_launch.package_name,
                                         self.parsed_launch.file_name,
                                         self.parsed_launch.path)
        
        output_args = sorted(self.parsed_launch.arg_dict.keys())
        input_vals = [self.parsed_launch.input_arg_dict[a]
                      if a in self.parsed_launch.input_arg_dict.keys() else ' '
                      for a in output_args]
        output_vals = [self.parsed_launch.arg_dict[a]
                      if a in self.parsed_launch.arg_dict.keys() else ' '
                      for a in output_args]
        args_dot_str = '{{'+'|'.join(input_vals)+'}|{'+\
                          '|'.join(output_args)+\
                          '}|{'+'|'.join(output_vals)+'}}'
        gv_graph.add_node(self.uid,
                          shape='record',
                          label='|'.join([header_dot_str, args_dot_str]),
                            URL=str(self.uid)
                          )
        for c in self.children:
            c.add_to_graph(gv_graph)
            gv_graph.add_edge(self.uid, c.uid, dir='forward')


class RINode(RIElement):

    def __init__(self, nodename, nodeparams):
        super(RINode, self).__init__()
        self.name = nodename
        self.params = nodeparams
        #self.input_args = input_args

    def add_to_graph(self, gv_graph):
        
        header_dot_str = '%s|{%s|%s}' % (self.name,
                                         self.params['package'],
                                         self.params['type'])
        
        gv_graph.add_node(self.uid,
                          shape='record',
                          label=header_dot_str,
                          )
        pass

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
            package = 'mdr_bringup'
            launch_file = 'openni2.launch'
            launch_file_paths = find_resource(package, launch_file)
            #print launch_file_paths
    
            for path in launch_file_paths:
                launch_file_obj = LaunchFileParser(path)
                self.root = RILaunch(launch_file_obj)
            pass
            
            graph = gv.AGraph(rankdir='LR')
            self.root.add_to_graph(graph)
            #graph.add_node('a', shape='record', label='Launchfile|{params|{a|s}|b}|3', pos='0,0!')

            #graph.add_subgraph(nbunch=nlist, name='asd', label='s')
            #n = RINode()
            #n.add_to_graph(graph)
            
        self.graph_view.set_dotcode(graph.to_string())
        print graph


    def select_cb(self, item, event):
        """Event: Click to select a graph node to display user data and update the graph."""
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