# -*- coding: utf-8 -*-
import sys
import wxversion
wxversion.select("2.8")
import wx
import xdot

import pygraphviz as gv


class RINode(object):
    """
    This class wraps pygraphviz nodes and takes care of
    their layout
    """
    #Unique id for the node
    uid_counter = 0
    
    def __init__(self):
        self._uid = RINode.uid_counter
        RINode.uid_counter += 1
    
    def add_to_graph(self, gv_graph):
        gv_graph.add_node(self._uid)
        
    def get_gv_attr(self):
        params = {}
        

        
class RoslaunchInspector(wx.Frame):
    """
    This class provides a GUI application for viewing roslaunch files.
    """
    def __init__(self, dot_filename=None):
        wx.Frame.__init__(self, None, -1, "Roslaunch inspector")
        self.graph_view = xdot.wxxdot.WxDotWindow(self, -1)
        self.graph_view.set_filter('neato')
        
        if not dot_filename is None:
            graph = gv.AGraph(filename=dot_filename)
            graph.node = {'shape':'record'}
        else:
            print 'no_file'
            graph = gv.AGraph(rankdir='LR')
            graph.add_node('a', shape='record', label='Launchfile|{params|{a|s}|b}|3', pos='0,0!')

            #graph.add_subgraph(nbunch=nlist, name='asd', label='s')
            #n = RINode()
            #n.add_to_graph(graph)
            
        self.graph_view.set_dotcode(graph.to_string())
        print graph
        
                


def main(argv):
    app = wx.App()
    frame = RoslaunchInspector(dot_filename=argv[1] if len(argv)>=2 else None)
    frame.Show()
    app.MainLoop()


if __name__ == '__main__':
    #assert len(sys.argv) == 2, 'dot filename input needed'
    main(sys.argv)