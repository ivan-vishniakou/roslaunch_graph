# -*- coding: utf-8 -*-
import sys
import wxversion
wxversion.select("2.8")
import wx
import xdot

import pygraphviz as gv


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
        else:
            print 'no_file'
            graph = gv.AGraph()
            graph.add_node('a', pos='0,0')
            
            graph.add_node('b', pos='10,10', label='test')
            
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