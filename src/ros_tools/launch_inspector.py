# -*- coding: utf-8 -*-
import sys
import wxversion
wxversion.select("2.8")
import wx
import xdot
from subprocess import call

from roslib.packages import find_resource, get_pkg_dir, get_dir_pkg, \
                            InvalidROSPkgException
from trace_launch_files import LaunchFileParser
from launch_parser import RoslaunchElement

class HTML(object):
    """
    Helper class for styling the graphvis nodes.
    """
    TABLE   = 'TABLE'
    TD      = 'TD'
    TR      = 'TR'
    BR      = '<BR/>'
    
    ST_TABLE = 'BORDER="0" CELLBORDER="1" CELLSPACING="0"'
    ST_HEADER = 'COLSPAN="2"'
    
    @staticmethod
    def wrap(TAG, string, attribs='', portname=None):
        """Wraps string into HTML as in the template:
        <TAG attribs port="portname">string</TAG>
        """
        if len(attribs)>0:
            attribs = ' ' + attribs
        if not portname is None:
            attribs += ' PORT="%s"' % portname
        return '<%s%s>%s</%s>' % (TAG,
                                  attribs,
                                  string,
                                  TAG)
    
    
class RIElement(object):
    """
    Base class for nodes drawn in graphvis.
    """
    #Unique id for the node
    uid_counter = 0
    
    @staticmethod
    def trim_str(s):
        return s[:50]+' ... '+s[-50:] if len(s)>100 else s

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
        s = '''%s [URL=%s, label="node %s" shape=record];''' % (self.uid,
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
            
        for r in self.parsed_launch.rosparams:
            self.children.append(RIRosparam(r))

    def get_dot_lines(self):
        header_dot_str = '<head>\n %s \t %s \n \t|%s' % (self.parsed_launch.package_name,
                                                         self.parsed_launch.file_name,
                                                         self.parsed_launch.path)
        output_args = sorted(self.parsed_launch.arg_dict.keys())
        input_vals = ['<in_%s> ' % a + RIElement.trim_str(
                                        self.parsed_launch.input_arg_dict[a]
                                                         )
                      if a in self.parsed_launch.input_arg_dict.keys() else ' '
                      for a in output_args]
        output_vals = ['<out_%s> ' % a + RIElement.trim_str(
                                        self.parsed_launch.arg_dict[a]
                                                            )
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
            
            u, v = self.uid, c.uid
            lines.append('%s:head -> %s:head;' % (u, v))
            
            lines.extend(c.get_dot_lines())
            edges = [('%s:out_%s' % (self.uid,a),
                     ('%s:in_%s' % (c.uid,a))) for a in output_args
                      if a in c.input_args]
            for u, v in edges:
                lines.append('%s -> %s [arrowsize=0.5, penwidth=0.5];' % (u, v))
        return lines


class RINode(RIElement):

    def __init__(self, nodename, nodeparams):
        super(RINode, self).__init__()
        self.name = nodename
        self.params = nodeparams

    def get_dot_lines(self):
        label = '{<head>\n %s \n\t |%s \t %s}' % (self.name,
                                                  self.params['package'],
                                                  self.params['type'])
        
        param_names = [p['name'] for p in self.params['params']]
        param_values = [RIElement.trim_str(p['value']) for p in self.params['params']]
        label += '|{{' + '|'.join(param_names) + '}|{' + '|'.join(param_values) + '}}'
        
        s = '''%s [URL=%s, label="%s" shape=record]''' % (self.uid,
                                                          self.uid,
                                                          label)
        lines = [s]
        return lines

        
class RIRosparam(RIElement):

    def __init__(self, params):
        super(RIRosparam, self).__init__()
        self.params = params

    def get_dot_lines(self):
        
        header_str = 'ROSPARAM \t NS: %s \t COMMAND: %s' % (self.params['ns'],
                                                            self.params['command'])
        label = HTML.wrap(HTML.TABLE,
                    HTML.wrap(HTML.TR,
                        HTML.wrap(HTML.TD, header_str,
                                  attribs=HTML.ST_HEADER,
                                  portname='head'
                                  )
                              )+\
                        HTML.wrap(HTML.TR, 
                            HTML.wrap(HTML.TD, self.params['file'],
                                      attribs=HTML.ST_HEADER
                                      )
                                 ),
                        attribs=HTML.ST_TABLE
                        )
        #label = '<head>\nrosparam \t ns: %s \t command:%s\n \t|%s' % (self.params['ns'],
        #                                                      self.params['command'],
        #                                                      self.params['file'])
           
        s = '''%s [URL=%s, label=<%s> shape=plaintext];''' % (self.uid,
                                                              self.uid,
                                                              label)
        print s + '\n'
        lines = [s]
        return lines


class RoslaunchInspector(wx.Frame):
    """
    This class provides a GUI application for viewing roslaunch files.
    """
    def __init__(self, dot_filename=None):
        wx.Frame.__init__(self, None, -1, title="Roslaunch inspector",
                          size=(640,480))
        self.CreateStatusBar()

        filemenu= wx.Menu()
        filemenu.Append(wx.ID_ABOUT, "&About"," Information about this program")
        filemenu.AppendSeparator()
        filemenu.Append(wx.ID_EXIT,"E&xit"," Terminate the program")
        
        menuBar = wx.MenuBar()
        menuBar.Append(filemenu,"&File") # Adding the "filemenu" to the MenuBar
        self.SetMenuBar(menuBar)  # Adding the MenuBar to the Frame content.
        
        
        self.graph_view = xdot.wxxdot.WxDotWindow(self, -1)
        #self.graph_view.set_filter('neato')
        
        self.graph_view.register_select_callback(self.select_cb)
        
        
        if not dot_filename is None:
            pass
        else:
            package = 'openni2_launch'
            launch_file = 'openni2.launch'

            package = 'cob_people_detection'
            launch_file = 'people_detection_with_viewer.launch'

            package = 'mdr_moveit_cob'
            launch_file = 'demo.launch'
            
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
        self.RequestUserAttention()

    def select_cb(self, item, event):
        """Event: Click to select a graph node."""
        if event.ButtonUp(wx.MOUSE_BTN_LEFT):
            el = self.root.find_by_uid(int(item.url))
            if isinstance(el, RILaunch):
                print call(["subl", el.parsed_launch.path])
                #print el.parsed_launch.path

def main(argv):
    app = wx.App()
    frame = RoslaunchInspector(dot_filename=argv[1] if len(argv)>=2 else None)
    frame.Show()
    app.MainLoop()


if __name__ == '__main__':
    #assert len(sys.argv) == 2, 'dot filename input needed'
    main(sys.argv)