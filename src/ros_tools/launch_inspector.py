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

import rospkg

class HTML(object):
    """
    Helper class for styling the graphvis nodes.
    """
    TABLE = 'TABLE'
    TD = 'TD'
    TR = 'TR'
    FONT='FONT'
    BR = '<BR/>'

    STYLE_TABLE_MAIN = ' BORDER="1" CELLBORDER="0" CELLPADDING="0" CELLSPACING="0" COLOR="#000000"'
    STYLE_PADDING = ' CELLPADDING="5"'
    STYLE_ALIGN_LEFT = ' BALIGN="LEFT" ALIGN="LEFT"'
    STYLE_INVISIBLE = '  BORDER="0" CELLBORDER="0" CELLSPACING="0"'
    STYLE_TD_GROUP = ' CELLPADDING="0" CELLBORDER="1"'
    STYLE_TABLE_INNER = ' BORDER="0" CELLBORDER="0" CELLPADDING="5" CELLSPACING="0"'
    STYLE_TABLE_CLEAR = ' BORDER="0" CELLBORDER="0" CELLSPACING="0" CELLPADDING="0"'
    STYLE_TABLE_HIGHLIGHT = ' BGCOLOR="#66FFFF"'

    @staticmethod
    def wrap(TAG, string, attributes='', port=None):
        """Wraps string into HTML as in the template:
        <TAG attribs port="portname">string</TAG>
        """
        if len(attributes) > 0:
            attributes = ' ' + attributes
        if not port is None:
            attributes += ' PORT="%s"' % port
        return '<%s%s>%s</%s>' % (TAG,
                                  attributes,
                                  string,
                                  TAG)

    @staticmethod
    def table(content, attributes=STYLE_TABLE_CLEAR, port=None):
        if port is not None:
            attributes += ' PORT="%s"' % port
        t = HTML.wrap(HTML.TABLE, HTML.wrap(
                HTML.TR,
                HTML.wrap(
                    HTML.TD,
                    content
                ),
            ),
            attributes=attributes
        )
        return t

    @staticmethod
    def list1_2_3(rows):
        trs = []
        for r in rows:
            if len(r) == 1: colspan = 'COLSPAN="6"'
            elif len(r) == 2: colspan = 'COLSPAN="3"'
            elif len(r) == 3: colspan = 'COLSPAN="2"'
            else: colspan = ''
        return HTML.wrap(HTML.TABLE, '<HR/>'.join(trs), attributes=HTML.STYLE_TABLE_CLEAR)

    @staticmethod
    def list(el_list):
        rows = [
            HTML.wrap(HTML.TR, HTML.wrap(HTML.TD, e, attributes=HTML.STYLE_ALIGN_LEFT))
            for e in el_list]
        return HTML.wrap(HTML.TABLE, '<HR/>'.join(rows), attributes=HTML.STYLE_TABLE_CLEAR)

    @staticmethod
    def header(label, sublabel = '', port=None):
        return HTML.wrap(
            HTML.TABLE,
            HTML.wrap(
                HTML.TR,
                HTML.wrap(
                    HTML.TD,
                    label,
                    attributes='ALIGN="CENTER" BGCOLOR="#DEDEDE"', port=port) +
                    HTML.wrap(
                        HTML.TD,
                        sublabel,
                        attributes='COLSPAN="9"'
                    )
                ),
            HTML.STYLE_TABLE_CLEAR
        )

    @staticmethod
    def labelled_cell(label, content, sublabel='', input_port=None, output_port=None):
        return HTML.wrap(HTML.TABLE, HTML.wrap(HTML.TR,
                                               HTML.wrap(HTML.TD,
                                                         label,
                                                         attributes='ALIGN="CENTER" BGCOLOR="#DEDEDE"',
                                                         port=input_port) +
                                               HTML.wrap(HTML.TD,
                                                         sublabel,
                                                         attributes='COLSPAN="9"')
                                               ) +
                         HTML.wrap(HTML.TR,
                                   HTML.wrap(HTML.TD, ' ', attributes='COLSPAN="10"'),
                                   ) +
                         HTML.wrap(HTML.TR,
                                   HTML.wrap(HTML.TD,
                                             HTML.table(
                                                content,
                                                attributes=HTML.STYLE_PADDING + HTML.STYLE_INVISIBLE
                                             ),
                                             attributes='COLSPAN="10" BALIGN="LEFT"'),
                                   ) +
                         HTML.wrap(HTML.TR,
                                   HTML.wrap(HTML.TD, ' ', attributes='COLSPAN="10"'),
                                   ), HTML.STYLE_TABLE_CLEAR, port=output_port)

    @staticmethod
    def invisible_table(string2darray):
        if len(string2darray)==0:
            return ''
        rows = []
        for row in string2darray:
            r = HTML.wrap(HTML.TR, ''.join(
                [HTML.wrap(HTML.TD, col, attributes='ALIGN="LEFT" BALIGN="LEFT" VALIGN="TOP"') for col in row]))
            rows.append(r)
        t = HTML.wrap(HTML.TABLE, ''.join(rows), attributes=HTML.STYLE_TABLE_INNER)
        return t


class DotElement(object):
    """
    Base class for nodes drawn in graphvis.
    """
    # Unique id for the node
    uid_counter = 0

    @staticmethod
    def trim_str(s):
        if ' ' in s:
            return HTML.BR.join(s.split())
        elif '/' in s and len(s)>50:
            return '.../'+'/'.join(s.split('/')[-2:])
        return s[:50] + ' ... ' + s[-50:] if len(s) > 100 else s

    def __init__(self, roslaunch_element=None, parent=None, text=None):
        self.children = []
        self.input_args = []
        self.uid = DotElement.uid_counter
        self.text = text if text is not None else roslaunch_element.text
        DotElement.uid_counter += 1
        self.roslaunch_element = roslaunch_element
        self.parent=parent
        self.condition = True

        if self.roslaunch_element is not None:

            if self.roslaunch_element.attributes.has_key('if'):
                if self.roslaunch_element.attributes['if']['resolved'] == RoslaunchElement.FALSE:
                    self.condition = False
            if self.roslaunch_element.attributes.has_key('unless'):
                if self.roslaunch_element.attributes['unless']['resolved'] == RoslaunchElement.TRUE:
                    self.condition = False

            if self.parent is not None:
                if self.parent.condition == False:
                    self.condition = False

            for c in self.roslaunch_element.children:
                if c.type == RoslaunchElement.MACHINE:
                    self.children.append(DotTableElement(c, parent=self))
                elif c.type == RoslaunchElement.GROUP:
                    self.children.append(DotGroup(c, parent=self))
                elif c.type == RoslaunchElement.LAUNCH or \
                        c.type == RoslaunchElement.INCLUDE:
                    self.children.append(DotRoslaunch(c, parent=self))
                elif c.type == RoslaunchElement.PARAM:
                    self.children.append(DotParam(c, parent=self))
                elif c.type == RoslaunchElement.NODE:
                    self.children.append(DotNode(c, parent=self))
                elif c.type == RoslaunchElement.ROSPARAM:
                    self.children.append(DotRosparam(c, parent=self))
                elif c.type == RoslaunchElement.ARG:
                    self.children.append(DotArg(c, parent=self))
                elif c.type == RoslaunchElement.REMAP:
                    self.children.append(DotRemap(c, parent=self))
                else:
                    self.children.append(DotGraphElement(c, parent=self))

    def get_condition_string(self):
        if self.roslaunch_element.attributes.has_key('if'):
            return ' if %s (%s)' % (self.roslaunch_element.attributes['if']['variable'],
                                    self.roslaunch_element.attributes['if']['resolved'])
        elif self.roslaunch_element.attributes.has_key('unless'):
            return ' unless %s (%s)' % (self.roslaunch_element.attributes['unless']['variable'],
                                        self.roslaunch_element.attributes['unless']['resolved'])
        else: return ''


    def find_by_uid(self, uid):
        if self.uid == uid:
            return self
        for c in self.children:
            ri_el = c.find_by_uid(uid)
            if ri_el is not None:
                return ri_el
        return None

    def find_graph_element(self):
        if isinstance(self, DotGraphElement):
            return self.uid
        if self.parent is None:
            return None
        else:
            return self.parent.find_graph_element()

    def get_table_style(self):
        st = HTML.STYLE_TABLE_MAIN
        if self.condition == False:
                st += ' BGCOLOR="#999999"'
        return st

    def get_header(self):
        return HTML.labelled_cell(self.roslaunch_element.type, self.roslaunch_element.attributes['name']['resolved'],
                                  input_port='head%s', output_port='tail%s' % (self.uid, self.uid))

    def get_inner_elements(self):
        inserts = []
        for c in self.children:
            if isinstance(c, DotTableElement):
                inner = c.get_inner_elements()
                if c.roslaunch_element.type==RoslaunchElement.MACHINE or \
                        c.roslaunch_element.type == RoslaunchElement.GROUP:
                    inserts.append(
                        HTML.table(
                            HTML.table(
                                HTML.list(
                                    [c.get_header()] + c.get_inner_elements()
                                ), attributes=c.get_table_style()
                            ),
                            attributes=HTML.STYLE_PADDING + HTML.STYLE_INVISIBLE
                        )
                    )
                else:
                    inserts.append(
                        HTML.table(
                                HTML.list(
                                    [c.get_header()]
                            ), attributes=HTML.STYLE_PADDING + HTML.STYLE_INVISIBLE
                        )
                    )
        return inserts

    def get_dot_lines(self, selected=None):
        label = HTML.table(
            HTML.table(
                HTML.list([self.get_header()] + self.get_inner_elements()),
                attributes='BORDER="0" CELLBORDER="0" CELLSPACING="0" CELLPADDING="0" COLOR="#CCCCCC"'
            ),
            attributes=self.get_table_style(),
            port='tail'
        )
        s = '''%s [URL=%s, label=<%s> shape=plaintext];''' % (self.uid,
                                                              self.uid,
                                                              label)
        lines = [s] + self.get_children_dot_lines(selected) + self.get_edges_dot_lines(selected)
        return lines

    def get_children_dot_lines(self, selected=None):
        lines = []
        for c in self.children:
            lines.extend(c.get_dot_lines(selected))
        return lines

    def get_edges_dot_lines(self, selected=None):
        lines = []
        for c in self.children:
            if not isinstance(c, DotTableElement):
                lines.append('%s:tail%s -> %s:head%s [arrowsize=0.5, penwidth=0.5];' % (self.find_graph_element(),
                                                                                        self.uid,
                                                                                        c.uid, c.uid))
        return lines


class DotGraphElement(DotElement):

    pass


class DotTableElement(DotElement):

    def get_header(self):
        return HTML.labelled_cell(
            self.roslaunch_element.type,'',
            self.roslaunch_element.attributes['ns']['resolved'],
            output_port='tail%s' % self.uid)


    def get_dot_lines(self, selected=None):
        return self.get_children_dot_lines(selected) + self.get_edges_dot_lines(selected)
    pass


class DotGroup(DotTableElement):

    def get_header(self):
        if self.roslaunch_element.attributes.has_key('ns'):
            sub_label = 'ns: %s;' % self.roslaunch_element.attributes['ns']['resolved']
        else:
            sub_label = ''

        label = self.get_condition_string()

        return HTML.labelled_cell(
            self.roslaunch_element.type, label,
            sub_label,
            output_port='tail%s' % self.uid)


class DotParam(DotTableElement):

    def get_header(self):
        return '<FONT POINT-SIZE="11">param %s %s</FONT><BR/>'% (self.roslaunch_element.attributes['type']['resolved'],
                                                                self.get_condition_string()) +\
            self.roslaunch_element.attributes['name']['resolved'] + ': ' +\
            self.roslaunch_element.attributes['value']['resolved']


class DotArg(DotTableElement):

    def get_header(self):
        return self.roslaunch_element.attributes['name']['resolved'] + ':' + \
               HTML.BR + self.trim_str(self.roslaunch_element.attributes['value']['resolved'])


class DotRemap(DotTableElement):

    def get_header(self):
        return '<FONT POINT-SIZE="11">remap%s</FONT><BR/>' % (self.get_condition_string()) +\
            self.roslaunch_element.attributes['from']['resolved'] + ' &rArr; ' + \
            self.roslaunch_element.attributes['to']['resolved']


class DotNode(DotGraphElement):

    def get_header(self):
        return HTML.labelled_cell(
            self.roslaunch_element.type,
            '%s %s' % (self.roslaunch_element.attributes['pkg']['resolved'],
                       self.roslaunch_element.attributes['type']['resolved']),
            self.roslaunch_element.attributes['name']['resolved'],
            input_port='head%s' % self.uid, output_port='tail%s' % self.uid
                                  )
        #return HTML.header(self.roslaunch_element.type, self.roslaunch_element.attributes['name']['resolved'], port='head%s' % self.uid)
        pass


class DotRosparam(DotGraphElement):

    def get_header(self):
        path = self.roslaunch_element.attributes['file']['resolved']
        return HTML.labelled_cell(self.roslaunch_element.type, rospkg.get_package_name(path) + '/' + path.split('/')[-1],
                                  self.roslaunch_element.attributes['command']['resolved'], input_port='head%s' % self.uid)


class DotRoslaunch(DotGraphElement):

    def get_header(self):
        path = self.roslaunch_element.attributes['file']['resolved']
        return HTML.labelled_cell(
            self.roslaunch_element.type, rospkg.get_package_name(path) + '  ' + path.split('/')[-1],
            self.roslaunch_element.attributes['ns']['resolved'], input_port='head%s' % self.uid, output_port='tail%s' % self.uid
        )


class RoslaunchInspector(wx.Frame):
    """
    This class provides a GUI application for viewing roslaunch files.
    """

    def __init__(self, dot_filename=None):
        wx.Frame.__init__(self, None, -1, title="Roslaunch inspector",
                          size=(640, 480))
        self.CreateStatusBar()

        filemenu = wx.Menu()
        filemenu.Append(wx.ID_ABOUT, "&About", " Information about this program")
        filemenu.AppendSeparator()
        filemenu.Append(wx.ID_EXIT, "E&xit", " Terminate the program")

        menuBar = wx.MenuBar()
        menuBar.Append(filemenu, "&File")  # Adding the "filemenu" to the MenuBar
        self.SetMenuBar(menuBar)  # Adding the MenuBar to the Frame content.

        self.graph_view = xdot.wxxdot.WxDotWindow(self, -1)
        #self.graph_view.set_filter('neato')

        self.selected = []
        self.graph_view.register_select_callback(self.select_cb)



        if not dot_filename is None:
            pass
        else:
            package = 'openni2_launch'
            launch_file = 'openni2.launch'

            package = 'cob_people_detection'
            launch_file = 'people_detection_with_viewer.launch'

            #package = 'mdr_moveit_cob'
            #launch_file = 'demo.launch'

            launch_file_paths = find_resource(package, launch_file)
            # print launch_file_paths

            for path in launch_file_paths:
                launch_file_obj = LaunchFileParser(path)
                # self.root = RILaunch(launch_file_obj)
                self.root = DotRoslaunch(RoslaunchElement(RoslaunchElement.load_xml(path)))
            pass

        self.update_graph()
        self.RequestUserAttention()
        self.graph_view.zoom_to_fit()

    def select_cb(self, item, event):
        """Event: Click to select a graph node."""
        if event.ButtonUp(wx.MOUSE_BTN_LEFT):
            el = self.root.find_by_uid(int(item.url))
            #self.graph_view.ge
            if isinstance(el, DotElement):
                print item.url
                item.highlight=None
                self.selected = item.url
                self.graph_view.set_highlight(item.item)
                #if 'file' in el.roslaunch_element.attributes.keys():
                print call(["subl", el.roslaunch_element.attributes['file']['resolved']])
                self.update_graph()

    def update_graph(self):
        t = '''digraph {
                       graph [rankdir=LR];
                       node [label="\N"];
                    '''
        t += '\n'.join(self.root.get_dot_lines(selected=self.selected))
        t += '}'
        self.graph_view.set_dotcode(t)
        self.graph_view.Refresh()
        pass


def main(argv):
    app = wx.App()
    frame = RoslaunchInspector(dot_filename=argv[1] if len(argv) >= 2 else None)
    frame.Show()
    app.MainLoop()


if __name__ == '__main__':
    # assert len(sys.argv) == 2, 'dot filename input needed'
    main(sys.argv)
