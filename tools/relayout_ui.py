#!/usr/bin/env python3
"""Convert mainwindow.ui from absolute geometry to real Qt layouts.

The original .ui positions every container with a fixed <geometry> rect and
buries the working grids inside absolutely-placed `layoutWidget` wrappers. That
is why the window ignores resizing and why several labels clip.

This rewrites the container level only. Inner grids keep their existing rows
and columns; they are simply promoted to be their parent's own layout so resize
events can reach them.

Run once from the repo root:  python3 tools/relayout_ui.py mainwindow.ui
"""
import sys
import xml.etree.ElementTree as ET


def find(root, name):
    for e in root.iter():
        if e.get('name') == name:
            return e
    raise SystemExit('not found: ' + name)


def parent_of(root, child):
    for p in root.iter():
        for c in list(p):
            if c is child:
                return p
    return None


def set_stretch(lay, value):
    """QBoxLayout stretch factors, e.g. '1,0' -- first item grows, second doesn't.

    This is an ATTRIBUTE on the <layout> element, not a <property> child. As a
    property, uic emits a bogus setStretch(QString) call that will not compile.
    """
    lay.set('stretch', value)


def strip_geometry(el):
    for prop in el.findall("./property[@name='geometry']"):
        el.remove(prop)


def item(child):
    it = ET.Element('item')
    it.append(child)
    return it


def layout(cls, name, *children, **attrs):
    lay = ET.Element('layout', {'class': cls, 'name': name})
    for k, v in attrs.items():
        p = ET.SubElement(lay, 'property', {'name': k})
        ET.SubElement(p, 'number').text = str(v)
    for c in children:
        lay.append(item(c) if c.tag in ('widget', 'layout') else c)
    return lay


def grid_item(child, row, col):
    it = ET.Element('item', {'row': str(row), 'column': str(col)})
    it.append(child)
    return it


def promote_wrapper(root, group_name):
    """Move a groupbox's layoutWidget-wrapped layout up to be its own layout."""
    grp = find(root, group_name)
    wrapper = None
    for c in list(grp):
        if c.tag == 'widget' and c.get('class') == 'QWidget':
            wrapper = c
            break
    if wrapper is None:
        return grp
    inner = wrapper.find('layout')
    grp.remove(wrapper)
    if inner is not None:
        grp.append(inner)
    return grp


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else 'mainwindow.ui'
    ET.register_namespace('', '')
    tree = ET.parse(path)
    root = tree.getroot()

    # ---- promote the wrapped grids inside every group box -------------------
    for g in ('ComSeetingsGroup', 'ServoListGroup',
              'groupBox_3', 'groupBox_4', 'groupBox_5', 'groupBox_6'):
        promote_wrapper(root, g)

    central = find(root, 'centralwidget')
    debug = find(root, 'DebugTab')
    prog = find(root, 'ProgrammingTab')

    # ---- DebugTab -----------------------------------------------------------
    # Capture every reference BEFORE detaching, or the lookups fail.
    graph = find(root, 'graphWidget')
    boxes = {n: find(root, n) for n in
             ('groupBox_3', 'groupBox_4', 'groupBox_5', 'groupBox_6')}

    # the free-floating checkbox/slider column beside the graph
    side_layout = None
    for c in list(debug):
        if c.tag == 'widget' and c.get('class') == 'QWidget' and c.find('layout') is not None:
            side_layout = c.find('layout')

    # Remove only child widgets/layouts. <attribute name="title"> lives here too
    # and carries the tab's label -- deleting it renames the tabs to "Page".
    for c in list(debug):
        if c.tag in ('widget', 'layout'):
            debug.remove(c)
    strip_geometry(graph)
    for el in boxes.values():
        strip_geometry(el)

    graph_row = ET.Element('layout', {'class': 'QHBoxLayout', 'name': 'graphRowLayout'})
    graph_row.append(item(graph))
    if side_layout is not None:
        graph_row.append(item(side_layout))
    set_stretch(graph_row, '1,0')          # graph absorbs width, side column does not

    controls = ET.Element('layout', {'class': 'QGridLayout', 'name': 'controlsGridLayout'})
    controls.append(grid_item(boxes['groupBox_3'], 0, 0))
    controls.append(grid_item(boxes['groupBox_5'], 0, 1))
    controls.append(grid_item(boxes['groupBox_4'], 1, 0))
    controls.append(grid_item(boxes['groupBox_6'], 1, 1))

    debug_layout = ET.Element('layout', {'class': 'QVBoxLayout', 'name': 'debugTabLayout'})
    debug_layout.append(item(graph_row))
    debug_layout.append(item(controls))
    set_stretch(debug_layout, '1,0')       # graph grows vertically, controls stay put
    debug.append(debug_layout)

    # ---- ProgrammingTab -----------------------------------------------------
    table = find(root, 'memoryTableView')
    memlabel = find(root, 'memLabel')
    textedit = find(root, 'plainTextEdit')
    toprow = None
    botrow = None
    for c in list(prog):
        if c.tag == 'widget' and c.get('class') == 'QWidget':
            lay = c.find('layout')
            if lay is None:
                continue
            if lay.get('name') == 'horizontalLayout':
                toprow = lay
            elif lay.get('name') == 'horizontalLayout_2':
                botrow = lay

    for c in list(prog):
        if c.tag in ('widget', 'layout'):
            prog.remove(c)
    for el in (table, memlabel, textedit):
        strip_geometry(el)

    right = ET.Element('layout', {'class': 'QVBoxLayout', 'name': 'progRightLayout'})
    right.append(item(textedit))
    right.append(item(memlabel))
    if botrow is not None:
        right.append(item(botrow))
    spacer = ET.Element('spacer', {'name': 'progRightSpacer'})
    op = ET.SubElement(spacer, 'property', {'name': 'orientation'})
    ET.SubElement(op, 'enum').text = 'Qt::Vertical'
    right.append(item(spacer))

    body = ET.Element('layout', {'class': 'QHBoxLayout', 'name': 'progBodyLayout'})
    body.append(item(table))
    body.append(item(right))
    set_stretch(body, '1,0')               # register table absorbs the width

    prog_layout = ET.Element('layout', {'class': 'QVBoxLayout', 'name': 'progTabLayout'})
    if toprow is not None:
        prog_layout.append(item(toprow))
    prog_layout.append(item(body))
    set_stretch(prog_layout, '0,1')        # table area absorbs the height
    prog.append(prog_layout)

    # ---- centralwidget ------------------------------------------------------
    com = find(root, 'ComSeetingsGroup')
    servolist = find(root, 'ServoListGroup')
    tabs = find(root, 'tabWidget')
    for el in (com, servolist, tabs):
        strip_geometry(el)

    for c in list(central):
        if c.tag in ('widget', 'layout'):
            central.remove(c)

    left = ET.Element('layout', {'class': 'QVBoxLayout', 'name': 'leftPanelLayout'})
    left.append(item(com))
    left.append(item(servolist))

    main_layout = ET.Element('layout', {'class': 'QHBoxLayout', 'name': 'mainLayout'})
    main_layout.append(item(left))
    main_layout.append(item(tabs))
    set_stretch(main_layout, '0,1')        # tabs absorb the width, side panel does not
    central.insert(0, main_layout)

    # ---- minimum widths for fields whose contents would otherwise clip ------
    # A layout gives a QLineEdit its sizeHint, which ignores the current text.
    # These hold paths or long numbers and scroll their contents out of view.
    for name, width in (('recFileNameLineEdit', 130),
                        ('goalLineEdit', 70),
                        ('timeLineEdit', 70),
                        ('speedLineEdit', 70),
                        ('accLineEdit', 70)):
        try:
            el = find(root, name)
        except SystemExit:
            continue
        if el.find("./property[@name='minimumSize']") is not None:
            continue
        prop = ET.Element('property', {'name': 'minimumSize'})
        size = ET.SubElement(prop, 'size')
        ET.SubElement(size, 'width').text = str(width)
        ET.SubElement(size, 'height').text = '0'
        el.insert(0, prop)

    # ---- window sizing ------------------------------------------------------
    # With everything in layouts the content has a real minimum. Measured with
    # MainWindow::minimumSizeHint() after the field minimums above: 1067x809.
    # Without an explicit minimumSize, resize() can still shrink the window
    # below it and labels clip again, which is what "cramped" looked like.
    # Re-measure and raise this if fields are added.
    win = root.find('widget')
    geo = win.find("./property[@name='geometry']")
    if geo is not None:
        rect = geo.find('rect')
        rect.find('width').text = '1500'
        rect.find('height').text = '900'

    # The original MainWindow declares sizePolicy Fixed/Fixed with minimumSize
    # equal to its geometry. That pins the window shut on its own -- layouts
    # alone would not have made it resizable. Relax it to Preferred.
    sp = win.find("./property[@name='sizePolicy']")
    if sp is not None:
        pol = sp.find('sizepolicy')
        pol.set('hsizetype', 'Preferred')
        pol.set('vsizetype', 'Preferred')

    # The original .ui already carries a minimumSize of 917x706, which is below
    # what the laid-out content needs -- overwrite it rather than skipping.
    existing = win.find("./property[@name='minimumSize']")
    if existing is not None:
        win.remove(existing)
    p = ET.Element('property', {'name': 'minimumSize'})
    size = ET.SubElement(p, 'size')
    ET.SubElement(size, 'width').text = '1080'
    ET.SubElement(size, 'height').text = '820'
    win.insert(list(win).index(geo) + 1 if geo is not None else 0, p)

    tree.write(path, encoding='UTF-8', xml_declaration=True)
    print('rewrote', path)


if __name__ == '__main__':
    main()
