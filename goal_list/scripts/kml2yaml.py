#!/usr/bin/env python
#
# convert a KML file into a route
#
# For now, use the python-kml package, which is based on Google's libkml

import sys
import kmldom
import kmlengine
import argparse
import yaml

# TODO: argparse for arguments

kmlfile = sys.argv[1]

kml_data = ""
if kmlfile.endswith('.kml'):
    with open(kmlfile) as f:
        kml_data = f.read()
elif kmlfile.endswith('.kmz'):
    (success, kml_data) = kmlengine.KmzFile_OpenFromFile(kmlfile).ReadKml()
    # check return code
    if not success:
        print "Failed to read KML from kmz file"
        sys.exit(-1)

kml = kmldom.ParseKml(kml_data)
kml_root = kmlengine.GetRootFeature(kml)

placemarks = []
nodes = [kml_root]

while len(nodes) > 0:
    node = nodes.pop()
    if node.Type() == kmldom.Type_Document:
        doc = kmldom.AsDocument(node)
        for i in range(doc.get_feature_array_size()):
            nodes.append(doc.get_feature_array_at(i))
    if node.Type() == kmldom.Type_Placemark:
        p = kmldom.AsPlacemark(node)
        placemarks.append(p)


if len(placemarks) < 1:
    print "Error: no placemarks found in KML file"
    sys.exit(-1)
if len(placemarks) == 1:
    print "Found 1 placemark. Using it"
    placemark = placemarks[0]
else:
    print "Select placemark:"
    for i,p in enumerate(placemarks):
        print " %3d: %s" % (i+1, p.get_name())
    pos = 0
    while pos < 1 or pos > len(placemarks):
        pos = raw_input("# ")
        try:
            pos = int(pos)
        except:
            pass
    placemark = placemarks[pos-1]
    print "Selected %s"%(placemark.get_name())

geometry = placemark.get_geometry()

coordinates = []

coords = None
if geometry.Type() == kmldom.Type_Polygon:
    poly = kmldom.AsPolygon(geometry)
    bound = poly.get_outerboundaryis()
    if bound and bound.has_linearring():
        ring = bound.get_linearring()
        if ring.has_coordinates():
            coords = ring.get_coordinates()
elif geometry.Type() == kmldom.Type_LineString:
    line = kmldom.AsLineString(geometry)
    if line.has_coordinates():
        coords = line.get_coordinates()
else:
    print "Unknown placemark geometry type", geometry.Type()

if coords:
    for i in range(coords.get_coordinates_array_size()):
        vec = coords.get_coordinates_array_at(i)
        coordinates.append( [vec.get_latitude(), vec.get_longitude()] )

mission = {
        'goals': coordinates,
        'loop': True
        }

if len(sys.argv) > 2:
    yamlfile = sys.argv[2]
    with open(yamlfile, 'w') as f:
        f.write(yaml.dump(mission))
else:
    print yaml.dump(mission)
