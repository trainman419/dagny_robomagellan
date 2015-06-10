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
    print "TODO: select placemark from list"
    print [ p.get_name() for p in placemarks ]
    sys.exit(1)

geometry = placemark.get_geometry()

coordinates = []

if geometry.Type() == kmldom.Type_Polygon:
    poly = kmldom.AsPolygon(geometry)
    bound = poly.get_outerboundaryis()
    if bound and bound.has_linearring():
        ring = bound.get_linearring()
        if ring.has_coordinates():
            coords = ring.get_coordinates()
            for i in range(coords.get_coordinates_array_size()):
                vec = coords.get_coordinates_array_at(i)
                coordinates.append( [vec.get_latitude(), vec.get_longitude()] )
else:
    print "Unknown placemark geometry type", geometry.Type()

mission = {
        'goals': coordinates,
        'loop': True
        }

print yaml.dump(mission)
