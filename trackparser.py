#!/usr/bin/env python2
import xmltodict
import math
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
from subprocess import call

parser = argparse.ArgumentParser(description='convert SpeedDreams\' map format to SUMO\'s')
parser.add_argument('inputFile', help='input file')
parser.add_argument("--debug", help="enable debug output", action="store_true")
parser.add_argument("--sumo", help="run netconvert and sumo afterwards", action="store_true")

args = parser.parse_args()

inputFile = args.inputFile
outputPath = os.getcwd() + "/sumoBuild"
filePrefix = outputPath + "/" + os.path.splitext(os.path.basename(inputFile))[0]

sumo = args.sumo
debug = args.debug

if not os.path.isdir(outputPath):
    os.mkdir(outputPath)

with open(inputFile) as fd:
	trackXml = xmltodict.parse(fd.read())

degreeStepSize = 10

nodes = [ (0,0) ]
helpNodes = []
directionDegree = 0

def parseSegment(segment):
	attrs = parseSegmentAttributes(segment)
	if attrs['type'] in ('lft', 'rgt'):
		parseCurve( parseSegmentAttributes(segment) )
	elif attrs['type'] == 'str':
		parseStraight( parseSegmentAttributes(segment) )
	else:
		print('I don\'t understand the segment type "%s"' % attrs['type'])


def parseSegmentAttributes(segment):
	attrs = {}

	for attr in segment['attstr']:
		attrs[attr['@name']] = attr['@val']
	for attr in segment['attnum']:
		attrs[attr['@name']] = attr['@val']

	return attrs


def parseCurve(attrs):
	global directionDegree
	center = None
	radius = float(attrs['radius'])
	# take 'end radius' into account (whatever this is)
	if(attrs.has_key('end radius')):
		radius = radius + float(attrs['end radius'])
		radius /= 2

	arc = float(attrs['arc'])
	radArc = np.radians(arc)
	startPoint = nodes[-1]
	rotationDirection = 1 if attrs['type'] == 'lft' else -1;
	lastDirection = (
		np.cos(np.radians(directionDegree)),
		np.sin(np.radians(directionDegree))
	)


	if attrs['type'] == 'lft':
		center = (
			startPoint[0] - lastDirection[1] * radius,
			startPoint[1] + lastDirection[0] * radius
		)
		# rotate left
		directionDegree = (directionDegree + arc) % 360
	elif attrs['type'] == 'rgt':
		center = (
			startPoint[0] + lastDirection[1] * radius,
			startPoint[1] - lastDirection[0] * radius
		)
		# rotate right
		directionDegree = (directionDegree - arc) % 360
	else:
		print('Wrong direction')

	if debug: print('Curve: center %s, radius %s, arc %s' % (center, radius, arc))
	helpNodes.append(center)

	curDegree = degreeStepSize
	while curDegree < arc:
		curRad = math.radians(curDegree)
		newNode = (
			center[0] + ( startPoint[0] - center[0] ) * np.cos(rotationDirection * curRad) + ( center[1] - startPoint[1] ) * np.sin(rotationDirection * curRad),
			center[1] + ( startPoint[1] - center[1] ) * np.cos(rotationDirection * curRad) + ( startPoint[0] - center[0] ) * np.sin(rotationDirection * curRad)
		)
		nodes.append(newNode)
		curDegree += degreeStepSize

	# add end node
	newNode = (
		center[0] + ( startPoint[0] - center[0] ) * np.cos(rotationDirection * radArc) + ( center[1] - startPoint[1] ) * np.sin(rotationDirection * radArc),
		center[1] + ( startPoint[1] - center[1] ) * np.cos(rotationDirection * radArc) + ( startPoint[0] - center[0] ) * np.sin(rotationDirection * radArc)
	)
	nodes.append(newNode)

	# rotate unit vector
	#rot=np.array([
	#	[ np.cos(radArc), -np.sin(radArc)],
	#	[ np.sin(radArc), -np.cos(radArc)]
	#])
	#b=np.array(*lastDirection)
	#print(rot.dot(b))


def parseStraight(attrs):
	if debug: print('Straight: length %s' % (float(attrs['lg']),))
	lastDirection = (
		np.cos(np.radians(directionDegree)),
		np.sin(np.radians(directionDegree))
	)
	print(lastDirection)
	newNode = (
		nodes[-1][0] + lastDirection[0] * float(attrs['lg']),
		nodes[-1][1] + lastDirection[1] * float(attrs['lg'])
	)
	nodes.append(newNode)


def parseTrack():
	track = None
	for section in trackXml['params']['section']:
		if section['@name'] == 'Main Track':
			track = section
			break

	if track is None:
		print( "Couldn't find Main Track" )
	else:
		print( "Main Track found")

	segments = None
	for section in track['section']:
		if section['@name'] == 'Track Segments':
			segments = section
			break

	if segments is None:
		print( "Couldn't find Track Segments" )
	else:
		print( "Track Segments found")

	for segment in segments['section']:
		parseSegment(segment)


def writeNodes(filePrefix):
	with open(filePrefix + '.nod.xml', 'w') as fd:
		id = 1
		fd.write('<nodes>\n')
		for node in nodes:
			fd.write('<node id="%s" x="%s" y="%s"/>\n' % (id, node[0], node[1]))
			id += 1
		fd.write('</nodes>\n')

def writeEdges(filePrefix):
	with open(filePrefix + '.edg.xml', 'w') as fd:
		fd.write('<?xml version="1.0" encoding="UTF-8"?>\n')
		fd.write('<edges xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.sf.net/xsd/edges_file.xsd">\n')

		for k in range(1, len(nodes)-1 ):
			fd.write('<edge from="' + str(k) +'" id="' + str(k) + 'to' + str(k+1) + '" to="' + str(k+1) + '"/>\n')

		# connect end to start
		fd.write('<edge from="' + str(k+1) +'" id="' + str(k+1) + 'to' + str(1) + '" to="' + str(1) + '"/>\n')
		fd.write('</edges>\n')

def writeRoutes(filePrefix):
	edgeIds = []
	for k in range(1, len(nodes)-1 ):
		edgeIds.append(str(k) + 'to' + str(k+1))
	edgeIds.append(str(len(nodes)-1) + 'to1') # connect end to start

	with open(filePrefix + '.rou.xml', 'w') as routesFile:
		routesFile.write('<routes>\n')
		routesFile.write('<vType accel="0.5" decel="5.0" id="Car" length="2.0" maxSpeed="1.0" sigma="0.0" />\n')
		routesFile.write('<route id="route0" edges="' + ' '.join(edgeIds) + '" />\n')
		routesFile.write('<vehicle depart="1" id="veh0" route="route0" type="Car" />\n')
		routesFile.write('</routes>\n')

def writeConf(filePrefix):
	conf = ('<configuration>\n'
    '	<input>\n'
    '	    <net-file value="' + os.path.splitext(os.path.basename(inputFile))[0] + '.net.xml"/>\n'
    '	    <route-files value="' + os.path.splitext(os.path.basename(inputFile))[0] + '.rou.xml"/>\n'
    '	</input>\n'
    '	<time>\n'
    '	    <begin value="0"/>\n'
    '	    <end value="1000000"/>\n'
    '	</time>\n'
	'</configuration>\n')
	with open(filePrefix + '.sumocfg', 'w') as fd:
		fd.write(conf)

def netconvert(filePrefix):
	# System call to use netconvert to bake nodes and edges to a net file
	call(["netconvert", '--node-files=' + filePrefix + '.nod.xml', '--edge-files=' + filePrefix + '.edg.xml', '--output-file=' + outputPath + '/' + filePrefix + '.net.xml'])

def sumoGUI(filePrefix):
	# Call of Sumo gui with configuration
	call(['sumo-gui', '-c', filePrefix + '.sumocfg'])

def showPoints():
	plt.scatter(*zip(*nodes))
	plt.scatter(*zip(*helpNodes), color='r')
	plt.plot(*zip(*nodes + [ nodes[0] ]))
	plt.show()

parseTrack()
writeNodes(filePrefix)
writeEdges(filePrefix)
writeRoutes(filePrefix)
writeConf(filePrefix)
if sumo:
    netconvert(filePrefix)
    sumoGUI(filePrefix)

if debug: showPoints()
