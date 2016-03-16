#!/usr/bin/env python2
import xmltodict
import math
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
import logging
from subprocess import call

logger = logging.getLogger('trackparser')

parser = argparse.ArgumentParser(description='convert SpeedDreams\' map format to SUMO\'s')
parser.add_argument('inputFile', help='input file')
parser.add_argument("--debug", help="enable debug output", action="store_true")
parser.add_argument("--sumo", help="run sumo (optional: path to sumo)", nargs="?", const="sumo-gui")
parser.add_argument("--net", help="run netconvert (optional: path to netconvert)", nargs="?", const="netconvert")
parser.add_argument("-d", "--degree", help="curve step size in degree", type=int, default=5)
parser.add_argument("-v", "--verbose", dest="count_verbose", default=0, action="count", help="increase verbosity")
parser.add_argument("-q", "--quiet", dest="count_quiet", default=0, action="count", help="decrease verbosity")

args = parser.parse_args()

LOGLEVELS = (logging.ERROR, logging.WARN, logging.INFO, logging.DEBUG)
loglevel = LOGLEVELS[min(1 + args.count_verbose - args.count_quiet, len(LOGLEVELS)-1)]
logging.basicConfig(level=loglevel)

inputFile = args.inputFile
outputPath = os.getcwd() + "/sumoBuild"
filePrefix = outputPath + "/" + os.path.splitext(os.path.basename(inputFile))[0]

sumoCommand = args.sumo
netconvertCommand = args.net
debug = args.debug

degreeStepSize = args.degree

if not os.path.isdir(outputPath):
	os.mkdir(outputPath)

with open(inputFile) as fd:
	trackXml = xmltodict.parse(fd.read())

nodes = [ (0,0) ]
helpNodes = []
directionDegree = 0

def parseSegment(segment):
	logger = logging.getLogger('trackparser')
	attrs = parseSegmentAttributes(segment)
	if attrs['type'] in ('lft', 'rgt'):
		parseCurve( parseSegmentAttributes(segment) )
	elif attrs['type'] == 'str':
		parseStraight( parseSegmentAttributes(segment) )
	else:
		logger.warn('I don\'t understand the segment type "%s"' % attrs['type'])


def parseSegmentAttributes(segment):
	attrs = {}

	if type(segment['attstr']) is list:
		for attr in segment['attstr']:
			attrs[attr['@name']] = attr['@val']
	else:
		attr = segment['attstr']
		attrs[attr['@name']] = attr['@val']

	if type(segment['attnum']) is list:
		for attr in segment['attnum']:
			attrs[attr['@name']] = attr['@val']
	else:
		attr = segment['attstr']
		attrs[attr['@name']] = attr['@val']

	return attrs


def parseCurve(attrs):
	global directionDegree
	logger = logging.getLogger('trackparser')
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
		logger.warn('Wrong direction')

	logger.debug('Curve: center %s, radius %s, arc %s' % (center, radius, arc))
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


def parseStraight(attrs):
	logger = logging.getLogger('trackparser')
	logger.debug('Straight: length %s' % (float(attrs['lg']),))
	lastDirection = (
		np.cos(np.radians(directionDegree)),
		np.sin(np.radians(directionDegree))
	)
	newNode = (
		nodes[-1][0] + lastDirection[0] * float(attrs['lg']),
		nodes[-1][1] + lastDirection[1] * float(attrs['lg'])
	)
	nodes.append(newNode)

def findByName(maybeList, name):
	result = None
	elementList = None

	if type(maybeList) is list:
		elementList = maybeList
	else:
		elementList = [maybeList]

	for element in elementList:
		if element['@name'] == name:
			result = element
			break

	return result


def parseTrack():
	logger = logging.getLogger('trackparser')
	track = findByName(trackXml['params']['section'], 'Main Track')

	if track is None:
		logger.error( "Couldn't find Main Track" )
		sys.exit(1)
	else:
		logger.info( "Main Track found")

	segments = findByName(track['section'], 'Track Segments')

	if segments is None:
		logger.error( "Couldn't find Track Segments" )
		sys.exit(1)
	else:
		logger.info( "Track Segments found")

	for segment in segments['section']:
		parseSegment(segment)


def writeNodes(filePrefix):
	with open(filePrefix + '.nod.xml', 'w') as fd:
		id = 0
		fd.write('<nodes>\n')
		for node in nodes:
			fd.write('<node id="%s" x="%s" y="%s"/>\n' % (id, node[0], node[1]))
			id += 1
		fd.write('</nodes>\n')

def writeEdges(filePrefix, width):
	with open(filePrefix + '.edg.xml', 'w') as fd:
		fd.write('<?xml version="1.0" encoding="UTF-8"?>\n')
		fd.write('<edges xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.sf.net/xsd/edges_file.xsd">\n')

		for k in range( len(nodes) ):
			nextK = (k + 1) % len(nodes)
			edgeFrom = str(k)
			edgeTo = str(nextK)
			edgeId = str(k) + 'to' + str(nextK)
			edgeWidth = width
			edgeLength = np.sqrt( np.square(nodes[k][0] - nodes[nextK][0]) + np.square(nodes[k][1] - nodes[nextK][1]) )
			fd.write('<edge from="{}" id="{}" to="{}" length="{}" width="{}"/>\n'.format(edgeFrom, edgeId, edgeTo, edgeLength, edgeWidth))

		fd.write('</edges>\n')

def writeRoutes(filePrefix):
	edgeIds = []
	for k in range( len(nodes) ):
		edgeIds.append(str(k) + 'to' + str( (k+1) % len(nodes)))

	with open(filePrefix + '.rou.xml', 'w') as routesFile:
		routesFile.write('<routes>\n')
		routesFile.write('<vType accel="0.5" decel="5.0" id="Car" length="2.0" maxSpeed="1.0" sigma="0.0" />\n')
		routesFile.write('<route id="route0" edges="' + ' '.join(edgeIds) + '" />\n')
		routesFile.write('<vehicle depart="1" id="veh0" route="route0" type="Car" />\n')
		routesFile.write('</routes>\n')

def writeConf(filePrefix):
	conf = ('<configuration>\n'
	'	<input>\n'
	'		<net-file value="' + os.path.splitext(os.path.basename(inputFile))[0] + '.net.xml"/>\n'
	'		<route-files value="' + os.path.splitext(os.path.basename(inputFile))[0] + '.rou.xml"/>\n'
	'	</input>\n'
	'	<time>\n'
	'		<begin value="0"/>\n'
	'		<end value="1000000"/>\n'
	'	</time>\n'
	'</configuration>\n')
	with open(filePrefix + '.sumocfg', 'w') as fd:
		fd.write(conf)

def netconvert(filePrefix, netconvertCommand):
	# System call to use netconvert to bake nodes and edges to a net file
	call([netconvertCommand, '--node-files=' + filePrefix + '.nod.xml', '--edge-files=' + filePrefix + '.edg.xml', '--output-file=' + filePrefix + '.net.xml'])

def sumo(filePrefix, sumoCommand):
	# Call of Sumo gui with configuration
	call([sumoCommand, '-c', filePrefix + '.sumocfg'])

def showPoints():
	plt.scatter(*zip(*nodes))
	plt.scatter(*zip(*helpNodes), color='r')
	plt.plot(*zip(*nodes + [ nodes[0] ]))
	plt.show()

def trackLength():
	trackLength = 0;
	for n in range( len(nodes) ):
		nextIndex = (n+1) % len(nodes)
		edgeLength = np.sqrt( np.square(nodes[n][0] - nodes[nextIndex][0]) + np.square(nodes[n][1] - nodes[nextIndex][1]) )
		trackLength += edgeLength
	return trackLength

def trackWidth():
	track = findByName(trackXml['params']['section'], 'Main Track')

	if type(track['attnum']) is list:
		for attr in track['attnum']:
			if attr['@name'] == 'width':
				return float(attr['@val'])
	else:
		attr = segment['attnum']
		if attr['@name'] is 'width':
			return float(attr['@val'])

	# default value
	logger.info('default width: 5m')
	return 5

parseTrack()
writeNodes(filePrefix)
writeEdges(filePrefix, trackWidth())
writeRoutes(filePrefix)
writeConf(filePrefix)

if netconvertCommand:
	netconvert(filePrefix, netconvertCommand)
if sumoCommand:
	sumo(filePrefix, sumoCommand)

logger.info("Track Length: %d" % trackLength())
if logger.isEnabledFor(logging.DEBUG):
	showPoints()
