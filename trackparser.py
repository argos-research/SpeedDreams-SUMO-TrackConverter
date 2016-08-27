#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
import logging
import collections
import sys
from subprocess import call
import csv

logger = logging.getLogger('trackparser')

parser = argparse.ArgumentParser(description='convert SpeedDreams\' map format to SUMO\'s')
parser.add_argument('inputFile', help='input file')
parser.add_argument("-w", "--width", help='track width', type=int, default=5)
parser.add_argument("-s", "--sumo", help="run sumo (optional: path to sumo)", nargs="?", const="sumo-gui")
parser.add_argument("-n", "--net", help="run netconvert (optional: path to netconvert)", nargs="?", const="netconvert")
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
track_width = args.width

degreeStepSize = args.degree

if not os.path.isdir(outputPath):
	os.mkdir(outputPath)

nodes = []
helpNodes = []
directionDegree = 0

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
		routesFile.write('<vehicle depart="0" id="veh0" route="route0" type="Car" />\n')
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
	plt.scatter(*nodes[0], color='g')
	plt.plot(*zip(*nodes + [ nodes[0] ]))

	for i, node in enumerate(nodes):
		plt.annotate(xy=node, s=str(i))

	plt.show()

def trackLength():
	trackLength = 0;
	for n in range( len(nodes) ):
		nextIndex = (n+1) % len(nodes)
		edgeLength = np.sqrt( np.square(nodes[n][0] - nodes[nextIndex][0]) + np.square(nodes[n][1] - nodes[nextIndex][1]) )
		trackLength += edgeLength
	return trackLength

def import_csv():
	with open(inputFile) as csvfile:
		trackreader = csv.reader(csvfile, delimiter=',', quotechar='|')
		for row in trackreader:
			coord = np.fromstring(' '.join(row), dtype="float64", sep=" ")
			x = (coord[0]+coord[2])/2
			y = (coord[1]+coord[3])/2
			nodes.append( (x,y) )

def main():
	import_csv()
	writeNodes(filePrefix)
	writeEdges(filePrefix, track_width)
	writeRoutes(filePrefix)
	writeConf(filePrefix)

	if netconvertCommand:
		netconvert(filePrefix, netconvertCommand)
	if sumoCommand:
		sumo(filePrefix, sumoCommand)

	logger.info("Track Length: %d" % trackLength())
	if logger.isEnabledFor(logging.DEBUG):
		showPoints()


if __name__ == '__main__':
	main()
