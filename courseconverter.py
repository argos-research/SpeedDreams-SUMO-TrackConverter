from subprocess import call
#call(["ls", "-l"])


#nodes = open("coursechanger.nod.xml","w")
dataset = open("Default-Dataset.csv")
edges = open("coursechanger.edg.xml","w")
routes = open("coursechanger.rou.xml", "w")
i = 261

# Nodes creating
#print("Creating nodes...")
#
#z= []
#
#for line in dataset:
#	(x,y) = line.split(', ')
#	x = float(x)
#	y = float(y)
#	z += [(x,y)]

#print(z)
#z.sort(key=lambda y: float(x)+float(y) , reverse=False)
#print(z)
#exit(0)




#nodes.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<nodes xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:noNamespaceSchemaLocation=\"http://sumo.sf.net/xsd/nodes_file.xsd\">\n")

#for line in z:
#	(x,y) = line
	# print for debugging
	# print(y);
#	nodes.write("<node id=\""+ str(i) + "\" x=\"" + str(x) + "\" y=\"" + str(-y) + "\" />\n")
#	i = i + 1

#nodes.write("</nodes>\n")
#nodes.close()

#print("Nodes generated successfully")

# Edges creating

print("Creating edges...")

edges.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<edges xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:noNamespaceSchemaLocation=\"http://sumo.sf.net/xsd/edges_file.xsd\">\n")
for k in range(1,(i-1)):
	edges.write("<edge from=\"" + str(k) +"\" id=\"" + str(k) + "to" + str(k+1) + "\" to=\"" + str(k+1) + "\"/>\n")



edges.write("<edge from=\"" + str(k+1) +"\" id=\"" + str(k+1) + "to" + str(1) + "\" to=\"" + str(1) + "\"/>\n")
edges.write("</edges>\n")

edges.close()
print("Edges generated successfully")


# Routes generation

print("Generating routes...")

routes.write("<routes>\n")
#Car configuration
routes.write("<vType accel=\"0.5\" decel=\"5.0\" id=\"Car\" length=\"2.0\" maxSpeed=\"1.0\" sigma=\"0.0\" />\n")

edges = open("coursechanger.edg.xml")

lines = edges.readlines()
lines = lines[2:-1]
routen = ""

for line in lines:
	#debug line
	#print(line)
	(e,q) = line.split("id=\"")
	(j,h) = q.split('\" to')
	routen = routen + " " + j 

c = 0
count = 0
for count in range (0,c):
	routen = routen + routen
	count = count -1

routes.write("<route id=\"route0\" edges=\"" + routen + " \"/>\n")
#Car configuration 2
routes.write("<vehicle depart=\"1\" id=\"veh0\" route=\"route0\" type=\"Car\" />\n")
routes.write("</routes>\n")

edges.close()
routes.close()
dataset.close()
print("Routes generated successfully....\nClosed all files\n")



# System call to use netconvert to bake nodes and edges to a net file
call(["netconvert", "--node-files=coursechanger.nod.xml", "--edge-files=coursechanger.edg.xml", "--output-file=coursechanger.net.xml"])


# Call of Sumo gui with configuration
call(["sumo-gui","-c","coursechanger.sumocfg"])

print("All proccesses finished.")
