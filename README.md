# SpeedDreams_SUMO_TrackConverter
Converts SpeedDreamTracks to SUMO Tracks


The courseconverter.py script is intended to convert a (Default-Dataset).csv file
with a lot of (x,y) values to a SUMO track. These values were derived from a picture
of the SpeedDreams track (for further details please visit the KIA4SM wiki).
As these points are not correct enough another script is developed to get the xxx.nod.xml
file out of the track xml file that is provided by SpeedDreams.
The script (as of now 23.Nov.2015) commented the parts necessary for the first use case,
because the second use case is more promissing, if you need them just uncomment the lines. (be aware of deletion in the future)

The courseconverter.py at the moment takes the coursechanger.nod.xml, as said above, and generates all necessary files e.g.
courschanger.edg.xml, courschanger.<net/.rou>.xml and so on.
A cycle course is generated and a car is set into the simulation. The cyclic route can be repeated with raising the parameter
in the script. (be aware too many rounds or points in the route may crash the simulation/execution)
Sumo/Sumo-gui is executed afterwards automaticly with the sumocfg file to show the simulation output.


bb