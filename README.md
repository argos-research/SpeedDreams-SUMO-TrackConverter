# SpeedDreams_SUMO_TrackConverter
Converts SpeedDreamTracks to SUMO Tracks

`trackparser.py` converts xml files in the form of [Track01.xml](exampleData/Track01.xml) to files readable by SUMO.
The SUMO files are placed inside a new directory `sumoBuild/` ([example data](exampleData/sumoBuild/)).

To run the conversion you have to execute:
```
./trackparser.py Track01.xml
```

Furthermore this script is also able to automatically run `netconvert` and `sumo-gui` (use this if you have SUMO installed):
```
./trackparser.py --sumo Track01.xml
```

Finally it is also possible to show a simple overview of the generated map by using the `--debug` option:
```
./trackparser.py --debug Track01.xml
```
