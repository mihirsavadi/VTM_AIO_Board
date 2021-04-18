# IMPROVEMENTS FOR NEXT REVISION

- Order schematic net ports directions properly.  
- Steering position net goes into a non ADC port in the Main Teensy3.5, this is a dumb mistake.
- Change name of AIO instead of DAQ_PDU AIO. 




# Various Older Comments Dont Delete
Fuse only, from the highside (no two wires - grounded to a pin in motec) [current sensors in all]
	*!- 2x injectors
	*!- 2x turbo solenoids
	*!- 4x shifting solenoids
	!- brake light
	!- egt sensor
	!- sync sensor

Fuse only, but power AND ground [current sensors in all]
	*!- g sensor
	*!- lambda to can (LTC) unit
	*!- ignition coil 
	*!- motec (two power and two ground to split the current)

full blown power stages with mosfet (p channel for no reason other than its nicer i.e the specific one i found)
	*!- fan (with 2 sets of connectors left and right)
	*!- fuel pump
	*!- auxiliary 


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^12V battery!^^^^^^^^^^^^^^^^^^^^^^^

**alternator also has current sensor.
**batter has beefy current sensor. 
^^^need to revise connection footprint.



------------------connectors need to add----------------
- engine kill switch repeat
- solenoid ground and kickback protection. (one wire from relay comes into button which grounds it). 


POWER NETS
+12V-
54-
55-
137-
66-
GND-
65-
64-
78-
138-
60-
164-
139-
46-
72-
119-
91-
71-
97-
68-
118-
122-
123-
130-
129-
127-
79-
124-
126-
+7.5V-
22-
50-
3-
4-
84-
135-
76-

















