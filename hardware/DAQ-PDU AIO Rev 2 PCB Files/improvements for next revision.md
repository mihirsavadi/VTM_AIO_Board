# IMPROVEMENTS FOR NEXT REVISION

- Order schematic net ports directions properly.  
- Steering position net goes into a non ADC port in the Main Teensy3.5, this is a dumb mistake.
- Change name of AIO instead of DAQ_PDU AIO.
- Use optoisolaters/optocouplers to buffer digital from power stages to mcu inputs, e.g. killsense on page 3 of schematic
- Fuse for servo connection
- Resistors u75 and u76 are 4.7k ohm but its attached to a pattern for a 360 resistor
- fill up middle gap make board dimensionally smaller
- maybe use raspberri pi pico or some rp2040 board and just one of them to replace both main mcu and teensy for parallel servo functionality (using the rp2040's PIO logic blocks)
- Use exclusively ACS781KLRTR-050B-T instead of the 150U's for greater resolution.
- Get rid of Batt and ACDC solder pads. Move literally all IO and power connections into superseal connections. Each superseal pin is good for 15A with normal thermal performance so unless really needed can just have one pin per V+ and GND. If need more just use another pin in parallel.
- Use up dead space in the middle a bit more.  
- Include a dedicated RTC instead of relying on GPS connection. Solder on directly since its a simple circuit. Can put under the GPS itself.

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

















