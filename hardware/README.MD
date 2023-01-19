## Electronics Hardware of Bijou Espresso Machine 

The main board schematic can be found in `main.sch` and the pcb layout in `main.brd`.  
The AC power measurement board schematic can be found in `AC_board.sch` and the pcb layout in `AC_board.brd`. This is currently not implemented (in software).  
The temperature readout board schematic can be found in `temperature_2.sch` and the pcb layout in `temperature_2.brd`.  
Issues:
- footprint of the mosfets for the LED (Switch) is faulty, needs to be fixed.
- capacitor missing on encoder
- temperature molex connector is the wrong way around (needs to be soldered from the back)
- attention! usb serial resistors are 22 Ohms, not 220 Ohms!!
