# fx201p-ram-replacement
FX201P RAM replacement

A PCB that replaces the RAM in an FX201P. Both HD36106 devices are replaced by an RP2040 based Pico-Zero
module. That replaces the memories and the program RAM. Additional features are adde:

* Save program space to RP2040 internal flash
* Load program space from flash
* Save memory space to flash
* Load memory space from flash
* Save all memory to flash
* Load all memory from flash
* atalog of flash slots.

  The flash has 250 slots, each is big enough to hold the entire RAm of the FX201P.

  
Command Mechanism
=================

To issue a command to the RP2040, put a number of the following format in M8:

ccsnnn E88

where:

cc     Command code
s      Status
nnn    Parameter


Command codes:

94          Zero all memory (both HD356106 RAM areas, but not flash)
95          Zero memories
96          Zero program space

11          Load program and memory areas from flash slot nnn
12          Load memory area from slot nnn
13          Load program area from slot nnn

15          Save program and memory areas to flash slot nnn
16          Save memory area to flash slot nnn
17          Save program area to flash slot nnn

47          Shows empty flash slots as a bitmap in octal in M9 A 1 denotes a non-blank slot

USB Interface
=============

The USB of the Pio-Zero has a menu driven interface.
