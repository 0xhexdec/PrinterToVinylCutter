# PrinterToVinylCutter

Converting an old HP DesignJet 130 to a vinyl cutter.

## Why and How?

Out of pure luck, I found an old HP DesignJet 130 for free. The printer was missing the printheads and ink cartridges, the ink tubes were clogged and the paper roll feed contraption was missing. The printer is able to print on A1+ paper (610mm or 24 inches width) so there should be enough space to fit a decent sized vinyl in it. (The standard 630mm fits without a problem)

The idea is, to use a much of the original parts as possible to make the conversion easy from the mechanical point of view.

## Hardware

- Arduino Uno, Nano or similar (anything that can run GRBL) (~3€)
- BluePill (STM32F103C8 board, full featured) (~3.50€)
- L298N driver board (~3€) or L293D driver board (~4€)
- solenoid version
  - 12 Volt solenoid (~3€)
  - 12 Volt power supply (you don't need a lot of wattage here, to have a buffer, go for 60 watts or so)  (~5€)
- servo version
  - cheap model servo like MG995 (~5€)
- male-male dupon wires
- a broken printer

***Keep in mind, the Bluepill can only be programmed using a dedicated programmer (like the ST-LINK V2). The programmer costs round about 3€.***

To modify the printer you also need:

- Screw driver (mine had a lot of Torx)
- Soldering Iron
- Thin wires
- Multimeter (if you can't find anything about the encoder pinout)

## Working principle

I don't wanted to build anything from scratch, that's why I use a basic Arduino to run a standard version of GRBL. Instead of using stepper drivers, I pass the STEP/DIR signals to the BluePill board that acts as a 2 channel motor driver. The BluePill than handles the motor control. The rest is more or less off the shelf.