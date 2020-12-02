# I4S_LoRaWanNode

The I4S* project investigates behavioral models of large scale LoRaWan networks used in smart contexts. This repository contains the micro-controller code for a _Harward architecture_ micro-controller for end-node testing. The AVR micro-controller performs regular transmissions in different configurations to test the performance of the network and the communication reliability. Transmission time samples will describe tbe behavior of the system and allow the creation of a model.

This is just one component of the I4S family. To explore all components, browse through the repositories starting with 'I4S_'.

## Project setup

The project is build on the Arduino core to ease development and speed-up results. It has been integrated in *Eclipse CDT* with the *Sloeber* plugin to allow standard C/C++ development without loosing the advantage of pre-made _Serial_ and _I/O_ classes. Future versions my factor out the Arduino source and rely merely on the AVR standard library. 

## Project structure

All core and library folders are supplied by Sloeber. 

## Directories

    .
    ├── LoRaMgmt.*		# routines to manage the test-depending LoRa communication
    ├── main.*		# Contains the startup code, setup and loop
    ├── TheThingsNetwork.*	# Interfacing class for the RN2483 LoRaWan module, modified from the TTN supplied solution
    ├── timer.*		# Hardware based timer routines to measure air-time.
    └── README.md		# this file
    
## Notes on versions

The used Arduino core is AVR 1.8.3 and the RN2483 module mounts a FW version 1.0.5, strictly needed to enable class C operation.
