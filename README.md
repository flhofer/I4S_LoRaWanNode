# I4S_LoRaWanNode

The I4S* project investigates behavioral models of large-scale LoRaWan networks used in smart contexts. This repository contains the micro-controller code for a _Harward architecture_ micro-controller for end-node testing. The AVR micro-controller performs regular transmissions in different configurations to test the performance of the network and the communication reliability. Transmission time samples will describe the system's behavior and allow the creation of a model.

This is just one component of the I4S family. Browse through the repositories to explore all components starting with 'I4S_'.

## Project setup

The project builds on the Arduino core to ease development and speed up results. It has been integrated through *Eclipse CDT* with the *Sloeber* plugin to allow standard C/C++ development without losing the advantage of pre-made _Serial_ and _I/O_ classes. Future versions may factor out the Arduino source and rely merely on the AVR standard library. 

## Project structure

Sloeber supplies all core and library folders. 

## Directories

    .
    ├── LoRaMgmt.*		# routines to manage the test-depending LoRa communication
    ├── main.*		# Contains the startup code, setup, and loop
    ├── TheThingsNetwork.*	# Interfacing class for the RN2483 LoRaWan module, modified from the TTN supplied solution
    ├── timer.*		# Hardware-based timer routines to measure air-time.
    └── README.md		# this file
    
## Notes on versions

The used Arduino core is AVR 1.8.3, and the RN2483 module mounts an FW version 1.0.5, strictly needed to enable class C operation. The module's firmware can be updated via `ICSP` or via USB using the Arduino as a bridge. NOTE: be careful to select the correct file for the USB update. The modem will get unusable and can only be recovered via ICSP. More info about Pickit3 and how to use the ICSP [here](https://components101.com/misc/pickit3-programmer-debugger-pinout-connections-datasheet).

## Usage

The latest test software has been boiled down to a simple (non-interactive) menu. Once booted, the prompt `Select test:` asks for user input on the test to be executed. It does not parse carriage return or new line inputs and does not wait for input. This means the commands and corresponding values must be sent character by character within the serial's read timeout. For example, `m0` to set to mode 0 must be sent in one go with `0` sent within one serial timeout delay after `m`.

The menu consists of generic commands, while specific commands depend on the chosen mode. Thus, before selecting those, we must select mode first.

Generic Commands are
```
'm' : test mode of the node, between 0 and 4. Default 0 is off (See modes)
'R' : run the test
'S' : stop test execution
'T' : print microcontroller type
'I' : print modem identification number, =EUI
'p' : set power index for tests accompanied by a digit number, [0..5], default 0.
'l' : random data length to send, 0-242/255, depending on mode. Default 1.
'r' : number of times to repeat a test, [0 to 100]. Default 5 repeats.
```
Other commands depend on the selected mode, `m`.

### Mode 0: Off

In this mode, the device does nothing. Even though it reacts to `R` run and `S` stop, there will be no output.

### Mode 1: LoRa Transmissions (Disabled, STUB)
For mode 1, plain LoRa packets, we start a continuous LoRa packet transmission with no breaks. It is intended as a simulation of interference signals. However, this mode is not implemented for this MCU type.

The options are the following:
```
'f' : transmission frequency in 100kHz steps, a number between 8630 and 8700. Default 8683(0000)Hz.
'b' : bandwith in kHz, one in [250, 125, 62, 41, 31, 20, 15, 10]. Default 250kHz.
'c' : code rate of the transmission, denominator value between 4/[5-8]. Default 8.
's' : spread factor to use in [7..12]. Default 12.
```

### Mode 2: LoRaWan Transmissions

This mode simulates LoRaWan transmissions with a specific interval. In particular, the transmissions repeat for `30` (hardcoded) times to execute measurements. If the repeat is set greater than 0, an unsuccessful experiment is repeated that many times. If the repeat is set to 0, the send continues until the `S` stop command is sent.

The options are the following.
```
'u' : set to unconfirmed test execution 
'c' : set to confirmed test execution 
'o' : set to OTAA mode join
'a' : set to ABP mode join
'n' : disable modem reset between tests
'f' : full air-time, disable duty cycle
'N' : Input Network session key ABP, 32 Hex characters, default NULL.
'A' : Input Application session key ABP, 32 Hex characters, default NULL.
'D' : Input device address ABP, 8 Hex characters, default NULL.
'K' : Input Application key OTAA, 32 Hex characters, default NULL.
'E' : Input device EUI address for OTAA, 16 Hex characters, default NULL.
'C' : Select channel mask, 2 Hex characters, setting mask for 16 possible channels.
'd' : set fixed data rate, [0-5,255]. Default 255, which is auto-starting with DR5.
'x' : set window delay in milliseconds [1000-15000]. Default 1000ms.
```

All keys and addresses, also the channel mask, are composed by hex strings. They may optionally be terminated by ending `h`. The channel mask refers to the default Semtech channels 1..8 (Mask 0-7) and 8 on the 867MHz frequency. 

### Mode 3: LoRaWan with remote control

This mode works the same way as mode 2, with the difference that we wait for a downlink command to start the experiment. Options for this mode are the same as for mode 2.

### Mode 4: LoRaWan join flood

In this mode, no package send is performed. Instead, we only repeat the join sequence without pause. Options for this mode are the same as for mode 2. However, some options may have no effect.

## Examples

To exemplify the usage of the microcontroller menu, we show here two examples for LoRa and LoRaWan communication. The codes can also be put together in one string, with or without spaces. Furthermore, all letters after 'R' may be ignored. This setup has been devised to be used with an external logging script.

### LoRa send

The following is an example of the command string:
```
m1f8650b125s7c5R
```
Sends plain LoRa signals with length 1, at 865.0MHz and Bandwith 125kHz, SF7, and code rate 4/5 until an `S` is supplied. The `R` at the end of the string triggers immediate test execution. There will be no further prints unless some errors occur during test execution.
```
Select Test:
Start test
Run test
```

Once a stop command `S` is supplied, the tests are stoped, and statistics are printed.
```
Test stop!
Evaluate
End test
Results:
01;0431982;0175857;000000.000;000000.000;000000.000;0x00;868600000;12;01;000;000
```
The values shown are `test number; total runtime in ms; total transmission count; time tx; time to rx; time after rx; channel mask; frequency; SF; power; RSSI; SNR`. Not all values are filled.

### LoRaWan send

Similarly, here is an example for a LoRaWan test string:
```
m2acr5CFFhp1d255l5D01234567hN01234567890ABCDEF01234567890ABCDhA01234567890ABCDEF01234567890ABCDhR
```
This sets to mode 2, confirmed sends on ABP and the set device address, network, and application session key. Data length is set to 5, data rate automatic to 255. Repeat count to 5.

At the end of the test sequence, or whenever the stop command is supplied `S`, the micro will print out the result statistics.
```
Results:
01;0001313;01;000108.444;000187.556;001296.424;0xFF;867500000;05;06;-104;003
...
30;0001312;01;000108.372;000187.484;001296.280;0xFF;868100000;05;06;-95;006
done
```
The values shown are `test number; total runtime in ms; time tx; time to rx; time after rx; channel mask; receive frequency; data rate; power in dBm; RSSI; SNR`.
