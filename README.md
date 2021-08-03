# Arduino_ECSensor
This repository contains a library to operate a simple EC sensor with an Arduino (various AVR processors are supported, a little work will allow others to use this code as well).

This is a very simple yet reliable and accurate EC sensor for use with Arduino. It measures the EC by measuring the discharge time of a capacitor, which is also used to create a high frequency AC current through the probe to prevent ion migration to affect the measurements.

The circuit requires only three external components: two 330Ω resistors (use metal film type with low temperature coefficient), and a 22nF film capacitor (best is PP film, PET/polyester is a good choice as well ).

Effective range: approx. 0.01 mS/cm up to 5 mS/cm (equivalent to a NaCl TDS mineral content of 50-2500 ppm), which is a range typical for use with e.g. hydroponic growing applications, fresh water fish, well water monitoring, etc.

## Background
The hardware and working of the sensor is based on this research paper: https://hal.inria.fr/file/index/docid/635652/filename/TDS_Logger_RJP2011.pdf

The range of the sensor is dictated by the choice of capacitor, and the resulting AC frequency. The test liquid is the resistance R to be measured, it forms an RC network with the probe cacapitor. The capacitor is charged and discharged through the test liquid repeatedly resulting in an alternating current - one half of the cycle is the measurement, the second half is the compensation: the duration of the compensation cycle is based on what was measured during the measurement cycle.

For good EC measurement the resulting frequency should be between 1 kHz and 1 MHz, with best results in the 3-300 kHz range, giving close to three orders of magnitude range in the EC as probed.

As the AC is actually pulsed through the test liquid with time in between for charge/discharge of the capacitor, the frequency mentioned here is based on the pulse period, disregarding the time it takes for the capacitor to charge or discharge. For low frequencies this is not much of a difference, for higher frequencies the capacitor charge/discharge takes longer than the actual measurement pulse.

## Limitations
Larger capacitance allows for higher EC to be measured, however as the capacitor is discharged through the pin of the microcontroller this limits the maximum discharge current, and thus limits the upper part of the range.

The clock speed of the microcontroller used to measure the discharge time puts a minimum on this, approx. 50 clock ticks is the minimum as the time resolution is now 2% (one clock tick more or less). On a 16 MHz Arduino that gives a maximum frequency of 320 kHz for the probe. 

## Shortcomings and caveats
The most obvious shortcoming is the *limited range*: it is not suitable for most brackish and seawater applications. Seawater has a typical EC of 60-80 mS/cm.

Galvanic *isolation* and *ground effects* are another issue. When the project is not fully isolated from the ground (e.g. battery operated) this may severely throw off the measurement. 

## Circuit diagram

![schematic](https://user-images.githubusercontent.com/2418004/127961949-128b4cca-5474-4c82-a935-3c020b8f5cdc.png)

## Suggested EC probes

![EC_probe](https://user-images.githubusercontent.com/2418004/127962574-c5a07c26-dac3-4be6-9efb-2cf672eff1cc.png)
This code and component choice is based on standard 1/4" EC/TDS probes, as commonly used in desalination or water filtering appliances. These probes can be found cheaply on many e-commerce platforms such as eBay, Amazon, Taobao, Aliexpress, etc. Most come with a standard JST-XH type conmector.
