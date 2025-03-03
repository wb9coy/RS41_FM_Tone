/*
 * config.h
 *
 *  Created on: Oct 25, 2021
 *      Author: eswiech
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#define VERSION_INFO ((char *)"Version 1.0")

#define CALL_SIGN ((char *)"W6SUN")

#define TRANSMIT_FREQUENCY  431.050f //Mhz

// TX Power
#define TX_POWER  0 // PWR 0...7 0- MIN ... 7 - MAX
// Power Levels measured at 434.650 MHz, using a Rigol DSA815, and a 10 kHz RBW
// Power measured by connecting a short (30cm) length of RG316 directly to the
// antenna/ground pads at the bottom of the RS41 PCB.
// 0 --> -1.9dBm
// 1 --> 1.3dBm
// 2 --> 3.6dBm
// 3 --> 7.0dBm
// 4 --> 10.0dBm
// 5 --> 13.1dBm - DEFAULT
// 6 --> 15.0dBm
// 7 --> 16.3dBm

#endif /* INC_CONFIG_H_ */
