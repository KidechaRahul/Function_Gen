1 Overview

The goal of this project is to design a low cost system capable of generating various waveforms on two analog outputs while also providing two analog inputs allow measurement of the amplitude of a
signal, providing feedback for automatic level control and load impedance estimation

This project has a command line interface capable of controlling the system and providing measurement data back to the user.






2 Hardware Description

The circuit and PSPICE simulation is provided for the analog processing stages. Microcontroller:
An ARM M4F core (TM4C123GH6PMI microcontroller) is required


Analog output processing:
The system uses an MCP4822 DAC with an internal 2.048V reference to create a two separate waveforms with a voltage range of +/-5V at frequencies up to 40 kHz. The DAC is connected to SSI2TX/CLK/FSS and a GPIO pin for ~LDAC.  After a reconstruction filter is applied to each DAC output (a PCM signal), an op amp (1/2 of TLC072) is used to level shift and gain up the DAC output signal to a
+/-5V range.  The DAC and op amp are bypassed with 0.1uF capacitors.

Analog input processing:
The system is capable of measuring two analog signals with a voltage range of +/- 5V.  The input is sent to an op amp that is used to form an ideal rectifier.  The input and rectified signal are then sent to an op amp that is configured to both full-wave rectifier the signal and to act as a leaky integrator to measure the amplitude of the signal.   These inputs are presented to the microcontroller on AIN0 and AIN1.

All four op amps are part of a TLV2374 rail-to-rail input/output quad op amp powered from +/-5V supplies. This device is bypassed with a 0.1uF capacitor.

+/- 5V supply:
The 5V supply is derived from the USB bus.  A -5V power rail is created using a MAX660 charge pump.
22uF capacitors provide the switched energy element and the output storage capacitor and a 1uF
capacitor and 0.1uF capacitor are connected to the positive rail.

Connections:
Five connections will be provided to allow access to the 2 outputs and 2 inputs and provide a method to connect the 2-port device under test. 







3 Suggested Parts List


Part                                                                             Quantity

TM4C123G evaluation board (ARM M4F)                         1

MCP4822 SPI DAC with internal reference                       1

MAX660 (analog negative rail charge pump)                    1

TLV2374 (quad rail-to-rail input/output op amp)                1

TLC072 (dual high current op amp)                                  1

47ohm, 1% resistor (DAC reconstruction filter)                 2

49.9ohm resistor (output series resistor)                           2

1k, 1% resistor (DAC signal conditioning)                         2

10k, 1% resistor (DAC signal conditioning)                       4

12k, 1% resistor (ADC signal conditioning)                       2

23.7k, 1% resistor (ADC signal conditioning)                    4

47k, 1% resistor (ADC signal conditioning)                       8

49.9k, 1% resistor (DAC signal conditioning)                    2

52.3k, 1% resistor (DAC signal conditioning)                    2

0.1uF capacitor (bypassing, reconstruct filter)                  6

1 uF capacitor (leaky integrator, dc-to-dc                          3 converter positive rail)

22 uF capacitor (dc-to-dc converter                                  2 commutation and negative rail)

1N914 diode (ideal rectifier)                                              4

2x10 double-row header, unshrouded                               1

14pin 300mil socket (for quad op amp)                             1

8pin 300mil socket (charge pump, DAC, op                      3 amp amp)

Wire (22-24 AWG solid wire, 3+ colors)                            1

PC board (approx 4.5x6”)                                                  1

ST-7565R based graphics LCD and parts                   Optional

Tools, safety glasses, …                                               1 each








4 Software Description


A virtual COM power using a 115200 baud, 8N1 protocol with no hardware handshaking shall be provided with support to the following commands.

General:

If “reset” is received, the hardware shall reset. 
Signal Generation:

If “dc OUT, VOLTAGE” is received, the output will be configured to be a DC signal with a voltage (V) on the requested output (OUT).

If “cycles N” is entered, then the cycle count for the waveform will be limited to N cycles.
If “cycles continuous” is entered, then the waveform will be continuous.  This is the default setting.

If “sine OUT, FREQ, AMP, [OFS]” is received, the output will be configured to be a sinusoidal signal with a frequency of FREQ (Hz), an amplitude of AMP (V) on the requested output (OUT), and an average of OFS (V), default OFS is 0V.

If “square OUT FREQ, AMP, [OFS]” is received, the output will be configured to be a square wave with frequency of FREQ (Hz), an amplitude of AMP (V) on the requested output (OUT), and an average of OFS (V), default OFS is 0V.

If “sawtooth OUT, FREQ, AMP, [OFS]” is received, the output will be configured to be a sawtooth wave with a frequency of FREQ (Hz), an amplitude of AMP (V) on the requested output (OUT), and an average of OFS (V), default OFS is 0V.

If “triangle OUT, FREQ, AMP, [OFS]” is received, the output will be configured to be a triangle wave with a frequency of FREQ (Hz), an amplitude of AMP (V) on the requested output (OUT), and an average of OFS (V), default OFS is 0V.

Modify square wave to add duty cycle support as follows:
If “square OUT, FREQ, AMP, [OFS], [DC]” is received, the output will be configured to be a square wave with frequency of FREQ (Hz), an amplitude of AMP (V), and an average of OFS (V), and DC is 0-100%. Note that the DC if not the average of Vmax and Vmin in this case so this causes ALC operation to work differently.

If “differential ON|OFF” is active (ON), then output 2 will be the inverse of output 1.  This mode can be supported when the offset = 0.

If “hilbert ON|OFF” is active (ON), then output 2 will match the Hilbert transform of output 1. The sine at minimum should be supported.

If “run” is entered, the waveforms on the two outputs should start as last configured.  If an output is not configured, the output should be 0V.

If “stop” is entered, then the waveform play should stop and 0V should be output from both outputs.

The system should be able to drive both outputs at the same time, with DAC sample updates at the same time to prevent a phase offset.  The amplifier is designed to reach amplitudes of +/- 5V with no load connected if the USB bus voltage is sufficient.  In practical use, voltages of +/- 4.5V are typically
available.


Voltage measurement:
If “voltage IN” is received, the hardware shall display the average of the absolute voltage on input IN on the UART.

Automatic level control:
if output N is connected to input N, then auto-leveling control (ALC) can be enabled.  If ALC is enabled, the circuit will attempt to compensate for various load impedances. 
If “alc ON|OFF” is entered, then the ALC will be turned on or off.  If ALC is on, the hardware should measure the amplitude of the output and attempt to determine the correct scaling for the varying load.

Two-port gain calculation:
Although limited by charge pump current, output impedance, signal bandwidth, and scalar (no phase) measurements, it is possible to make a number of primitive swept frequency measurements.  By sweeping the frequency over a range of frequencies, the transfer function of the hardware can be calculated using the following commands:  For this mode to operation, OUT1 drive a two-port network under test, IN1 is connected to OUT1, and IN2 is connected to the output of the two-port network under test.  OUT2 is not used for this mode.

If “gain FREQ1, FREQ2” is received, the system shall sweep a sinusoidal output from FREQ1 to FREQ2 and capture IN1 and IN2 to calculate the gain.  The calculated gain at each frequency should be presented in a tabular format on UART.  A log(f) sweep should be used to quicken the calculation.

