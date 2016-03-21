# Boost_Converter
Arduino code to control the Boost converter battery charger.

Pretty simple program that outputs a 53KHz PWM waveform to a control the MOSFET of a boost converter.

The duty cycle is ramped up until either the voltage_setpoint or the current_setpoint is reached and ramps down otherwise. The ramping occurs relatively slowly. There is overvoltage and undervoltage protection.
