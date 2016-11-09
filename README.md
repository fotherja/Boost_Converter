# Boost_Converter
Arduino code to control the Boost converter battery charger.

Pretty simple program that outputs a 53KHz PWM waveform to a control the MOSFET of a boost converter.

The duty cycle is ramped up until either the voltage_setpoint or the current_setpoint is reached and ramps down otherwise. The ramping occurs relatively slowly. There is overvoltage and undervoltage protection.

The circuit monitors input and output voltage with 11:1 voltage dividors. It also measures output current using the ACS712 5A current sensor. The code could easily be used without current sensing by grounding the analogue input pin for the current sense.

A series of tests are run every 15seconds to detect whether a battery is connected or not. The boost converter is idled by zeroing the duty cycle. The rate of decay of the output voltage is then analysed. If it decays then the output capacitance must be being discharged through the resistor divider pins for the ADC. If the voltage is maintianed, there must be a battery connected.
