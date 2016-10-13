# SolarLogger

![alt tag](https://github.com/TomasTT7/SolarLogger/blob/master/SolarLogger.jpg)

This is a low power Arduino Pro Mini (3.3V) based logging device utilizing an external serial flash memory. In this particular configuration it uses 3 ADC channels to sample voltage on 2-4 solar cells, LTC3105 output, a shunt rezistor (connected to the low side of a charging battery) and reads the current temperature on the DS18B20 thermometer. These values are stored in the external flash memory.

Reading the flash memory and updating the settings can be done by entering a BOOT loop in the first 5 seconds after powerup by issuing a 'B' command via a 250000 baud rate serial connection.  


**LIST OF PARTS:**

- Arduino Pro Mini
  
- W25Q64BV Flash Memory
  
- DS18B20 Digital Thermometer
  
- LTC3105 Converter
  
- 4x Solar Cell
  
- 1Ω Shunt Rezistor
  
- 4x Voltage Divider Rezistor 
  


![alt tag](https://github.com/TomasTT7/SolarLogger/blob/master/voltage(mV)_temperature(m%C2%B0C)_8s_charging.png)

![alt tag](https://github.com/TomasTT7/SolarLogger/blob/master/current(mA)_8s_charging.png)
