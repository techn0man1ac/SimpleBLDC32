# SimpleBLDC32

This is simple STM32 based 1000W BLDC motor(with Hall sensors) controller. Designed with [KiCAD](https://www.kicad.org/). In addition, the project supports Field-Oriented Control (FOC), which is implemented using the [SimpleFOC library](https://github.com/simplefoc). 

3-Phase STM32 BLDC motor controller specs:

- Battery: 12S;
- VCC: 36.0..50.4V;
- I(max): +/- 20A (have diodes for regenerative braking);
- Сurrent measurements per phase.
