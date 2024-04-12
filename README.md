# Arduino MAX7219 Spectrum Analyzer with Peak Hold
[<img src="https://i.ytimg.com/vi/D2GJO4K_w3I/maxresdefault.jpg" width="100%">](https://www.youtube.com/watch?v=D2GJO4K_w3I "Arduino MAX7219 Spectrum Analyzer with Peak Hold")

Original code is from HAZI TECH. I modified the code, add bar mode and peak hold.

I build this using spare Arduino Nano and MAX7219 matrix display which I have been using in my previous DIY. So I just need to buy RTC DS3231 and other small parts for this.

# Schematics
![Arduino_SpectrumAnalyzer_Schematics_bb](https://github.com/ChrisSuryanto/ArduinoSAPH/assets/146957789/4aff2e46-4199-4af0-8fc6-2cdde6749a92)

Diode on VCC line is 1N4007, or any other standard diode, just for protection, so 5V pin on Arduino will function as supply input only. LED matrix will be powered from external supply only, not from arduino's 5v pin, thanks to diode's 1 way current direction characteristics.
