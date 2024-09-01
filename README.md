# Örgryte Watch Winder
This is a hobby project. Building a Watch winder controlled by a Raspberry pi Pico.

The RTC PIO program works when I have a cheap logic analyzer (https://www.az-delivery.de/en/products/saleae-logic-analyzer?_pos=1&_psq=logic&_ss=e&_v=1.0)  connected. But behaves strange when disconnected.
So the analyzers input impedance or something helps the circuit. I have not had time to look into that yet. (And software is my domain...)

BOM
- Rapberry Pi Pico
- [RTC ds1302 board](https://www.az-delivery.de/en/products/rtc-modul?_pos=2&_psq=RTC&_ss=e&_v=1.0)
- [Oled Screen 128x64, ssd1306](https://www.az-delivery.de/en/products/0-96zolldisplay)
- [Stepper motor 28BYJ-48]( https://www.mouser.com/datasheet/2/758/stepd-01-data-sheet-1143075.pdf )
- [Motor driver board ULN2003](https://www.hadex.cz/spec/m513.pdf )
- 4 button switches

Circuit 
![image](https://github.com/user-attachments/assets/0f97ff8a-f416-4822-b9b0-700ad1891d10)


Prototype
![IMG20240718102837](https://github.com/user-attachments/assets/c5e7f0cf-24f8-479e-9a0b-cec28afd60ee)


Video

https://github.com/user-attachments/assets/534dc787-c206-47b4-83e5-13a05a5b2c3c

