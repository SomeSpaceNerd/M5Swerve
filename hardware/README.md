# Hardware

This directory contains all of the files relating to the hardware component of M5Swerve, including the 3D CAD files and bill of materials (BOM)

## Printed Parts

Basically you need 4 of each printed part that is part of the swerve module (because there are 4 modules) with the exception being needing 2 Main Mount As an 2 Main Mount Bs   
You need one of each printed part that is part of the drivebase itself (mainly bellypan segments, corner brackets, and bumpers)

### Materials

All solid parts were printed using just under 1 whole spool of Overture PLA+/PLA Professional in space gray on an Ender 5 Mercury One.1, you don't really need anything crazy in terms of material properties or print quality for this.   
All TPU parts (tires and bumpers) were printed using ERYONE 85A TPU in transparent blue on a Voron 0.2R1, this filament is quite tricky to print but with a well tuned printer it's not too big of a deal. You might be able to get away with using 90A or 95A TPU, as that is much more common, but I can't say for sure because I did not test that.   
The RIL cover specifically was printed using Overture Transparent PETG but really any translucent/transparent white/clear filament will work.   

## Custom Proto Module

Attached to the bottom of the CoreMP135 is an M5Stack Proto module containing a StampIO and a few breakouts.   
The parts used are as follows:

* [M5Stack Proto Module](https://shop.m5stack.com/products/proto-module)
* [M5Stack StampIO](https://shop.m5stack.com/products/m5stamp-extend-i-o-module-stm32f0)
* [30AWG Solid Core wire](https://www.amazon.com/TUOFENG-Solid-Tinned-Copper-Colors/dp/B0C1J5M3P7/ref=sr_1_1?sr=8-1)
* [8-pin right angle female header](https://www.amazon.com/MTCELL-Single-Female-Header-Connector/dp/B0GM6JW5QN)
* [3-pin right angle JST-XH female](https://www.amazon.com/MECCANIXITY-2-54mm-Connector-Adapter-Connection/dp/B0BMDZR7RZ)
* [M3x35mm Screws](https://www.amazon.com/BNUOK-Socket-Stainless-Threads-Spanner/dp/B0DJQH1HT4)

There are a few ways to go about laying out and assembling it, but this is what is required

* StampIO on I2C1
* I2C1, UART2, 5V, 2x GND breakout on 8-pin right angle female header (for servo driver and IMU/INA228)
* StampIO Pin 4, 5V, GND breakout on 3-pin right angle JST-XH female (for neopixel sequin)

By default the StampIO only drives one neopixel per pin, the sequin has 7 neopixels so I created a [fork of the firmware](https://github.com/SomeSpaceNerd/M5Unit-EXTIO2-Internal-FW) to work properly for this application

## Other Parts

This is a section for parts that may not be contained in or immediately obvious in the generated BOM file, as well as their purchase links (not affiliate links) when applicable

* [Roll-In T Nuts](https://www.amazon.com/M3-M3x0-5mm-T-Nuts-Extrusions-Replacement/dp/B0DPK1TXBQ)
* [Voron Spec (M3x5x4) Heat Set Inserts](https://www.amazon.com/ruthex-Threaded-Inserts-VORON-Components/dp/B0CDH36ZMX)
* Robot Controller: [M5Stack CoreMP135](https://shop.m5stack.com/products/m5stack-coremp135-w-stm32mp135d)
* Robot Radio: [GL.iNet GL-MT300N-V2 (Mango)](https://www.amazon.com/GL-iNET-GL-MT300N-V2-Repeater-300Mbps-Performance/dp/B073TSK26W)
* Radio Power Supply: [Generic "Waterproof car adapter" 12V to 5V 3A Micro USB module](https://www.amazon.com/dp/B08Q34TYV5)
* Main Breaker/Power Switch: [goBILDA Floodgate V2](https://www.gobilda.com/floodgate-power-switch-xt30-current-sensing/)
* Battery: [TATTU 850mAh 3S LiPo with XT30](https://www.amazon.com/TATTU-850mAh-11-1V-LiPo-Battery/dp/B0BWRR3FFP) (Really any similar size/capacity 3S LiPo with an XT30 will work)
* Ethernet Cable: [15cm Cat6 Low-Profile](https://www.amazon.com/JUXINICE-Ethernet-Low-Profile-Connector-High-Speed/dp/B0FMY1VMK1/) (when I bought this cable it had a 6in/15cm option, it seems that has been removed, but the 1ft version should work fine)
* [XT30 Female Pigtail](https://www.amazon.com/ZHOFONET-Pigtail-Connector-Adapter-Silicone/dp/B09LYHHS9J)
* Power Wiring: [Good quality 16-18AWG Silicone Wire](https://www.amazon.com/Fermerry-Electric-Silicone-Cables-Stranded/dp/B089CPH72F)
* Data Wiring: [Good quality 20AWG Silicone Wire](https://www.amazon.com/TUOFENG-Flexible-Silicone-Tinned-Copper/dp/B07G2GLKMP)
* Ferrules: [Single and Dual Wire](https://www.amazon.com/Lytool-Double-Ferrule-1200Pcs-ferrules/dp/B0BY2CS16J)
* RS-485 Termination Resistors: [120ohm Through-hole](https://www.amazon.com/EDGELEC-Resistor-Tolerance-Multiple-Resistance/dp/B07QH5QLR1)
* Radio Buck Connector: [Generic Male DC Barrel Jack Screw Terminal](https://www.amazon.com/California-JOS-Male-Female-2-1x5-5mm/dp/B0CR8TZ41W)
* Battery Straps: [Hook/Loop Tape](https://www.amazon.com/Strips-Adhesive-Double-Backing-Crafting/dp/B0CGVPMJBP) and/or [Hook/Loop Cable Ties](https://www.amazon.com/Reusable-Newlan-Adjustable-Organizer-Management/dp/B081HH5X61/ref=sr_1_13?sr=8-13)
* STEMMA QT Cables: [Generic STEMMA QT Male to Male and Male to 2.54mm Header Cables](https://www.amazon.com/elechawk-SparkFun-Development-Breadboard-Connector/dp/B08HQ1VSVL)
* 2.54mm Crimp Headers: [Generic Crimp Set](https://www.amazon.com/IWISS-1550PCS-Connector-Headers-Balancer/dp/B08X6C7PZM)
* Lots of zip ties/cable ties, particularly the 4mm and 2.5mm wide variety in 150mm length

## General Build Notes

The assembly process for M5Swerve is pretty straightforward if you use the CAD for reference, but here are a few notes that will be useful

* The Roller Mount part requires 8 heat set inserts, 4 are placed in the bottom of the legs and 4 at the top, the top ones MUST be inserted from the back of the part (the side that was on the printbed)
* The servo has a limited range of motion so it must be aligned correctly, install the 2 M2x4 screws and M2 nuts that hold the servo in place loosely, as well as the gear and it's M2x6 screw (the gear is a tight press fit), command the servo to 30 steps (default minimum position) and align the steering mount so the notch in the steering mount aligns with the notch in the retaining ring closest to the long side with the screw holes (so when looking at the modules top-down with the servo at the bottom that is the notch on the right side for A variant modules and the notch on the left side for B), then slide the servo into place and fully secure it down
* If you notice the drive gears are rubbing (specifically the input gear and sun gear) add 1-2 M3 washers between the legs of the roller mount and face of the main mount to shim the parts a bit

## Custom Firmwares

I have made 2 forks of M5Stack firmware for this project , one for the Roller485s and one for the StampIO. Both must be flashed using an ST-LINK or similar programmer and instructions can be found in their respective repo

### Roller485

My custom Roller485 firmware allows them to communicate at 1MBaud instead of the default maximum of 115200, which is necessary for the tight control loop used by the robot software to work. [Firmware download and flashing instructions are available here.](https://github.com/SomeSpaceNerd/M5Unit-Roller485-Internal-FW)

### StampIO

My custom StampIO firmware simply makes it drive a chain of 7 neopixels instead of just 1 per pin, specifically for the 7 pixel neopixel sequin used in this project as the RIL. [Firmware download and flashing instructions are available here](https://github.com/SomeSpaceNerd/M5Unit-EXTIO2-Internal-FW)

## Wiring

The wiring can be slightly complex and convoluted at times, so again here is a simple list of what you need to know to assemble it

* The XT30 pigtail connects to the output of the Floodgate, it's ground wire goes to the CoreMP135's PWR485 port, and the positive wire goes into the INA228 (in high-side sensing mode with VBUS bridged), the output from the other sense terminal of the INA228 goes into the CoreMP135's PWR485 port
* Power is distributed to the swerve modules through a bunch of dual-wire ferrules at each drive motor, starting at the CoreMP135 then going to the front-right, back-right, back-left, and front-right the front right splits the power input into a wire for the Servo buck input (just cut off and strip one of the wires it comes with)
* The RS-485 wiring is very similar, the start and end of the bus (CoreMP135 and front-left module) are single ferrules with a 120 ohm resistor placed in the ferrule as well, all the others are dual wire ferrules in the same configuration as the power wires, with twisted pair wires
* The servo wiring goes from the output of the buck into the servo driver, then the output of the servo driver into a splitter, that splitter has 2 splitters connected to it, one connects to the left front/back servos and the other connects to the right front/back servos, there is no particular order
* The servo driver UART wiring is a simple 3 pin 2.54mm header cable from the proto module's UART2 breakout to the UART pins on the driver
* The I2C wiring goes from the 2.54mm header breakout on the proto module to the BNO055, then to the INA228
* The Neopixel wiring is a simple 3 pin JST-XH/2.54mm header cable with the other end soldered into the sequin
* The radio buck converter connects to the CoreMP135's DC jack using the screw terminal breakout, you may need to splice and extend the buck converter input wires
* The radio wiring is self-explanatory, the output Micro USB of the buck converter goes into the radio's Micro USB Port, the Ethernet cable connects between the CoreMP135's Ethernet 1 port to the radio's LAN port
