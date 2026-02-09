# Two-Wheel Balancing Robot

A PID-controlled two-wheel self-balancing robot powered by CircuitPython running on a Raspberry Pi Pico.

<img src="/docs/media/Two-Wheel Balancer - Front.jpeg" alt="Photo of the 'front' side of the robot" width="250"/> <img src="/docs/media/Two-Wheel Balancer - Back.jpeg" alt="Photo of the 'back' side of the robot" width="250"/>

<img src="/docs/media/Two-Wheel Balancer in Action.gif" alt="GIF of a two-wheeled balancing robot doing its best to stay upright" width="250"/>

## Project Status and Next Steps
Current Status: **On hold**. 

The proof of concept works. The robot successfully prevents itself from falling over for a respectable period of time using only IMU tilt data. Horizontal station-keeping (preventing displacement or vehicle drift) would be the next step for development. This may or may not require energizing and using the motor encoders that are part of the motor assembly.

## Project Motivation
I wanted to create a mobile platform that would allow me to explore robotics safely through trial and error. I came across the two-wheel balancing kit while shopping for electronic parts and figured it might be a helpful bundle to get started. With the kit, I was able to explore an interest in embedded systems and control theory.

Near the beginning of 2025, I began paying closer attention to LLMs, eventually deploying a Docker container hosting local open source models and connecting to third-party LLM APIs. These LLMs have been very helpful providing me with information about topics and ideas I had not known before (including robotics relevant concepts), allowing me to explore quickly and transition to testing more confidently.

## Technologies and Tools
The following is a list of equipment used during testing, building, and assembly of the robot.

### Personal Protective Equipment (PPE)
- Safety glasses
- Exhaust fan and an open window

### Electronics
- Benchtop power supply 
- EV-Peak 50W 6Amp multi-chemistry balance charger & discharger
- LiPo battery (4C)
- LiPo battery voltage tester with low voltage buzzer alarm
- Voltage regulators (12V 2A; and 3.3V 0.6A)
- Blade fuse (3A)
- Raspberry Pi Pico
- Adafruit 9-DOF Orientation IMU Fusion Breakout - BNO085
- 2x JGA25-371 DC Gearmotors with Encoders
- L298 H-Bridge motor driver
- Double-sided PCB
- Micro USB to USB-C Cable

### Hardware
- M2 and M3 bolts, nuts, and standoffs
- Vernier Caliper

### Software
- VSCode
- Python (v3.13.2)
- CircuitPython (v10.0.3)

### Fabrication
- CAD Modelling: Onshape 
- 3D Printing: Prusa i3 Mk3
- Soldering: Miniware TS100 Mini

## Installation/Usage
### Hardware
This robot build began with acquiring a [two-wheel balancing kit](/docs/media/Two%20Wheel%20Balance%20Car%20Chassis%20with%20JGA25%20Motor%20Kit.pdf). This kit contained two JGA25-371 DC gearmotors with encoders, metal brackets, acrylic boards, and a few screws and standoffs. The acrylic sheets that came in the kit had asymmetric cutouts embedded throughout which added inconsistency to my build leading me to design my own structural parts.

While a decent kit to get the ball rolling, if I were to start over again I would prefer to spend more time planning out what parts I wanted and needed. I would then design the robot from the ground up rather than retrofitting to the kit parts, which is what I ended up doing.

In any case, the CAD models I created to harden the robot structure can be accessed and viewed in two ways as listed below,
1. Via Onshape:
    - [Two-wheel balancer frame - Onshape Link](https://cad.onshape.com/documents/7dfefa6e2921ad39be2e3821/w/3f55fc51df92af93037a4f11/e/0a32c4429e5ca3bb6b5eb229?renderMode=0&uiState=69571ee25eaa326608b95544)
    - [4C LiPo Battery Power Distribution Layout - Onshape Link](https://cad.onshape.com/documents/d5dfdf18f97619740b08314e/w/11a66787474f13b8d7b80358/e/5259f1931b61786a48ea3b8f?renderMode=0&uiState=69571fcdb97c0f56a7a46870)

2. Via STEP files:
    - [Two-wheel balancer frame - STEP file](/docs/CAD/Two-Wheel%20Balancer%20Frame.step)
    - [4C LiPo Battery Power Distribution Layout - STEP file](/docs/CAD/LiPo%20PDB.step)

### Software
A core component of the build makes use of the BNO085 IMU. As libaries exist for this IMU, I opted to use [CircuitPython](https://circuitpython.org/) running on a Raspberry Pi Pico due to Pythons ease of use, and my familiarity with Raspberry Pi's hardware products. The libraries required to be loaded onto the Pico alongside the main `code.py` file can be seen in the following image:

<img src="/docs/media/CircuitPython Files.png" alt="Screenshot of the files located on the Raspberry Pi Pico. boot_out.txt, code.py, and a lib folder with the dependencies adafruit_bno08x, adafruit_bus_device, and adafruit_register" width="200"/>

#### Pico USB Serial Connection
During testing and iteration of the software, my approach was to connect the Pico to my laptop via a USB cable, allowing me to access and update the onboard `code.py` directly using VSCode. 

I could then also monitor serial data from the Pico on my laptop using a tool such as `minicom`.
> On a MacOS device, I run the terminal command `ls /dev/tty.*` to find the ID of the USB port connected to the Pico, and then I can monitor the USB serial output by running `minicom -D /dev/tty.usbmodem<PORT_ID> -b 115200`, where the previously identified port ID replaces `<PORT_ID>`, and the baud rate of 115200 is default for the Pico.

#### Python Scripts and Tests
The `code.py` file onboard the Pico would be loaded with contents from any one of the files under the `/src` folder.

- `code_alt_IMU_only.py` is the latest "working" code, which maintains an upright posture, but does not prevent drift.
- `code_alt_encoder_added.py` is an alternate script that intends to introcude a cascaded PID control loop, but this is not yet properly implemented.
- All other scripts in `/src` are preceded with "test" and are used to focus on individual tests to isolate and validate to the user that hardware is working.

`/examples` contains scripts created by Adafruit Industries. These scripts were used as a reference for building my own. 

`/experiments` contains a couple of scripts and CSV files, which were quick and dirty attempts to capture and display tilt data. The idea here was to capture data under controlled tests to try and identify an inflection point indicating the angle where the robot is at the unstable equilibrium, between toppling forward or backward. On testing, it was clear that more rigorous experiments and analysis would be required to get any significant value from this, and so this approach was deprioritized as other more pressing problems arose.
