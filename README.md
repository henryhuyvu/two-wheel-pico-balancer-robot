# Two-Wheel Balancing Robot

A PID-controlled two-wheel self-balancing robot powered by CircuitPython running on a Raspberry Pi Pico.

<img src="/docs/media/Two-Wheel Balancer - Front.jpeg" alt="Photo of the 'front' side of the robot" width="250"/>
<img src="/docs/media/Two-Wheel Balancer - Back.jpeg" alt="Photo of the 'back' side of the robot" width="250"/>

<img src="/docs/media/Two-Wheel Balancer in Action.gif" alt="GIF of a two-wheeled balancing robot doing its best to stay upright" width="250"/>

## Project Status
Current Status: **MVP / Proof of Concept**. The robot successfully prevents itself from falling over for a respectable period of time using only IMU tilt data. Horizontal station-keeping (preventing displacement or vehicle drift) is identified as the next step for development.

## Tech Stack
The following is a list of equipment used to test, build, and assemble the robot into its current form.

### PPE
- Safety glasses
- Exhaust fan and an open window

### Hardware
- EV-Peak 50W 6Amp Multi-chemistry Balance Charger & Discharger
- LiPo Battery (4C)
- LiPo Battery Voltage Tester (Low Voltage Buzzer Alarm)
- Buck converters (12V 2A; and 3.3V 0.6A )
- Blade fuse and DIY blade fuse mount
- Power Supply - 0 to 30V, 0 to 5A

- Raspberry Pi Pico
- Adafruit 9-DOF Orientation IMU Fusion Breakout - BNO085
- 2x JGA25-371 DC Gearmotors with Encoders
- L298D Motor driver
- Double-sided PCB
- Two-wheel balancer frame
- M2 and M3 bolts, nuts, and standoffs

- Micro USB to USB-C Cable
- Vernier Caliper

### Software
- VSCode
- MacOS Terminal (Zsh)
- Python (v3.13.2)
- CircuitPython (v10.0.3)
- PID Control Logic
- I2C Communication

### Fabrication
- CAD Modelling: Onshape 
- 3D Printing: Prusa i3 Mk3
- Miniware TS100 Mini Soldering Iron

## Installation/Usage
### Hardware
The creation of this robotic build spawned from online exploration and ultimately led to the purchase of a [two-wheel balancing kit](/docs/media/Two%20Wheel%20Balance%20Car%20Chassis%20with%20JGA25%20Motor%20Kit.pdf). The key components of this kit were the two JGA25-371 DC gearmotors with encoders, metal brackets, and acrylic boards that came with the kit. While a decent kit, the structural parts needed to be recreated anyway, and the motors and encoders likely could have been purchased independently with more forethought. Otherwise, the list of equipment can be found in the previous section.

The CAD files I created to refine the robot can be accessed and viewed in several ways. Two ways are listed below.
1. Via Onshape:
    - [Two-wheel balancer frame - Onshape Link](https://cad.onshape.com/documents/7dfefa6e2921ad39be2e3821/w/3f55fc51df92af93037a4f11/e/0a32c4429e5ca3bb6b5eb229?renderMode=0&uiState=69571ee25eaa326608b95544)
    - [4C LiPo Battery Power Distribution Layout - Onshape Link](https://cad.onshape.com/documents/d5dfdf18f97619740b08314e/w/11a66787474f13b8d7b80358/e/5259f1931b61786a48ea3b8f?renderMode=0&uiState=69571fcdb97c0f56a7a46870)

2. Via STEP files:
    - [Two-wheel balancer frame - STEP file](/docs/CAD/Two-Wheel%20Balancer%20Frame.step)
    - [4C LiPo Battery Power Distribution Layout - STEP file](/docs/CAD/LiPo%20PDB.step)

### Software
This project is written in Python with an emphasis on CircuitPython for the onboard Raspberry Pi Pico, which acts as the central processing unit.

The process for downloading CircuitPython and its dependencies onto the Raspberry Pi Pico can be followed as instructed on the [CircuitPython webpages](https://circuitpython.org/).

The files loaded onto the Raspberry Pi can be seen in the following image:
<img src="/docs/media/CircuitPython Files.png" alt="Screenshot of the files located on the Raspberry Pi Pico. boot_out.txt, code.py, and a lib folder with the dependencies adafruit_bno08x, adafruit_bus_device, and adafruit_register" width="200"/>

Connecting the Pico to my laptop via a USB cable then allows me to update the `code.py` file on the Pico through VSCode. I could then also monitor any serial outputs from the Pico on my laptop using `ls /dev/tty.*` to find the ID of the USB port connected to the Pico, and then monitor minicom terminal outputs using the terminal command `minicom -D /dev/tty.usbmodem101 -b 115200`, or whatever usbmodem ID is connected to the Pico.

## Why this project exists
I developed an interest in robotics years ago after seeing polished robotic systems on social media and how these tools can be applied to impact the physical world.

After successfully building an FPV quadcopter and validating that it could fly without catching on fire, I quickly realized the limitations of this type of system when it came to safety, given my inexperience flying racing-quality aerial machines. With the sudden realization and focus on acquiring a simpler platform to explore robotics safely, I rashly purchased a kit that came with two wheels, motors, encoders, and a few metal and plastic structural parts.

It was only recently that I had the time and mental energy to consistently reflect on designing and building a terrestrial robot using these purchased kit parts, scavenging from other hardware I had on hand, and buying any outstanding components. Through this, I was able to explore an interest in embedded systems and control theory.

Around this time, LLMs began exploding into mainstream adoption and utilization. I leveraged the capabilities these new tools enabled throughout building and testing this project to reach its MVP. From Gemini APIs, to self-hosted sharded Ollama models, and even an exploration into Cursor. I spent a good amount of time prompt "engineering", debugging hallucinations, and wrestling with frustrating LLM outputs. I am not fully convinced of the consistency or quality of LLM outputs in the absence of human intervention, nor the cost-effectiveness of deploying AI agents. At the very least, however, I was able to learn many new things throughout this foray into robotics supported by LLMs.