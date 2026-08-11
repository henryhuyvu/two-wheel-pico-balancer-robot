# Two-Wheel Balancer Robot

The following is a scattering of lessons learned from my first attempt at building a "robot" in a resource constrained build.


## Overview
Robotics is cool but costly. For the most part, I made use of the electronic components which I had aggregated over time, thus, the final state of the electromechanical assembly took advantage of these resources.

This project used an L298N breakout board motor driver which was intended as a placeholder until delivery of a more efficient TB6612 motor Driver, as the TB6612 has improved power distribution with far fewer losses compared to the L298N and L293D ICs. In my crude construction of a power distribution board, replacing the L293D with a TB6612 felt like too much of a hassle, and so I opted to stick with the parts I had put in place.

## An attempt at C programming for embedded systems
Wanting to learn a lower level programming language, I had initially attempted to dive into C programming for processing data from a BNO085 IMU using a Raspberry Pi Pico.

Having never used C but reading online about its speed and proliferation in embedded systems, I wanted to get SPI communications working. Through this approach, I learned about concepts such as TVL, Tag-Length-Value (aka Type-Length-Value), bit banging, and some binary and hexadecimal numbering systems. It was interesting peering into finer detail of how data is communicated between the BNO085 IMU and Pico, but ultimately I abandoned this route as I wanted to iterate faster, rather than trying to aim for performance gains in a system I had not yet even proven to have worked. This meant switching away from C and SPI, to Python and I2C.

In any case, the brief attempt at C programming began by simply blinking the built-in LED on the Pico in C, and got as far as capturing some HEX output from the IMU as can be seen below:

```text
User input detected. Beginning main()
BNO085 SPI communication initialization complete.
BNO085 reset complete.
BNO085 interrupt (INT) asserted (active low). Ready for communication!
Attempting to read initial SHTP advertisement packet...
Raw Header Bytes: 0x14 0x01 0x00 0x00
SHTP Packet received: Channel=00, SeqNum=00, PayloadLen=276
Received 280 bytes (Advertisement Packet):
0x14 0x01 0x00 0x00 0x00 0x0A 0x00 0x00 0x80 0x00 0x82 0x00 0x00 0x00
0x03 0x18 0x97 0x18 0x17 0x18 0x00 0x01 0x01 0x00 0x00 0x81 0x81 0x7F
0x01 0x00 0x00 0x82 0x81 0x7F 0xBF 0x84 0x02 0xA9 0xA4 0x2A 0x28 0x00
0x80 0x04 0x84 0x31 0xB7 0xB7 0x3A 0x39 0x37 0xB6 0x00 0x00 0x82 0x00
0x00 0x04 0x05 0xB2 0xBC 0x32 0xB1 0xBA 0xBA 0x30 0xB1 0x36 0x32 0x80
0x80 0x84 0x83 0xB2 0x32 0xBB 0x34 0xB1 0xB2 0x80 0x00 0x82 0x01 0x00
0x04 0x05 0x39 0xB2 0xB7 0x39 0xB7 0xB9 0x34 0x3A 0xB1 0x00 0x03 0x00
0x84 0x31 0xB7 0xB7 0x3A 0x39 0x37 0xB6 0x00 0x03 0x00 0x81 0x84 0x86
0x38 0x3A 0xBA 0x27 0x37 0xB9 0x36 0xB0 0xB6 0x00 0x03 0x80 0x82 0x04
0xB7 0x38 0x3A 0xBA 0x2B 0xB0 0xB5 0xB2 0x80 0x03 0x00 0x82 0x84 0x86
0x38 0x3A 0xBA 0x23 0xBC 0xB9 0x37 0xA9 0x3B 0x00 0x40 0x03 0x18 0x97
0x18 0x00 0x40 0xB2 0x7C 0x08 0x7A 0x82 0x79 0x88 0x78 0x88 0x7D 0x82
0xFE 0x08 0xF7 0x81 0x00 0x85 0x01 0x05 0x01 0x85 0x02 0x05 0x02 0x87
0x03 0x88 0x04 0x06 0x04 0x87 0x05 0x04 0x05 0x84 0x06 0x03 0x06 0x83 
0x07 0x88 0x08 0x02 0x88 0x86 0x09 0x03 0x09 0x83 0x0A 0x08 0x0A 0x88
0x0B 0x80 0x0C 0x04 0x0C 0x83 0x0D 0x00 0x0D 0x80 0x0E 0x03 0x0E 0x80
0x0F 0x80 0x10 0x00 0x10 0x80 0x11 0x00 0x11 0x80 0x12 0x00 0x12 0x80
0x13 0x80 0x14 0x07 0x14 0x86 0x15 0x07
Raw advertisement data captured.
Requesting Feature ID 0x01 with interval 60000 us...

Starting main loop to read sensor data...
Raw Header Bytes: 0x14 0x00 0x02 0x00
SHTP Packet received: Channel=02, SeqNum=00, PayloadLen=20
(A) Received 0x02 channel packet, for Report ID: 0x14
Raw Header Bytes: 0x05 0x00 0x01 0x00
SHTP Packet received: Channel=01, SeqNum=00, PayloadLen=5
(A) Received 0x01 channel packet, for Report ID: 0x05

0x14 0x01 0x00 0x00 0x00 0x02 0x80 0x00 0x20 0x00 0x20 0x80 0x00 0x00
0x00 0xC6 0x25 0xC6 0x05 0xC6 0x00 0x00 0x40 0x40 0x00 0x20 0x60 0x5F
0x80 0x40 0x00 0x20 0xA0 0x5F 0xEF 0xE1 0x00 0xAA 0x69 0x0A 0x8A 0x00
0x20 0x01 0x21 0x0C 0x6D 0xED 0xCE 0x8E 0x4D 0xED 0x80 0x00 0x20 0x80
0x00 0x01 0x01 0x6C 0xAF 0x0C 0xAC 0x6E 0xAE 0x8C 0x2C 0x4D 0x8C 0xA0
0x20 0x21 0x20 0xEC 0x8C 0xAE 0xCD 0x2C 0x6C 0xA0 0x00 0x20 0x80 0x40
0x01 0x01 0x4E 0x6C 0xAD 0xCE 0x6D 0xEE 0x4D 0x0E 0xAC 0x40 0x00 0xC0
0x21 0x0C 0x6D 0xED 0xCE 0x8E 0x4D 0xED 0x80 0x00 0xC0 0x20 0x61 0x21
0xCE 0x0E 0xAE 0x89 0xCD 0xEE 0x4D 0xAC 0x2D 0x80 0x00 0xE0 0x20 0x81
0x2D 0xCE 0x0E 0xAE 0x8A 0xEC 0x2D 0x6C 0xA0 0x00 0xC0 0x20 0xA1 0x21
0xCE 0x0E 0xAE 0x88 0xEF 0x2E 0x4D 0xEA 0x4E 0xC0 0x10 0x00 0xC6 0x25
0xC6 0x00 0x10 0x2C 0x9F 0x02 0x1E 0xA0 0x9E 0x62 0x1E 0x22 0x1F 0x60
0xBF 0x82 0x3D 0xE0 0x40 0x21 0x40 0x41 0x40 0x61 0x40 0x81 0x40 0xA1
0x40 0xE2 0x01 0x01 0x81 0x21 0xC1 0x41 0x01 0x61 0x01 0x80 0xC1 0xA0
0xC1 0xE2 0x02 0x00 0xA2 0x21 0x82 0x40 0xC2 0x60 0xC2 0x82 0x02 0xA2
0x02 0xE0 0x03 0x01 0x03 0x20 0xC3 0x40 0x03 0x60 0x03 0x80 0xC3 0xA0
0x03 0xE0 0x04 0x00 0x04 0x20 0x04 0x40 0x04 0x60 0x04 0x80 0x04 0xA0
0x04 0xE0 0x05 0x01 0xC5 0x21 0x85 0x41
```

Getting any sort of response from the SPI connection was neat, but interpreting the results and verifying the IMU firmware with it's unique SH-2 Sensor Hub Transport Protocol (SHTP) communication was more complex that I had anticipated. With essentially no C, SPI, or embedded experience, I tried to vibe code my way into a solution. I like to think my writing abilities are quite decent, and with plenty of detailed prompting in pursuit of obtaining a working output, LLM responses mostly generated a lot of trash code that ended up being primarily useless. Starting from VSCode's Pico SDK extension was significantly more reliable and usable.

As I read further into the IMU datasheets and documentation, I accepted that coordinating the IMU's SHTP protocol into C code (with proper timing of GPIO signals + order of operations for SPI communication), in addition to managing the SH-2 software methodologies was too much effort for what I was aiming to achieve.

With more unknown and tangential topics coming to light, I decided to transition to the I2C communication protocol as it was much easier to implement, and the data transmission speeds indicated in the IMU datasheets specified reporting intervals at 100 Kbps and 400 Kbps (depending of which IMU report is called), which seemed to be sufficient for balancing response times.

## Voltage regulation and component layout
Knowing that I would make use of a 4C LiPo battery I had from a prior quadcopter project, I planned out a simple circuit for the power distribution such that the LiPo battery power would be sent through a fuse, and then divided into two different power rails regulated by a 12V and 3.3V voltage regulator. The 12V regulator would strictly power the L298N motor driver, and the 3.3V regulator for the IMU, Pico, and any additional peripherals.

I drew rough outlines of the power components onto a sheet of paper and cut them out to see how the components would be laid out with respect to each other and get a better sense for the positioning and size of the final product.

The layered structure that I had decided on was roughly as follows:
- Bottom: Power distribution board and motor controller.
- Middle: LiPo Battery.
- Top: Raspberry Pi Pico, IMU, LiPo battery monitor.

## Space utilization and structural parts
I had been using parts that originally came with the two-wheel chassis kit, and so my design revolved around this. I decided to swap these acrylic structural pieces with my own 3D printed parts to more consistently position screw holes and mounting points.

In retrospect, had I decided to design my own robotic frame from the start, I likely would have been better able to optimize space and incorporate features that would lead to easier assembly and disassembly.

With fear of sudden and accidental combustion of the LiPo battery, my main concern structurally was ensuring the battery housing was enclosed to reduce any risk of punctures and impacts that would lead to a battery fire.

## I2C and poor breadboard contact
Transitioning to I2C and testing Python scripts to read and display IMU data was very simple. On successful testing of the IMU and motor controller, the next step was to combine the two scripts into a cohesive loop with PID controls.

Continued testing to verify that I can regularly capture and report IMU data ended up resulting in inconsistent errors. 

I initially believed that there have been errors resulting from issues with the BNO085 IMU itself (through reading forum posts about this IMU, and failure to debug the issue using LLMs). Returning to this problem in a couple days with nothing changed gradually revealed to me that it was likely a wiring issue. The single IMU breakout board LED had previously shone with varying intensity. Varying the amount of tension in the power wires connected to the IMU on the breadboard was directly correlated to the presence of software errors, and so the component was getting insufficient power to properly function.

This poor wiring contact was resolved by transitioning the components to a PCB and hand soldering each contact so there would be little "play" in the contact area of the connections.

## Final changes and moving on
With the major hardware issues resolved one way or another, my focus could shift to software. I used LLMs quite significantly as this technology was quite intriguing at this point in time for me, but this was not without it's issues. There was plenty of refactoring needed, and many frustrations dealing with the lack of logical reasoning that LLM results produced. Essentially every bit of code produced by the various LLMs had to be reviewed, scrutinized, and reworked so that they would function in the context of my robot design.

Something like the orientation of the IMU placed on the robot where the "up" direction corresponded to the IMUs negative y-axis conflicted greatly with the data these LLMs were trained on. One shot LLM solutions just were not possible, even when given detailed prompt instructions.

Not wanting to mess with any IMU firmware flashing to address this orientation issue, I modified my scripts to account for this axis remapping. I further introduced a tilt angle cut off so that beyond a certain tilt angle, the motors would cut off to avoid motor runoff when the robot is not upright.

With the IMU accounted for, the PID loop coefficients for the P, I, and D parameters were tuned via trial and error. With crude test design and plenty of adjustments, the PID coefficients were sufficient to prevent the robot from falling over, but not without forward or backward drift. Using only tilt meant that there is no accountability for acceleration or displacement of the robot.

Next steps for this project would involve incorporating the motor encoders to track rotation of the wheels and record how far the robot has moved so as to enable the possibility of balancing on the spot, and minimizing vehicle drift. The things I've learned through this project have helped me to understand that my interests lay more toward instrumentation and data processing than control problems, and so this robot will be scraped for parts.

# A comment on LLM as of 2026
While these tools can be very powerful, they should not stand in place of critical thinking and fact checking. Understanding how these tools work and what they can do is quite helpful, but hoping that natural language can be used to generate an exact solution to your unique problems is not the right approach, no matter how much you wish it so. 

I have found the best use case for me is to use these tools as an extension to what search engines can do. Helping you to identify possible solutions to problems. The pattern recognition these tools provide is really something else, but to think that they can do the work for you is misplaced. An accelerator for sure, but not a substitute for actually engaging your brain, unfortunately.
