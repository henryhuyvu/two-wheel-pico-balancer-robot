# Two-Wheel Balancer Robot

Welcome. This is a bit of a longer rambling on the trajectory I had throughout this project. By no means is this all encompassing of the challenges and solutions I came across, but it is at least a quick glimpse.

My initial vision for this thing was more grandiose than the final result that I arrived at. While a tad disappointing at first, I feel this was the necessary and ideal outcome for me.

I have had some prior hands-on laboratory experience using scientific equipment, but no end-to-end familiarity with robotics. My understanding of hardware limitations and caveats of electronics evolved as I made my way through this project.

The trajectory of this project was not thoroughly pre-meditated. Of course online resources were explored, but for the most part, I tackled problems as they arose.

## Purpose
My initial goal was to create a robust platform for exploring sensors and computer vision. I could have likely achieved this goal simply by choosing a platform that did not have to worry about balancing, but at the time that felt too simple. 

It was not until I was nearing the final stages of the MVP that I realized I was too ambitious given the limitations of the hardware I had available to me. As such, I opted to lobotomize my desire for this device to perform complex automated tasks, and opted to be satisfied if this thing can simply balance itself.

## Project Investment
Robotics is expensive. It takes some form of spending or creativity to create a physical object. I had purchased a two-wheel balancer kit in the heat of passion and had not planned too in depth to ensure all my electronics would be easy to work with. Sometimes you have to just dive in and adjust accordingly. Given my inexperience with robotics and my desire to get started, this is how I went about beginning.

## Component Validation
I purchased practically no backup equipment, so some added caution was taken to try and mitigate any potential short circuiting or catastrophic failure of parts.

### Underpowered Motor Driver IC
An unfortunate yet educational outcome of little planning was purchasing underpowered parts.

I had gone to a local hardware store and purchased an L293D IC. This IC was intended to act as the motor driver for the two DC motors, however the voltage output of this IC was well below the capabilities of the motors. Testing this IC using a desktop power supply and breadboard wiring over the period of about 5 minutes resulted in the IC becoming uncomfortably hot to touch. This heat issue led me down a tangential path of considering the use of a Peltier cooler to temperature regulate the IC in addition to using cooling fins and thermal paste. While a neat idea, properly reviewing the IC and motor documentation revealed the underpowered nature of the IC and led to scrapping the idea in exchange for swapping to a more powerful motor driver, the L298N. 

I had purchased a TB6612 Motor Driver due to its drastic improvement in efficiently compared to the L293N and L298N motor drivers thanks to the insignificant voltage drop when compared to the L29XX motors, however there were significant delays in delivery and thus a L298N breakout board was purchased in the meantime to get a proof of concept up and running.

When I was still figuring out this motor driver issue, I was simultaneously investigating how I could best implement the IMU. Being that the robot will need to iteratively maintain and act on telemetry data, I figured real-time processing would be the best option, and thus, I did a brief dive into C programming for the Raspberry Pi Pico.

## C Programming for Embedded Systems
Having never used C but reading online about its speed, I wanted to get SPI communications working. Ultimately, I abandoned this route as I needed to start from scratch to understand what I was doing. Through this approach, however, I learned about concepts such as TVL, Tag-Length-Value (aka Type-Length-Value), bit banging, and some binary and hexadecimal numbering systems.

I had used the VSCode Pico extension and began by simply blinking the built-in LED on the Pico. I had gotten as far as getting some sort of HEX output from the C code, as can be seen below:

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

Getting the SPI connection to work was okay. Getting C code to align with the IMU firmware and its unique SH-2 firmware and protocols was more challenging than I had hoped for. With essentially no C or SPI experience, vibe coding something to work produced a lot of trash code that ended up being mostly useless. Using the functionality of VSCode's Pico SDK extension was significantly more reliable.

The more I dug into the IMU datasheets and documentation, the more I felt that transferring the IMU's SHTP protocol into C code with proper timing of the Picos GPIO signals, orders of operation for SPI communication, in addition to managing the SH-2 software methodologies was too much effort for the payoff (which was determined after a few days of frustration and mulling).

With what felt like many more unknown and tangential topics I needed to learn to get the embedded C to work, I began to lose patience as my inexperience in C and desire for a tangible working product led me to finally transition into near-real-time processing that Python would have provided. I opted then to switch to I2C communications as it was much easier to implement, and the data transmission speeds suggested by datasheets for the IMU indicated 100 kbps to 400 kbps, which seemed to be sufficient for balancing.

## Power Distribution
Eventually I would need to devise some solution for providing power to the full system safely. Knowing that I would make use of the 4C LiPo batteries I had from a prior quadcopter project, I planned out a schematic for the power distribution such that the LiPo battery power would be sent through a fuse, and then divided into two different power rails regulated by a 12V and 3.3V voltage regulator. The 12V regulator would be strictly for the L298N motor driver, and the 3.3V regulator for the IMU, Pico, and any additional peripherals.

To figure out the positioning of the various electrical components for this robot, I drew rough outlines of the power components onto a sheet of paper and cut them out to see how the components would be laid out with respect to each other and get a better sense for the positioning and size of the final product.

The structure that I had decided on was roughly as follows:
- Bottom Layer: Power distribution boards, motor controller, wiring, and shielded battery wires (did not end up shielding them).
- Middle Layer: LiPo Battery
- Top Layer: CPU (Raspberry Pi Pico), IMU

## Robotic Frame
Note that around this time, I had been using the acrylic structural parts that came with the two-wheel chassis kit, and so my planning revolved around this. It was only later that I decided to swap these acrylic parts with my own 3D printed parts to better accomodate for space and assembly consistency. 

Space optimization, symmetry, and easy of assembly were part of my main focus for designing the CAD models of the robots' structure. Ensuring the battery compartment was shielded was also a priority as I did not want to introduce the risk of the LiPo battery potentially being punctured and causing a battery fire.

## Software 

## I2C and Breadboard Wiring
After transitioning to I2C and testing simple Python scripts to read and display IMU data, I realized that I made the right choice. With the initial testing of the IMU and motor controller performing admirably. The final step in my mind was to combine the two scripts into one cohesive loop in combination with PID controls.

Upon continued testing of preexisting code to ensure I can capture and report IMU data, it seemed that some reporting features of the IMU were producing errors. What I thought may have been errors propagating from issues with the I2C communications of the BNO085 through reading forum postings about this IMU, and failure to debug the issue using LLMs, I was ready to give up on the project until a later time. In the following days of planning to switch to different hardware, I powered on the IMU again, and this time I sought to ensure the single green LED on the device was shining as bright as possible (I had known that there was a correlation between tension in breadboard wiring and the IMUs onboard LED brightness).

There had been evidence of a variable level of light intensity from the IMUs onboard LED - which I began to believe was tied to the power in the breakout board. In the evening when I attempted again to see what IMU reports were available to me, after fidgeting with the breadboard wiring and getting the light to be as bright as possible, it suddenly fixed any reporting errors that occured. 

The poor contact points in the IMU power and signal wiring needed to be reinforced, which led to me building PCBs that would have soldered wires and electrical contacts to inevitably create much more consistent and reliable connections for the power and signal wiring between the IMU, Pico, and other electrical components.


### Control Loop
With the majority of major hardware issues resolved one way or another, the final step was to combine everything via software. The balancing problem is fairly well understood and my intention was to use a PID loop as others before me have done. 

Simply put, I used LLMs quite significantly to arrive at the cleaned up, class heavy version of the MVP script. There was plenty of rework that needed to be done, and reasonably just as many frustrations dealing with the lack of reasoning that LLMs provide. Essentially every bit of code that I had used that was produced by LLMs had to be reviewed, scrutinized, and finally reworked so that they would apply in my case.

Due to the mechanical orientation of the IMU on the robot, the "up" direction corresponded to the IMUs negative y-axis. Not wanting to mess with any IMU firmware flashing, I restructured my scripts to account for this axis remapping. I further introduced a tilt angle cut off so that beyond a certain tilt angle, the motors would cut off to avoid any potential motor runoff. 

After the IMU was properly accounted for, the PID loop coefficients for the P, I, and D parameters were tuned via trial and error. In the latest MVP, the robot is now able to prevent itself from falling, but not without significant forward or backward drift. The MVP uses only tilt data, and the limitations of this mean that there is no accountability for acceleration or displacement of the robot.

Next steps for this project would involve incorporating the motor encoders to track rotation of the wheels and record keeping of how far the robot as moved so as to enable the possibility of balancing on the spot, and minimizing vehicle drift.

# A diversion (rant) into LLMs
Over the past year, it has been impossible to escape word of LLMs, ChatGPT, OpenAI, Gemini. Any news of LLMs and improvements is very public in North America.

Truly a powerful tool when implemented properly, I also did my own exploration into this world. Starting with ChatGPT Plus, before the start of that month long subscription I knew I wanted to use an API so I could regulate my use and payment of such a tool. By the end of my ChatGPT Plus subscription I had set up my own self-hosted Docker container running OpenWebUI so that I could use LLM APIs from Gemini, Anthropic, OpenAI, Mistral, and local Ollama models.

With this tool at my fingertips, I used them quite extensively for many things, this robotics project included. Being exposed to LLM outputs for many months, I have come to better understand the high-level theory behind LLMs, and more importantly their limitations and the frustrations that can arise from them. Hallucinations, poor context utilization, the requirement for extremely detailed prompting. The idea of LLM agents is great, but as an individual knowing the limitations of such tools, the thought of deploying agents on my own dime does not feel worthwhile, even if it would be glorious to witness.

While a useful tool. Oh my goodness. The level of trash code and outputs that can be produced by these clankers is unfortunate. Cursor is beautiful during its execution, but the final results need to be vetted else you subject yourself to so many attack vectors or output errors. Installing potentially malicious or even non-existent software libraries is a very real concern. I spend a significant amount of time simply vetting the results produced to ensure reliability and safety of such black-box system outputs.

The code quality produced by LLMs for this robotics project was one instance of significant discovery of the frustrations of using these tools. You can practically turn off your brain, produce lengthy "code" and then run into many errors trying to get it to simply run. The lack of reasoning does not help when the code you are trying to update has nuances such as the fact that your motor rotation directions or IMU orientations are unique to your system, and these LLMs "forget" that this is the case and they rework the entirety of your code base, changing a `<` to `>` here and there, overriding your 90 degree coordinate change because the training data is primarily normalized to 0. 

Many small possibly unnoticed changes that compound into a giant heap of trash code that because both impressive and frustrating to work with. Small code bases and changes are reasonable, but expecting to build out complex systems in a safe, reliable, and consistent manner is something that will likely require a significant overhaul in how LLMs function, or a non-trivial budget to enable agentic programming. 

LLMs and AGI? I think not in its current form. There is no reasoning. A glorified word prediction tool at best. Can it be useful? Sure, but not without critical thinking and intervention from a human perspective. Critical thinking which can be impaired easily due to the perceived correctness and confidence of outputs from such tools.