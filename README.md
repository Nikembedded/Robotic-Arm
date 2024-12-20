# Robotic-Arm
Robotic Arm Pick and Place Controller Project

Designed and assembled by Nikhil Premjani

**1. Project Overview**
The project involved designing a robotic arm that can pick and place objects from one location to another with a precision error of ±5cm. A custom PCB was designed in Altium, and both SMT and through-hole components were assembled and soldered. A code was also developed to control the robotic arm's movement.

**2. Objective and Real-World Application**
The primary objective of this project was to create an automated robotic arm capable of performing pick-and-place operations in industrial settings. Such arms are critical in automated manufacturing processes, reducing human intervention and enhancing precision.

**3. Tools Used**
**Software Tools:**
- Code Composer for microcontroller programming
- Visual Studio for MFC programming
- Altium Designer for PCB circuit design
**Hardware and Testing Tools:**
- Oscilloscope
- Spectrum Analyzer
- Digital Multimeter
- Soldering Station
- 10x Zoom Eye Loop for component soldering


**4. PCB Design (Altium)**
The PCB for the robotic arm's controller was designed using Altium Designer. It involved the creation of schematics, component placement, and routing to ensure signal integrity and minimize noise. Screenshots of the design (top and bottom layers) would be attached here.

 ![image](https://github.com/user-attachments/assets/26e4dd3b-b093-4d4b-8b9d-104a0d9f9fff)

 ![image](https://github.com/user-attachments/assets/fb2566de-4c8a-459a-8401-cd5d48225878)

 ![image](https://github.com/user-attachments/assets/b3e1e7ce-76f6-4046-a0d4-60d48020aaad)



**5. Code Overview**
The code for the robotic arm involved creating several tasks such as external I/O heartbeat, servo control, and main loop logic. The heartbeat task ensures that the system is functioning properly, while the servo control task manages the movement of the robotic arm. Below is a sample snippet:

void Heartbeat_Task() {
   if (system_ok) {
      heartbeat_led_on();
   }
}

Additional code for tasks related to coordinate calculation, servo control, and communication between the components were developed using FreeRTOS. Full source code is available upon request.


**6. Assembly and Debugging**
The assembly process involved soldering both SMT and through-hole components in stages. First, the power supply components were assembled and tested. Then, communication and microcontroller circuits were soldered and debugged. After ensuring that power and communication were working, relays and connectors were soldered. The PCB was then mounted in an industry-standard case and attached to an industrial rail for final testing.

**7. Results and Conclusion**
The robotic arm performed as expected, with a final error of 1.5 cm during pick-and-place operations. The coordinate system was controlled through an MFC interface where the user inputs start and end points and the rest of the calculations were handled by the system. The math and physics involved in calculating coordinates were challenging but ultimately rewarding when the system worked successfully. Photos of final outcome are attached here.

![image](https://github.com/user-attachments/assets/29740b98-5504-4d2b-896c-dc216699f541)

![image](https://github.com/user-attachments/assets/f709549c-d585-4456-ac89-c2273c1476cc)

 
**8. Appendix**
Additional files such as the Gerber files and the BOMs are available upon request.

