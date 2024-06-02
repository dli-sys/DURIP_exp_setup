**Edition 3, advised: 06-02-2024**
# 1 Robot arm safety

## 1.1 Robot arm check list
Make sure you check each of the settings before running any program
- [ ] TCP correctly defined
- [ ] Payload
	- [ ] Payload mass defined
	- [ ] Payload location defined
- [ ] Tool position defined
- [ ] Safety plane
	- [ ] WB_distal
	- [ ] WB_ground
- [ ] When you done with the previous settings, make sure you manually test the safety plane using the free drive button. If things properly defined, when you move the tip of the robot, it will then generate a warning saying you are close to the safety plane.

## 1.2 Safety settings
1. Unlock the key-rest switch and all other emergency stop
2. Go to **Installation** to confirm **mounting** angle and direction
3. Adjust **TCP** according to the tool you mounted to the robot arm, in most case, the TCP should be at the end of your tool, for example, if your extension arm is 200 mm long, TCP should be set as X: 0, Y: 0 and Z:200, Save your payload with your initial in the front, for example, if you are Dr. Good Student, your TCP should be called "GS_tool_date". Press "**Set now**" to save your change.
4. Go to **Payload** to **Measure** the payload and center of gravity. This step is really ***important*** as a incorrect payload will cause the robot arm move itself can leads to unpredictable damage to the load cell, robot arm and workbench, making it a 60,000 Dollar mistake.  Save your payload with your initial in the front, for example, if you are a good student, your payload should be called "GS_exp_date".  Press "**Set now**" to save your change. **When you install any new payload, press the e-stop to lock the robot joint.** This step makes sure the robot will not move during the installation. Then **Set Now** the new payload to match what you have, then unlock the robot joint.
5. Inspect the safety plane (WB stand for workbench, Labelled on the bench already)
	1. WB_prox -- optional
	2. WB_distal -- Must turn on
	3. WB_ground -- Must turned on for experiments on the table
	4. WB_right -- optional
6. Important step! Adjust your tool size in **Installation** -- **Tool position**. You do not have to make a perfect match but try to make it as close as possible. The radius should be the maximize radius of your tool and location should the distal point of your virtual tool. For example, If you install the following: Millbar 150 extension arm (150 mm height, $h_1$, 35 mm radius, $r_1$), ATI-delta load cell (90 mm height, $h_2$, radius 90 mm, $r_2$)  and a custom attachment (230 mm height, $h_3$, max radius 90 mm, $r_3$). The tool direction should be calculated as following (Consult dongting if you are not sure):
	1. Radius: $max{ \left\{ r_1, r_2, r_3 \right\}}$
	2. location: $h_1 + h_2 + h_3 - max{ \left\{ r_1, r_2, r_3 \right\}}$
7. When you create a new program for your experiments, use "exp" folder in the main directory
8. When you are done:
	1. home the robot arm using "safe_home.urp" in the main directory. During moving, make sure the cable is not fighting with each other.
	2. Power off the load cell and make sure its in a safe configuration
	3. clean off the table --  clean means, make everything exactly like the original condition or even better
	4. push the key-reset e-stop to prevent unauthorized access

# 2 Python environment settings
**GitHub repo hosting the codes: https://github.com/dli-sys/DURIP_exp_setup**. Ask dongting to add you to the repo. Check out the "readme.md" and following the instruction
# 3 Load cell safety

1. Connect the load cell to the DAQ box
2. Power on the load cell by plugging in the ethernet cable to the POE switch, no additional power supply is required
3. Go to the load cell configuration parge, 192.168.0.121
	1. Confirm if the system is healthy
	2. Confirm the following settings
		1. Sampling rate
		2. Transformation matrix
		3. Low pass filter
