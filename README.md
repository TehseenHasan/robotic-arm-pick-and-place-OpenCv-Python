These are the files/code of my pick and place robotic arm using OpenCV-Python.
These code files are not so much organized as I did not find spare time to  clean the code or write a good documentation/tutorial.
I will try my best to wrate a complete guide and documentation about that complete system.

**Steps:**
1. You have to install Python 3
2. Use this Robotic arm: https://github.com/20sffactory/community_robot_arm 
3. install OpenCv on your python environment
3. use a Digital camera
4. Mount the camera above the robotic arm at 90 degree position in such a way that it can capture the working area of the robotic arm clearly.
5. use a (~50mm wide, please read the code and comments in the code for details) blue rectangle cardboard as a place holder of the robotic arm and place it where you will place the robotic arm and run the robot_position_estimation.py code
6. This code will find the coordinates of the blue cardboard piece and considered it mid point as robotic arm origin.
7. place the robotic arm above that bliue cardboard precisely.
8. Now you can use the main code file. you have to calibrate the parameters in the code according to your environment and camera setup.
9. There are a lot of things to learn first before understanding these codes
10. Please watch tutorial videos from this great robotics cource:  http://www.robogrok.com/index.html
11. Watach atleast these 3 videos to understand my codes: ![Contents](https://user-images.githubusercontent.com/25352528/116832263-a7fdb700-abcd-11eb-84db-5413ca39865c.jpg)
