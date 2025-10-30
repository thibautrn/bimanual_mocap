In this github there is the code for the Watch to connect and also the code for the ros2 sim.

Wear_mocap :
The arm-pose-estimation is the code that makes the calculation from the watch. I can't remember if I actually changed something from the original code but I'm giving you the one I'm running so we're sure its up to date. if any problem, check the fabian's main project : https://github.com/wearable-motion-capture
then take his arm-pose-estimation code. 

The code that connects to the watch is watch_phone_pocket. the watch_phone_pocket_vm is the one I'm using with my vm but you probably don't need it.
Watch_phone_pocket.py is the one used. I'm importing the files with "
script_dir = os.path.dirname(os.path.realpath(__file__))
src_path = os.path.join(script_dir, "arm-pose-estimation","src")
sys.path.append(src_path)"
but you can change it to make it working with Linux. The original installs wearmocape with pip but I didn't do it so I could change the files myself. (not sure if I changed anything but I had to while testing. TLDR : don't care about that, it should work as is).

You need to be on either of the lab's wifi and get their IP. The computer and the phone have to be on the same wifi, so remember to also change the wifi of the phone, don't care about the watch. you can then change the ip in the middle of the phone's screen on the ap. 
The apps are already downloaded on each phones and watch so you just need to open them. 
On the watch, you have to select the second option, which is "pocket" . then you have to select the first button for the "Stream IMU". Always try to have the arm perpendicular to your shoulder, in a way that the elbow is perpendicular and the watch is at the same height of your shoulder, for better results. The phone can be in any pocket I believe.
Everytime the phone closes or even goes in horizontal mode or vertical, you'll lose the connection with the watch so you'll need to toggle "recording" again. Remember that you sometimes need to recalibrate your watch like I often do at the lab, just to be sure, untoggle and retoggle. IF PROBLEMS WITH THE APPS : you can clone the Android studio and connect the watch and phone to android studio and rebuild the 2 apps, don't believe it'll be needed tho. but https://github.com/wearable-motion-capture/sensor-stream-apps

On the computer side, just need to run the code, with the right ip as a variable, and you're supposed to be good to go. 
Checkmarks : 
-Computer to private WIFI
-Phone to private WIFI
-Correct IP settings in the phone's app.
-Open the watch/ select the second button/ toggle the start recording.

Now onto the robot side:
I'll give you the full repository I use just in case I changed any other file, that made my code run, but I'll tell you just the main ones needed. 

the main files :
in bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/:
- move_right_arm_fast is the basic 40 hz position input.
- move_right_arm_fast_hz same but with 2 (or 3) more points in between each of the recorded points, which makes 120 (or 160) hz input. It might be the better up to date code.
- move_right_arm_fast_wrist : is my newest code with the wrist movement. However as talked with Heni and Sachin, it comes with some problems. The wrist works well tho. The code is made with the first fast code and not fast_hz. If I have time on thursday night I'll do the same but for fast_hz_wrist.

All the codes normally have the collision prevention, however the camera on the head is not into the files so there's a possibility that we can hit it if someones is not careful enough. You either need to get rid of it (just to be sure) or quickly model it into the urdf. I almost hit it when playing with it on wednesday, sachin was there too.

don't forget to update the CMAKELIST. 

other files that probably have changed ? :
bimanual_urdf : I'm using a full .urdf, so maybe change the pin settings with your urdf.xacro or use the same as me. remember to change the URDF_path in my codes. 
arm_ros2_controller.urdf.xacro : changed back the gains to 10. Maybe can try to go up to 15 but 20 is a bit too shaky. 
manipulator_h.xacro : efforts at 250 and velocity at 6.5 I haven't changed much so you could also check what is the best for the presentation if you have time.
setup.sh : export PYTHONPATH=/home/thibaut/miniconda3/lib/python3.12/site-packages:$PYTHONPATH I have that line but I think you already have your own version of it.

for the gazebo running I don't think I changed anything, just run it as usual, I'm still following the lines I wrote in the google doc. 

When running, it'll wait to receive the data from udp on port 50003. once its done it'll do as usual. The first input is always a bit fast depending on what the robot position is at first. So maybe to not scare people, make them start with the arm that is on their left side, to ressemble an arm straight to the left. Nothing will break if you don't but its just so the small acceleration doesn't scare anyone. 

Remember that if we don't plan in using any of the wrist joints, we could be using the left arm of the robot. if people want to see the left arm moving with the watch on the left arm. If so I have this code:
move_left_arm_basic : which isn't up to date, however, you can change a few things in any of the fast/fast_hz code. to do so :
-change any rightarm/right_arm for the joints or group or link into left
- SHOULDER_ANCHOR = np.array([0.045, -0.2925, 1.526], dtype=float) to SHOULDER_ANCHOR = np.array([0.045, 0.2925, 1.526], dtype=float). just inverse the Y of the anchor ( from right to left)
- in scale_watch_to_right_robot() get rid of : Sh[1] = -Sh[1]; El[1] = -El[1]; Wr[1] = -Wr[1]  


