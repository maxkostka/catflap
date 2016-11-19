# catflap
The catflap for detecting prey in the cats snout.
Based on a raspberry pi 2 with the raspicam and the catflap sureflap.
The raspberry runs ubuntu with ros indigo, python 2.x opencv.
The system has 4 sensors, two ir sensors to detect the presence of a cat in the catflap tunnel, a sensor to check, whether the status of the flap and a camera to take pictures of the cats profile.

I further think that the original idea was not created by the three youngsters or at least they were not the only ones.
I was further inspired by joakim soderberg and the catcierge https://joakimsoderberg.github.io/catcierge/ (Nice work Joakim!) and the flow control project which can no longer be found on the web (origial url: http://www.quantumpicture.com/Flo_Control/flo_control.htm) but we can find some remaints of its impact on others http://www.openscience.org/blog/?p=109.

At least four years after the flo control project, three germans participated in a contest called 'jugend forscht' http://www.neumarktonline.de/art.php?newsid=59507). The price was patent application http://www.freepatentsonline.com/DE202009002677.html which is currently owned by the Feldberglicht GmbH. They are still developing a commercial application - check it out http://www.mouse-lock.de/. But don't expect a working catflap before a working fusion reactor ;)

usage so far
> roslaunch catflap_training.launch <
to start the ros nodes to aquire images, when a cat enters passes the flap

i added the startup.sh via
> sudo crontab -e <
just add a line like 
> @reboot /home/max/projects/catflap/ros_catkin_ws/src/startup.sh >>/home/max/projects/catflap/log_cronrun 2>&1 <

to start it at at reboot
