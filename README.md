# Autonomous Surveillance and Security Drone

For my senior thesis for my degree of Bachelor of Science (B.S.) in Computer Science from the University of Illinois at Urbana-Champaign, I designed and implemented an autonomous unmanned aerial vehicle which can patrol an area and identify humans.

You can view my thesis here: http://njlochner.com/thesis.html

My thesis is also available on this github repository: https://github.com/njlochner/secdrone/blob/master/thesis.pdf

### Overview

The drone includes a Raspberry Pi computer which controls it. The computer runs an on-board web server and a secure WiFi access-point running a controller web-application built with Python.

The web application uses the Google Maps API and allows for the drone to be sent to a GPS way-point, specify the speed and altitude, and specify a circuit for the drone to patrol.

The drone has a camera, and sends images to a computer connected to the drone’s WiFi network. This computer continuously processes images with OpenCV in Python to detect humans. Humans are identified using a Histogram of Oriented Gradients method conjunction with a Linear Support Vector Machine. When a person is detected, the machine sends an email to notify the user.

### Licencing info

All code in this repository is licensed under the GNU General Public License Version 3: https://www.gnu.org/licenses/gpl-3.0.en.html

server.py contains code snippits which are are licensed under the Apache 2 license, which is compatable with this project's license which is GNU GPLv3. For more information see: https://www.apache.org/licenses/GPL-compatibility.html

detect.py and detectbest.py contain code snippits from http://pyimagesearch.com which is licensed under the MIT license, thus compatable with the GNU GPLv3. For more information see: https://en.m.wikipedia.org/wiki/MIT_License
