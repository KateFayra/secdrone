# secdrone
Autonomous surveillance &amp; security drone

For my senior thesis for my degree of Bachelor of Science (B.S.) in Computer Science from the University of Illinois at Urbana-Champaign, I designed and implemented an autonomous unmanned aerial vehicle which can patrol an area and identify humans.

The drone includes a Raspberry Pi computer which controls it. The computer runs an on-board web server and a secure WiFi access-point running a controller web-application built with Python.

The web application uses the Google Maps API and allows for the drone to be sent to a GPS way-point, specify the speed and altitude, and specify a circuit for the drone to patrol.

The drone has a camera, and sends images to a computer connected to the drone’s WiFi network. This computer continuously processes images with OpenCV in Python to detect humans. Humans are identified using a Histogram of Oriented Gradients method conjunction with a Linear Support Vector Machine. When a person is detected, the machine sends an email to notify the user.

You can view my thesis here: http://njlochner.com/thesis.html

My thesis is also available on this github repository.
