# Autonomous Surveillance and Security Drone

For my senior thesis for my degree of Bachelor of Science (B.S.) in Computer Science from the University of Illinois at Urbana-Champaign, I designed and implemented an autonomous unmanned aerial vehicle which can patrol an area and identify humans.

You can view my thesis paper here: http://njlochner.com/thesis.html

My thesis paper is also available on this GitHub repository: https://github.com/njlochner/secdrone/blob/master/Autonomous%20Surveillance%20and%20Security%20Drone.pdf

### Overview

The drone includes a Raspberry Pi computer which controls it. The computer runs an on-board web server and a secure WiFi access-point running a controller web-application built with Python.

The web application uses the Google Maps API and allows for the drone to be sent to a GPS way-point, specify the speed and altitude, and specify a circuit for the drone to patrol.

The drone has a camera, and sends images to a computer connected to the drone’s WiFi network. This computer continuously processes images with OpenCV in Python to detect humans. Humans are identified using a Histogram of Oriented Gradients method conjunction with a Linear Support Vector Machine. When a person is detected, the machine sends an email to notify the user.

### Thesis Abstract

Autonomous surveillance drones, also known as unmanned aerialvehicles, would improve security for individuals, businesses, and government agencies. Drones present an affordable, effective method toautomatically surveil a large open area to identify unauthorized personnel at close range.

This paper fully describes the design and implementation of an autonomous unmanned aerial vehicle which can patrol an area and identify humans. Potential future improvements of my aircraftand some alternative uses of unmanned aerial surveillance, other thansurveilling an area of private property, are also discussed.

### Licencing/Copyright Info

#### Source Code

All code in this repository is licensed under the GNU General Public License Version 3: https://www.gnu.org/licenses/gpl-3.0.en.html

server.py contains code snippits which are are licensed under the Apache 2 license, which is compatable with this project's license which is GNU GPL v3. For more information see: https://www.apache.org/licenses/GPL-compatibility.html

detect.py and detectbest.py contain code snippits from http://pyimagesearch.com which is licensed under the MIT license, thus compatable with the GNU GPL v3. For more information see: https://en.m.wikipedia.org/wiki/MIT_License

#### Thesis Paper

All original materials created by the author of this paper are Copyright ©2016 Nicholas Lochner, All Rights Reserved, except for images which depict copyrighted and/or trademarked third-party software or materials which are used under the fair use exception of the US Copyright Act of 1979.

Redistribution of any source code part contained in the thesis paper is licensed under the GNU GPL v3 and is subject to the terms and conditions of the GNU General Public License Version 3. A copy can be obtained here: http://www.gnu.org/licenses/gpl-3.0.en.html

Redistribution of any (non-source code) part of this document copyrighted by the author of this paper is permitted provided that all of the following conditions are met:

•The name of the author may not be used to endorse or promote products derivedfrom this paper without specific prior written permission of the author.

•The purpose of redistribution falls under Section 107 of the United States Copyright Act of 1976.

•Proper attribution is given.

•The redistribution is unmodified, unless it is an image or a section of text of a paragraph or less. Redistribution not meeting the above conditions is prohibited without specific priorwritten permission of the author.

See the last few pages of the paper for full copyright and redistribution details.
