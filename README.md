# ADPSLdriver
An EPICS areaDetector driver for the X-ray FDS detector from Photonic Science Limited (PSL). It needs to be build inside the AreaDetector environment.

First version of the driver. Needs to be tested and debugged. You need the original PSL files to compile and use it:

Compiling:
- in PSLdriverSupport create the folders os\win32-x86 and/or os\windows-x64
- copy fdscontrol.h to PSLdriverSupport
- copy fdscontrol.dll and fdscontrol.lib to os\win32-x86 and/or os\windows-x64
- compile

Before starting the IOC:
- to be tested: fdscontrol.dll needs to be in ...\iocs\PSLdriverIOC\iocBoot\iocPSLdriver to run the IOC ?? just copy it there...
- edit PSLdriverConfig in your st.cmd: locate your camera_files (read vendor manual) and adapt the path
- connect the camera, close vendor software if running and run the IOC
