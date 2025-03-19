# UAV

PI SETUP

Flash latest PI OS

sudo apt-get update
sudo apt-get upgrade
sudo apt-get install python3-pip
sudo apt-get install python3-dev
sudo apt-get install screen python3-wxgtk4.0 python3-lxml

pip install future --break-system-packages
pip install pyserial --break-system-packages
pip install dronekit --break-system-packages
pip install MAVProxy --break-system-packages

Pip install --break-system-packages --upgrade opencv-python opencv-contrib-python

Sudo apt install libraspberrypi-bin
Sudo apt-get install fswebcam 
Sudo apt-get install libcamera-apps
Sudo apt-get install libcamera-dev
Sudo apt-get install libcamera0

pip install geopy


SSH Connection:
Connect both computers to hotspot running zerotier
In windows/mac terminal:
ssh uav@172.28.51.11
Should ask for pi passowrd if connection is successful.(may take a moment or multiple tries)


