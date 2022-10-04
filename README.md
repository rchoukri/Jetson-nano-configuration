# Jetson-nano_Pixhawk_configuration
## Prepare for setup
The detailed guide  based on the Jetson Nano Developer Kit is on this link :

https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-2gb-devkit
 
 
### microSD Card
The Jetson Nano Developer Kit uses a microSD card as a boot device and for main storage. the minimum recommended is a 32 GB UHS-1 card.
### Micro-USB Power Supply
You’ll need to power the developer kit with a good quality power supply that can deliver 5V⎓2A at the developer kit’s Micro-USB port. Not every power supply promising “5V⎓2A” will do this.
### Mouse
### Keyboard
### Screen & HDMI Cable

## Write Image to the microSD Card

To prepare your microSD card, you’ll need a computer wan with an Internet connection and the ability to read and write SD cards, either via a built-in SD card slot or adapter.
1. Download the Jetson Nano Developer Kit SD Card Image, and note where it was saved on the computer.
2. Format your microSD card using SD Memory Card Formatter from the SD Association.
3. Use Etcher to write the Jetson Nano Developer Kit SD Card Image to your microSD card

Link to Download Etcher Balena : https://www.balena.io/etcher/
Link to SD card frmatter: https://www.sdcard.org/downloads/formatter/

## Write Image

### Setup 
1. Insert the microSD card (with system image already written to it) into the slot on the underside of the Jetson Nano module.
2. Set the developer kit on top of the paper stand.
3. Power on your computer display and connect it.
4. Connect the USB keyboard and mouse.
5. Connect your Micro-USB power supply (or see the Jetson Nano Developer Kit User Guide for details about using DC a power supply with a barrel jack connector). The developer kit will power on and boot automatically.
### First boot
In this, Tutorial we propose two versions of Jetson nano SDcard image.
#### Version 1
- Jetson Nano 2GB Developer Kit SD Card Image  is the version proposed by the Nvidia website, with Ubuntu 18.04 and python 6.9.
If you use this version, you need to configure the Jetson.
A green LED next to the Micro-USB connector will light as soon as the developer kit powers on. When you boot the first time, the developer kit will take you through some initial setup, including:
Review and accept NVIDIA Jetson Software EULA
Select system language, keyboard layout, and time zone
Create username, password, and computer name.

You can download this version from : https://developer.nvidia.com/embedded/downloads
#### Version 2

- JetsonNanoUb20 is the version proposed by Qengineering team with Ubuntu 20.04 and python3.8 .
this version is already configured, after writing the image in the SDcard, the jetson will boot automaticaly.
The password for this version is : "jetson"
All instructions are in this link: https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image

You can download this version from:
https://ln5.sync.com/dl/3a81c82d0/kngeabjw-qtt943v6-mz7kqtpu-xzqcekgy/view/default/11289357380004

In the rest of this tutorial we will work with the first version
## Setup the RPi HQ V1.0 camera with the Jetson
The RPi HQ V1.0 camera it's not supported on the Jetson nano, The CSI interface only works with the Pi Versions 2.1 or later  camera modules.

### First method: Manual Installation

for it to be functional it is necessary to install some drivers, the detailed instructions are in this tutorial: 
https://www.okdo.com/getting-started/get-started-with-jetson-nano-2gb-and-csi-camera/

Download the.deb package:

Download and install the driver: 
The link to below the driver is :
 https://github.com/ArduCAM/MIPI_Camera/tree/master/Jetson/IMX477/driver/Nano

Install package:
Go to the downloads folder

cd ~/Downloads

To Run the installation, the tutorial proposes to use the command bellow:
sudo dpkg -i xxx.deb
Replace the "xxx" with the name of the downloaded file.
But The driver package should match with our current kernel version and L4T release number, in our case it doesn’t match,
so if you use the line above, you will get some errors.
our kernel: Linux 4.9.253
the package kernel: 4.9.140
to avoid that we will use the command line:

sudo dpkg --foce-all -i xxx.deb

After the installation of the package, we must reboot the device

## Automatic Installation:

Automatic driver installation is supported only for L4T32.4.3 and later versions
Our version is: L4R32.7.1, the automatic installation works perfectly on our devices
	
Detailed instructions on this tutorial: 

https://www.arducam.com/docs/camera-for-jetson-nano/native-jetson-cameras-imx219-imx477/imx477/#5-automatic-driver-installation-the 

### Driver Setup
After writing the image to the SD Card, we must modify the camera interface (CSI) through jetson-io with the command:

sudo /opt/nvidia/jetson-io/jetson-io.py

### Download automatic installation script

cd ~
wget https://github.com/ArduCAM/MIPI_Camera/releases/download/v0.0.3/install_full.sh


### Install the driver

chmod +x install_full.sh 
./install_full.sh -m imx477


## Configuration of OpenCVpencv version to use Jompy_lib_aruco_poseMarzo_ubuntu/windows.py

https://automaticaddison.com/how-to-install-opencv-4-5-on-nvidia-jetson-nano/

The Jetson nano uses a pre-installed version of OpenCV (4.1), this version is built with the pre-installed version of python(3.6), this version of Opencv is not compatible with our library.

Also, we can't create a virtual environment and install the version we want, because, on the jetson nano (Linux in general), the camera requires an object called Gstreamer to be launched, this object can not be installed with the pip or pip3 tool in a virtual environment, so we can only use the pre-installed version of python/Opencv on Linux if we want to use a camera.

The solution is to overwrite the pre-installed version of OpenCV (which is not compatible), and build the new version at first, then install GStreamer for this new version.

This solution works only with the pre-installed version of python. (In our case Python3.6)


This installation takes between 4h and 5h, the different steps are in the link above:

I note that the version installed in this tutorial is not compatible, so we have to change some lines of code to install the versions we want.
(Opencv and Opencv-contrib V4.5.4 and Gstreamer V1.0 ).

For that, you will find above, a Bash script “build_opencv_camera_driver.sh that allows the full installation and configuration of camera and OpenCV. 

To execute the script, you need to use these this two commandlines:
cd ~/Download
chmod +x build_opencv_camera_driver.sh
bash build_opencv_camera_driver.sh

## Using Mavlink protocol to connect the Jetson to the Pixhawk. (MAVSDK)

To be able to connect the jetson nano to the pixhawk with serial connection, you need to configure the pixhawk, all instructions are in this links (1 and 2): 
https://ardupilot.org/dev/docs/companion-computer-nvidia-tx2.html  (1)
https://docs.px4.io/v1.9.0/en/peripherals/serial_configuration.html  (2)



https://mavsdk.mavlink.io/main/en/python/quickstart.html (3)

The installation of mavsdk is too simple, but it couldn’t be installed with the pre-installed versions of python on the Jetson nano (python3.6), because it requires python3.7 or more.
it could be installed with python3-pip with the following command line:

python3 -m pip install asyncio aioconsole mavsdk 

The python scripts for connecting the Jetson to The pixhawk and to get global_position are Above.




