# ArduPilot SITL and Gazebo Setup

This document describes the steps required to install and configure ArduPilot SITL together with the Gazebo simulator. It also includes instructions for installing DroneKit. When you install all neededs you should replace /ardupilot_gazebo/models folder with our folder and /ardupilot_gazebo/worlds folder with our world folder.

---

## 1. Installing Git


```bash
sudo apt-get update
sudo apt-get upgrade

sudo apt-get install git
sudo apt-get install gitk git-gui
```

---

## 2. Cloning the ArduPilot Repository

```bash 
git clone https://github.com/ArduPilot/ardupilot.git

cd ardupilot
git submodule update --init --recursive
```

---

## 3. Installing Required Components
```bash 
sudo apt install python-matplotlib python-serial python-wxgtk4.0 python-wxtools python-lxml python-scipy python-opencv ccache gawk python-pip python-pexpect

gedit ~/.bashrc

export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH

. ~/.bashrc
```

---

## 4. Installing MAVProxy

```bash 
sudo pip install future pymavlink MAVProxy

```
---

## 5. Running ArduPilot SITL


```bash 
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -w --console --map


```
---

## 6. Installing Gazebo
```bash 
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

sudo apt update
sudo apt-get install gazebo9
sudo apt-get install libgazebo9-dev

```
---

## 7. Installing the Gazebo ArduPilot Plugin

```bash 
git clone https://github.com/khancyr/ardupilot_gazebo
cd ardupilot_gazebo

mkdir build
cd build
cmake ..
make -j4
sudo make install
```

---

## 8. Installing DroneKit


```bash 
sudo apt-get install python-pip python-dev python3-pip python3-dev

pip install dronekit
pip3 install dronekit

pip install dronekit-sitl
pip3 install dronekit-sitl

```
---

## 9. Download Python_files Folder 

- Check path dependencies in the `mission.py` and run it.
---
