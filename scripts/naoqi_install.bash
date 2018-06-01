#!/bin/bash

#ros-naoqi driver installation
sudo apt update
sudo apt install ros-kinetic-naoqi-driver

echo 'export HOME_NAOQI=/media/ubuntu/naoqi' >> ~/.bashrc
source ~/.bashrc

#naoqi python SDK download, installation and config
cd $HOME_NAOQI
mkdir lib && cd lib
#wget https://developer.softbankrobotics.com/Software/Python/2.5.5/Linux/pynaoqi-python2.7-2.5.5.5-linux64.tar.gz
wget https://lcas.lincoln.ac.uk/owncloud/index.php/s/1PLbRNtgklY6NCB/download -O pynaoqi-python2.7-2.5.5.5-linux64.tar.gz
tar -xzvf pynaoqi-python2.7-2.5.5.5-linux64.tar.gz
rm pynaoqi-python2.7-2.5.5.5-linux64.tar.gz

echo 'export PYTHONPATH=${PYTHONPATH}:$HOME_NAOQI/lib/pynaoqi-python2.7-2.5.5.5-linux64/lib/python2.7/site-packages' >> ~/.bashrc

#naoqi C++ SDK download, installation and config
cd $HOME_NAOQI/lib
#wget https://developer.softbankrobotics.com/Software/C%2B%2B/2.5.5/Linux/naoqi-sdk-2.5.5.5-linux64.tar.gz
wget https://lcas.lincoln.ac.uk/owncloud/index.php/s/424z8mYr9TKX7J7/download -O naoqi-sdk-2.5.5.5-linux64.tar.gz
tar -xzvf naoqi-sdk-2.5.5.5-linux64.tar.gz
rm naoqi-sdk-2.5.5.5-linux64.tar.gz

#qibuild download, installation and config
pip install qibuild --user
cd $HOME_NAOQI
mkdir qi_ws && cd qi_ws
qibuild init
qitoolchain create linux64 $HOME_NAOQI/lib/naoqi-sdk-2.5.5.5-linux64/toolchain.xml
qibuild add-config linux64 -t linux64

#pepper_tools downdload
cd $HOME_NAOQI
mkdir src && cd src
git clone https://bitbucket.org/mtlazaro/pepper_tools.git
cd $HOME_NAOQI


cd $HOME_NAOQI
mkdir choregraphe
wget https://developer.softbankrobotics.com/Software/Choregraphe/2.5.5/Linux/choregraphe-suite-2.5.5.5-linux64-setup.run
chmod u+x choregraphe-suite-2.5.5.5-linux64-setup.run
echo "When you launch Choregraphe, the license key is requested. Please copy and paste the following key: 654e-4564-153c-6518-2f44-7562-206e-4c60-5f47-5f45"
echo "Choose advanced installation and find /media/ubuntu/naoqi/choregraphe folder"
./choregraphe-suite-2.5.5.5-linux64-setup.run

