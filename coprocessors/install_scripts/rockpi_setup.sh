echo "" >> ~/.bashrc
echo "export PYTHONPATH=/home/pi/ChargedUp2023DepthAI:${PYTHONPATH}" >> ~/.bashrc
source ~/.bashrc

sudo apt-get -y update
sudo apt-get -y install python-opencv python3-pyqt5 usbtop usbutils vnstat dos2unix network-manager bridge-utils rsync bc dkms
sudo dpkg -i /opt/linux-headers-*.deb
sudo wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash
pip3 install --find-links=https://tortall.net/~robotpy/wheels/2023/raspbian/ robotpy-cscore robotpy-apriltag