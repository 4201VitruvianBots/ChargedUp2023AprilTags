echo "" >> ~/.bashrc
echo "export PYTHONPATH=/home/pi/ChargedUp2023DepthAI:${PYTHONPATH}" >> ~/.bashrc
source ~/.bashrc

sudo apt-get -y update
sudo apt-get -y install python-opencv
sudo apt-get -y install python3-pyqt5
sudo apt-get -y install usbtop
sudo apt-get -y install usbutils
sudo apt-get -y install vnstat
sudo apt-get -y install dos2unix
sudo apt-get -y install network-manager
sudo apt-get -y install bridge-utils
sudo apt-get -y install rsync
sudo apt-get -y install bc
sudo apt-get -y install dkms
sudo dpkg -i /opt/linux-headers-*.deb
sudo wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash
pip3 install --find-links=https://tortall.net/~robotpy/wheels/2023/raspbian/ robotpy-cscore robotpy-apriltag