echo "" >> ~/.bashrc
echo "alias ntpup='sudo service ntp stop; sudo ntpd -qa; sudo ntp service start'" >> ~/.bashrc
source ~/.bashrc

sudo apt-get -y update
sudo apt-get -y install python-opencv
sudo apt-get -y install python3-pyqt5
sudo apt-get -y install usbtop
sudo apt-get -y install vnstat
sudo wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash