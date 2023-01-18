echo "" >> ~/.bashrc
echo "alias ntpup='sudo service ntp stop; sudo ntpd -qa; sudo ntp service start'" >> ~/.bashrc
source ~/.bashrc

sudo apt-get update
sudo apt-get install python-opencv
sudo apt-get install python3-pyqt5
sudo apt-get install usbtop
sudo apt-get install vnstat
sudo curl -fL https://docs.luxonis.com/install_dependencies.sh | bash