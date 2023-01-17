echo "" >> ~/.bashrc
echo "alias ntpup='sudo service ntp stop; sudo ntpd -qa; sudo ntp service start'" >> ~/.bashrc
source ~/.bashrc

sudo apt-get update
sudo apt-get install python-opencv
sudo apt install python3-pyqt5
sudo curl -fL https://docs.luxonis.com/install_dependencies.sh | bash