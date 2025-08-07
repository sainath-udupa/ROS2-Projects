#step01-install wsl in your windows
wsl --install

#step02-install ubuntu via wsl in your terminal/command-line
wsl --install -d Ubuntu-22.04

#step03-launch ubuntu and set-up your username and password for the linux environment
wsl -d Ubuntu-22.04

#update and upgrade packages
sudo apt update && sudo apt upgrade -y

#check the installed version in your ubuntu
lsb_release -a
