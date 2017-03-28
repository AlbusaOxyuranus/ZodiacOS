#prepare system, download and install tools
clear
sudo wget http://flatassembler.net/fasm-1.71.60.tgz
sudo tar xfvz  fasm-1.71.60.tgz
sudo cp -avr fasm /home/denis
sudo ln -s -f /home/denis/fasm/fasm.x64 /usr/local/bin
sudo rm -r fasm
sudo rm -v *.tgz.*
sudo rm -v *.tgz
sudo apt-get install build-essential;
sudo apt-get install bin86 kvm qemu gcc build-essential
sudo apt-get install g++;
sudo apt-get install gcc;
sudo apt-get install dd;
sudo apt-get install ld;
sudo apt-get install qemu-system-i386;
sudo apt-get install gcc-multilib;
sudo apt-get install g++-multilib;
sudo apt-get update;