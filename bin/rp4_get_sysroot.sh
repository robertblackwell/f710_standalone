rpi_user=robert
rpi_ip_addr=192.168.1.22

mkdir $HOME/rpi-sysroot/opt
mkdir $HOME/rpi-sysroot/usr
cd $HOME
sudo rsync -avzS --rsync-path="rsync" --delete ${rpi_user}@${rpi_ip_addr}:/lib/* rpi-sysroot/lib
sudo rsync -avzS --rsync-path="rsync" --delete ${rpi_user}@${rpi_ip_addr}:/usr/include/* rpi-sysroot/usr/include
sudo rsync -avzS --rsync-path="rsync" --delete ${rpi_user}@${rpi_ip_addr}:/usr/lib/* rpi-sysroot/usr/lib
sudo rsync -avzS --rsync-path="rsync" --delete ${rpi_user}@${rpi_ip_addr}:/opt/vc rpi-sysroot/opt/vc

mkdir -p ~/pi-sysroot/usr ~/pi-sysroot/lib
rsync -avz --delete pi@raspberrypi.local:/lib/ ~/pi-sysroot/lib/
rsync -avz --delete pi@raspberrypi.local:/usr/lib/ ~/pi-sysroot/usr/lib/
rsync -avz --delete pi@raspberrypi.local:/usr/include/ ~/pi-sysroot/usr/include/
