rpi_user=robert
rpi_ip_addr=192.168.1.22

cd $HOME
sudo rsync -avzS --rsync-path="rsync" --delete ${rpi_user}@${rpi_ip_addr}:/lib/* rpi-sysroot/lib
mkdir $HOME/rpi-sysroot/usr
sudo rsync -avzS --rsync-path="rsync" --delete ${rpi_user}@${rpi_ip_addr}:/usr/include/* rpi-sysroot/usr/include
sudo rsync -avzS --rsync-path="rsync" --delete ${rpi_user}@${rpi_ip_addr}:/usr/lib/* rpi-sysroot/usr/lib
mkdir $HOME/rpi-sysroot/opt
 
sudo rsync -avzS --rsync-path="rsync" --delete ${rpi_user}@${rpi_ip_addr}:/opt/vc rpi-sysroot/opt/vc
