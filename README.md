# S01_Embedded

## Install Deps

- [gcc-11](https://stackoverflow.com/questions/67298443/when-gcc-11-will-appear-in-ubuntu-repositories?answertab=votes#tab-top)

- [asio](https://think-async.com/Asio/)

- [fmt](https://fmt.dev/8.1.0/)

- [yaml-cpp](https://github.com/jbeder/yaml-cpp)

## compile

cmake option: **CM4_VERSION** default on
```bash
cmake .. -D RF24_DRIVER=pigpio
```
##imu-setup-process

1.```bash
sudo raspi-config
```  choose-interware configuation choose-serial #ban console and open serial
2.```bash
sudo nano /boot/config.txt 
```
add "dtoverlay=disable-bt"
3.```bash
systemctl disable bluetooth
``` #ban the bluetooth service 
4.``` bash
sudo reboot
``` 
the procedure above is aimed to configure raspi. after configuration we change the code
5.```bash
cd rpi-embedded 
cd src
cd share
cd proto
```
change the proto : exactly the zss_cmd.proto 
compile the files of proto:```bash
protoc --experimental_allow_proto3_optional --cpp_out=. zss_cmd.proto 
```
6.change the src code  mainly four file: device_pigpio.h device_pigpio.cpp robotz.cpp robotz.h
then make the bin file
```bash
cd build
cmake ..
make
```
111
7.reboot and finish 