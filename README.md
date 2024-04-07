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
imu-setup-process

1.sudo raspi-config #ban console and open serial
2.sudo nano /boot/firmware/config.txt 
add "dtoverlty=bluetooth-bt"
3.systemctl disable bluetooth #ban the bluetooth service 
4.sudo reboot 
5.change the src code and make a new one 