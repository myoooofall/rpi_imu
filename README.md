# S01_Embedded
Dependence:

	Threads

	ZOS::rate
	
	gcc-11

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
