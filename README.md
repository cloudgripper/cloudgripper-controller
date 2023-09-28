# CloudGripper Controller

## Install PlatformIO Core
Follow the instructions at https://platformio.org/install/cli to install


## Cloning the CloudGripper Controller project
```
cd ~/cloudgripper/
git clone https://github.com/cloudgripper/cloudgripper-controller.git
```

## Build and upload to microcontroller
```
cd ~/cloudgripper/cloudgripper-controller/
pio run --target upload
```

## To delete compiled objects
```
pio run --target clean
```
