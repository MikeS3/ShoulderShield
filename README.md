# ShoulderShield
How to set up the Shoulder Cuff Injury Prevention device

## Clone the GitHub repository
Input the following commands
```bash
cd ~
```

```bash
git clone https://github.com/MikeS3/ShoulderShield
```

## First Steps after Cloning
After cloning, enter the project directory with
```bash
cd Shoulder_Shield
```
Next run the following commands to prepare the build:

```bash
git submodule update --init --recursive
```

```bash
sed -i '23s/cmake_minimum_required(VERSION 2.8.12)/cmake_minimum_required(VERSION 3.5..3.27)/' lib/pico-sdk/lib/mbedtls/CMakeLists.txt
```
This initializes the pico-sdk git submodule and modifys one of the CMakeLists file to work with the newer versions of Cmake

## Setting up the container
---
First make sure you install docker\
You can do this on Ubuntu with the command 
```bash
sudo snap install docker
```

Once docker is intalled, build the container by running
 ```bash
sudo docker build -t shoulder_shield_pico2w .
```
### Running the docker container
After building the container, in the base directory of shoulder_shield run
```bash
sudo docker run -it --rm -v "$PWD":/app -w /app shoulder_shield_pico2w bash
```

Inside the container create a build directory and enter it with
 ```bash 
mkdir build
cd build
```
Next run Cmake with the correct parameters for the pico2w with
 ```bash
cmake -DPICO_BOARD=pico2_w ..
```

Finally build with
 ```bash 
make
```
The generated uf2 files will appear in a directory titled src
