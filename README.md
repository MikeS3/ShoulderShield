# ShoulderShield
## First Steps after Cloning
After cloning, run the following commands to prepare the build:

```bash
git submodule update --init --recursive
sed -i '23s/cmake_minimum_required(VERSION 2.8.12)/cmake_minimum_required(VERSION 3.5..3.27)/' lib/pico-sdk/lib/mbedtls/CMakeLists.txt
```
This initializes the pico-sdk git submodule and modifys one of the CMakeLists file to work with the newer versions of Cmake\

## Setting up the container
---
First make sure you install docker\
You can do this on Ubuntu with the command ```bash sudo apt install docker```
Once docker is intalled, build the container by running
 ```bash
 sudo docker build -t shoulder_shield_pico2w .
```
### Running the docker container
After building the container, in the base directory of shoulder_shield run
```bash
docker run -it --rm -v "$PWD":/app -w /app shoulder-shield-pico2w bash
```
You may need to run with root permissions\
Inside the container create a build directory and enter it with
 ```bash 
mkdir build
cd build
```
Next run Cmake with the correct parameters for the pico2w with `cmake -DPICO_BOARD=pico2_w ..`\
Finally build with `make`\
The generated uf2 files will appear in a directory titled src\
