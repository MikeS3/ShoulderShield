# ShoulderShield
## First Steps after Cloning
After cloning, run the following commands to prepare the build:\

```bash
git submodule update --init --recursive
sed -i '23s/cmake_minimum_required(VERSION 2.8.12)/cmake_minimum_required(VERSION 3.5..3.27)/' lib/pico-sdk/lib/mbedtls/CMakeLists.txt
```
## Setting up the container
---

### Running the docker container
After building the container, in the base directory of shoulder_shield run
```docker run -it --rm -v "$PWD":/app -w /app shoulder-shield-pico2w bash```
You may need to run with root permissions\
Inside the container create a build directory and enter it with ```mkdir build
cd build```
\Next run Cmake with the correct parameters for the pico2w with `cmake -DPICO_BOARD=pico2_w ..`\
Finally build with `make`\
The generated uf2 files will appear in a directory titled src\
