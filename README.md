# FineNav2D

## How to use

1. clone the repository
   
```shell
mkdir nav_ws && cd nav_ws/
git clone https://gitee.com/huigg-practice/FineNav2D.git
cd FineNav2D/
git submodule update --init --recursive 
```

2. build the project

```shell
cd ..
colcon build --symlink-install
```

3. launch

```shell
# Make sure you are in the root directory of the project
source install/setup.bash
```
