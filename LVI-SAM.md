# LVI-SAM

## Install

BRIEF.cpp 54行需改为

```
cv::cvtColor(image, aux, cv::COLOR_RGB2GRAY);
```

CMakeLists.txt加上：

```
set(CMAKE_CUDA_COMPILER "/usr/local/cuda-11.3/bin/nvcc")
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda")
set(OpenCV_DIR /home/xu/下载/opencv-4.5.4/build)
set(cv_bridge_DIR "/usr/local/share/cv_bridge/cmake")
```

catkin_make之后需要source devel/setup.bash

## code

保存建好的地图：config/KITTI_lidar.yaml中的`savePCD: True`
