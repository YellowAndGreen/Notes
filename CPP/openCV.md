# 安装

1. [下载链接](https://opencv.org/releases/)，下载后解压，并执行cmake

2. 依赖，有时依赖冲突可用aptitude解决

   ```bash
   # 安装必要的依赖项
   sudo apt-get install build-essential libgtk2.0-dev libvtk7-dev libjpeg-dev libtiff5-dev libjasper-dev libopenexr-dev libtbb-dev
   ```

3. 将openCV安装：`sudo make install`

## 在已有OpenCV4的情况下安装OpenCV3

- 第一步：在网上下载opencv3的源码编译包[OpenCV Download](https://opencv.org/releases/)
- 第二步：cmake时，输入：`cmake -D CMAKE_INSTALL_PREFIX=/home/xu/opencv3 -D CMAKE_BUILD_TYPE="Rlease" -D OPENCV_GENERATE_PKGCONFIG=ON ..`
- 第三步：编译安装

```
make
make install
```

- 第四步：配置环境

```
sudo vim  /etc/ld.so.conf.d/opencv.conf
输入：/home/xu/opencv3/lib
```

• 第五步：配置bashrc
`vim ~/.bashrc`,在最后添加：

```
    #opencv-3.4.12
export PKG_CONFIG_PATH=${PKG_CONFIG_PATH}:/home/xu/opencv3/lib/pkgconfig
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/home/xu/opencv3/lib
```