# 安装

1. [下载链接](https://opencv.org/releases/)，下载后解压，并执行cmake

2. 依赖，有时依赖冲突可用aptitude解决

   ```bash
   # 安装必要的依赖项
   sudo apt-get install build-essential libgtk2.0-dev libvtk7-dev libjpeg-dev libtiff5-dev libjasper-dev libopenexr-dev libtbb-dev
   ```

3. 将openCV安装：`sudo make install`