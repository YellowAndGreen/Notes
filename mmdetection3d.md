# mmdetection3d

## 安装测试

单模态测试

```
python demo/pcd_demo.py demo/data/kitti/kitti_000008.bin  ./configs/second/hv_second_secfpn_6x8_80e_kitti-3d-car.py ./checkpoints/hv_second_secfpn_6x8_80e_kitti-3d-car_20200620_230238-393f000c.pth
```

> 结果默认在demo目录下，需要使用meshlab打开

## 使用kitti数据集

1. 按照官网教程组织文件

2. 建立软链接：`ln [-sf] 来源文件 目标文件`

> ln -s /home/xu/3Det/OpenPCDet/data/kitti /home/xu/3Det/mmdetection3d/data/kitti

3. 对数据预处理

```
python tools/create_data.py kitti --root-path ./data/kitti --out-dir ./data/kitti --extra-tag kitti --with-plane
```

## demo

1. second

```
python tools/test.py ./configs/second/hv_second_secfpn_6x8_80e_kitti-3d-car.py ./checkpoints/hv_second_secfpn_6x8_80e_kitti-3d-car_20200620_230238-393f000c.pth --show --show-dir ./data/kitti/show_results
```

2. 多模态

 MVX-Net 

```
python demo/multi_modality_demo.py demo/data/kitti/kitti_000008.bin demo/data/kitti/kitti_000008.png demo/data/kitti/kitti_000008_infos.pkl configs/mvxnet/dv_mvx-fpn_second_secfpn_adamw_2x8_80e_kitti-3d-3class.py checkpoints/dv_mvx-fpn_second_secfpn_adamw_2x8_80e_kitti-3d-3class_20210831_060805-83442923.pth
```

 ImVoteNet（数据集不同，不能用）

```
python demo/multi_modality_demo.py demo/data/kitti/kitti_000008.bin demo/data/kitti/kitti_000008.png demo/data/kitti/kitti_000008_infos.pkl configs/imvotenet/imvotenet_stage2_16x8_sunrgbd-3d-10class.py checkpoints/imvotenet_stage2_16x8_sunrgbd-3d-10class_20210819_192851-1bcd1b97.pth
```

3. 3D 分割

```
python demo/pc_seg_demo.py demo/data/scannet/scene0000_00.bin configs/pointnet2/pointnet2_ssg_16x2_cosine_200e_scannet_seg-3d-20class.py checkpoints/pointnet2_ssg_16x2_cosine_200e_scannet_seg-3d-20class_20210514_143644-ee73704a.pth
```

## Test

```
python tools/test.py configs/mvxnet/dv_mvx-fpn_second_secfpn_adamw_2x8_80e_kitti-3d-3class.py checkpoints/dv_mvx-fpn_second_secfpn_adamw_2x8_80e_kitti-3d-3class_20210831_060805-83442923.pth --show --show-dir ./data/kitti/show_results --eval mAP
```

+ `--show-dir`：如果被指定，检测结果会被保存在指定文件夹下的 `***_points.obj` 和 `***_pred.obj` 文件中
+ --eval：评估方式可选为mAP或img_bbox

## Train

```
python tools/train.py configs/mvxnet/dv_mvx-fpn_second_secfpn_adamw_2x8_80e_kitti-3d-3class.py
python tools/train.py /configs/second/hv_second_secfpn_6x8_80e_kitti-3d-car.py
```

