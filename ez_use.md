# Example
如example_data中放入的json数据
运行命令：
```python
# 生成html 
# eg. LiDAR 2 IMU: soure = LiDAR, target = IMU
# --axis-length 5 每根轴的长度
python extrinsic_visualizer_plus.py --target IMU --source LiDAR --file example_data/example.json --axis-length 5 --output example_data/lidar_to_imu.html

```
```bash
# 运行html in ubuntu
xdg-open example/lidar_to_imu.html

```

