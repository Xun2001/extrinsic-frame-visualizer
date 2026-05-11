# Extrinsic Visualizer

用于验证多传感器外参矩阵的 Python 可视化工具，主要面向 LiDAR、IMU、Camera 等坐标系之间的外参检查。

主脚本为 [extrinsic_visualizer.py](./extrinsic_visualizer.py)。

## 功能

- 支持输入 `4x4` 齐次外参矩阵 `T_target_source`
- 默认数学约定为 `p_target = T_target_source @ p_source`
- 支持命令行直接传入矩阵
- 支持从 `txt` 或 `json` 文件加载矩阵
- 支持 `--inverse`，可将输入矩阵先取逆后再显示
- 支持通过 `--source` 和 `--target` 指定坐标系名称
- 使用 Plotly 生成可交互 HTML 页面
- 在同一 3D 场景中显示 `target` 和 `source` 两个坐标系
- 支持预设视角切换
- 在终端输出旋转矩阵诊断信息

## 安装依赖

项目只依赖：

- `numpy`
- `plotly`

安装方式：

```bash
pip install numpy plotly
```

## 数学约定

输入矩阵采用如下定义：

```text
p_target = T_target_source @ p_source
```

其中：

- `T[:3, :3]` 表示从 `source` 坐标系到 `target` 坐标系的旋转矩阵
- `T[:3, 3]` 表示 `source` 坐标系原点在 `target` 坐标系下的位置

如果你手里的是反方向矩阵 `T_source_target`，可通过 `--inverse` 自动取逆后再显示。

## 输入格式

### 1. 直接通过命令行传矩阵

支持以下格式：

```bash
python extrinsic_visualizer.py \
  --matrix "1 0 0 0.4; 0 0 -1 0.1; 0 1 0 0.2; 0 0 0 1" \
  --source LiDAR \
  --target IMU
```

也支持 JSON 风格：

```bash
python extrinsic_visualizer.py \
  --matrix "[[1,0,0,0.4],[0,0,-1,0.1],[0,1,0,0.2],[0,0,0,1]]" \
  --source Camera \
  --target LiDAR
```

### 2. 从 txt 文件加载

`txt` 文件可以是逐行 4x4：

```text
1 0 0 0.4
0 0 -1 0.1
0 1 0 0.2
0 0 0 1
```

使用方式：

```bash
python extrinsic_visualizer.py \
  --matrix-file extrinsic.txt \
  --source LiDAR \
  --target Camera
```

### 3. 从 json 文件加载

支持以下 JSON 结构：

```json
[[1, 0, 0, 0.4], [0, 0, -1, 0.1], [0, 1, 0, 0.2], [0, 0, 0, 1]]
```

或带字段名：

```json
{
  "T_target_source": [
    [1, 0, 0, 0.4],
    [0, 0, -1, 0.1],
    [0, 1, 0, 0.2],
    [0, 0, 0, 1]
  ]
}
```

已识别字段包括：

- `matrix`
- `T`
- `transform`
- `extrinsic`
- `T_target_source`
- `T_source_target`

## 使用示例

### 生成 HTML

```bash
python extrinsic_visualizer.py \
  --matrix "1 0 0 0.4; 0 0 -1 0.1; 0 1 0 0.2; 0 0 0 1" \
  --source LiDAR \
  --target IMU \
  --output lidar_to_imu.html
```

### 输入反方向矩阵并取逆

```bash
python extrinsic_visualizer.py \
  --matrix-file imu_to_lidar.json \
  --inverse \
  --source LiDAR \
  --target IMU \
  --output lidar_in_imu.html
```

### 查看帮助

```bash
python extrinsic_visualizer.py --help
```

## 输出内容

脚本会生成一个可直接在浏览器打开的 HTML 文件，并显示：

- `target` 坐标系固定在原点
- `source` 坐标系按输入外参旋转和平移后显示
- 两个坐标系的 `+X / +Y / +Z` 箭头
- 坐标轴标签，如 `IMU +X`、`LiDAR +Z`
- 原点 marker 和名称标签
- legend，用于区分各条轴所属坐标系

颜色约定：

- `X` 轴红色
- `Y` 轴绿色
- `Z` 轴蓝色

## 视角切换

页面顶部提供 Plotly 下拉菜单，支持：

- 默认斜视图
- 沿 `target` 坐标系的 `+X / -X / +Y / -Y / +Z / -Z` 方向查看
- 沿 `source` 坐标系的 `+X / -X / +Y / -Y / +Z / -Z` 方向查看

切换后会自动更新 Plotly 的 `camera eye/up` 参数，实现对应正视图。

## 终端诊断输出

运行时终端会打印：

- 输入矩阵
- 用于显示的矩阵
- 平移向量 `t`
- `det(R)`
- `||R.T @ R - I||_F`

当以下情况发生时会输出 warning：

- `det(R)` 明显不接近 `1`
- 旋转矩阵正交误差较大

这些 warning 通常意味着：

- 坐标变换方向可能反了
- 行列约定可能不一致
- 旋转矩阵数值精度有问题
- 输入矩阵本身不是合法刚体变换

## 命令行参数

```text
--matrix         直接输入 4x4 矩阵
--matrix-file    从 txt/json 文件读取矩阵
--inverse        先对输入矩阵求逆
--source         source 坐标系名称
--target         target 坐标系名称
--output         输出 HTML 文件路径
```

## 典型使用流程

1. 准备 `T_target_source`
2. 明确 `source` 和 `target` 的命名
3. 如果拿到的是反方向矩阵，则加上 `--inverse`
4. 运行脚本生成 HTML
5. 打开 HTML，检查两个坐标系的相对姿态和轴向
6. 对照终端里的 `det(R)` 与正交误差判断矩阵质量

## 注意事项

- 本工具假设输入是刚体变换矩阵
- 如果矩阵不可逆，`--inverse` 会报错
- 如果当前 Python 环境缺少依赖，脚本无法运行
- 生成的是独立 HTML，可直接分享或离线打开
