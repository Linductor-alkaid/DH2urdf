# DH2URDF

## 项目简介

`DH2URDF` 是一个用于根据 Denavit-Hartenberg (DH) 参数生成 URDF 文件的工具。该工具能够将 DH 坐标系中的参数（如关节角度 `theta`，连杆长度 `a`，连杆偏移 `d` 和连杆扭转角度 `alpha`）转换为 URDF 格式的机器人模型文件。该工具支持从文本文件输入 DH 参数，并自动生成对应的 URDF 文件，适用于 ROS（Robot Operating System）等机器人开发平台。

## 功能

- 通过输入 DH 参数，自动计算每个关节的旋转轴（`axis`）。
- 根据 DH 参数生成带有链接、关节及其属性（如限制条件）的 URDF 文件。
- 支持通过读取格式化的文本文件输入 DH 参数，简化用户操作。
- 输出的 URDF 文件包含机器人的连杆、关节、惯性、视觉和碰撞信息。

## 依赖

- C++11 或更高版本的编译器
- 标准 C++ 库（`<iostream>`、`<fstream>`、`<cmath>`、`<vector>`、`<string>`、`<sstream>`）

## 项目结构

```
DH2URDF/
├── DH2urdf.cpp         # 主程序源代码
├── README.md           # 本文档
└── dh_params.txt       # 示例 DH 参数输入文件
```

## 输入文件格式

输入文件（如 `dh_params.txt`）应包含每个关节的 DH 参数，每行的格式如下：

```
theta, d, a, alpha
```

例如，文件内容是：

```
0,0,0.1,0
0,0,0.1,1.57
0,0,1,0
0,0,1,0
```

每个参数之间使用逗号分隔。每行对应一个关节的 DH 参数。

## 使用说明

### 编译和运行

1. 克隆该仓库或下载源代码。
2. 使用 `g++` 或其他支持 C++11 的编译器进行编译。

#### 示例编译命令（使用 `g++`）：

```bash
g++ -o DH2urdf DH2urdf.cpp -std=c++11
```

3. 运行程序：

```bash
./DH2URDF
```

程序将提示输入 DH 参数文件的路径。

### 输入 DH 参数文件

- 当程序启动时，您将被提示输入一个包含 DH 参数的 `.txt` 文件路径。例如，输入 `dh_params`。

### 输入输出文件名

- 在成功读取 DH 参数文件后，程序会生成一个 `.urdf` 格式的机器人描述文件。

### 示例

假设输入文件 `dh_params.txt` 内容如下：

```txt
0,0,0.1,0
0,0,0.1,1.57
0,0,1,0
0,0,1,0
```

程序运行后会生成一个名为 `dh_params.urdf` 的文件。

#### 运行示例：

```bash
Enter the input file name (with .txt extension): dh_params.txt
URDF file generated: dh_params.urdf
```

这将生成 `dh_params.urdf` 文件，包含一个带有三个关节的机器人模型。

## 生成的 URDF 文件

URDF 文件包含机器人的连杆（`link`）和关节（`joint`）信息，连杆具有圆柱形状，关节设置为可旋转类型（`revolute`）。此外，程序还为每个连杆添加了惯性、视觉和碰撞体属性。

例如，生成的 URDF 文件将包含类似如下内容：

```xml
<robot name="robot">
  <link name="link_0">
    <inertial>
      <origin xyz="0.25 0 0.25" rpy="0 0 0"/>
    </inertial>
    <visual>
      <origin xyz="0.25 0 0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
      <material name="default_material">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.25 0 0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
    </collision>
  </link>
  <joint name="joint_0" type="revolute">
    <parent link="link_0"/>
    <child link="link_1"/>
    <origin rpy="0 0 0" xyz="0.5 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100.0" velocity="1.0"/>
  </joint>
  <!-- More links and joints... -->
</robot>
```

## 贡献

如果您发现问题或希望为该项目做出贡献，请提交 issue 或 pull request。欢迎参与讨论和改进！

## 授权

该项目使用 MIT 许可证，详见 [LICENSE](LICENSE) 文件。
