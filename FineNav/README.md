# FineNav：NeuPAN 机载部署源码

该目录是可独立传输的源码整包。`finenav_ws` 与 `neupan_cpp_ws` 并列，构建脚本按自身位置寻找依赖，不要求固定用户名或固定解压路径。

```text
FineNav/
├── README.md
├── tools/package_project.py
├── finenav_ws/
│   ├── src/FineNav-Engine/
│   │   └── finenav_example/create_custom_neupan_planner/
│   │       ├── models/       # 已训练的 0.4 × 0.4 m 足迹模型
│   │       ├── tools/        # 构建、模拟验证和重训工具
│   │       └── neupan_sim.yaml
│   └── neupan_dependencies/
│       ├── src/             # OSQP、osqp-eigen、QDLDL 源码
│       └── COLCON_IGNORE
└── neupan_cpp_ws/
    └── src/neupan_cpp/       # 完整 C++ 项目源码与许可证
        ├── libneupan/       # 引擎实际编译的算法核心
        └── neupan_cpp_ros/  # 上游独立 ROS 示例，不参与本接入构建
```

## 通过 GitHub 获取源码（推荐）

将本目录作为一个 Git 仓库的根目录上传，包含 `finenav_ws/`、`neupan_cpp_ws/`、`tools/`、`.gitignore` 和本说明。内层项目使用普通源码目录，不依赖 Git 子模块；原 FineNav-Engine 的 Git 历史已另行备份，不需要上传备份。

机载电脑执行（将 `<你的仓库URL>` 替换成实际地址）：

```bash
mkdir -p ~/robot_navigation
cd ~/robot_navigation
git clone <你的仓库URL> FineNav
cd FineNav/finenav_ws
bash src/FineNav-Engine/finenav_example/create_custom_neupan_planner/tools/build.sh
```

之后按下文启动导航引擎。只需要普通 `git clone`，不需要额外克隆算法库，也不需要初始化子模块。

提交前在根目录执行 `git status --short` 检查内容。根目录 `.gitignore` 已排除编译、安装、日志、压缩包和本地工具缓存；模型与依赖源码必须提交。原项目自带的地图及定位评估录包由其安装规则引用，保留以保证完整构建。验证记录也保留，便于实车测试时对照。

## 可选：打包与传输

在本目录执行（Python 标准库，无额外依赖）：

```bash
python3 tools/package_project.py
```

按需生成 `dist/FineNav-source.tar.gz` 和相邻的 `.sha256` 校验文件；源码交付目录不预放压缩包。若同名文件已存在，使用 `--output dist/FineNav-source-v2.tar.gz` 指定新名称。

打包脚本只收集源码、模型、文档和所需依赖，排除 Git 元数据、工具私有目录、Python 缓存以及所有生成的 `build/`、`install/`、`log/`。无需传输外部的 `~/neupan_cpp_ws` 或 `~/NeuPAN`。原电脑上的外部目录保留作备份，默认构建不再使用它们。

将压缩包和校验文件一起传到机载电脑。在校验文件所在目录执行：

```bash
sha256sum -c FineNav-source.tar.gz.sha256
```

选择一个没有现有 `FineNav` 目录的新位置解压，例如：

```bash
mkdir -p ~/robot_navigation
tar -xzf FineNav-source.tar.gz -C ~/robot_navigation
```

不要把其他电脑的 CMake 缓存或 `install` 中的符号链接一起用于新电脑构建。即使两个电脑的用户名相同，也应在目标机重新编译。x86 与 ARM 可共用源码和训练模型，二进制由目标机生成。

## 在机载电脑构建

前提：ROS 2 Humble、编译工具和 FineNav 开发依赖已安装。以下命令以解压到 `~/robot_navigation/FineNav` 为例：

```bash
cd ~/robot_navigation/FineNav/finenav_ws
bash src/FineNav-Engine/finenav_example/create_custom_neupan_planner/tools/build.sh
source install/setup.bash
ctest --test-dir build/finenav_example -R test_neupan_adapter --output-on-failure
```

脚本自动编译包内求解器依赖和 `../neupan_cpp_ws/src/neupan_cpp/libneupan`，随后构建 `finenav_example` 及引擎依赖；不需要在 `neupan_cpp_ws` 中另行执行 colcon，也不启动独立的 NeuPAN ROS 节点。

构建脚本使用系统工具 PATH 和 `/usr/bin/python3` 生成 ROS 接口，避免从 Conda 的 PATH 自动选择 Python、fmt 或 yaml-cpp；只影响脚本子进程，不改变当前终端。默认按顺序构建包，并限制编译并发为 4。

默认路径不需要额外环境变量。仅在有意替换算法核心时，才使用 `NEUPAN_SOURCE_DIR=/absolute/path/to/libneupan` 覆盖。

## 第 2 步：启动导航引擎

```bash
cd ~/robot_navigation/FineNav/finenav_ws
source install/setup.bash
ros2 run finenav_example hello_finenav --ros-args \
  --params-file src/FineNav-Engine/finenav_example/getting_started/params/sim_localizer.yaml \
  --params-file src/FineNav-Engine/finenav_example/create_custom_neupan_planner/neupan_sim.yaml
```

机载电脑已有的 `fins_localization` 和 `fins_driver` 保持原路径和启动方式，不包含在本压缩包中。输入为 `/localization_pose`、`/aligned_cloud`，输出为 `/cmd_vel`。`neupan_sim.yaml` 按原要求保留名称，实际使用实车墙上时钟，覆盖前一个配置的仿真时钟设置。

运行无需 Python NeuPAN 或 PyTorch。模型、配置、行为树都在整包内。可选重训工具也包含所需的原 ObsPointNet 架构副本及 C++ 导出工具；仅重训时额外需要 NumPy 和 PyTorch，或用 `--upstream` 指定其他 NeuPAN Python 源码。

## 文档与验证边界

整包已在另一临时路径解压，从零构建通过（9 个包），11 项 C++ 测试全部通过。动态库检查确认无缺失库，也未引用原工作区或 Conda 路径。

- [插件说明、参数及重训](finenav_ws/src/FineNav-Engine/finenav_example/create_custom_neupan_planner/README.md)
- [算法集成验证及既有框架问题](finenav_ws/src/FineNav-Engine/finenav_example/create_custom_neupan_planner/VALIDATION.md)

本次目录调整不改变算法、驱动、定位或框架源码。暂定车体尺寸为 0.4 × 0.4 × 0.5 m。实车尺寸、安装外参和制动能力仍需现场核对；导航成功后 Ctrl+C 的既有 Action Server 异常退出问题仍按不修改框架的约束保留。
