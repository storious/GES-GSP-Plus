# GES-GSP-Plus


[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)   [![C++](https://img.shields.io/badge/C++-17-blue.svg)](https://isocpp.org/)   [![OpenCV](https://img.shields.io/badge/OpenCV-4.12.0-green.svg)](https://opencv.org/)   [![Platform](https://img.shields.io/badge/Platform-Windows-lightgrey.svg)](https://www.microsoft.com/windows)

**对 [GES-GSP-Stitching](https://github.com/flowerDuo/GES-GSP-Stitching) 的优化与现代化版本，拥有更精简的构建流程、最新的依赖库，并且无需任何外部 DLL。**

## 中文 | [English](./README.md)

## 🚀 快速开始

通过我们简单的设置指南，几分钟内即可上手。

### 📋 实验环境

请确保你的 Windows 11 机器上已安装以下工具：

- **[CMake](https://cmake.org/)** (版本 >= 3.23)
- **[Vcpkg](https://vcpkg.io) or [Conan](https://conan.io/)** (版本 >= 2.0)
- **MSBuild** (来自 Visual Studio 2022)
> **💡 提示:** 如果你使用 Visual Studio 2026 那么请务必使用vcpkg (目前最新支持opencv4.11.0 不影响复现)
- **IDE**: [VSCode](https://code.visualstudio.com/) (推荐)

> **💡 提示:** 我们强烈推荐使用 [Scoop](https://scoop.sh/) 在 Windows 上轻松安装 CMake 和 Conan。
```powershell
# 安装 Scoop
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
Invoke-RestMethod -Uri https://get.scoop.sh | Invoke-Expression
# 使用 Scoop 安装工具
scoop install cmake conan # 如果不想使用 VS 自带的vcpkg, 也可以使用 scoop 单独安装一个 
```
### 🛠️ 构建与运行

请按照以下步骤构建并运行项目：

#### 对于使用 Vcpkg 的用户
```powershell
# 1. 克隆仓库
git clone https://github.com/storious/GES-GSP-Plus.git

# 2. 进入源代码目录
cd Code

# 3. 使用 CMake 预设配置项目
cmake --preset vcpkg-debug

# 4. 在 Debug 模式下构建项目
cmake --build --preset vcpkg-debug

# 如需 Release 构建，请使用: 
# cmake --preset vcpkg-release
# cmake --build --preset vcpkg-release

# 7. 运行应用程序
cd .. # 确保你位于项目根目录
.\Code\build\Debug\ges_stitching.exe <input_data_name>
# 示例: .\Code\build\Debug\ges_stitching.exe AANAP-01_skyline

```

#### 对于使用 Conan 的用户
> **💡 提示:** 如果你使用 visual Studio 2026，下载编译opencv4.12.0 时会有问题，请改用 vcpkg

```powershell
# 1. 克隆仓库
git clone https://github.com/storious/GES-GSP-Plus.git

# 2. 进入源代码目录
cd Code

# 3. 初始化 Conan 配置
conan profile detect --force
# 这里有可能会检测不到系统编译环境，可以使用 `conan profile show default`查看是否正常

# 4. 为 Debug 模式安装依赖
conan install . --output-folder=build --build=missing -s build_type=Debug

# 5. 使用 CMake 预设配置项目
cmake --preset conan-default

# 6. 在 Debug 模式下构建项目
cmake --build --preset conan-debug

# 如需 Release 构建，请使用: cmake --build --preset conan-release
# 7. 运行应用程序
cd .. # 确保你位于项目根目录
.\Code\build\Debug\ges_stitching.exe <input_data_name>
# 示例: .\Code\build\Debug\ges_stitching.exe AANAP-01_skyline
```
### 📜 脚本
为了方便，你可以使用我们提供的脚本：
- **`RUN.EXE.bat`**: 构建完成后，修改 `RUN_FILE.txt` 即可轻松运行不同的数据集。
- **`debug.bat`**: 一步完成重新构建和运行。非常适合在修改代码后进行快速调试。
---

## ✨ 核心特性与改进
- 🧹 **简化的工作流**: 无需再寻找 DLL。所有依赖均由 CMake 控制生成，带来无忧体验。
- 📦 **现代化依赖管理**: 使用 [Conan](https://conan.io/) / [Vcpkg](https://vcpkg.io) 实现可靠且可复现的 C++ 依赖管理。
- 🔄 **更新的核心库**: 基于较新版本的 [OpenCV](https://opencv.org/) 和 [Eigen](https://eigen.tuxfamily.org/) 构建。
- 🗑️ **移除 VLFeat**: SIFT 实现已完全迁移至 OpenCV，减少了外部依赖。
- 🛠️ **CMake 预设**: 利用现代 CMake 预设功能，实现标准化且简单的构建配置。
---

## 🤝 贡献与反馈
贡献是让开源社区成为一个学习、激励和创造的绝佳之地。你所做的任何贡献都将**被高度赞赏**。
如果你有任何能让这个项目变得更好的建议，请 Fork 这个仓库并创建一个 Pull Request。你也可以直接创建一个带有 "enhancement" 标签的 Issue。
别忘了给这个项目点个 Star！再次感谢！
- **常见问题**: 如有任何疑问，欢迎随时在 [GitHub Issues](https://github.com/storious/GES-GSP-Plus/issues) 中提出。
---

## 📄 许可证
本项目基于 MIT 许可证开源。详情请参阅 [LICENSE](LICENSE) 文件。