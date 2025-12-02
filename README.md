# GES-GSP-Plus

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) [![C++](https://img.shields.io/badge/C++-17-blue.svg)](https://isocpp.org/) [![OpenCV](https://img.shields.io/badge/OpenCV-4.12.0-green.svg)](https://opencv.org/) [![Platform](https://img.shields.io/badge/Platform-Windows-lightgrey.svg)](https://www.microsoft.com/windows)
**An optimized and modernized version of [GES-GSP-Stitching](https://github.com/flowerDuo/GES-GSP-Stitching), featuring a streamlined build process, up-to-date dependencies, and a DLL-free experience.**

## [中文](./README_CN.md) | English
## 🚀 Quick Start
Get up and running in minutes with our simple setup guide.
### 📋 Prerequisites
Ensure you have the following tools installed on your Windows 11 machine:
- **[CMake](https://cmake.org/)** (version >= 3.23)
- **[Conan](https://conan.io/)** (version >= 2.0)
- **MSBuild** (from Visual Studio 2022)
- **IDE**: [VSCode](https://code.visualstudio.com/) (Recommended)
> **💡 Tip:** We highly recommend using [Scoop](https://scoop.sh/) for easy installation of CMake and Conan on Windows.
```powershell
# Install Scoop
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
Invoke-RestMethod -Uri https://get.scoop.sh | Invoke-Expression
# Install tools with Scoop
scoop install cmake conan
```
### 🛠️ Build & Run
Follow these steps to build and run the project:
```powershell
# 1. Clone the repository
git clone https://github.com/storious/GES-GSP-Plus.git
# 2. Navigate to the source directory
cd Code
# 3. Initialize the Conan profile
conan profile detect --force
# 4. Install dependencies (OpenCV, Eigen) for Debug mode
conan install . --output-folder=build --build=missing -s build_type=Debug
# 5. Configure the project using CMake presets
cmake --preset conan-default
# 6. Build the project in Debug mode
cmake --build --preset conan-debug
# For a release build, use: cmake --build --preset conan-release
# 7. Run the application
cd .. # Make sure you are in the project root
.\Code\build\Debug\ges_stitching.exe <input_data_name>
# Example: .\Code\build\Debug\ges_stitching.exe AANAP-01_skyline
```
### 📜 Scripts
For convenience, you can use the provided scripts:
- **`RUN.EXE.bat`**: After building, modify `RUN_FILE.txt` to easily run different datasets.
- **`debug.bat`**: Rebuilds and runs the project in one step. Ideal for quick debugging after code changes.
---
## ✨ Key Features & Improvements
- 🧹 **Simplified Workflow**: No more hunting for DLLs. Everything is statically linked for a hassle-free experience.
- 📦 **Modern Dependency Management**: Uses [Conan](https://conan.io/) for reliable and reproducible C++ dependency management.
- 🔄 **Updated Core Libraries**: Built with the latest versions of [OpenCV](https://opencv.org/) and [Eigen](https://eigen.tuxfamily.org/).
- 🗑️ **VLFeat Removed**: The SIFT implementation has been fully migrated to OpenCV, reducing external dependencies.
- 🛠️ **CMake Presets**: Leverages modern CMake presets for a standardized and simple build configuration.
---
## 🤝 Contributing & Feedback
Contributions are what make the open-source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.
If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!
- **Frequently Asked Questions**: For any questions, please feel free to open an issue on [GitHub Issues](https://github.com/storious/GES-GSP-Plus/issues).
---
## 📄 License
Distributed under the MIT License. See `LICENSE` for more information.
---