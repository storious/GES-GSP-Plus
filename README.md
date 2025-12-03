# GES-GSP-Plus

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)   [![C++](https://img.shields.io/badge/C++-17-blue.svg)](https://isocpp.org/)  [![OpenCV](https://img.shields.io/badge/OpenCV-4.12.0-green.svg)](https://opencv.org/)  [![Platform](https://img.shields.io/badge/Platform-Windows-lightgrey.svg)](https://www.microsoft.com/windows)

**An optimized and modernized version of [GES-GSP-Stitching](https://github.com/flowerDuo/GES-GSP-Stitching), featuring a streamlined build process, up-to-date dependencies, and a self-contained build with no external DLLs required.**

## English | [中文](./README_CN.md)
## 🚀 Quick Start
Get up and running in minutes with our simple setup guide.

### 📋 Prerequisites
Ensure you have the following tools installed on your Windows 11 machine:
- **[CMake](https://cmake.org/)** (version >= 3.23)
- **[Vcpkg](https://vcpkg.io)** or **[Conan](https://conan.io/)** (version >= 2.0)
- **MSBuild** (from Visual Studio 2022)
> **💡 Tip:** If you are using Visual Studio 2026, we strongly recommend using vcpkg, as Conan may encounter issues compiling OpenCV 4.12.0. (Using vcpkg with OpenCV 4.11.0 will not affect reproduction of results.)
- **IDE**: [VSCode](https://code.visualstudio.com/) (Recommended)

> **💡 Tip:** We highly recommend using [Scoop](https://scoop.sh/) to easily install CMake and Conan on Windows.
```powershell
# Install Scoop
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
Invoke-RestMethod -Uri https://get.scoop.sh | Invoke-Expression
# Use Scoop to install tools
scoop install cmake conan # You can also use 'scoop install vcpkg' if you prefer not to use the version bundled with Visual Studio
```
### 🛠️ Build and Run
Follow the steps below to build and run the project.
#### For Vcpkg Users
```powershell
# 1. Clone the repository
git clone https://github.com/storious/GES-GSP-Plus.git

# 2. Navigate to the source directory
cd Code

# 3. Configure the project using CMake presets
cmake --preset vcpkg-debug

# 4. Build the project in Debug mode
cmake --build --preset vcpkg-debug

# For a Release build, use:
# cmake --preset vcpkg-release
# cmake --build --preset vcpkg-release

# 5. Run the application
cd .. # Ensure you are in the project root directory
.\Code\build\Debug\ges_stitching.exe <input_data_name>
# Example: .\Code\build\Debug\ges_stitching.exe AANAP-01_skyline
```
#### For Conan Users
> **💡 Tip:** If you are using Visual Studio 2026, you may face issues when compiling OpenCV 4.12.0. In this case, please switch to vcpkg.
```powershell
# 1. Clone the repository
git clone https://github.com/storious/GES-GSP-Plus.git

# 2. Navigate to the source directory
cd Code
 
# 3. Initialize Conan profile
conan profile detect --force
# If the compiler is not detected correctly, you can check with `conan profile show default`

# 4. Install dependencies for Debug mode
conan install . --output-folder=build --build=missing -s build_type=Debug

# 5. Configure the project using CMake presets
cmake --preset conan-default

# 6. Build the project in Debug mode
cmake --build --preset conan-debug
# For a Release build, use: cmake --build --preset conan-release

# 7. Run the application
cd .. # Ensure you are in the project root directory
.\Code\build\Debug\ges_stitching.exe <input_data_name>
# Example: .\Code\build\Debug\ges_stitching.exe AANAP-01_skyline
```
### 📜 Scripts
For your convenience, we provide the following scripts:
- **`RUN.EXE.bat`**: After building, simply edit `RUN_FILE.txt` to easily run different datasets.
- **`debug.bat`**: Rebuilds and runs the application in one step. Ideal for quick debugging after code changes.
---
## ✨ Key Features & Improvements
- 🧹 **Simplified Workflow**: No more hunting for DLLs. All dependencies are managed and built by CMake for a hassle-free experience.
- 📦 **Modern Dependency Management**: Relies on [Conan](https://conan.io/) or [Vcpkg](https://vcpkg.io) for reliable and reproducible C++ dependency management.
- 🔄 **Updated Core Libraries**: Built with modern versions of [OpenCV](https://opencv.org/) and [Eigen](https://eigen.tuxfamily.org/).
- 🗑️ **VLFeat Removed**: The SIFT implementation has been fully migrated to OpenCV, reducing external dependencies.
- 🛠️ **CMake Presets**: Leverages modern CMake presets for standardized and simple build configurations.
---
## 🤝 Contributing & Feedback
Contributions are what make the open-source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.
If you have a suggestion that would make this project better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!
- **Questions & Issues**: For any questions, please feel free to open an issue in our [GitHub Issues](https://github.com/storious/GES-GSP-Plus/issues) tracker.
---
## 📄 License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.