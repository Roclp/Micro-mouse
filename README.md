# Micro-mouse
# TQD-Micromouse-OC v2.0 迷宫机器人虚拟仿真评测系统

[![GitHub license](https://img.shields.io/github/license/yourname/yourproject)](https://github.com/yourname/yourproject/blob/master/LICENSE)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](http://makeapullrequest.com)

## 简介

TQD-OC V2.0 是由天津启诚伟业科技有限公司开发的迷宫机器人虚拟仿真评测系统，专为虚拟仿真教育教学需求设计。系统支持3D模型搭建、多传感器融合、智能算法验证和数据实时反馈等功能，提供沉浸式的第一视角体验，并支持多种算法实现迷宫搜索和路径优化。

## 特性

- **3D模型搭建**：系统内置3D迷宫和机器人模型，可根据竞赛需求自由切换。
- **多传感器融合**：支持迷宫机器人的传感器数据处理，提供真实的仿真体验。
- **智能算法验证**：支持左手算法、右手算法、中心算法和洪水算法等，轻松实现迷宫搜索和路径优化。
- **数据实时反馈**：在仿真运行过程中，实时反馈机器人的速度、偏移量、转弯角度等数据。
- **成绩记录**：实时显示迷宫时间和运行时间，记录最优成绩，并在竞赛结束后自动排序。
- **模式切换**：支持调试模式，允许参赛者通过校正参数优化机器人运行效率。
- **国际化支持**：一键切换中英文界面，满足国际选手与国内选手的竞技需求。

## 安装

在开始使用之前，请确保您的系统满足以下要求：

- Ubuntu 22.04 或更高版本
- 网络连接以完成安装和配置

安装步骤如下：

1. 打开终端并导航至评测系统文件夹。
2. 拖动 `app` 文件到终端窗口并启动。
3. 在登录窗口输入正确的账号密码以打开评测系统启动工具。
4. 依次点击“一键安装”、“一键配置”和“一键启动”以完成系统设置。

## 使用

系统启动后，您可以通过以下步骤运行仿真：

1. 点击“启动仿真”以打开仿真视图。
2. 点击“启动”按钮运行竞赛程序。
3. 使用“设置”菜单切换到调试模式进行参数调试。

## 目录结构

- `app`：系统启动文件。
- `config.ini`：系统配置文件，包含初始化信息。
- `contest.py`：程序文件，包含迷宫机器人的底层驱动和顶层逻辑。
- `debug.py`：参数调试文件。
- `urdf`：迷宫机器人模型文件夹。
- `world`：迷宫模型文件夹。
- `TQD`：迷宫描述文本文件的文件夹。

## 贡献

我们欢迎任何形式的贡献！请fork本项目，提出您的问题或建议，并提交pull request。

## 许可证

Distributed under the MIT License. See `LICENSE` for more information.

## 联系我们

如有任何问题，请通过以下方式联系我们：

- Email: [your_email@example.com](mailto:your_email@example.com)
- Website: [https://www.yourwebsite.com](https://www.yourwebsite.com)
