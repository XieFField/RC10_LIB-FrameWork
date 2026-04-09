## RC10_LIB-FrameWork GDUT-R1-代码框架/仓库
GDUT 2026ROBOCON R1仓库 本项目基于STM32H723ZGT6开发
---
文件目录结构说明
- `RC10_LIB-FrameWork/`
  - `README.md`         #项目总说明
  - `其他手册`          #控制方案以及相关参考手册
  - `matlab`            #用于验证逻辑的仿真相关代码
  - `RC10_LIB`          #此架构下的Lib
    - `BSP_Driver`      #基层类
    - `Module`          #模块类
    - `APP`             #工具/算法类
    - `Motor`           #电机类
    - `Lua`             #航模遥控器脚本
  - `User`              #应用层
    - `Control`         #机构驱动
    - `Setup`           #任务层/初始化
---
RC10_LIB-FrameWork是一个基于Robcon 2026比赛建立的通用电控软件架构，此架构基于GDUT Robocon2025和 Robocon2024框架改进而来。

