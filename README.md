<img src="https://download.blender.org/branding/blender_logo_socket.png" width="280"> <img src="https://www.ros.org/wp-content/uploads/2013/10/rosorg-logo1.png" width="250"/> 

# Blender URDF 导入插件（兼容 Blender 5.1+）

这是一个面向 Blender 的 URDF 导入插件，可将机器人模型导入 Blender，并自动构建用于姿态调整与动画的骨骼系统。

## 项目来源

本项目基于 [Victorlouisdg/blender-urdf-importer](https://github.com/Victorlouisdg/blender-urdf-importer) 修改，针对 **Blender 5.1.1** 及现代版本进行了全面适配和功能改进。

## 主要改进与新功能

### 1. **Blender 版本兼容性**
- 支持 Blender 5.1.1 及以上版本
- 兼容 Blender 4.1+ 的新导入 API（`wm.stl_import`, `wm.obj_import` 等）
- 向下兼容旧版 Blender API
- 自动尝试多个导入方法确保可靠性

### 2. **多种网格格式支持**
- **STL** (.stl)：使用新的 `wm.stl_import` API，同时兼容旧版导入方式
- **OBJ** (.obj)：支持 `wm.obj_import` 与 `import_scene.obj`
- **Collada/DAE** (.dae, .xml)：完整支持 Collada 导入流程

### 3. **网格文件路径解析**
- 支持相对于 URDF 文件的相对路径
- 支持 ROS `package://` 协议引用
- 支持绝对路径直接加载
- 智能路径解析，多种策略组合确保文件找到
- 针对标准解压结构 `./<robot_name>/{meshes,urdf,xml}` 优化了 `package://` 解析
- 在未设置 `ROS_PACKAGE_PATH` 时也可优先按标准目录结构尝试定位 mesh

### 4. **几何体和变换改进**
- 正确处理 `mesh` 元素的 `scale` 属性
- 支持多个 `<visual>` 元素（原版仅支持单个）
- 精确应用 `origin` 变换（位置与 RPY 旋转）
- 自动清理导入过程中的相机和灯光对象
- 新增导入后自动贴地：当模型最低点低于地面时，自动整体上移到 `Z=0`
- 导入结束后自动恢复 Blender 3D 游标，避免游标停留在关节或脚底位置

### 5. **骨骼和蒙皮系统**
- 为每个 `revolute` 关节创建骨骼，支持正确的旋转轴约束
- 完整的蒙皮系统（Armature 绑定）
- 自动创建顶点组并应用蒙皮权重
- 支持逆向运动学（IK）约束

### 6. **错误处理与日志**
- 详细的错误信息和故障排查指南
- 改进的异常捕获机制

## 安装

1. 克隆或下载本仓库
2. 将 `urdf_importer` 目录复制到 Blender 的插件目录：
   - Linux/Mac：`~/.config/blender/5.1/scripts/addons/`
   - Windows：`%APPDATA%\Blender Foundation\Blender\5.1\scripts\addons\`

3. 打开 Blender，进入 `Edit > Preferences > Add-ons`，搜索 `URDF Importer` 并启用

## 使用方法

安装成功后，在 `File > Import` 菜单中会出现 `URDF (.urdf)` 选项。

### 重要提示

许多 URDF 文件通过 `package://` 协议引用 ROS 包中的网格文件。为了正确解析这些路径：

1. 确保 `ROS_PACKAGE_PATH` 环境变量已设置正确
2. 在终端中 `source` 你的 ROS workspace 的 `devel/setup.bash`
3. 从同一终端启动 Blender

如果未配置 ROS 环境变量，插件仍会优先按常见的标准目录结构尝试解析 `package://` 路径：

```text
./<robot_name>
├─meshes
├─urdf
└─xml
```

如果 URDF 中的网格文件使用相对路径，确保路径相对于 URDF 文件位置有效。

**网格显示效果优化**：如果导入后的网格表面出现放射状阴影伪影，可在 Outliner 中选中 Armature 对象后按 `A` 全选相关网格，然后右键选择“自动平滑着色”或“平滑着色”以改善显示效果。

## 获取 URDF 文件

ROS 机器人描述包通常命名为 `<robot>_description`，例如：
- [ur_description](https://github.com/ros-industrial/universal_robot)：Universal Robots UR 系列
- 许多包包含 `.xacro` 文件，需先使用 `xacro` 命令生成 URDF

## 添加逆向运动学

1. 在编辑模式中于末端执行器前创建一个新骨骼
2. 为末端执行器添加 IK 约束
3. 将约束目标设置为新建的骨骼

## 支持的功能

✅ Revolute 关节轴向约束  
✅ 多个 visual 元素  
✅ 多种网格格式  
✅ Origin 变换（位置和旋转）  
❌ 关节限位（Joint limits）  
❌ Prismatic 关节  
❌ Planar 关节
