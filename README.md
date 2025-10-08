# 基于树莓派5的电子积木
本仓库用于收集和封装树莓派5控制各个模块的代码, 供以后参考使用

硬件: Raspberry Pi 5 Model B Rev 1.0
*操作系统: Raspberry Pi OS(64-bit)*
操作系统: Ubuntu22.04 LTS

[树莓派5学习手册](https://p9v7kwu259.feishu.cn/wiki/OlN3wt5jwiIwppkCE1rchdPKnHg?fromScene=spaceOverview)

## 环境安装
创建python虚拟环境时一定要使用让虚拟环境能够使用系统包，否则使用调用gpio会报错

```bash
sudo apt-get install python3-pip
python3 -m venv --system-site-packages ~/ros2_venv
source ~/ros2_venv/bin/activate
```

**2025/10/8 更新**
在Raspberry Pi OS中安装ros2 docker控制树莓派的gpio老是出问题, 所以我在树莓派上重新安装了系统, 新系统是ubuntu22.04 LTS, 然后在ubuntu中安装了ros2, 避免使用docker控制gpio
具体参看树莓派5学习手册

## 示例代码
`example`中的代码都是单个可运行的小模块, 直接放在树莓派中运行即可
`ros2_example`中放的都是节点代码, 请自行创建ros2工作空间然后放入节点代码之后编译运行