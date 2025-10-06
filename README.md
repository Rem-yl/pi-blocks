# 基于树莓派5的电子积木
本仓库用于收集和封装树莓派5控制各个模块的代码, 供以后参考使用

硬件: Raspberry Pi 5 Model B Rev 1.0
操作系统: Raspberry Pi OS(64-bit)

[树莓派5学习手册](https://p9v7kwu259.feishu.cn/wiki/OlN3wt5jwiIwppkCE1rchdPKnHg?fromScene=spaceOverview)

## 环境安装
创建python虚拟环境时一定要使用让虚拟环境能够使用系统包，否则使用调用gpio会报错

```bash
sudo apt-get install python3-pip
python3 -m venv --system-site-packages ~/ros2_venv
source ~/ros2_venv/bin/activate
```