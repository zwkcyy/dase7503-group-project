# 电控代码

`firmware/` 是指向 [7503esp32](https://github.com/CaspianW/7503esp32) 的 Git submodule，
当前固定在提交 `2b56582`，源码入口为 `firmware/7503esp32/7503esp32.ino`。

首次获取代码：

```bash
git clone --recurse-submodules https://github.com/zwkcyy/dase7503-group-project.git
```

已经克隆公共仓库的用户可运行：

```bash
git submodule update --init ec/firmware
```

上游固件仓库目前是私有的。只有获得该仓库读取权限的用户才能初始化子模块；
公共仓库只保存仓库地址和固定提交号，不包含固件文件。电控负责人更新私有仓库后，
还需要更新公共仓库中的子模块提交号。

当前固件已经验证电机、编码器、轮速闭环、升降舵机和串口键盘测试；
它目前使用 ASCII 串口指令，原生 micro-ROS 通信尚未实现。
