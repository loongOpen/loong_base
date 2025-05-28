Copyright 2025 人形机器人（上海）有限公司, https://www.openloong.net/

# 简介

OpenLoong framework控制业务主程序

将编译为各个任务的可执行文件，调用相应的动态库

* loong_driver：实机驱动
* loong_locomotion：运动控制器
* loong_manipulation：操作控制器
* loong_interface：交互

# 使用

所有操作均在tools文件夹内完成

* 编译：./make.sh [参数]，参数：xs、xr、as、ar（含义见脚本内），生成可执行文件于nabo_output目录内
* 开启驱动：./run_driver.sh [参数]，参数：空、s
* 开启运动控制：./run_locomotion.sh [参数]，参数：空、s
* 开启上肢运动控制：./run_manipulation.sh [参数]，参数：空、s
* 开启交互：./run_interface.sh [参数]，参数：空、s
  (有些脚本需要权限)

注：loong_driver_sdk依赖libethercat.so、libmodebus.so(LGPL协议)，可由loong_thirdparty编译并拷贝至本目录的third_party/lib_lin_x64(或a64)目录下
