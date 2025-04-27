Copyright 2025 国地共建人形机器人创新中心/人形机器人（上海）有限公司, https://www.openloong.net/

# 简介

OpenLoong framework控制业务主程序

将编译为各个任务的可执行文件，调用相应的动态库

* loong_driver：实机驱动
* loong_locomotion：运动控制器
* loong_manipulation：操作控制器
* loong_interface：交互

# 使用

所有操作均在tools文件夹内完成

* 编译：./make.sh [参数]，参数：xs、xr、as、ar
* 开启驱动：./run_driver.sh [参数]，参数：空、s
* 开启运动控制：./run_locomotion.sh [参数]，参数：空、s
* 开启上肢运动控制：./run_manipulation.sh [参数]，参数：空、s
* 开启交互：./run_interface.sh [参数]，参数：空、s
  (有些脚本需要权限，首次使用前需提前修改密码，方便调用)
