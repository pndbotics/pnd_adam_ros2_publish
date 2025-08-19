### 启动adam_demo

1. 启动adam_demo脚本，在`adam_demo/bin/`目录下执行

    ```bash
    sudo su
    sh run.sh

### 编译指令

1. 启动adam_demo的脚本中使用了sudo权限，所以需要在编译前先切换到root用户下

    ```bash
    sudo su
2. 使用如下指令编译ros2包

    ```bash
    source build.sh

### 数据接收

1. adam_demo启动后即可运行下面的指令接收上肢数据

    ```bash
    sh run_subscriber.sh

### 数据发布

1. 当机器人处于站立状态时，xbox手柄中`xx`的正按键按下，终端打印`real time retarget start`表示机器人进入接收外部数据状态，此时执行如下命令

    ```bash
    sh run_publisher.sh
2. 使用方法参考`ros2_test/src/robot_state_publisher/robot_state_publisher/robot_state_publisher_node.py`中描述，文件的`62-68`行为控制姿态和高度和上肢的示例，文件的`61`行为控制手指的示例

3. 机器人停止接收外部数据时，xbox手柄中`xx`的负按键按下，终端打印`real time retarget stop`表示机器人停止接收外部数据状态
