tinker2014
==========

tinker2014

------

#重构设想

##文件结构

假设沿用之前的三层架构：

tinker2014/

|-dev/

|-downloaded/

|-driver/

|-logic/

|-decision/

在tinker2014文件夹下放5个catkin workspace，
各自独立开发。
其中downloaded中放下载下来的包，
dev中放刚创建的包

另外，
share文件夹中放资源文件(如*.jpg)，
ui文件夹中放启动脚本等等非ROS的程序。

~~缺点是编译时需要cd到各个workspace下执行catkin_make。~~
目前最上层的编译管理使用Makefile，
但单独编译某个workspace时，
不能自动分析不同workspace之间的依赖来决定是否编译其他workspace。

##使用方法

####配置

按此方法下载tinker2014的源码：

    cd ~
    git clone https://github.com/<YOUR_NAME>/tinker2014

在.bashrc中添加以下内容：

    TINKER_WORKSPACE=~/tinker2014         # 可自行修改
    source $TINKER_WORKSPACE/tinkersetup.bash

建议在.bashrc中添加：

    ROS_WORKSPACE=$TINKER_WORKSPACE

####编译

与直接使用catkin管理ROS工作空间类似，
TINKER_WORKSPACE下执行make相当于在各工作空间执行catkin_make，
执行make clean相当于在各工作空间执行catkin_make clean。
也支持make dev或make clean_dev等用法。

    make                # 全部编译
    make driver         # 编译driver workspace
    make clean          # 全部清理
    make clean_driver   # 清理driver workspace

##命名规则

目前存在不常用的package、node很难找的情况。
暂时规定package和node的命名规则：

downloaded中的包保留原名。

driver中的包以d开头，然后按驼峰命名法命名，如dSerial

logic中的包以l开头，然后按驼峰命名法命名，如lParser

decision中的包直接按驼峰命名法命名，首字母大写，如AnswerQuestions

##协作模式

依然是fork+pull的模式。

代码开发都在dev等分支下进行，
开发好后将dev分支合并到主帐户(tinkerfuroc)的master分支(pull request)。
其他帐户的master分支用于同步主帐户master分支的更改。

临时的修改之类的先放在dev workspace里。
