tinker2014
==========

tinker2014

------

#重构设想

##文件结构

假设沿用之前的三层架构：

tinker2014/

|-dev/

|-driver/

|-logic/

|-decision/

在tinker2014文件夹下放4个catkin workspace，
各自独立开发。
（可能要加一个包放下载来的package）

缺点是编译时需要cd到各个workspace下执行catkin_make。
不能自动分析不同workspace之间的依赖来决定是否编译其他workspace。

可以在.bashrc中添加以下内容：

    source ~/tinker2014/dev/devel/setup.bash
    source ~/tinker2014/driver/devel/setup.bash --extend
    source ~/tinker2014/logic/devel/setup.bash --extend
    source ~/tinker2014/decision/devel/setup.bash --extend

##命名规则

目前存在不常用的package、node很难找的情况。
建议规定一下package和node的命名规则。
（还没想好怎么起名比较合适）

##协作模式

依然是fork+pull的模式。

代码开发都在dev等分支下进行，
开发好后将dev分支合并到主帐户的master分支。
其他帐户的master分支用于同步主帐户master分支的更改。

临时的修改之类的先放在dev workspace里。
