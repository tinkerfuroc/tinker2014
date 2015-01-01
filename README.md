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

|-share/

|-ui/

在tinker2014文件夹下放5个catkin workspace，
各自独立开发。
其中downloaded中放下载下来的包，
dev中放刚创建或临时的包

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

```bash
cd ~
git clone https://github.com/<YOUR_NAME>/tinker2014
```

在.bashrc中添加以下内容：

```bash
TINKER_WORKSPACE=~/tinker2014         # 可自行修改
source $TINKER_WORKSPACE/tinkersetup.bash
```

建议在.bashrc中添加：

```bash
ROS_WORKSPACE=$TINKER_WORKSPACE
```

之后建议先完整编译一次，再做别的操作。

####编译

与直接使用catkin管理ROS工作空间类似，
TINKER_WORKSPACE下执行make相当于在各工作空间执行catkin_make，
执行make clean相当于在各工作空间执行catkin_make clean。
也支持make dev或make clean_dev等用法。

```bash
make                # 全部编译
make driver         # 编译driver workspace
make clean          # 全部清理
make clean_driver   # 清理driver workspace
```

##命名规则

目前存在不常用的package、node很难找的情况。
暂时规定package和node的命名规则：

downloaded中的包保留原名。

driver中的包以d开头，
然后按
~~驼峰命名法命名，如dSerial~~
ROS建议的方法命名：
所有字母小写，
以下划线分隔，
如d_serial

logic中的包以l开头，
然后按
~~驼峰命名法命名，如lParser~~
ROS建议的方法命名：
所有字母小写，
以下划线分隔，
如l_parser

decision中的包直接按
~~驼峰命名法命名，首字母大写，如AnswerQuestions~~
ROS建议的方法命名，
如answer_questions

##协作模式

依然是Fork+Pull的模式。

代码开发都在dev等分支下进行，
开发好后将dev分支合并到主帐户(tinkerfuroc)的master分支(pull request)。
其他帐户的master分支用于同步主帐户master分支的更改。

临时的修改之类的先放在dev workspace里。


具体可以按照此流程进行：

1. 到 https://github.com/tinkerfuroc/tinker2014 Fork版本库到自己的帐户下
2. git clone
3. 创建并推送自己的dev分支：
  
  ```bash
  cd tinker2014
  git checkout -b dev
  echo -e '[branch "dev"]\n    remote = origin\n    merge = refs/heads/dev' >> .git/config
  git push origin dev
  ```
  - 如果在自己的github帐户上已经有dev分支，则可以这样操作：
  ```bash
  git branch -va    # 检查是否确实有远程dev分支（有时候会不显示，不过没关系）
  git checkout -b dev remotes/origin/dev
  # 这时 .git/config 应该已经被自动修改了
  ```
4. 在dev分支下开发（可以合理使用git的分支机制做一些更复杂的操作）
5. 将自己的dev分支推送到tinkerfuroc下的master分支（pull request）：
  - 到github页面上，切换到dev分支，点创建pull request
  - pull request的路径应是 tinkerfuroc:master  ...  YOUR_NAME:dev
  - 认真填写改动说明后提交
  - tinkerfuroc:master分支更新后，创建一个路径为 YOUR_NAME:master  ...  tinkerfuroc:master 的pull request，并自己合并之
  - 再依据YOUR_NAME:master更新YOUR_NAME:dev，创建pull request或者删了重建都可以
