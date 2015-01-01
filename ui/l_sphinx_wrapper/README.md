##l_sphinx_wrapper的附加脚本


###init.py

初始化一个任务文件夹

####使用方法

```bash
./init.py --init TASK_NAME
```

这条命令会创建$TINKER_WORKSPACE/share/l_sphinx_wrapper/launches/TASK_NAME/，
其中有文件 sent.txt, gram.jsgf, TASK_NAME.launch 。

sent.txt 和 gram.jsgf 的编辑方法：
- 编辑sent.txt，写入可能出现的词语，如
  ```
  hello tinker
  my name is
  alex
  bob
  ```

- 然后编辑gram.jsgf，写入语法，注意单词必须是大写，如
  ```
  #JSGF v1.0;
  grammar furoc;
  public <furocCmd> = <myname> | <hellotinker>;
  <myname> = MY NAME IS <names>;
  <names> = TOM | BOB | JACK;
  <hellotinker> = HELLO <names>;
  ```

编辑完成后，生成 .dict, .fsg 文件：

```bash
./init.py -t TASK_NAME
```

这条命令等价于

```bash
python input2dict.py -t TASK_NAME
python jsgf2fsg.py -t TASK_NAME
```

之后便可以正常使用：

```bash
roslaunch l_sphinx_wrapper TASK_NAME.launch
```

------

###toUpper.py

将文件(例如gram.jsgf)中的字母转换为大写

####使用方法

```bash
python toUpper.py -i ../../share/l_sphinx_wrapper/answer_questions/gram.jsgf
```

