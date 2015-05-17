##d_firmware 命令发送package
###node: fw_send_service.py
将与固件路由器的通信封装为ros service，**所有**的发给固件的命令都应该先发给这个service
####service: SendCommand
#####Service定义
参见/src/fw_send/srv/SendCommand.srv
传入: `string command`
返回: `string echo` 
#####运行方式
```bash
rosrun fw_send fw_send_service.py
```
#####Node名
send_command
#####
说明
先到先得的独占式发送，使用线程加锁完成
###node: fw_send_client_demo.py
fw_send_service的使用样例，在其他节点中需要调用fw_send_service可以用相似的方式完成
####运行方式
```bash
rosrun fw_send fw_send_client_demo.py yourCommand
```

