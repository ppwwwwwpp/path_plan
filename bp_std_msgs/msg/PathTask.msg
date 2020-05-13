Header header
uint8 path_task_id  #,0表示停止测试,1表示添加路线, 2表示测试路线
uint8 path_id    #路线id号，0表示不测试任何路线，1表示测试路线1，2表示测试路线2，等等
geometry_msgs/PoseArray path   #路线的pose
