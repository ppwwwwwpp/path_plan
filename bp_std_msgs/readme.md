弓形线和沿墙任务显示：
安卓下发给机器人话题/bp_android_to_nav_alongwallbow,类型bp_std_msgs/AlongWallBowTask.msg
机器人反馈给安卓话题/bp_nav_to_android_alongwallbow,类型bp_std_msgs/AlongWallBow.msg


开启沿墙任务话题
topic name = /start_alongwall_task，msg type = StartAlongWallOrBow.msg
开启弓字型任务话题
topic name = /start_bow_task，msg type = StartAlongWallOrBow.msg


任务操作话题，暂停，恢复，停止
topic name = /task_option，msg type = TaskOption.msg

定点导航话题
topic name = /simple_pose_nav type = StartSimplePointNav.msg