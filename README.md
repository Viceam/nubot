# nubot

 

defend version2:

当球被对方控制并且球位于己方半场时：

​	带球机器人离我方球门最近：

​		当前机器人离球最近

​			带球者越过x = -875：

​				目标点设为ball_pos_.pointofline(opp_pos_, 75.0)

​			没有越过：

​				在x = -1000线上，纵坐标为射门线与 x = -1000的交点

​		其他机器人：

​				执行mark()

​	带球机器人不离我方球门最近：

​		距球最近的机器人 defend1v1()，距防守点最近的机器人 block(), 剩余两个机器人中靠左的机器人 mark(), 靠右的 pre_attack(), 运动到便于接球与进攻的位置

其余情况的策略与2021年相同



2022 7.15 修改：

​	staticpass.cpp, staticpass.h  内实现各种比赛模式下的跑位

​	nubot_control.cpp 中loopControl()函数, position()函数与position()内调用的站位函数

​	void Plan::moveAvoidBall(DPoint *target*) 躲避其他机器人与球进行移动，用于开角球等情况下的跑位

​	defend()函数 将接球部分与防守部分分开放入if else中，避免接球时同时执行防守策略

​	bool Plan::moveBall(DPoint)中处理带球不超过300cm规则

​		
