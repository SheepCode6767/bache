#ifndef ROBOT_H
#define ROBOT_H

/**
 * @brief 整车初始化,请在开启实时系统之前调用,是main()中唯一需要调用的初始化函数
 */
void RobotInit();

/**
 * @brief 整车任务,把各个应用的任务放在一个任务中运行的接口
 *        (当前freertos.c里给每个应用各创建了一个任务,所以暂时没有用到它)
 */
void RobotTask();

#endif // ROBOT_H
