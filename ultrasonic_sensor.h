/******************************************************************************

  版权所有 (C), 2017-2028 惠州市蓝微电子有限公司

 ******************************************************************************
  文件名称: ultrasonic_sensor.h
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年7月3日
  最近修改:
  功能描述   : 超声波传感器类声明
  函数列表:
  修改历史:
  1.日     期: 2017年7月3日
    作     者: Leon
    修改内容: 创建文件

******************************************************************************/
#ifndef __ULTRASONIC_SENSOR_H__
#define __ULTRASONIC_SENSOR_H__

/*******************************************************************************
 * 包含头文件                                                                       *
 *******************************************************************************/

/*******************************************************************************
 * 外部变量说明                                                                      *
 *******************************************************************************/

/*******************************************************************************
 * 外部函数原型说明                                                                    *
 *******************************************************************************/

/*******************************************************************************
 * 内部函数原型说明                                                                    *
 *******************************************************************************/

/*******************************************************************************
 * 全局变量                                                                        *
 *******************************************************************************/

/*******************************************************************************
 * 模块级变量                                                                       *
 *******************************************************************************/

/*******************************************************************************
 * 常量定义                                                                        *
 *******************************************************************************/

/*******************************************************************************
 * 宏定义                                                                         *
 *******************************************************************************/
#define DEBUG_ULTRASONIC_SENSOR

#define ENABLE_FRONT_ULTRASONIC_SENSOR     //启动前方的超生波传感器
//#define ENABLE_BACK_ULTRASONIC_SENSOR      //启动后方的超生波传感器

using namespace std;

class ultrasonic_sensor
{
public:
	ultrasonic_sensor();

	void initialize(void);
	bool get_sensor_distance(int sensor_index, float& distance);
	bool get_optimization_distance(int sensor_index, float &distance);
	
private:
	void trig_ultrasonic_sensor(int pin);
	bool get_ultrasonic_sensor_value(int pin, float& distance);
	void init_ultrasonic_sensor_pins(int trig, int echo);
	void init_ultrasonic_sensor(void);
	
private:
	static const int SENSOR_NUM = 2;
	
	static const int trigpin[SENSOR_NUM];
	static const int echopin[SENSOR_NUM];
};

#endif /* __ULTRASONIC_SENSOR_H__ */

