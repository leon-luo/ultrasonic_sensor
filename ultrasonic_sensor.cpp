/******************************************************************************

  版权所有 (C), 2017-2028 惠州市蓝微电子有限公司

 ******************************************************************************
  文件名称: ultrasonic_sensor.cpp
  版本编号: 初稿
  作     者: Leon
  生成日期: 2017年7月3日
  最近修改:
  功能描述   : 超声波传感器功能定义
  函数列表:
              execute_app
              init_setup
              ultrasonic_sensor.echopin
              ultrasonic_sensor.echo_sensor_distance
              ultrasonic_sensor.get_ultrasonic_sensor_value
              ultrasonic_sensor.initialize
              ultrasonic_sensor.init_ultrasonic_sensor
              ultrasonic_sensor.init_ultrasonic_sensor_pins
              ultrasonic_sensor.trigpin
              ultrasonic_sensor.trig_ultrasonic_sensor
              ultrasonic_sensor.ultrasonic_sensor
  修改历史:
  1.日     期: 2017年7月3日
    作     者: Leon
    修改内容: 创建文件
    s
******************************************************************************
传感器参数说明：
１、本模块性能稳定，测度距离精确。能和国外的SRF05,
SRF02等超声波测距模块相媲美。模块高精度，盲区（2cm）超近,
稳定的测距是此产品成功走向市场的有力根据！此模块完全谦容GH-311防盗模块

2 主要技术参数：
        1：使用电压：DC5V         2：静态电流：小于2mA
        3：电平输出：高5V           4：电平输出：底0V
        5：感应角度：不大于15度       6：探测距离：2cm-450cm

3:高精度：可达0.3cm

 板上接线方式，VCC、trig（控制端）、  echo（接收端）、 out（空脚）、 GND

  注：  
TRIP引脚是内部上拉10K的电阻，用单片机的IO口拉低TRIP引脚，然后给一个10us以上的脉冲信号。
 OUT脚为此模块作为防盗模块时的开关量输出脚，测距模块不用此脚！ 
******************************************************************************

******************************************************************************/

/*******************************************************************************
 * 包含头文件                                                                       *
 *******************************************************************************/
#include "ultrasonic_sensor.h"

#include <ros.h>

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


const int ultrasonic_sensor::trigpin[ultrasonic_sensor::SENSOR_NUM] = {7, 4};
const int ultrasonic_sensor::echopin[ultrasonic_sensor::SENSOR_NUM] = {3, 5};


/*****************************************************************************
 函 数 名: ultrasonic_sensor.ultrasonic_sensor
 功能描述  : 构造函数
 输入参数  : 无
 输出参数: 无
 返 回 值: ultrasonic_sensor
 调用函数:
 被调函数:
 
 修改历史:
  1.日     期: 2017年7月3日
    作     者: Leon
    修改内容: 新生成函数

*****************************************************************************/
ultrasonic_sensor::ultrasonic_sensor()
{

}

/*****************************************************************************
 函 数 名: ultrasonic_sensor.init_ultrasonic_sensor_pins
 功能描述  : 初始化设置超声波传感器使用的引脚
 输入参数: int trig  
           int echo  
 输出参数: 无
 返 回 值: void
 调用函数:
 被调函数:
 
 修改历史:
  1.日     期: 2017年7月3日
    作     者: Leon
    修改内容: 新生成函数

*****************************************************************************/
void ultrasonic_sensor::init_ultrasonic_sensor_pins(int trig, int echo)
{
	pinMode(trig, OUTPUT);
	pinMode(echo, INPUT);
}

/*****************************************************************************
 函 数 名: ultrasonic_sensor.init_ultrasonic_sensor
 功能描述  : 初始化传感器
 输入参数: void  
 输出参数: 无
 返 回 值: void
 调用函数:
 被调函数:
 
 修改历史:
  1.日     期: 2017年7月3日
    作     者: Leon
    修改内容: 新生成函数

*****************************************************************************/
void ultrasonic_sensor::init_ultrasonic_sensor(void)
{
	int index = 0;
#ifdef ENABLE_FRONT_ULTRASONIC_SENSOR
	init_ultrasonic_sensor_pins(trigpin[index], echopin[index]);
#endif

#ifdef ENABLE_BACK_ULTRASONIC_SENSOR
	index = 1;
	init_ultrasonic_sensor_pins(trigpin[index], echopin[index]);
#endif
}

/*****************************************************************************
 函 数 名: ultrasonic_sensor.initialize
 功能描述  : 初始化设置超声波传感器使用的引脚
 输入参数: void  
 输出参数: 无
 返 回 值: void
 调用函数:
 被调函数:
 
 修改历史:
  1.日     期: 2017年7月3日
    作     者: Leon
    修改内容: 新生成函数

*****************************************************************************/
void ultrasonic_sensor::initialize(void)
{
	init_ultrasonic_sensor();
}

/*****************************************************************************
 函 数 名: ultrasonic_sensor.trig_ultrasonic_sensor
 功能描述  : 触发测量
 输入参数: int pin  
 输出参数: 无
 返 回 值: void
 调用函数:
 被调函数:
 
 修改历史:
  1.日     期: 2017年7月3日
    作     者: Leon
    修改内容: 新生成函数
时序图表明你只需要提供一个10uS以上脉冲触发信号，该模块内部将发出8个40kHz周期
电平并检测回波。一旦检测到有回波信号则输出回响信号。回响信号的脉冲宽度与所测
的距离成正比。由此通过发射信号到收到的回响信号时间间隔可以计算得到距离。
公式：uS/58=厘米或者uS/148=英寸；或是：距离=高电平时间*声速（340M/S）/2；
建议测量周期为60ms以上，以防止发射信号对回响信号的影响。
*****************************************************************************/
void ultrasonic_sensor::trig_ultrasonic_sensor(int pin)
{
	digitalWrite(pin, LOW);
	delayMicroseconds(2);
	digitalWrite(pin, HIGH);
	delayMicroseconds(10);
	digitalWrite(pin, LOW);
}

/*****************************************************************************
 函 数 名: ultrasonic_sensor.get_ultrasonic_sensor_value
 功能描述  : 获取超声波检测到的距离
 输入参数: int pin                  
           unsigned long& distance  
 输出参数: 无
 返 回 值: bool
 调用函数:
 被调函数:
 
 修改历史:
  1.日     期: 2017年7月3日
    作     者: Leon
    修改内容: 新生成函数

*****************************************************************************/
bool ultrasonic_sensor::get_ultrasonic_sensor_value(int pin, float& distance)
{
	long ret = false;
	long value = 0;
	unsigned long time_us = 0;
	
	/*
	unsigned long pulseIn (uint8_t pin, uint8_t state, unsigned long timeout)
	读脉冲：
	读引脚的脉冲, 脉冲可以是 HIGH 或 LOW. 如果是 HIGH, 函数将先等引脚变为高电平, 然后 
	开始计时, 一直到变为低电平为止. 返回脉冲持续的时间长短, 单位为微秒.
	如果超时还没有 读到的话, 将返回0.
	参数:
	pin 引脚编号; state 脉冲状态; timeout 超时时间(us)
	pulseIn()单位为微秒，声速340m/s，所以距离cm=340*100/1000000*pulseIn()/2约等于pulseIn()/58.0
	*/
	time_us = pulseIn(pin, HIGH); //存储回波等待时间,
	if ( 0 != time_us )
	{
		distance = float(time_us*17.0/1000);//cm
		ret = true;
		#ifdef DEBUG_ULTRASONIC_SENSOR
		Serial.print("time_us = ");
		Serial.print(time_us);//串口输出等待时间的原始数据
		Serial.print(" us ; Distance = ");
		Serial.print(distance);//串口输出距离换算成cm的结果
		Serial.println(" cm");
		#endif
	}
	else
	{
		#ifdef DEBUG_ULTRASONIC_SENSOR
		Serial.print("time_us = ");
		Serial.print(time_us);
		Serial.println("us. Time out!");
		#endif
	}

	return ret;
}

/*****************************************************************************
 函 数 名: ultrasonic_sensor.get_sensor_distance
 功能描述  : 测量并读取指定超声波传感器的距离
 输入参数: int sensor_index         
           unsigned long& distance  
 输出参数: 无
 返 回 值: bool
 调用函数:
 被调函数:
 
 修改历史:
  1.日     期: 2017年7月3日
    作     者: Leon
    修改内容: 新生成函数

*****************************************************************************/
bool ultrasonic_sensor::get_sensor_distance(int sensor_index, float& distance)
{
	bool ret = false;
	int trig = 0;
	int echo = 0;
	int i = sensor_index;
	int sum = SENSOR_NUM;

	if ((0 > i) || (i >= sum))
	{
		return - 1;
	}

	trig = trigpin[i];
	echo = echopin[i];

	trig_ultrasonic_sensor(trig);
	ret = get_ultrasonic_sensor_value(echo, distance);

	return ret;
}

/*****************************************************************************
 函 数 名: ultrasonic_sensor.get_optimization_distance
 功能描述  : 获取筛选优化的数据
 输入参数: int sensor_index  
           float &distance   
 输出参数: 无
 返 回 值: bool
 调用函数:
 被调函数:
 
 修改历史:
  1.日     期: 2017年7月4日
    作     者: Leon
    修改内容: 新生成函数

*****************************************************************************/
bool ultrasonic_sensor::get_optimization_distance(int sensor_index, float &distance)
{
	bool ret = false;
	int i = 0;
	int real_samples = 0;
	const int valid_samples = 20;
	float totle =  0.0;
	float average =  0.0;
	float curr_value = 0.0;
	const float min = 1.7;  //探测距离：2cm-450cm;      高精度：可达0.3cm
	const float max = 450;

	while ( 1 )
	{
		real_samples ++;
		ret = get_sensor_distance(sensor_index, curr_value);
		if ( ret == false )
		{
			#ifdef DEBUG_ULTRASONIC_SENSOR
			Serial.println("[Errors]get_sensor_distance(...)");
			#endif
			continue;
		}
		if ((min <= curr_value ) && (curr_value <= max ))
		{
			i++;
			totle += curr_value;
		}
		else
		{
			#ifdef DEBUG_ULTRASONIC_SENSOR
			Serial.print("[Errors]invalid sample curr_value = ");
			Serial.print(curr_value);
			Serial.print(" us ; Distance = ");
			Serial.print(distance);//串口输出距离换算成cm的结果
			Serial.println(" cm !");
			#endif
		}

		if (i >= valid_samples)
		{
			average = totle/valid_samples;
			distance = average;
			ret = true;
			#ifdef DEBUG_ULTRASONIC_SENSOR
			Serial.print("[OK]average = totle/valid_samples = ");
			Serial.print(totle);
			Serial.print(" / ");
			Serial.print(valid_samples);
			Serial.print(" = ");
			Serial.println(average);
			#else
			Serial.print(distance);
			Serial.println("cm");
			#endif
			
			break;
		}

		if (real_samples > (valid_samples+5))
		{
			#ifdef DEBUG_ULTRASONIC_SENSOR
			Serial.print("[error] real_samples = ");
			Serial.println(real_samples);
			#endif
			break;
		}
	}

	return ret;
}

