#ifndef NUOGENG_H_
#define NUOGENG_H_
#include<iostream>
#include<ros/ros.h>
#include<cmath>
#include<serial/serial.h>
#include<boost/thread.hpp>
#include<boost/bind.hpp>
#include"gps_msgs/Inspvax.h"
#include<nav_msgs/Odometry.h>
#include"gps_msgs/Satellite.h"
#include"gps_msgs/Satellites.h"
#include <tf2_ros/transform_broadcaster.h>


#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))

const static uint16_t crc16ccitt_false_table[] = {
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a,
	0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
	0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401, 0x64e6,
	0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672,
	0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d,
	0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af,
	0x8948, 0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd,
	0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03,
	0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6, 0x5ed5,
	0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
	0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004,
	0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290,
	0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c,
	0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8,
	0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c,
	0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1,
	0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37,
	0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
	0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b,
	0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0 
};

PACK(
struct pkg20Msgs_t
{
	//数据头格式
	uint8_t lrc;//数据头LRC-用于对数据头进行校验
	uint8_t header0;//数据包ID-0~19表示系统数据包，20~179是状态数据包，180~255是配置数据包
	uint8_t header1;//数据包长度-定义接下来要发送的数据的长度
	uint16_t crc16_ccitt;//CRC16校验-仅对数据内容进行校验

	//系统状态数据包-pkg20
	uint16_t system_status;//系统状态，每一位代表一个状态,0 表示故障,1 表示正常
	uint16_t filter_status;//滤波器状态
	uint32_t time_s;//Unix时间秒
	uint32_t time_ms;//毫秒
	double latitude;//纬度
	double longitude;//经度
	double height;//高度
	float north_velocity;//北向速度
	float east_velocity;//东向速度
	float down_velocity;//地向速度
	float body_acceleration_x;//载体加速度x
	float body_acceleration_y;//载体加速度y
	float body_acceleration_z;//载体加速度z
	float gravitational_acceleration;//重力加速度
	float roll;//横滚
	float pitch;//俯仰
	float yaw;//航向
	float angle_velocity_x;//角速度x
	float angle_velocity_y;//角速度y
	float angle_velocity_z;//角速度z
	float latitude_std_deviation;//纬度标准差
	float longitude_std_deviation;//经度标准差
	float height_std_deviation;//高度标准差
});


PACK(
struct pkg31Msgs_t//卫星详细信息数据包
{
//	uint8_t lrc;
//	uint8_t header0;
//	uint8_t header1;
//	uint16_t crc16_ccitt;
	
	uint8_t navigation_system;//导航系统
	uint8_t satellite_num;//卫星编号
	int8_t  satellite_frequency;//卫星频率
	uint8_t elevation;//仰角
	uint16_t azimuth;//方位角
	uint8_t snr;//信噪比
});

double deg2rad(const double& deg)
{
	return deg*M_PI/180.0;//角度与弧度的转换
}

class Nuogeng
{
public:
	Nuogeng();
	~Nuogeng();
	Nuogeng(const Nuogeng& obj) = delete;
	Nuogeng& operator=(const Nuogeng& obj) = delete;
	bool init();
	void startReading();
	void stopReading();
private:
	bool openSerial(const std::string& port,int baudrate);//baudrate-波特率
	void closeSerial();
	void readSerialThread();
	void parseIncomingData(uint8_t* buffer,size_t len);//size_t一般用来表示一种计数，比如有多少东西被拷贝
	void parseId20Pkg(const uint8_t* buffer);
	void parseId31Pkg(const uint8_t* buffer);
	
	uint8_t LRC(const uint8_t* buf,size_t len)
	{
		uint8_t sum = 0;
		for(size_t i=0; i<len; ++i)
		{
			sum += buf[i];
			//cout << hex << int(buf[i]) << " ";
		}
		return ~sum +1;//～按位取反（按位取反再加1得到补码）
	}
	
	//Crc校验
	uint16_t getCrc16ccittFalseByTable(const uint8_t *buf, int len)
	{
		uint16_t  crc = 0xffff; //规定开始的校验值是0xFFFF
		for (int i = 0; i <len; ++i) 
			crc = (crc<<8) ^ crc16ccitt_false_table[buf[i] ^ (crc>>8)];
		return crc;
	}
	
private:
	ros::Publisher m_pub_id20;//创建发布者对象
	ros::Publisher m_pub_id31;
	ros::Publisher m_pub_ll2utm;
	
	//创建一个serial类的指针
	serial::Serial *m_serial_port;
	//! shared pointer to Boost thread for listening for data from novatel
	boost::shared_ptr<boost::thread> m_read_thread_ptr;
	bool m_reading_status;  //!< True if the read thread is running, false otherwise.
	uint8_t * const m_pkg_buffer;//指针常量
	gps_msgs::Inspvax m_inspax;//创建微发布的数据（gps_msgs功能包下的Inspvax消息类型）
	
	bool m_is_pub_ll2utm;
	bool m_is_pub_tf;

	tf2_ros::TransformBroadcaster m_tf_br;
	std::string m_child_frame_id, m_parent_frame_id;
	
	
};


#endif
