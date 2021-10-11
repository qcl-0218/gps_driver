#include<iostream>
#include"nuogeng/nuogeng.h"
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geographic_msgs/GeoPoint.h>
#include <Eigen/Dense>

Nuogeng::Nuogeng():
	m_reading_status(false),
	m_pkg_buffer(new uint8_t[500])
{
}

Nuogeng::~Nuogeng()
{
	this->closeSerial();
	if(m_pkg_buffer!=NULL)
		delete [] m_pkg_buffer;
}

bool Nuogeng::openSerial(const std::string& port,int baudrate)
{
	m_serial_port = new serial::Serial(port,baudrate,serial::Timeout::simpleTimeout(10)); //port-对应的是哪一个端口

	if (!m_serial_port->isOpen())
	{
		std::stringstream output;
        output << "Serial port: " << port << " failed to open." << std::endl;
		delete m_serial_port;
		m_serial_port = NULL;
		return false;
	} 
	else 
	{
		std::stringstream output;
		output << "Serial port: " << port << " opened successfully." << std::endl;
	}

	m_serial_port->flush();
	return true;
}

void Nuogeng::closeSerial()
{
	if(m_serial_port!=NULL)
	{
		delete m_serial_port;
		m_serial_port = NULL;
	}
}

bool Nuogeng::init()
{
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	m_pub_id20 = nh.advertise<gps_msgs::Inspvax>(nh_private.param<std::string>("gps_topic","/gps"),1);//话题名为gps_topic
	m_pub_ll2utm = nh.advertise<nav_msgs::Odometry>(nh_private.param<std::string>("odom_topic","/odom"),1);
	
	//私有节点获取参数                  坐标系参数名           存储变量        默认值
	nh_private.param<std::string>("parent_frame_id", m_parent_frame_id, "world");
	nh_private.param<std::string>("child_frame_id", m_child_frame_id, "gps");
	
	nh_private.param<bool>("pub_odom", m_is_pub_ll2utm, false);
	nh_private.param<bool>("pub_tf", m_is_pub_tf, true);
	
	std::string port_name = nh_private.param<std::string>("port_name","/dev/ttyUSB0");//查询参数名为port_name的值，没有的话返回/dev/ttyUSB0
	int baudrate = nh_private.param<int>("baudrate",115200);//查询参数名为baudrate的值，没有查询到的话返回115200
	if(!openSerial(port_name,baudrate))
		return false;
	return true;
}

void Nuogeng::startReading()
{
	if(m_reading_status)
		return ;
	m_read_thread_ptr = boost::shared_ptr<boost::thread>(new boost::thread(&Nuogeng::readSerialThread,this));
}

void Nuogeng::readSerialThread()
{
	m_reading_status = true;
	
	const int read_size = 200;
	const int buffer_size = read_size * 2;
	const int left_reserve_len = 5;//数据头占用5个字节
	//存储区数据的缓存
	uint8_t * const raw_data_buf = new uint8_t[buffer_size+left_reserve_len];//动态分配整型数组的定义：new int[数组大小]，返回值是指定类型的指针
	//所要读取数据的缓存数组
	uint8_t * const buffer =  raw_data_buf+ left_reserve_len;//指针raw_data_buf向后移动了left_reserve_len个元素，进入pkg20
	
	size_t offset = 0, get_len, total_len;
		   
	while(ros::ok() && m_reading_status)
	{
		try
		{
			get_len = m_serial_port->read(buffer+offset,read_size);//将大小为read_size的数据读到buffer数组中
		}
		catch(std::exception &e)
		{
			ROS_ERROR("Error reading from serial port: %s",e.what());
		}
		
		total_len = offset + get_len;//offset就是上次已读的数据
		if(total_len < read_size)//只有当读取的数据大于200个字节的时候才会提交解析
			offset = total_len;//否则，通过指针偏移接着读取上次未读完的数据
		else
		{
			//cout << total_len << endl;
			parseIncomingData(buffer, total_len);//buffer是指向pkg20首地址的整型数组
			offset = 0;//本次读取的数据完成，置为0
		}
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));//休眠10ms
	}
	
	delete [] raw_data_buf;//手动删除new出来的在堆区的指针
}

void Nuogeng::parseIncomingData(uint8_t* buffer,size_t len)
{
	static size_t pkg_buffer_index = 0; //数据包缓存定位索引，指向m_pkg_buffer的首地址，用来指示m_pkg_buffer已存放数据的个数
	static size_t pkg_len = 0;			//数据包长度，在找到数据头后执行pkg_len = real_buffer[i+2];即可得到数据包长度
	static size_t remainded = 0;		//包头(5bytes)搜索完毕后buffer的剩余长度；remainded <=4；有可能未读取的剩余的数据包含包头
										//防止这一帧数据的丢失，影响稳定性
	//real_buffer指针位于buffer指针之前
	//上次搜索剩余的数据位于buffer之前
	//再次搜索时应从real_buffer开始
	static uint8_t *real_buffer = buffer - remainded;//buffer指针数组前移remained个字节
	
	size_t real_buffer_len = len + remainded;//如果有剩余的数据，下次读入时候buffer的真实长度就等于上次剩余的加上再来的
	remainded = 0; //复位
	for(size_t i=0; i<real_buffer_len; ++i)//i=0表示检索buffer数组的第一个位置
	{
		if(pkg_buffer_index == 0) //还未找到数据头
		{
			//只用来实现找包头，一旦找到，定位索引pkg_buffer_index+1退出当前找包头任务，转入将读到的数据拷贝到m_pkg_buffer
			if(real_buffer_len-i > 4 ) //剩余数据可能包含数据头(剩余bytes>=数据头bytes)
			{
				//real_buffer[i]即为LRC校验值，如果是想要的数据头，满足数据头LRC=LRC(real_buffer+i+1,4),real_buffer+i+1为紧接着LRC的4位
				if(LRC(real_buffer+i+1,4)==real_buffer[i]) //LRC校验查找数据头，前移一位（LRC,数据ID,数据长度，2个字节CRC校验位）
				{
					m_pkg_buffer[pkg_buffer_index++] = real_buffer[i];
					pkg_len = real_buffer[i+2];
					//cout << pkg_len << endl;
				}
			}
			else if(remainded ==0) //剩余数据不足且暂(未开始拼接)
			{
				remainded = real_buffer_len-i; //剩余个数=真实的数据长度-检索过得数据个数，执行一次即可，并让更新到real_buffer_len
				//开始拼接，剩余bytes有序放在buffer前面，且保证数据的连续性
				buffer[i-real_buffer_len] = real_buffer[i];//把剩余的数据放在buffer前面
			}
			else
				buffer[i-real_buffer_len] = real_buffer[i]; //继续拼接
		}
		//数据头已经找到，根据包长逐个拷贝(不包括最后一个字节)
		else if(pkg_buffer_index < pkg_len+4)//未拷贝到最后一位
			m_pkg_buffer[pkg_buffer_index++] = real_buffer[i];
		else//只剩这一帧数据的随后一个字节，拷贝下来表示完成这一帧数据的拷贝，接下来需要尾部crc校验
		{	//拷贝最后一个字节,over
			m_pkg_buffer[pkg_buffer_index] = real_buffer[i];
			pkg_buffer_index = 0; //定位索引复位
			
			if(m_pkg_buffer[3]+m_pkg_buffer[4]*256 != getCrc16ccittFalseByTable(m_pkg_buffer+5,pkg_len))
				continue; //校验失败，结束本次循环，进行下一次循环，而不终止整个循环的执行
				
			if(20==m_pkg_buffer[1]) //数据头ID
				parseId20Pkg(m_pkg_buffer);
			else if(31 == m_pkg_buffer[1])
				;//parseId31Pkg(m_pkg_buffer);
		}
	}
}

void Nuogeng::parseId20Pkg(const uint8_t* buffer)
{
	auto gpsPtr = (const pkg20Msgs_t *)buffer;//将buffer强制转换成数据定义的格式
	
	m_inspax.header.stamp = ros::Time::now();
	m_inspax.header.frame_id = "gps";
	m_inspax.latitude = gpsPtr->latitude *180.0/M_PI;
	m_inspax.longitude = gpsPtr->longitude *180.0/M_PI;
	m_inspax.height = gpsPtr->height;
	m_inspax.north_velocity = gpsPtr->north_velocity;
	m_inspax.east_velocity = gpsPtr->east_velocity;
	m_inspax.up_velocity = gpsPtr->down_velocity;
	m_inspax.roll = gpsPtr->roll;
	m_inspax.pitch = gpsPtr->pitch;
	m_inspax.azimuth = gpsPtr->yaw *180.0/M_PI;
	m_inspax.latitude_standard_deviation = gpsPtr->latitude_std_deviation;
	m_inspax.longitude_standard_deviation = gpsPtr->longitude_std_deviation;
	m_inspax.height_standard_deviation = gpsPtr->height_std_deviation;
	
	m_pub_id20.publish(m_inspax);
	
	geographic_msgs::GeoPoint point;
	point.latitude = m_inspax.latitude;
	point.longitude = m_inspax.longitude;
	point.altitude = m_inspax.height;
	
	geodesy::UTMPoint utm;
	geodesy::fromMsg(point, utm);
	
	Eigen::AngleAxisd xAngle(deg2rad(m_inspax.roll), Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd yAngle(deg2rad(m_inspax.pitch), Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd zAngle(-deg2rad(m_inspax.azimuth-90.0), Eigen::Vector3d::UnitZ());
	
	Eigen::Quaterniond quat = xAngle * yAngle * zAngle;
	quat.normalized();
	
	if(m_is_pub_ll2utm)
	{
		nav_msgs::Odometry odom;
		
		odom.header.stamp = m_inspax.header.stamp;
		odom.header.frame_id = m_parent_frame_id;
		odom.child_frame_id  = m_child_frame_id;
		
		//east-north-sky
		odom.pose.pose.position.x = utm.easting;
		odom.pose.pose.position.y = utm.northing;
		odom.pose.pose.position.z = utm.altitude;
		
		//yaw is the direction from east(x) ccw
		double yaw = 2*M_PI-deg2rad(m_inspax.azimuth-90.0);
		while(yaw > M_PI) yaw -= 2*M_PI;
		while(yaw <-M_PI) yaw += 2*M_PI;
		odom.pose.covariance[0] = yaw;
		odom.pose.covariance[1] = point.longitude;
		odom.pose.covariance[2] = point.latitude;
		
		odom.pose.pose.orientation.x = quat.x();
		odom.pose.pose.orientation.y = quat.y();
		odom.pose.pose.orientation.z = quat.z();
		odom.pose.pose.orientation.w = quat.w();
		
		m_pub_ll2utm.publish(odom);
	}
	
	if(m_is_pub_tf)
	{
		geometry_msgs::TransformStamped transformStamped;
		transformStamped.header.stamp = ros::Time::now();
		transformStamped.header.frame_id = m_parent_frame_id;
		transformStamped.child_frame_id = m_child_frame_id;
		transformStamped.transform.translation.x = utm.easting;
		transformStamped.transform.translation.y = utm.northing;
		transformStamped.transform.translation.z = 0.0;

		transformStamped.transform.rotation.x = quat.x();
		transformStamped.transform.rotation.y = quat.y();
		transformStamped.transform.rotation.z = quat.z();
		transformStamped.transform.rotation.w = quat.w();

		m_tf_br.sendTransform(transformStamped);
	}
	
}

/*
void Nuogeng::parseId31Pkg(const uint8_t* buffer)
{
	int satellite_count = buffer[2]/7; //pkglen/7
	gps_msgs::Satellite satellite;
	gps_msgs::Satellites satellites;
	
	satellites.header.stamp = ros::Time::now();
	satellites.header.frame_id = "gps";
	satellites.satellites.resize(satellite_count);
	satellites.count = satellite_count;
	
	for(int i=0; i<satellite_count; ++i)
	{
		auto satellitePtr = (const pkg31Msgs_t*)(buffer+5+i*7);
		satellite.navigation_system = satellitePtr->navigation_system;
		satellite.satellite_num = satellitePtr->satellite_num;
		satellite.satellite_frequency = satellitePtr->satellite_frequency;
		satellite.elevation = satellitePtr->elevation;
		satellite.azimuth = satellitePtr->azimuth;
		satellite.snr = satellitePtr->snr;
		
		satellites.satellites[i] = satellite;
	}
	m_pub_id31.publish(satellites);
}
*/

void Nuogeng::stopReading()
{
	m_reading_status = false;
}


int main(int argc,char** argv)
{
	ros::init(argc,argv,"nuogeng_node");
	
	Nuogeng gps;
	
	if(gps.init())
	{
		gps.startReading();
		ROS_INFO("%s initial ok...",ros::this_node::getName().c_str());
	}
	
	ros::spin();
}



