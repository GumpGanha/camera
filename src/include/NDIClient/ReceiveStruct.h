#pragma once

#pragma pack (1)

struct  XS_ReceiveStruct {
    short			 m_head;						//头，666为正确
    unsigned char	 m_programStatus;				//程序是否执行完毕
    unsigned char	 m_targetStatus;				//接收到的点是否可达
    unsigned char	 m_torqueStatus;				//扭矩是否超出上限
    unsigned char	 m_robotStatus;					//机器人当前状态
    short			 m_sendStatus;					//
    double			 m_endPosition[12];				//机器人模型XYZABC   A1-A6
    int				 m_sensorForce6[6];				//六维力传感器值
    double			 m_driverTorque6[6];			//驱动器扭矩反馈
    short			 m_driverAlarm6[6];				//驱动器报警
    int				 m_tail;						//尾，888为正确
};