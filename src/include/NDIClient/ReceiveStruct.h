#pragma once

#pragma pack (1)

struct  XS_ReceiveStruct {
    short			 m_head;						//ͷ��666Ϊ��ȷ
    unsigned char	 m_programStatus;				//�����Ƿ�ִ�����
    unsigned char	 m_targetStatus;				//���յ��ĵ��Ƿ�ɴ�
    unsigned char	 m_torqueStatus;				//Ť���Ƿ񳬳�����
    unsigned char	 m_robotStatus;					//�����˵�ǰ״̬
    short			 m_sendStatus;					//
    double			 m_endPosition[12];				//������ģ��XYZABC   A1-A6
    int				 m_sensorForce6[6];				//��ά��������ֵ
    double			 m_driverTorque6[6];			//������Ť�ط���
    short			 m_driverAlarm6[6];				//����������
    int				 m_tail;						//β��888Ϊ��ȷ
};