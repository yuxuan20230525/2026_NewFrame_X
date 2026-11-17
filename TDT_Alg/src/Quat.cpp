#include "Quat.h"



Quaternion Euler2Quat(const EulerAngle ang)
{
   
    Quaternion qx = Quaternion(cos(ang.roll/2),sin(ang.roll/2),0,0);
    Quaternion qy = Quaternion(cos(ang.pitch/2),0,sin(ang.pitch/2),0);
    Quaternion qz = Quaternion(cos(ang.yaw/2),0,0,sin(ang.yaw/2));
    
    return qz*qy*qx;

}
EulerAngle Quat2Euler(const Quaternion q)
{
    EulerAngle euler;

    euler.roll = atan2(2 * (q.getW() * q.getX() + q.getY() * q.getZ()), 1 - 2 * (q.getX() * q.getX() + q.getY() * q.getY()));
    euler.pitch = asin(2 * (q.getW() * q.getY() - q.getZ() * q.getX()));
    euler.yaw = atan2(2 * (q.getW() * q.getZ() + q.getX() * q.getY()), 1 - 2 * (q.getY() * q.getY() + q.getZ() * q.getZ()));

    return euler;
}
Quaternion QuatCoorTrans(Quaternion q, EulerAngle ang)
{
	Quaternion q_trans = Euler2Quat(ang);
	Quaternion _q_trans = ~q_trans;
	q = q_trans*q*_q_trans;
    return q;
}

EulerAngle EulerCoorTrans(EulerAngle ang, EulerAngle coor_ang)
{
    Quaternion q1, q2;
    q1 = Euler2Quat(ang);
    q2 = QuatCoorTrans(q1, coor_ang);
    EulerAngle euler = Quat2Euler(q2);
    return euler;
}