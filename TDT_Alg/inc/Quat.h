#ifndef _QUAT_H
#define _QUAT_H


#include "board.h"


/**
 * @class Quaternion
 * @brief The Quaternion class encapsulates the functionality related to quaternions.
 * 
 * It stores the components of a quaternion (a scalar part and three imaginary parts)
 * and provides methods to perform operations on quaternions such as multiplication,
 * addition, finding the conjugate, and calculating the norm.
 */
class Quaternion {
public:
    /**
     * @brief Default constructor.
     * 
     * Initializes the quaternion with all components set to 0.
     */
    Quaternion() : w(1), x(0), y(0), z(0) {}

    /**
     * @brief Parameterized constructor.
     * 
     * @param w_ The scalar part (real part) of the quaternion.
     * @param x_ The coefficient of the imaginary unit i.
     * @param y_ The coefficient of the imaginary unit j.
     * @param z_ The coefficient of the imaginary unit k.
     * 
     * Initializes the quaternion with the given values for its components.
     */
    Quaternion(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}

    /**
     * @brief Get the scalar part (real part) of the quaternion.
     * 
     * @return The value of the scalar part (real part) of the quaternion.
     */
    double getW() const { return w; }

    /**
     * @brief Get the coefficient of the imaginary unit i.
     * 
     * @return The value of the coefficient of the imaginary unit i.
     */
    double getX() const { return x; }

    /**
     * @brief Get the coefficient of the imaginary unit j.
     * 
     * @return The value of the coefficient of the imaginary unit j.
     */
    double getY() const { return y; }

    /**
     * @brief Get the coefficient of the imaginary unit k.
     * 
     * @return The value of the coefficient of the imaginary unit k.
     */
    double getZ() const { return z; }

    /**
     * @brief Overload the multiplication operator for quaternion multiplication.
     * 
     * @param other The other quaternion to multiply with.
     * @return The result of the multiplication of this quaternion with the given quaternion.
     * 
     * Performs the multiplication of two quaternions according to the rules of quaternion arithmetic.
     * The resulting quaternion represents the combined rotation if quaternions are used to represent rotations.
     */
    Quaternion operator*(const Quaternion& other) const {
        double new_w = w * other.w - x * other.x - y * other.y - z * other.z;
        double new_x = w * other.x + x * other.w + y * other.z - z * other.y;
        double new_y = w * other.y - x * other.z + y * other.w + z * other.x;
        double new_z = w * other.z + x * other.y - y * other.x + z * other.w;
		double norm = sqrt(new_w*new_w + new_x*new_x + new_y*new_y + new_z*new_z);
        return Quaternion(new_w/norm, new_x/norm, new_y/norm, new_z/norm);
    }
	
	
	Quaternion operator~ () const {
		double norm = sqrt(w*w + x*x + y*y + z*z);
		double new_w = w/norm;
		double new_x = -x/norm;
		double new_y = -y/norm;
		double new_z = -z/norm;
		return Quaternion(new_w,new_x,new_y,new_z);
	}
	
	
	Quaternion& operator*=(const Quaternion& other) {
        *this = *this * other;  
        return *this; 
    }

    /**
     * @brief Overload the addition operator for quaternion addition.
     * 
     * @param other The other quaternion to add.
     * @return The result of the addition of this quaternion with the given quaternion.
     * 
     * Performs the addition of two quaternions by adding their corresponding components.
     */
    Quaternion operator+(const Quaternion& other) const {
        return Quaternion(w + other.w, x + other.x, y + other.y, z + other.z);
    }

    /**
     * @brief Calculate the conjugate of the quaternion.
     * 
     * @return The conjugate of the quaternion.
     * 
     * The conjugate of a quaternion has the same scalar part and the opposite of each imaginary part.
     * It is useful in various operations involving quaternions, such as division and normalization.
     */
    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }

    /**
     * @brief Calculate the norm (magnitude) of the quaternion.
     * 
     * @return The norm (magnitude) of the quaternion.
     * 
     * The norm of a quaternion is calculated using the formula: sqrt(w * w + x * x + y * y + z * z).
     * It is used to normalize quaternions and check their magnitudes in different applications.
     */
    double norm() const {
        return std::sqrt(w * w + x * x + y * y + z * z);
    }

private:
    double w;  /**< The scalar part (real part) of the quaternion. */
    double x;  /**< The coefficient of the imaginary unit i. */
    double y;  /**< The coefficient of the imaginary unit j. */
    double z;  /**< The coefficient of the imaginary unit k. */
};


typedef struct 
{
	float yaw;
	float pitch;
	float roll;
}EulerAngle;

//Euler :x-y-z
Quaternion Euler2Quat(const EulerAngle ang);
EulerAngle Quat2Euler(const Quaternion q);
Quaternion QuatCoorTrans(Quaternion q, EulerAngle ang);
EulerAngle EulerCoorTrans(EulerAngle ang, EulerAngle coor_ang);



#endif