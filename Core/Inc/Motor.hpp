/*
 * LChika.hpp
 *
 *  Created on: 2022/07/25
 *      Author: leopi
 */

#ifndef INC_MOTOR_HPP_
#define INC_MOTOR_HPP_

//モータークラス

typedef enum {
		ML,
		MR
}MotorName;

class Motor{
//classはデフォルトがprivateらしい. structはpublic
private:
	MotorName name;
	int out;
	float duty_ratio_max;

public:
	Motor(MotorName motor_name);
	void Init();
	inline int GetValue()
	{
		return out;
	}
	inline void SetValue(int value)
	{
		out = value;
	}
	void Output();

};
#endif /* INC_MOTOR_HPP_ */
