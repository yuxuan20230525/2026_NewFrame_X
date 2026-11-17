#ifndef __FF_TEST_TASK_H
#define __FF_TEST_TASK_H

#include "task_virtual.h"
#include "board.h"
#include "DjiMotor.h"

class FFTestTask : public VirtualTask
{
public:
	float test_a;

	PidParam testPidInnerParam;
	PidParam testPidOuterParam;
	LK_Motor testMotor = LK_Motor(MHF7015, CAN1, 0x140);

	void init() override;
	void run() override;
};

#endif
