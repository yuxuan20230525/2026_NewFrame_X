#ifndef __VOFA__
#define __VOFA__


#include "board.h"
#include "vision.h"
struct Frame {
    float fdata[6];
    unsigned char tail[4]{0x00, 0x00, 0x80, 0x7f};
};


class VOFA
{
public:
	void init();
	void run();
	void DMA_Send(uint8_t *data, uint16_t size);
	Frame sendpack={0};
	void printf(float value,u8 id);  //×î¶à0-5
	struct SendMessage
	{
		uint8_t *data;
		uint16_t size;
	};
	myqueue<SendMessage> sendPackage;
};

extern VOFA vofa;







#endif