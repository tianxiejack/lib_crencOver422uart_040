#ifndef _GPIO_RDWR_H_
#define _GPIO_RDWR_H_

//#ifdef __cplusplus
//extern "C"{
//#endif

#define GPIO_DIRECTION_OUT	(1)
#define GPIO_DIRECTION_IN	(0)

int GPIO_create(unsigned int num, unsigned int mode);
int GPIO_close(unsigned int num);
int GPIO_get(unsigned int nPin);
int GPIO_set(unsigned int nPin, unsigned int nValue);

//#ifdef __cplusplus
//}
//#endif

#endif
