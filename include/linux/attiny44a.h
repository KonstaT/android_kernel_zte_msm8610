//[ECID:000000] ZTEBSP wanghaifei start 20111226, for attiny44a
#ifndef __ATTINY44A_H
#define __ATTINY44A_H

#define ATTINY_NAME "attiny44a"
struct attiny_platform_data {
	void (*enable)(int);
	int (*get_irq_level)(void);
	int irq;
};

#endif
//[ECID:000000] ZTEBSP wanghaifei end 20111226, for attiny44a
