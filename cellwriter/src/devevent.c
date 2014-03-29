
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/input.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include "devevent.h"

struct input_event event_key_start,event_mouse_start, event_end;

#define EV_RELEASED 0
#define EV_PRESSED 1

void select_key(int fd, unsigned short my_key){
		if(my_key == 0x3131)
				event_key_start.code = KEY_R;
		else if(my_key == 0x3134)
				event_key_start.code = KEY_S;
		else if(my_key == 0x3137)
				event_key_start.code = KEY_E;
		else if(my_key == 0x3139)
				event_key_start.code = KEY_F;
		else if(my_key == 0x3141)
				event_key_start.code = KEY_A;
		else if(my_key == 0x3142)
				event_key_start.code = KEY_Q;
		else if(my_key == 0x3145)
				event_key_start.code = KEY_T;
		else if(my_key == 0x3147)
				event_key_start.code = KEY_D;
		else if(my_key == 0x3148)
				event_key_start.code = KEY_W;
		else if(my_key == 0x314A)
				event_key_start.code = KEY_C;
		else if(my_key == 0x314B)
				event_key_start.code = KEY_Z;
		else if(my_key == 0x314C)
				event_key_start.code = KEY_X;
		else if(my_key == 0x314D)
				event_key_start.code = KEY_V;
		else if(my_key == 0x314E)
				event_key_start.code = KEY_G;

		else if(my_key == 0x314F)
				event_key_start.code = KEY_K;
		else if(my_key == 0x3150)
				event_key_start.code = KEY_O;
		else if(my_key == 0x3151)
				event_key_start.code = KEY_I;
		else if(my_key == 0x3153)
				event_key_start.code = KEY_J;
		else if(my_key == 0x3154)
				event_key_start.code = KEY_P;
		else if(my_key == 0x3155)
				event_key_start.code = KEY_U;
		else if(my_key == 0x3157)
				event_key_start.code = KEY_H;
		else if(my_key == 0x315B)
				event_key_start.code = KEY_Y;
		else if(my_key == 0x315C)
				event_key_start.code = KEY_N;
		else if(my_key == 0x3160)
				event_key_start.code = KEY_B;
		else if(my_key == 0x3161)
				event_key_start.code = KEY_M;
		else if(my_key == 0x3163)
				event_key_start.code = KEY_L;

		else if(my_key == 0x3143){
				event_key_start.code=KEY_RIGHTSHIFT;
				write(fd, &event_key_start, sizeof(event_key_start));
				event_key_start.code = KEY_Q;
		}
		else if(my_key == 0x3149){
				event_key_start.code=KEY_RIGHTSHIFT;
				write(fd, &event_key_start, sizeof(event_key_start));
				event_key_start.code = KEY_W;
		}
		else if(my_key == 0x3138){
				event_key_start.code=KEY_RIGHTSHIFT;
				write(fd, &event_key_start, sizeof(event_key_start));
				event_key_start.code = KEY_E;
		}
		else if(my_key == 0x3132){
				event_key_start.code=KEY_RIGHTSHIFT;
				write(fd, &event_key_start, sizeof(event_key_start));
				event_key_start.code = KEY_R;
		}
		else if(my_key == 0x3146){
				event_key_start.code=KEY_RIGHTSHIFT;
				write(fd, &event_key_start, sizeof(event_key_start));
				event_key_start.code = KEY_T;
		}
		else if(my_key == 0x3152){
				event_key_start.code=KEY_RIGHTSHIFT;
				write(fd, &event_key_start, sizeof(event_key_start));
				event_key_start.code = KEY_O;
		}
		else if(my_key == 0x3156){
				event_key_start.code=KEY_RIGHTSHIFT;
				write(fd, &event_key_start, sizeof(event_key_start));
				event_key_start.code = KEY_P;
		}

		move_mouse();
}

void keyboard_event(unsigned short ch)
{
		int fd = open("/dev/input/event3", O_RDWR);
		if(!fd){
				printf("Error open keyboard:\n");
		}
		memset(&event_key_start, 0, sizeof(event_key_start));
		memset(&event_end, 0, sizeof(event_end));
		event_key_start.type = EV_KEY;
		event_end.type = EV_SYN;
		event_end.code = SYN_REPORT;
		event_end.value = 0;

		event_key_start.value = EV_PRESSED;

		select_key(fd,ch);
		write(fd, &event_key_start, sizeof(event_key_start));

		event_key_start.value = EV_RELEASED;
		write(fd, &event_key_start, sizeof(event_key_start));
		event_key_start.code=KEY_RIGHTSHIFT;
		write(fd, &event_key_start, sizeof(event_key_start));
		write(fd, &event_end, sizeof(event_end));

}

void mouse_event(int fd, unsigned short code,int value){

	event_mouse_start.code = code;
	event_mouse_start.value = value;
	write(fd, &event_mouse_start, sizeof(event_mouse_start));// Move the mouse
	write(fd, &event_end, sizeof(event_end));// Show move


}


void move_mouse(){

		int fd = open("/dev/input/event4", O_RDWR);
		memset(&event_mouse_start, 0, sizeof(event_mouse_start));
		memset(&event_end, 0, sizeof(event_end));
		//memset(&event_btn, 0, sizeof(event_btn));

		gettimeofday(&event_mouse_start.time, NULL);
		gettimeofday(&event_end.time, NULL);
		//gettimeofday(&event_btn.time, NULL);

		event_mouse_start.type = EV_REL;
		event_end.type = EV_SYN;
		event_end.code = SYN_REPORT;
		event_end.value = 0;

	   mouse_event(fd,REL_X,-1000);
	   mouse_event(fd,REL_Y,-1000);
	   mouse_event(fd,REL_X,200);
	   mouse_event(fd,REL_Y,200);

}
