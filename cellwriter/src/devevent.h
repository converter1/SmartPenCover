
//#include <gtk/gtk.h>
//extern struct input_event event_key_start,event_mouse_start, event_end;
#define EV_RELEASED 0
#define EV_PRESSED 1

#define KEYBOARD 0
#define MOUSE 1
void select_key(int fd, unsigned short my_key);
void keyboard_event(unsigned short ch);
void mouse_event(int fd,unsigned short code,int value);
void move_mouse();
