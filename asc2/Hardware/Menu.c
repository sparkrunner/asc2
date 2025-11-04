#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "Encoder.h"
#include "Key.h"
#include <string.h>
#include <stdio.h>

#define NORMAL_MODE 0
#define CHANGE_MODE 1

typedef struct Menu {
    char *name;
    struct Menu **son;
    int sonCount;
    struct Menu *father;
    int index;
    int fixed;
	int step;
	int limit;
} Menu;

Menu main_Menu, LED_Control, Image, PID, LED_Speed, LED_Direction, Angle, Set_KP, Set_KI, Set_KD;
Menu *cur;
Menu *main_Menu_sons[] = {&LED_Control, &PID, &Image, &Angle};
Menu *LED_Control_sons[] = {&LED_Speed, &LED_Direction};
Menu *PID_sons[] = {&Set_KP, &Set_KI, &Set_KD};
Menu *Image_sons[] = {&Image};
Menu *Angle_sons[] = {&Angle};

char *Mode_Name[] = {" ", "E"};
int modeIndex;

void Menu_Init() {
    main_Menu = (Menu) {
        "Main Menu",
        main_Menu_sons,
        4,
        NULL,
        0,
        0,
		0,
		0
    };
    LED_Control = (Menu) {
        "LED Control",
        LED_Control_sons,
        2,
        &main_Menu,
        0,
        0,
		0,
		0
    };
    Image = (Menu) {
        "Image",
        Image_sons,
        1,
        &main_Menu,
        0,
        0,
		0,
		0
    };
    PID = (Menu) {
        "PID",
        PID_sons,
        3,
        &main_Menu,
        0,
        0,
		0,
		0
    };
    LED_Speed = (Menu) {
        "LED_speed",
        NULL,
        0,
        &LED_Control,
        0,
        1,
		10,
		30
    };
    LED_Direction = (Menu) {
        "LED_dir",
        NULL,
        0,
        &LED_Control,
        0,
        1,
		10,
		20
    };
    Angle = (Menu) {
        "Angle",
        Angle_sons,
        1,
        &main_Menu,
        0,
        0,
		0,
		0
    };
    Set_KP = (Menu) {
        "kp",
        NULL,
        0,
        &PID,
        0,
        1,
		1,
		0
    };
    Set_KI = (Menu) {
        "ki",
        NULL,
        0,
        &PID,
        0,
        1,
		1,
		0
    };
    Set_KD = (Menu) {
        "kd",
        NULL,
        0,
        &PID,
        0,
        1,
		1,
		0
    };
    cur = &main_Menu;
}

int Numlen(int Num) {
	if (Num == 0) return 1;
	int res = 0;
	while (Num > 0) {
		Num /= 10;
		res++;
	}
	return res;
}

void Menu_Show() {
	int top;
    if (!strcmp(cur->name, "Main Menu")) {
		top = 1;
    } else {
		top = 2;
		OLED_ShowString(1, 1, cur->name);
	}
	if (modeIndex == CHANGE_MODE)
	{
		OLED_ShowChar(1, 15, 'E');
	}
    for (int i = 0; i < cur->sonCount; i++) {
		Menu* p = cur->son[i];
        if (i == cur->index) {
            OLED_ShowChar(top + i, 1, '>');
        } else {
            OLED_ShowChar(top + i, 1, ' ');
        }
        OLED_ShowString(top + i, 2, p->name);
        if (p->fixed) {
            if (cur->son[i]->step == 10) {
				int len = Numlen(p->index / 10);
				OLED_ShowNum(top + i, 16 - len, p->index / 10, len);
			} else {
				int len = Numlen(p->index) + 1 + (p->index < 10) + 2 * (p->index < 0);
				OLED_ShowFloatNum(top + i, 16 - len, (float)p->index / 10, 1);
			}
        }
    }
}

uint8_t Menu_Mode(void)
{
	return modeIndex;
}

int Menu_LED_Speed(void)
{
	return LED_Speed.index / LED_Speed.step;
}

int Menu_LED_Direction(void)
{
	return LED_Direction.index / LED_Direction.step;
}

void Menu_Option(uint32_t opt)
{
	if (opt != 0)
	{
		OLED_Clear();
	}
	Menu* p = cur->son[cur->index];
    switch (opt) {
		case 1:
			if (modeIndex == NORMAL_MODE) {
				if (cur->sonCount > 0) {
					cur->index--;
					if (cur->index < 0) {
						cur->index = cur->sonCount - 1;
					}
				}
			} else {
				p->index += p->step;
				if (p->limit > 0) {
					if (p->index >= p->limit) {
						p->index = 0;
					}
				}
			}
			break;
		case 2:
			if (modeIndex == NORMAL_MODE) {
				cur->index++;
				if (cur->index >= cur->sonCount) {
					cur->index = 0;
				}
			} else {
				p->index -= p->step;
				if (p->index < 0) {
					if (p->limit > 0) {
						p->index = p->limit - p->step;
					}
				}
			}
			break;
		case 3:
			if (modeIndex == CHANGE_MODE) {
				modeIndex = NORMAL_MODE;
			} else if (p->fixed) {
				modeIndex = CHANGE_MODE;
			} else if (cur->sonCount > 0) {
				cur = p;
			}
			break;
		case 4:
			if (modeIndex == CHANGE_MODE) {
				modeIndex = NORMAL_MODE;
			} else if (cur->father != NULL) {
				cur = cur->father;
			}
			break;
	}
	if (modeIndex == CHANGE_MODE && !strcmp(cur->name, "PID")) {
		int16_t EncoderNum = Encoder_Get();
		if (EncoderNum) {
			p->index += EncoderNum;
			OLED_Clear();
			Menu_Show();
		}
		if (Key_Check(0, KEY_REPEAT)) {
			p->index++;
			OLED_Clear();
			Menu_Show();
		}
		if (Key_Check(1, KEY_REPEAT)) {
			p->index--;
			OLED_Clear();
			Menu_Show();
		}
	}
}
