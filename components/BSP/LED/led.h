#ifndef __LED_H_
#define __LED_H_

#include "driver/gpio.h"
#include <stdbool.h>


/* 引脚定义 */

#define BLED_GPIO_PIN    GPIO_NUM_21  /* LED连接的GPIO端口 */

/* 引脚的输出的电平状态 */
enum GPIO_OUTPUT_STATE
{
    PIN_RESET,
    PIN_SET
};

/* LED端口定义 */

#define BLED(x)          do { gpio_set_level(BLED_GPIO_PIN, (x) ? PIN_SET : PIN_RESET); } while(0)
/* LED取反定义 */
#define BLED_TOGGLE()    do { gpio_set_level(BLED_GPIO_PIN, !gpio_get_level(BLED_GPIO_PIN)); } while(0)  /* LED翻转 */

#ifdef __cplusplus
extern "C" {
#endif

/* 函数声明*/
void bled_init(void);    /* 初始化BLED */
void bled_set(bool on);  /* 控制补光灯，true=开，false=关 */
void bled_on(void);
void bled_off(void);


#ifdef __cplusplus
}
#endif

#endif
