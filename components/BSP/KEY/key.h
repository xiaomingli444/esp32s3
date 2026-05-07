#ifndef __KEY_H_
#define __KEY_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"


/* 引脚定义 */
// #define KEY1_PIN   GPIO_NUM_47
// #define KEY2_PIN   GPIO_NUM_38
// #define KEY3_PIN   GPIO_NUM_46


// #define KEY1_PIN   GPIO_NUM_46
// #define KEY2_PIN   GPIO_NUM_38
// #define KEY3_PIN   GPIO_NUM_47

#define KEY1_PIN   GPIO_NUM_38
#define KEY2_PIN   GPIO_NUM_46
#define KEY3_PIN   GPIO_NUM_47

/*IO操作*/
#define KEY1            gpio_get_level(KEY1_PIN)
#define KEY2            gpio_get_level(KEY2_PIN)
#define KEY3            gpio_get_level(KEY3_PIN)

/* 按键按下定义 */
#define KEY1_PRES       1       /* BOOT按键按下 */
#define KEY2_PRES       2       /* BOOT按键按下 */
#define KEY3_PRES       3       /* BOOT按键按下 */


#ifdef __cplusplus
extern "C" {
#endif


/* 函数声明 */
void key_init(void);            /* 初始化按键 */
uint8_t key_scan(uint8_t mode); /* 按键扫描函数 */

#ifdef __cplusplus
}
#endif

#endif