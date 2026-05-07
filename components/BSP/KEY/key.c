#include "key.h"

/**
 * @brief       初始化按键
 * @param       无
 * @retval      无
 */
void key_init(void)
{
    gpio_config_t gpio_init_struct;

    gpio_init_struct.intr_type = GPIO_INTR_DISABLE;         /* 失能引脚中断 */
    gpio_init_struct.mode = GPIO_MODE_INPUT;                /* 输入模式 */
    gpio_init_struct.pull_up_en = GPIO_PULLUP_ENABLE;       /* 使能上拉 */
    gpio_init_struct.pull_down_en = GPIO_PULLDOWN_DISABLE;  /* 失能下拉 */
    gpio_init_struct.pin_bit_mask = (1ull << KEY1_PIN) |
									(1ull << KEY2_PIN) |
									(1ull << KEY3_PIN);  /* BOOT按键引脚 */

    gpio_config(&gpio_init_struct);                         /* 配置使能 */
}

uint8_t key_scan(uint8_t mode)
{
	static uint8_t  key_up = 1;
	
	if(mode)			//mode==1时,支持连按
		key_up = 1;
	
	if( key_up&&(KEY1==0||KEY2==0||KEY3==0))
	{
		vTaskDelay(10);             /* 去抖动 */
		key_up = 0;
		if(KEY1==0)		return KEY1_PRES;
		if(KEY2==0)		return KEY2_PRES;
		if(KEY3==0)		return KEY3_PRES;
	}else if(KEY1==1&&KEY2==1&&KEY3==1)
		key_up = 1;
	return 0;
}


