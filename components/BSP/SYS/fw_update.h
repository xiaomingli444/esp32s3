#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* STA 模式进入主界面后触发一次检查（满足条件才会真正发起网络请求） */
void fw_update_notify_main_screen_entered(void);

/* UI 回调：用户在“固件升级”弹窗中选择 升级/拒绝 */
void fw_update_user_decide(bool accept_upgrade);

/* OTA 拉取升级期间用于互斥（避免与其它 OTA 流程并发） */
bool fw_update_is_busy(void);

#ifdef __cplusplus
}
#endif

