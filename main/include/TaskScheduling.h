#pragma once
#include "lvgl_ui.h"
#include "lcd.h"
#include "camera.h"
#include "uartp.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>



#ifdef __cplusplus
extern "C" {
#endif

/* 启动任务调度：监听 UI 按钮，控制摄像头→LCD 显示任务的启停 */
void TaskScheduling_Init(void);
void TaskScheduling_PauseForSettings(void);
void TaskScheduling_ResumeAfterSettings(void);
/* Non-blocking: request pause/resume handled by ui_watch task. */
void TaskScheduling_RequestPauseForSettings(void);
void TaskScheduling_RequestResumeAfterSettings(void);
/* 从 UI 确认任务选择时请求立即重启指定任务（即使任务未变） */
void TaskScheduling_RequestRestartSelected(lvgl_ui_task_t task);
/* 在主界面展开任务下拉时，主动停止当前任务，避免画面遮挡 */
void TaskScheduling_StopForDropdown(void);
/* 查询当前运行任务（无任务则返回 LVGL_UI_TASK_NONE） */
lvgl_ui_task_t TaskScheduling_GetRunningTask(void);

#ifdef __cplusplus
}
#endif
