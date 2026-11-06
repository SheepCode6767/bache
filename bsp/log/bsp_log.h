#ifndef _BSP_LOG_H
#define _BSP_LOG_H

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"
#include <stdio.h>

#define BUFFER_INDEX 0

/**
 * @brief 日志系统初始化
 *
 */
void BSPLogInit();

/**
 * @brief 日志功能原型,供下面的LOGI,LOGW,LOGE等使用
 *
 */
/* 颜色复位序列直接拼进格式串,末尾**不留 %s 参数** —— 这是防弹衣,不是美学问题:
 *   SEGGER RTT 的 printf 只认 c/d/u/x/X/s/p/% 这几个格式符(见 SEGGER_RTT_printf.c),
 *   碰到它不认识的(最典型的就是 %f,以及写错的格式符)它**不会消耗对应的参数**,
 *   于是后面所有参数整体错位一格:原来末尾那个 %s(颜色复位)会拿一个错位的值当 char* 去解引用。
 *   实测后果:0.1 秒内 HardFault,PC 停在 SEGGER_RTT_printf.c 的 "%s" 循环里
 *   (c = *s; 那条 ldrb),CFSR 是精确总线错误,BFAR 是个几 GB 的野地址 —— 很难往"日志格式"
 *   上想。所以:
 *     ① 复位序列改成拼在格式串里(下面这样),错位也不会再变成"解引用野指针";
 *     ② 要打浮点,用 Float2Str() 转成字符串再 %s,不要用 %f(本宏的调用方也遵守这条)。
 *   %p 只有 8 位宽、且不做 0x 前缀之外的处理,%lx/%lu 里的 l 只是被跳过(不影响参数数量)。
 */
#define LOG_PROTO(type, color, format, ...)                                 \
        SEGGER_RTT_printf(BUFFER_INDEX, "  %s%s" format RTT_CTRL_RESET "\r\n", \
                          color,                                            \
                          type,                                             \
                          ##__VA_ARGS__)

/*----------------------------------------下面是日志输出的接口-------------------------------------------------*/

/* 清屏 */
#define LOG_CLEAR() SEGGER_RTT_WriteString(0, "  " RTT_CTRL_CLEAR)

/* 无颜色日志输出 */
#define LOG(format, ...) LOG_PROTO("", "", format, ##__VA_ARGS__)

/**
 *  有颜色格式日志输出,建议使用这些宏来输出日志
 *  @attention 注意这些接口不支持浮点格式化输出,若有需要,请使用Float2Str()函数进行转换后再打印
 *  @note 在release版本上车使用时,与makefile中添加的宏DISABLE_LOG_SYSTEM一起使用,可以关闭日志系统
 */
#if DISABLE_LOG_SYSTEM
#define LOGINFO(format, ...) 
#define LOGWARNING(format, ...) 
#define LOGERROR(format, ...) 
#else
// information level
#define LOGINFO(format, ...) LOG_PROTO("I:", RTT_CTRL_TEXT_BRIGHT_GREEN, format, ##__VA_ARGS__)
// warning level
#define LOGWARNING(format, ...) LOG_PROTO("W:", RTT_CTRL_TEXT_BRIGHT_YELLOW, format, ##__VA_ARGS__)
// error level
#define LOGERROR(format, ...) LOG_PROTO("E:", RTT_CTRL_TEXT_BRIGHT_RED, format, ##__VA_ARGS__)
#endif //  DISABLE_LOG_SYSTEM

/**
 * @brief 通过segger RTT打印日志,支持格式化输出,格式化输出的实现参考printf.
 * @attention !! 此函数不支持浮点格式化,若有需要,请使用Float2Str()函数进行转换后再打印 !!
 *
 * @param fmt 格式字符串
 * @param ... 参数列表
 * @return int 打印的log字符数
 */
int PrintLog(const char *fmt, ...);

/**
 * @brief 利用sprintf(),将float转换为字符串进行打印
 * @attention 浮点数需要转换为字符串后才能通过RTT打印
 *
 * @param str 转换后的字符串
 * @param va 待转换的float
 */
void Float2Str(char *str, float va);

#endif
