#pragma once
#ifndef HAL_MULTIBUTTON_H
#define HAL_MULTIBUTTON_H

#include <Arduino.h>
#include <vector>
#include <cstring>

#define MAX_MATRIX_PINS 4 // 最大支持矩阵按键数量

extern "C"
{
    typedef void (*btnCbFunction)(void);
    typedef void (*btnCbFunctionParameter)(uint8_t, uint8_t);
}

/// @brief 按钮事件类型
enum KEYEVENT_TYPE : int
{
    CLICK = 0,              // 单击
    DOUBLECLICK = 1,        // 双击
    LONGPRESS = 2,          // 长按
    ENCODER_RIGHT = 3,      // 右转
    ENCODER_LEFT = 4,       // 左转
    MATRIX_CLICK = 5,       // 矩阵按键按下
    MATRIX_DOUBLECLICK = 6, // 矩阵按键按下
    MATRIX_LONGPRESS = 7    // 矩阵按键长按
};

class hal_key
{
public:
    // define FiniteStateMachine 定义状态机
    enum stateMachine_t : int
    {
        OCS_INIT = 0,     // 按钮初始状态
        OCS_DOWN = 1,     // 按下按钮
        OCS_UP = 2,       // 松开按钮
        OCS_COUNT = 3,    // 按下并松开，开始计数
        OCS_PRESS = 6,    // 长按按钮
        OCS_PRESSEND = 7, // 释放按钮
    };

    // 定义按钮结构体，用于存储按钮相关信息
    struct stru_button
    {
        int8_t pin = -1;                                  // 引脚号
        int8_t assistPin = -1;                            // 辅助引脚号,用于编码按键的判断
        uint8_t buttonPressed = 0;                        // 低电平有效，触发模式
        bool pullupActive;                                // 上拉电阻使能
        uint8_t btnType = 0;                              // 按钮类型，0-普通按键，1-编码按键
        stateMachine_t btnState = OCS_INIT;               // 状态机,当前按钮状态
        int debouncedPinLevel = -1;                       // 防抖后的引脚电平
        int lastPinLevel = -1;                            // used for detecting button up/down
        int lastDebouncePinLevel = -1;                    // used for pin debouncing
        unsigned long lastDebounceTime = 0;               // millis()
        unsigned long now = 0;                            // millis()
        unsigned long startTime = 0;                      // start of current input change to checking debouncing
        int nClicks = 0;                                  // 单次点击次数，用于判断是否是双击
        bool encoderflag = false;                         // 编码器状态
        int8_t encoderdirection = -1;                     // 编码器方向 1 left 0 right -1 none
        int encoderCW1 = 0;                               // 编码器电平
        int encoderCW2 = 0;                               // 编码器电平
        btnCbFunctionParameter clickEventCallback = NULL; // 带参回调函数
        uint8_t rowsPin[MAX_MATRIX_PINS] = {0, 0, 0, 0};  // 按键矩阵行引脚
        uint8_t rowid = 0;                                // 按键矩阵行号
        uint8_t colid = 0;                                // 按键矩阵列号
    };

    static hal_key *_instance;
    hal_key();
    ~hal_key() = default;
    void tick(void);
    // 添加普通按键引脚 activeLow 是否低电平有效，pullupActive 是否上拉电阻使能，clickEventCallback 单击回调函数
    void addButton(int8_t pin, bool activeLow = true, bool pullupActive = false, btnCbFunctionParameter clickEventCallback = NULL);
    // 添加编码按键引脚，pin=编码器A引脚，assistPin=编码器B引脚，内部通过中断方式判断编码方向,activeLow 是否低电平有效，pullupActive 是否上拉电阻使能，clickEventCallback 单击回调函数
    void addButton(int8_t pin = -1, int8_t assistPin = -1, bool activeLow = true, bool pullupActive = false, btnCbFunctionParameter clickEventCallback = NULL);
    // 添加矩阵按键引脚，只能添加一次  rowsPin 行引脚数组，colsPin 列引脚数组 ,activeLow 是否低电平有效(固定为False)，pullupActive 是否上拉电阻使能，clickEventCallback 单击回调函数
    void addButton(uint8_t rowsPin[MAX_MATRIX_PINS], uint8_t colsPin[MAX_MATRIX_PINS], bool activeLow = true, bool pullupActive = false, btnCbFunctionParameter clickEventCallback = NULL);
    std::vector<stru_button> *getButtons();

private:
    unsigned int _debounce_ms = 25; // 消抖时间，单位毫秒
    unsigned int _click_ms = 85;    // 单击时间，单位毫秒 (ms)
    unsigned int _press_ms = 500;   // 长按时间，单位毫秒 (ms)

    std::vector<stru_button> _buttons; // 按钮数组

    /// @brief 消抖函数
    /// @param value
    /// @return 消抖后的电平
    int debounce(uint8_t btnIndex, const int value);
    int debouncedValue(uint8_t btnIndex) const;

    void _newState(uint8_t btnIndex, stateMachine_t nextState);
    void _fsm(uint8_t btnIndex, bool activeLevel);
    void reset(uint8_t btnIndex);
};

extern hal_key hal_multiButton;
void IRAM_ATTR encoderParse_Inter(); // 编码器 中断函数

#endif // HAL_MULTIBUTTON_H