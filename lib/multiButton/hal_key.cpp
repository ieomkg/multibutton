#include "hal_key.h"

hal_key hal_multiButton = hal_key();
hal_key *hal_key::_instance = NULL;

hal_key::hal_key()
{
    _instance = this;
}

/// @brief 消抖函数
/// @param btnIndex 按钮索引
/// @param value 目前的按钮电平
/// @return debouncedPinLevel
int hal_key::debounce(uint8_t btnIndex, const int value)
{
    // 记录上一次的按下时间和电平，如果当前时间和上一次的按下时间差值超过阈值，则认为是一次有效的按下
    // 否则认为是抖动，不做处理
    _buttons[btnIndex].now = millis(); // current (relative) time in msecs.
    if (_buttons[btnIndex].lastDebouncePinLevel == value)
    {
        if (_buttons[btnIndex].now - _buttons[btnIndex].lastDebounceTime >= _debounce_ms)
            _buttons[btnIndex].debouncedPinLevel = value;
    }
    else
    {
        _buttons[btnIndex].lastDebounceTime = _buttons[btnIndex].now;
        _buttons[btnIndex].lastDebouncePinLevel = value;
    }
    return _buttons[btnIndex].debouncedPinLevel;
}

int hal_key::debouncedValue(uint8_t btnIndex) const
{
    return _buttons[btnIndex].debouncedPinLevel;
};

void hal_key::_newState(uint8_t btnIndex, stateMachine_t nextState)
{
    _buttons[btnIndex].btnState = nextState;
};

/// @brief 状态机处理（判断按钮按下的状态，并触发单击，双击，长按等事件）
/// @param btnIndex  处理的按钮索引
/// @param activeLevel 代表是否按下或释放
void hal_key::_fsm(uint8_t btnIndex, bool activeLevel)
{
    // 状态流程  OCS_INIT->OCS_DOWN->OCS_UP->OCS_COUNT->OCS_PRESS->OCS_PRESSEND->OCS_INIT

    unsigned long waitTime = (_buttons[btnIndex].now - _buttons[btnIndex].startTime);

    // Implementation of the state machine
    switch (_buttons[btnIndex].btnState)
    {
    case OCS_INIT:
        // waiting for level to become active.
        // 等待按钮被按下，设置状态为OCS_DOWN,并记录开始时间，初始化单击次数
        if (activeLevel)
        {
            _newState(btnIndex, OCS_DOWN);
            _buttons[btnIndex].startTime = _buttons[btnIndex].now; // remember starting time
            _buttons[btnIndex].nClicks = 0;
        } // if
        break;
    case OCS_DOWN:
        // waiting for level to become inactive.
        // 原来是按下，现在松开，设置状态为OCS_UP,并记录开始时间
        if (!activeLevel)
        {
            _newState(btnIndex, OCS_UP);
            _buttons[btnIndex].startTime = _buttons[btnIndex].now; // remember starting time
        }
        else if ((activeLevel) && (waitTime > _press_ms))
        {
            // 原来是按下，现在还是按下，并且超过 长按 时间，设置状态为OCS_LONGPRESS ，调用长按回调函数
            if (_buttons[btnIndex].clickEventCallback)
            {
                if (_buttons[btnIndex].btnType == 0)
                    _buttons[btnIndex].clickEventCallback(_buttons[btnIndex].pin, KEYEVENT_TYPE::LONGPRESS);
                else if (_buttons[btnIndex].btnType == 2) // 矩阵按键触发不同的事件 ,valuid 代表行与列的组合值
                    _buttons[btnIndex].clickEventCallback(_buttons[btnIndex].rowid << 4 | _buttons[btnIndex].colid, KEYEVENT_TYPE::MATRIX_LONGPRESS);
            }
            _newState(btnIndex, OCS_PRESS);
        } // if
        break;

    case OCS_UP:
        // level is inactive
        // count as a short button down
        // 扫描到按钮从按下到松开，设置状态为OCS_COUNT, 并记录 按下次数
        _buttons[btnIndex].nClicks++;
        _newState(btnIndex, OCS_COUNT);
        break;

    case OCS_COUNT:
        // dobounce time is over, count clicks
        // 按下一次，并释放后，进行按键次数计数
        if (activeLevel)
        {
            // button is down again
            // 再次按下按钮
            _newState(btnIndex, OCS_DOWN);
            _buttons[btnIndex].startTime = _buttons[btnIndex].now; // remember starting time
        }
        else if ((waitTime >= _click_ms) || (_buttons[btnIndex].nClicks == 2))
        {
            // now we know how many clicks have been made.
            // 超过 单击 时间，或者 按下两次，触发单击或双击，并还原按钮状态

            if (_buttons[btnIndex].nClicks == 1)
            {
                // this was 1 click only.
                if (_buttons[btnIndex].clickEventCallback)
                {
                    if (_buttons[btnIndex].btnType == 0)
                        _buttons[btnIndex].clickEventCallback(_buttons[btnIndex].pin, KEYEVENT_TYPE::CLICK);
                    else if (_buttons[btnIndex].btnType == 2) // 矩阵按键触发不同的事件 ,valuid 代表行与列的组合值
                        _buttons[btnIndex].clickEventCallback(_buttons[btnIndex].rowid << 4 | _buttons[btnIndex].colid, KEYEVENT_TYPE::MATRIX_CLICK);
                }
            }
            else if (_buttons[btnIndex].nClicks == 2)
            {
                // this was a 2 click sequence.
                if (_buttons[btnIndex].clickEventCallback)
                {
                    if (_buttons[btnIndex].btnType == 0)
                        _buttons[btnIndex].clickEventCallback(_buttons[btnIndex].pin, KEYEVENT_TYPE::DOUBLECLICK);
                    else if (_buttons[btnIndex].btnType == 2) // 矩阵按键触发不同的事件 ,valuid 代表行与列的组合值
                        _buttons[btnIndex].clickEventCallback(_buttons[btnIndex].rowid << 4 | _buttons[btnIndex].colid, KEYEVENT_TYPE::MATRIX_DOUBLECLICK);
                }

            } // if

            reset(btnIndex);
        } // if
        break;

    case OCS_PRESS:
        // waiting for pin being release after long press.
        // 长按后，等待按钮被释放，设置状态为OCS_PRESSEND,并记录开始时间
        if (!activeLevel)
        {
            _newState(btnIndex, OCS_PRESSEND);
            _buttons[btnIndex].startTime = _buttons[btnIndex].now;
        } // if
        break;

    case OCS_PRESSEND:
        // button was released.
        // 还原按钮状态
        reset(btnIndex);
        break;

    default:
        // unknown state detected -> reset state machine
        _newState(btnIndex, OCS_INIT);
        break;
    } // if
    return;
}

/// @brief 循环对所有按钮进行状态机处理
/// @param
void hal_key::tick(void)
{
    for (uint8_t i = 0; i < _buttons.size(); i++)
    {
        if (_buttons[i].pin >= 0 && _buttons[i].btnType == 0)
        {
            // 普通按键处理规则
            // buttonPressed 代表 判断按下的电平是高还是低 0 代表低电平 1 代表高电平
            // debounce 代表 去抖动函数，返回去抖动后的电平值 0 代表低电平 1 代表高电平
            // _fsm 处理 按钮按下的状态（按下或释放），并触发单击，双击，长按等事件
            _fsm(i, debounce(i, digitalRead(_buttons[i].pin)) == _buttons[i].buttonPressed);
        }
        else if (_buttons[i].pin >= 0 && _buttons[i].btnType == 1 && _buttons[i].encoderdirection != -1)
        {
            // 编码器处理规则不一样
            // 通过中断模式处理
            if (_buttons[i].encoderdirection > 0 && _buttons[i].clickEventCallback)
            {
                _buttons[i].clickEventCallback(_buttons[i].pin, KEYEVENT_TYPE::ENCODER_LEFT);
            }
            else
            {
                _buttons[i].clickEventCallback(_buttons[i].pin, KEYEVENT_TYPE::ENCODER_RIGHT);
            }
            _buttons[i].encoderdirection = -1; // 重置编码器方向
        }
        else if (_buttons[i].pin >= 0 && _buttons[i].btnType == 2)
        {
            // 矩阵按键处理规则、 每行置低电压
            for (uint8_t row = 0; row < MAX_MATRIX_PINS; row++)
                digitalWrite(_buttons[i].rowsPin[row], LOW);

            // 当前引脚高电平
            digitalWrite(_buttons[i].pin, HIGH);

            // 读列引脚的电平
            _fsm(i, debounce(i, digitalRead(_buttons[i].assistPin)) == _buttons[i].buttonPressed);

            // Serial.println("btnIndex:" + String(i) + " rowid = " + String(_buttons[i].rowid) + " colid = " + String(_buttons[i].colid));

            // return;
        }
    }
} // tick()

/// @brief 重置按钮的状态
/// @param btnIndex
void hal_key::reset(uint8_t btnIndex)
{
    _buttons[btnIndex].btnState = OCS_INIT;
    _buttons[btnIndex].nClicks = 0;
    _buttons[btnIndex].startTime = 0;
}

/// @brief
/// @param pin 按键引起，ID，公用
/// @param activeLow 是否低电平触发
/// @param pullupActive 是否上升沿触发
/// @param clickEventCallback 带参按键事件回调函数，案例：keyevent(uint8_t keyPin, uint8_t eventType)
void hal_key::addButton(int8_t pin, bool activeLow, bool pullupActive, btnCbFunctionParameter clickEventCallback)
{
    this->addButton(pin, -1, activeLow, pullupActive, clickEventCallback);
}

/// @brief
/// @param pin 按键引起，ID，公用
/// @param assistPin 辅助引脚，用于编辑器模式
/// @param activeLow 是否低电平触发
/// @param pullupActive 是否上升沿触发
/// @param clickEventCallback 带参按键事件回调函数，案例：keyevent(uint8_t keyPin, uint8_t eventType)
void hal_key::addButton(int8_t pin, int8_t assistPin, bool activeLow, bool pullupActive, btnCbFunctionParameter clickEventCallback)
{
    if (pin <= 0)
        return;
    stru_button btn;
    btn.pin = pin;
    btn.assistPin = assistPin;
    if (assistPin >= 0)
        btn.btnType = 1; // 2: assist button
    btn.buttonPressed = activeLow ? LOW : HIGH;
    btn.pullupActive = pullupActive;
    btn.clickEventCallback = clickEventCallback;

    _buttons.push_back(btn);

    // 引脚设置
    if (btn.btnType == 0)
    {
        if (pullupActive)
            pinMode(pin, INPUT_PULLUP);
        else
            pinMode(pin, INPUT);
    }
    else if (btn.btnType == 1)
    {
        // 通过 中断模式处理
        pinMode(btn.pin, INPUT);       // PIN_AIO
        pinMode(btn.assistPin, INPUT); // PIN_BIO
        attachInterrupt(digitalPinToInterrupt(btn.pin), encoderParse_Inter, CHANGE);
    }

    Serial.println("buttons:" + String(_buttons.size()));
}

/// @brief 矩阵按键初始化
/// @param rowsPin
/// @param colsPin
/// @param activeLow
/// @param pullupActive
/// @param clickEventCallback
void hal_key::addButton(uint8_t rowsPin[MAX_MATRIX_PINS], uint8_t colsPin[MAX_MATRIX_PINS], bool activeLow, bool pullupActive, btnCbFunctionParameter clickEventCallback)
{

    for (uint8_t i = 0; i < MAX_MATRIX_PINS; i++)
    {
        for (uint8_t j = 0; j < MAX_MATRIX_PINS; j++)
        {
            stru_button btn = stru_button();
            btn.pin = rowsPin[i];       // 行引脚
            btn.assistPin = colsPin[j]; // 列引脚
            btn.btnType = 2;            // 矩阵按键
            btn.buttonPressed = HIGH;   // 矩阵按键默认高电平触发
            btn.pullupActive = pullupActive;
            btn.clickEventCallback = clickEventCallback;

            if (pullupActive)
            {
                pinMode(btn.pin, OUTPUT);
                pinMode(btn.assistPin, INPUT_PULLDOWN); // INPUT，INPUT_PULL 都不好使
            }
            else
            {
                pinMode(btn.pin, OUTPUT);
                pinMode(btn.assistPin, INPUT);
            }
            std::memcpy(btn.rowsPin, rowsPin, MAX_MATRIX_PINS);

            btn.rowid = i;
            btn.colid = j;

            _buttons.push_back(btn);

            // Serial.println("buttons: " + String(btn.pin) + " : " + String(btn.assistPin));
        }
    }

    // Serial.println("buttons total:" + String(_buttons.size()));
}

std::vector<hal_key::stru_button> *hal_key::getButtons()
{
    return &_buttons;
}

/// @brief 编码器中断处理函数,测试好用
/// @return
void IRAM_ATTR encoderParse_Inter()
{
    std::vector<hal_key::stru_button> *btns = hal_multiButton.getButtons();

    for (uint8_t btnInx = 0; btnInx < btns->size(); btnInx++)
    {
        if ((*btns)[btnInx].pin >= 0 && (*btns)[btnInx].btnType == 1)
        {
            int alv, blv;
            alv = digitalRead((*btns)[btnInx].pin);       // PIN_AIO
            blv = digitalRead((*btns)[btnInx].assistPin); // PIN_BIO
            // Serial.println("alv = " + String(alv) + " blv = " + String(blv));
            if (!(*btns)[btnInx].encoderflag && alv == LOW)
            {
                (*btns)[btnInx].encoderCW1 = blv;
                (*btns)[btnInx].encoderflag = true;
            }
            if ((*btns)[btnInx].encoderflag && alv)
            {
                (*btns)[btnInx].encoderCW2 = !blv;
                if ((*btns)[btnInx].encoderCW1 && (*btns)[btnInx].encoderCW2)
                {
                    (*btns)[btnInx].encoderdirection = 1; // 设置编码器方向 0
                }
                if ((*btns)[btnInx].encoderCW1 == false && (*btns)[btnInx].encoderCW2 == false)
                {
                    (*btns)[btnInx].encoderdirection = 0; // 设置编码器方向 0
                }
                (*btns)[btnInx].encoderflag = false;
            }
        }
    }
}
