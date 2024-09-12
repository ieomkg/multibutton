#include <Arduino.h>
#include <hal_key.h>
#include <U8g2lib.h>
#include <Wire.h>

U8G2_SH1107_PIMORONI_128X128_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

#define oled_sda 27
#define oled_scl 26
#define beepPin 14

void beep();

// 4x4 距阵键盘
uint8_t rowspin[4] = {15, 2, 4, 16};  // 行引脚配置
uint8_t colspin[4] = {17, 5, 18, 19}; // 列引脚配置
char keyChar[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}};

// u8g2 打印信息
void printInfo(std::string _msg)
{
  static std::vector<std::string> _infoCache = {};
  static const uint8_t _max = 128 / 12;
  static const uint8_t _fontHeight = 12;

  if (_infoCache.size() >= _max)
    _infoCache.clear();
  _infoCache.push_back(_msg);

  u8g2.clearBuffer();
  u8g2.setDrawColor(1); // 反色显示
  for (uint8_t i = 0; i < _infoCache.size(); i++)
  {
    u8g2.drawStr(0, _fontHeight + i * (1 + _fontHeight), _infoCache[i].c_str());
  }
  u8g2.sendBuffer();
  u8g2.setDrawColor(0); // 反色显示 // 回归实色显示
}

/// @brief 按键事件处理
/// @param keyPin
/// @param eventtype
void keyEvent(uint8_t keyPin, uint8_t eventtype)
{
  if (eventtype == 5 || eventtype == 6 || eventtype == 7) // 按下
  {
    // 矩阵键盘特殊处理。转换为行与列索引
    int col = (keyPin >> 4) & 0xF;
    int row = keyPin & 0xF;

    Serial.println("keyEvent,key:" + String(keyChar[row][col]) + ",eventtype: " + String(eventtype));
    printInfo("key: " + std::string(1, keyChar[row][col]) + " , event:" + std::to_string(eventtype));
  }
  else
  {
    // 其他按键处理（旋转编码器、中间按键等）
    Serial.println("keyEvent,key:" + String(keyPin) + ",eventtype:" + String(eventtype));
    printInfo("key: " + std::to_string(keyPin) + " , event: " + std::to_string(eventtype));
  }
  beep();
}

void oled_init()
{
  Wire.begin(oled_sda, oled_scl);
  u8g2.setBusClock(1000000); // 硬件IIC接口使用
  u8g2.begin();
  u8g2.setDisplayRotation(U8G2_R0);
  u8g2.setContrast(10);
  u8g2.enableUTF8Print();
  u8g2.setFont(u8g2_font_wqy12_t_chinese1);
  u8g2.drawUTF8(0, 64, "Hello World! 世界你好!");
  u8g2.drawStr(0, 12, "Top Left");
  u8g2.drawStr(93, 12, "Right");
  u8g2.drawStr(0, 128, "Bottom Left");
  u8g2.drawStr(93, 128, "Right");
  u8g2.sendBuffer();
}

void setup()
{

  Serial.begin(115200);

  // 以下代码增加两个旋转编码器，并带中间的按压按键
  hal_multiButton.addButton(22, 23, true, false, keyEvent);
  hal_multiButton.addButton(21, true, false, keyEvent);
  hal_multiButton.addButton(32, 33, true, false, keyEvent);
  hal_multiButton.addButton(25, true, false, keyEvent);
  // 以下代码增加一个4x4 距阵键盘
  hal_multiButton.addButton(rowspin, colspin, true, true, keyEvent);

  oled_init();
  pinMode(beepPin, OUTPUT);
}

void beep()
{
  uint8_t channel = 0;
  int freq = 2000;     // 设置频率
  int resolution = 8;  // 计数位数，2的8次幂=256
  int dutyCycle = 125; // 占空比，0~255
  pinMode(beepPin, OUTPUT);
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(beepPin, channel);
  ledcWriteTone(channel, freq);  // 设置频率 值越大频率越高，声音越尖
  ledcWrite(channel, dutyCycle); // 设置占空比, 0~255 值越大响度越大
  delayMicroseconds(2000);
  ledcDetachPin(beepPin);
}

void loop()
{
  hal_multiButton.tick();
}
