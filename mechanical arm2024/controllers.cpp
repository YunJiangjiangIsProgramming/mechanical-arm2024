#define _CRT_SECURE_NO_WARNINGS 1

#include <Wire.h>
#include <MPU6050_tockn.h>
#include <SSD1306Wire.h>
#include <SoftwareSerial.h>
#include <OneButton.h> // 引入 OneButton 库

SoftwareSerial mySerial(20, 21); // RX, TX

MPU6050 mpu6050(Wire);
SSD1306Wire display(0x3c, 5, 4);

unsigned long now, lastTime = 0;
float dt; // 微分时间

const int buttonPin = A3; // 按键连接到 A3 位置
OneButton button(buttonPin, true); // 初始化按键，高电平触发

// 定义卡尔曼滤波参数
struct KalmanFilter {
    float P;
    float R;
    float K;
    float S;
    float V;
    float Q;
};

KalmanFilter kalmanX = { 1, 0, 0, 0, 0, 1 }; // X轴卡尔曼滤波参数
KalmanFilter kalmanZ = { 1, 0, 0, 0, 0, 1 }; // Z轴卡尔曼滤波参数

float prevAgz = 0; // 上一个Z轴角度
float prevAccx = 0; // 上一个X轴加速度
float prevAngleX = 0; // 上一个X轴角度

float buttonState = 0.00; // 初始化按键状态为未按
bool longPressFlag = false; // 长按标志位
bool clickFlag = false; // 单击标志位

void setup() {
    Wire.begin();
    Serial.begin(115200);
    mySerial.begin(9600);

    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);

    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_16);

    // 初始化按键回调函数
    button.attachClick(handleClick);
    button.attachDoubleClick(handleDoubleClick);
    button.attachLongPressStart(handleLongPressStart);
}

void kalmanFilter(float& angle, float& acc, KalmanFilter& kalman) {
    kalman.S = 0;
    kalman.R = acc - kalman.S;
    kalman.K = kalman.P / (kalman.P + kalman.R);
    angle = angle + kalman.K * (acc - angle);
    kalman.P = (1 - kalman.K) * kalman.P;
}

void displayData(float deltaAngleX, float deltaAgz, float accX) {
    display.clear();
    display.drawString(0, 0, String(deltaAngleX));
    display.drawString(0, 16, String(deltaAgz));
    display.drawString(0, 32, String(accX));
    display.display();
}

void sendData(float deltaAngleX, float deltaAgz, float accX) {
    if (!longPressFlag) {
        mySerial.print(deltaAngleX);
        mySerial.print(" ");
        mySerial.print(deltaAgz);
        mySerial.print(" ");
        mySerial.print(accX);
        mySerial.print(" ");
        mySerial.println(buttonState);
    }
}

void calculateAnglesAndAcc() {
    mpu6050.update();

    float accX = mpu6050.getAccX();
    float accZ = mpu6050.getAccZ();
    float accY = mpu6050.getAccY();
    float angleX = mpu6050.getAngleX();
    float angleZ = mpu6050.getAngleZ();
    float angleY = mpu6050.getAngleY();

    kalmanFilter(angleX, accX, kalmanX);
    kalmanFilter(angleZ, accZ, kalmanZ);

    float deltaAgz = angleZ - prevAgz;
    float deltaAccx = accX - prevAccx; // 计算X轴加速度的变化量
    float deltaAngleX = angleX - prevAngleX; // 计算X轴角度的变化量

    prevAgz = angleZ;
    prevAccx = accX;
    prevAngleX = angleX; // 更新上一个X轴角度

    displayData(deltaAngleX, deltaAgz, accX); // 显示变化量
    sendData(deltaAngleX, deltaAgz, accX); // 发送变化量
}

// 按键单击回调函数
void handleClick() {
    if (!clickFlag) {
        buttonState = 1.00; // 第一次单击设置为 1.00
        clickFlag = true; // 设置单击标志位为 true
    }
    else {
        buttonState = 0.00; // 再次单击设置为 0.00
        clickFlag = false; // 重置单击标志位
    }
}

// 按键双击回调函数
void handleDoubleClick() {
    buttonState = 2.00;
    mySerial.println("0.00 0.00 0.00 2.00");
    delay(2000);
    ESP.restart();
}

// 按键长按回调函数
void handleLongPressStart() {
    if (!longPressFlag) {
        longPressFlag = true; // 设置长按标志位为 true
        mySerial.println("0.00 0.00 0.00 3.00");
    }
    else {
        buttonState = 0.00;
        longPressFlag = false; // 重置长按标志位
    }
}

void loop() {
    unsigned long now = millis();
    dt = (now - lastTime) / 1000.0;
    lastTime = now;

    calculateAnglesAndAcc();

    button.tick(); // 检查按键状态

    delay(50);
}
