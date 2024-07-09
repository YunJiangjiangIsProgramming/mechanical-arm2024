#define _CRT_SECURE_NO_WARNINGS 1

#include <Wire.h>
#include <MPU6050_tockn.h>
#include <SSD1306Wire.h>
#include <SoftwareSerial.h>
#include <OneButton.h> // ���� OneButton ��

SoftwareSerial mySerial(20, 21); // RX, TX

MPU6050 mpu6050(Wire);
SSD1306Wire display(0x3c, 5, 4);

unsigned long now, lastTime = 0;
float dt; // ΢��ʱ��

const int buttonPin = A3; // �������ӵ� A3 λ��
OneButton button(buttonPin, true); // ��ʼ���������ߵ�ƽ����

// ���忨�����˲�����
struct KalmanFilter {
    float P;
    float R;
    float K;
    float S;
    float V;
    float Q;
};

KalmanFilter kalmanX = { 1, 0, 0, 0, 0, 1 }; // X�Ῠ�����˲�����
KalmanFilter kalmanZ = { 1, 0, 0, 0, 0, 1 }; // Z�Ῠ�����˲�����

float prevAgz = 0; // ��һ��Z��Ƕ�
float prevAccx = 0; // ��һ��X����ٶ�
float prevAngleX = 0; // ��һ��X��Ƕ�

float buttonState = 0.00; // ��ʼ������״̬Ϊδ��
bool longPressFlag = false; // ������־λ
bool clickFlag = false; // ������־λ

void setup() {
    Wire.begin();
    Serial.begin(115200);
    mySerial.begin(9600);

    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);

    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_16);

    // ��ʼ�������ص�����
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
    float deltaAccx = accX - prevAccx; // ����X����ٶȵı仯��
    float deltaAngleX = angleX - prevAngleX; // ����X��Ƕȵı仯��

    prevAgz = angleZ;
    prevAccx = accX;
    prevAngleX = angleX; // ������һ��X��Ƕ�

    displayData(deltaAngleX, deltaAgz, accX); // ��ʾ�仯��
    sendData(deltaAngleX, deltaAgz, accX); // ���ͱ仯��
}

// ���������ص�����
void handleClick() {
    if (!clickFlag) {
        buttonState = 1.00; // ��һ�ε�������Ϊ 1.00
        clickFlag = true; // ���õ�����־λΪ true
    }
    else {
        buttonState = 0.00; // �ٴε�������Ϊ 0.00
        clickFlag = false; // ���õ�����־λ
    }
}

// ����˫���ص�����
void handleDoubleClick() {
    buttonState = 2.00;
    mySerial.println("0.00 0.00 0.00 2.00");
    delay(2000);
    ESP.restart();
}

// ���������ص�����
void handleLongPressStart() {
    if (!longPressFlag) {
        longPressFlag = true; // ���ó�����־λΪ true
        mySerial.println("0.00 0.00 0.00 3.00");
    }
    else {
        buttonState = 0.00;
        longPressFlag = false; // ���ó�����־λ
    }
}

void loop() {
    unsigned long now = millis();
    dt = (now - lastTime) / 1000.0;
    lastTime = now;

    calculateAnglesAndAcc();

    button.tick(); // ��鰴��״̬

    delay(50);
}
