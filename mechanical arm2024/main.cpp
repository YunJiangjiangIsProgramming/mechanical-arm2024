#define _CRT_SECURE_NO_WARNINGS 1

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // 最小脉冲长度
#define SERVOMAX  600 // 最大脉冲长度

float servoAngles[5] = { 90.0, 90.0, 90.0, 90.0, 90.0 };

// 核心ID
const BaseType_t servoCore = 0;
const BaseType_t displayCore = 1;

TaskHandle_t Task1;
TaskHandle_t Task2;

void updateDisplay(char* buffer) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Received data:");
    Serial.println(buffer);
    display.println(buffer);
    display.display();
}

void setServoAngle(int servoIndex, float angle) {
    servoAngles[servoIndex] = constrain(angle, 0.0, 180.0);
    int pulse = map((int)angle, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(servoIndex, 0, pulse);
}

void processSerialData(char* buffer) {
    char* token = strtok(buffer, " ");
    if (token != NULL) {
        float angleChange1 = atof(token);
        if ((angleChange1 >= 0.5 && angleChange1 <= 10) || (angleChange1 <= -0.5 && angleChange1 >= -10)) {
            setServoAngle(2, servoAngles[2] + angleChange1);
        }


        token = strtok(NULL, " ");
        if (token != NULL) {
            float angleChange2 = atof(token);
            if ((angleChange2 >= 0.5 && angleChange2 <= 10) || (angleChange2 <= -0.5 && angleChange2 >= -10)) {
                setServoAngle(0, servoAngles[0] + angleChange2 * 0.9);
            }
        }
        token = strtok(NULL, " ");
        if (token != NULL) {
            float angleChange3 = atof(token);
            if (angleChange3 < -0.7) {
                setServoAngle(1, servoAngles[1] + 2);
                setServoAngle(2, servoAngles[2] - 2);
                setServoAngle(3, servoAngles[3] + 1);
            }
            else if (angleChange3 > 0.7) {
                setServoAngle(1, servoAngles[1] - 2);
                setServoAngle(2, servoAngles[2] + 2);
                setServoAngle(3, servoAngles[3] - 1);
            }
        }
        token = strtok(NULL, " ");
        if (token != NULL) {
            float action = atof(token);

            if (action == 1.00) {
                setServoAngle(4, 45.0);
            }
            else if (action == 0.00) {
                setServoAngle(4, 90.0);
            }
            else if (action == 2.00) {
                ESP.restart();
            }
            else if (action == 3.00) {
                setServoAngle(0, 0.0);
                setServoAngle(1, 0.0);
                setServoAngle(2, 180.0);
                setServoAngle(3, 90.0);
                setServoAngle(4, 45.0);
            }
        }
    }
}


// 关节控制任务
void servoControlTask(void* pvParameters) {
    (void)pvParameters;
    char buffer[50];

    while (1) {
        if (Serial2.available() > 0) {
            String data = Serial2.readStringUntil('\n');

            data.toCharArray(buffer, 50);
            Serial.println("Data received on Serial2: " + String(buffer));
            processSerialData(buffer);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
// 显示任务
void displayTask(void* pvParameters) {
    (void)pvParameters;
    char buffer[50];

    while (1) {
        if (Serial2.available() > 0) {
            String data = Serial2.readStringUntil('\n');

            data.toCharArray(buffer, 50);
            updateDisplay(buffer);
        }
        vTaskDelay(60 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(9600);
    Serial2.begin(9600);

    pwm.begin();
    pwm.setPWMFreq(60);

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.display();

    for (int i = 0; i < 5; i++) {
        setServoAngle(i, 90.0); // 初始化舵机角度为90度
    }
    // 创建并启动任务
    xTaskCreatePinnedToCore(servoControlTask, "ServoControl", 10000, NULL, 1, &Task1, servoCore);
    xTaskCreatePinnedToCore(displayTask, "DisplayUpdate", 10000, NULL, 1, &Task2, displayCore);
}

void loop() {
    vTaskDelay(1 / portTICK_PERIOD_MS);
}