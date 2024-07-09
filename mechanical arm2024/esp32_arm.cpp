#define _CRT_SECURE_NO_WARNINGS 1

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

#define SERVOMIN  150 // ��С���峤��
#define SERVOMAX  600 // ������峤��

// ����Ƕȱ�������ʼΪ90��
float servoAngles[5] = { 90.0, 90.0, 90.0, 90.0, 90.0 };

// ����ID
const BaseType_t servoCore = 0;
const BaseType_t displayCore = 1;

// ������
TaskHandle_t Task1;
TaskHandle_t Task2;

void setup() {
    Serial.begin(9600); // ��ʼ������0
    Serial2.begin(9600); // ��ʼ������2

    pwm.begin();
    pwm.setPWMFreq(60);

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.display();

    delay(10);

    // ��ʼ�������90��
    for (int i = 0; i < 4; i++) {
        setServoAngle(i, 90.0);
    }
    setServoAngle(4, 90.0);
    display.clearDisplay();
    display.display();

    // ��������
    xTaskCreatePinnedToCore(servoControlTask, "ServoControl", 10000, NULL, 1, &Task1, servoCore);
    xTaskCreatePinnedToCore(displayTask, "DisplayUpdate", 10000, NULL, 1, &Task2, displayCore);
}

void loop() {
    // ��ѭ�����У������ڸ��Ժ�����ִ��
    vTaskDelay(1 / portTICK_PERIOD_MS);
}

void servoControlTask(void* pvParameters) {
    (void)pvParameters;
    char buffer[50];

    while (1) {
        if (servoAngles[4] != 3.00) {
            if (Serial2.available() > 0) {
                String data = Serial2.readStringUntil('\n');

                data.toCharArray(buffer, 50);
                Serial.println("Data received on Serial2: " + String(buffer)); // ��ӡ����2���յ�������

                processSerialData(buffer);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void processSerialData(char* buffer) {
    char* token = strtok(buffer, " ");
    if (token != NULL) {
        float angleChange1 = atof(token);
        applyFilterAndSetAngle(2, angleChange1);

        token = strtok(NULL, " ");
        if (token != NULL) {
            float angleChange2 = atof(token);
            applyIncrementalAngle(0, angleChange2);

            token = strtok(NULL, " ");
            if (token != NULL) {
                // �ճ�����λ����

                token = strtok(NULL, " ");
                if (token != NULL) {
                    float action = atof(token);

                    if (action == 0.00) {
                        // ���ֵ�ǰ״̬�������κβ���
                        setServoAngle(4, 90.0);
                    }
                    else if (action == 1.00) {
                        // ���״̬ת��Ϊ45��
                        setServoAngle(4, 45.0);
                    }
                    else if (action == 2.00) {
                        // ��������״̬��Ϊ1.00
                        if (action != 1.00) {
                            ESP.restart();
                            action = 1.00; // ��״̬��Ϊ1.00
                        }
                    }
                    else if (action == 3.00) {
                        // ���״̬�趨Ϊ1�Ŷ��180�ȣ�2�Ŷ��90�ȣ�3�Ŷ��180�ȣ�4�Ŷ��90�ȣ�5�Ŷ��45��
                        setServoAngle(0, 0.0);
                        setServoAngle(1, 0.0);
                        setServoAngle(2, 180.0);
                        setServoAngle(3, 90.0);
                        setServoAngle(4, 45.0);
                    }
                }
            }
        }
    }
}

void toggleServo5() {
    if (servoAngles[4] == 90.0) {
        setServoAngle(4, 45.0);
    }
    else {
        setServoAngle(4, 90.0);
    }
}


void displayTask(void* pvParameters) {
    (void)pvParameters;
    char buffer[50];

    while (1) {
        if (Serial2.available() > 0) {
            String data = Serial2.readStringUntil('\n');
            data.toCharArray(buffer, 50);
            updateDisplay(buffer);
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

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

void applyFilterAndSetAngle(int servoIndex, float angleChange) {
    float targetAngle = 90.0 + angleChange;
    targetAngle = constrain(targetAngle, 0.0, 180.0);

    float currentAngle = servoAngles[servoIndex];
    while (abs(currentAngle - targetAngle) > 1.0) {
        currentAngle += (targetAngle - currentAngle) * 0.1;
        setServoAngle(servoIndex, currentAngle);
        delay(5); // ����ƽ���ٶ�
    }
    setServoAngle(servoIndex, targetAngle);
}

void applyIncrementalAngle(int servoIndex, float angleChange) {
    float targetAngle = servoAngles[servoIndex] + angleChange;
    targetAngle = constrain(targetAngle, 0.0, 180.0);

    setServoAngle(servoIndex, targetAngle);
}