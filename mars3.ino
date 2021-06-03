#define MOTOR_MAX 255  // максимальный сигнал на мотор (max 255)
#define JOY_MAX 40    // рабочий ход джойстика (из приложения)

#define IN1 24
#define IN2 6        // IN2 обязательно должен быть ШИМ пином!!!
#define IN3 28
#define IN4 3      // IN4 обязательно должен быть ШИМ пином!!!

//#define ENA 4
//#define ENB 5
 
#define BT_TX 13
#define BT_RX 12

#include <Servo.h>

Servo myservo1;
Servo myservo2;
/*
  Bluetooth шлёт пакет вида $valueX valueY;
  моя функция parsing разбивает этот пакет в массив intData
  Парсинг полностью прозрачный, не блокирующий и с защитой от битого пакета,
  так как присутствует строгий синтаксис посылки. Без хешсуммы конечно, но и так норм
*/
#define PARSE_AMOUNT 2         // число значений в массиве, который хотим получить

int intData[PARSE_AMOUNT];     // массив численных значений после парсинга
boolean recievedFlag;
int dutyR, dutyL;
int signalX, signalY;
int dataX, dataY;

#include "GyverMotor.h"
GMotor motorR(IN1, IN2);
GMotor motorL(IN3, IN4);

#include <SoftwareSerial.h>
SoftwareSerial btSerial(12, 13); // TX, RX

void setup() {
  Serial.begin(9600);
  btSerial.begin(9600);
  //PWMfrequency(IN2, 1);   // 31 кГц
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  //pinMode(ENA, OUTPUT);
  //pinMode(ENB, OUTPUT);
  
  myservo1.attach(9);
  myservo2.attach(10);
}

void loop() {
  parsing();              // функция парсинга
  if (recievedFlag) {     // если получены данные
    recievedFlag = false;
    dataX = intData[0];
    dataY = intData[1];
    for (byte i = 0; i < PARSE_AMOUNT; i++) { // выводим элементы массива
      Serial.print(intData[i]); Serial.print(" ");
      } Serial.println();
    Serial.print(dutyR);
    Serial.print(" ");
    Serial.println(dutyL);
  }

  if (dataX == 0 && dataY == 0) {   // если мы в "мёртвой" зоне
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);          // не двигаемся;
    dutyR = 0;
    dutyL = 90;
  } else {
    signalY = map((dataY), -JOY_MAX, JOY_MAX, -MOTOR_MAX, MOTOR_MAX);         // сигнал по У
    signalX = map((dataX), -JOY_MAX, JOY_MAX, 0, 180); // сигнал по Х

    dutyR = signalY;
    dutyL = signalX;

    if (dutyR > 0){ 
      motorR.setMode(FORWARD);
      motorL.setMode(FORWARD);
    }
    else{ 
      motorR.setMode(BACKWARD);
      motorL.setMode(BACKWARD);
    }
    
    dutyR = constrain(abs(dutyR), 0, MOTOR_MAX);
    dutyL = constrain(abs(dutyL), 0, 180);
  }
  
  myservo1.write(dutyL);
  myservo2.write(dutyL);
  

  //motorR.setSpeed(dutyR);
  
  //digitalWrite(IN1, HIGH);
  //analogWrite(IN2, dutyR);
  /*if (dutyR>0){
    analogWrite(ENA, dutyR);
    //Serial.println(ENA);
    analogWrite(ENB, dutyR);
    digitalWrite(IN2, LOW);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN4, LOW);
    digitalWrite(IN3, HIGH);
  }
  else if(dutyR < 0){
    analogWrite(ENA, abs(dutyR));
    analogWrite(ENB, abs(dutyR));
    digitalWrite(IN2, HIGH);
    digitalWrite(IN1, LOW);
    digitalWrite(IN4, HIGH);
    digitalWrite(IN3, LOW);
  } else{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }*/
 // motorL.setMode(FORWARD);
  //motorR.setMode(FORWARD);
  motorL.setSpeed(dutyR);
  motorR.setSpeed(dutyR);
}

boolean getStarted;
byte index;
String string_convert = "";
void parsing() {
  if (btSerial.available() > 0) {
    char incomingByte = btSerial.read();        // обязательно ЧИТАЕМ входящий символ
    if (getStarted) {                         // если приняли начальный символ (парсинг разрешён)
      if (incomingByte != ' ' && incomingByte != ';') {   // если это не пробел И не конец
        string_convert += incomingByte;       // складываем в строку
      } else {                                // если это пробел или ; конец пакета
        intData[index] = string_convert.toInt();  // преобразуем строку в int и кладём в массив
        string_convert = "";                  // очищаем строку
        index++;                              // переходим к парсингу следующего элемента массива
      }
    }
    if (incomingByte == '$') {                // если это $
      getStarted = true;                      // поднимаем флаг, что можно парсить
      index = 0;                              // сбрасываем индекс
      string_convert = "";                    // очищаем строку
    }
    if (incomingByte == ';') {                // если таки приняли ; - конец парсинга
      getStarted = false;                     // сброс
      recievedFlag = true;                    // флаг на принятие
    }
  }
}
