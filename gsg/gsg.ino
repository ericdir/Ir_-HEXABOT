//Hexabot ini menggunakan IR remote control
#include "IRremote.h"

#include "ir_command_codes.h"

#include <Servo.h>

// // Analog pin untuk IR receiver sensor
const int IR_PIN = A0;

// pin untuk servo
const int LEFT_SERVO_PIN = 2;
const int CENTRAL_SERVO_PIN = 4;
const int RIGHT_SERVO_PIN = 7;

// posisi angle servo pada posisi awal
const long LEFT_SERVO_ZERO_VALUE = 90;
const long RIGHT_SERVO_ZERO_VALUE = 90;
const long CENTRAL_SERVO_ZERO_VALUE = 90;

// Amplitude untuk servo samping kanan dan kiri
const long SIDE_SERVOS_FULL_AMPLITUDE = 30;
// setengah amplitude untuk servo kanan kiri saat robot berputar.
const long SIDE_SERVOS_HALF_AMPLITUDE = 15;
// amplitude untuk servo tengah
const long CENTRAL_SERVO_AMPLITUDE = 20;

// perbedaan periode pada setiap action
const long STEP_PERIOD_VERY_SLOW = 2000;
const long STEP_PERIOD_SLOW = 1500;
const long STEP_PERIOD_FAST = 1000;
const long STEP_PERIOD_VERY_FAST = 500;

long lastMillis;
long globalPhase;
float angleShiftLeftServo;
float angleShiftRightServo;
float angleShiftCentralServo;
long stepPeriod;
long amplitudeLeftServo;
long amplitudeRightServo;
boolean isAttached;
boolean isStopped;

// // EN: IRreceiver code
IRrecv irrecv(IR_PIN);

Servo LeftServo;
Servo RightServo;
Servo CentralServo;

void attachServos() {
  if (!isAttached) {
    LeftServo.attach(LEFT_SERVO_PIN);
    RightServo.attach(RIGHT_SERVO_PIN);
    CentralServo.attach(CENTRAL_SERVO_PIN);
    isAttached = true;
  }
}

// dibeberapa posisi mungkin servo akan mengeluarkan bunyi atar bergetar
//     untuk menghindari hal ini servo akan diarahkan berhenti.
void detachServos() {
  if (isAttached) {
    LeftServo.detach();
    RightServo.detach();
    CentralServo.detach();
    isAttached = false;
  }
}

void setup() {
  // memulai IR Receiver
  irrecv.enableIRIn();
 
  attachServos();
  isStopped = true;
  lastMillis = millis();

  angleShiftLeftServo = 0;
  angleShiftRightServo = 0;
  angleShiftCentralServo = 0;
  
  stepPeriod = STEP_PERIOD_FAST;
}

// // mencari angle untuk servo
//     amplitudo tray proses,
//     Param phaseMillis - saat ini durasi tray,
//     param shiftAndle - oscillating process.
int getAngle(long amplitude, long phaseMillis, float shiftAngle) {
  float alpha = 2 * PI * phaseMillis / stepPeriod + shiftAngle;
  float angle = amplitude * sin(alpha);
  return (int)angle;
}

template<typename T,size_t N>
boolean hasCode(T (&commandCodes)[N], long code) {
  for (int i = 0; i < N; i++) {
    if (commandCodes[i] == code) {
      return true;
    }
  }
  return false;
}

void loop() {
  long millisNow = millis();
  long millisPassed = millisNow - lastMillis;
  if (isStopped) {
    //  Kita harus menunggu setengah detik. Setelah itu kita berpikir bahwa servos 
    // berada dalam posisi nol dan kita dapat melepaskan mereka.
    if (millisPassed >= 500) {
      lastMillis = 0;
      detachServos();
    }

    globalPhase = 0;
  } else {
    lastMillis = millisNow;
    
    globalPhase += millisPassed;
    globalPhase = globalPhase % stepPeriod;
  }
  
  // Deklarasi struktur yang digunakan untuk menerima dan diterjemahkan IR perintah

  decode_results results;
  
  // Kita dapat menangani perintah IR jika diterima dan diterjemahkan berhasil.
  if (irrecv.decode(&results)) {
    
    if (hasCode(IR_COMMAND_FORWARD_CODES, results.value) ||
        hasCode(IR_COMMAND_FORWARD_LEFT_CODES, results.value) ||
        hasCode(IR_COMMAND_FORWARD_RIGHT_CODES, results.value)) {
    
      attachServos();
      isStopped = false;
      angleShiftLeftServo = 0;
      angleShiftRightServo = 0;
      angleShiftCentralServo = PI/2;
        
      amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
      amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
      if (hasCode(IR_COMMAND_FORWARD_LEFT_CODES, results.value)) {
        amplitudeLeftServo = SIDE_SERVOS_HALF_AMPLITUDE;
      } else if (hasCode(IR_COMMAND_FORWARD_RIGHT_CODES, results.value)) {
        amplitudeRightServo = SIDE_SERVOS_HALF_AMPLITUDE;
      }
    } else if(hasCode(IR_COMMAND_BACKWARD_CODES, results.value) ||
              hasCode(IR_COMMAND_BACKWARD_LEFT_CODES, results.value) ||
              hasCode(IR_COMMAND_BACKWARD_RIGHT_CODES, results.value)) {

      attachServos();
      isStopped = false;
      angleShiftLeftServo = 0;
      angleShiftRightServo = 0;
      angleShiftCentralServo = -PI/2;

      amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
      amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
      if (hasCode(IR_COMMAND_BACKWARD_LEFT_CODES, results.value)) {
        amplitudeRightServo = SIDE_SERVOS_HALF_AMPLITUDE;
      } else if (hasCode(IR_COMMAND_BACKWARD_RIGHT_CODES, results.value)) {
        amplitudeLeftServo = SIDE_SERVOS_HALF_AMPLITUDE;
      }
    } else if (hasCode(IR_COMMAND_TURN_LEFT_CODES, results.value)) {
      attachServos();
      isStopped = false;
      angleShiftLeftServo = 0;
      angleShiftRightServo = PI;
      angleShiftCentralServo = -PI/2;
      amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
      amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
    } else if (hasCode(IR_COMMAND_TURN_RIGHT_CODES, results.value)) {
      attachServos();
      isStopped = false;
      angleShiftLeftServo = 0;
      angleShiftRightServo = PI;
      angleShiftCentralServo = PI/2;
      amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
      amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
    } else if (hasCode(IR_COMMAND_STOP_CODES, results.value)) {
      attachServos();
      isStopped = true;
      angleShiftLeftServo = 0;
      angleShiftRightServo = 0;
      angleShiftCentralServo = 0;
      amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
      amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
    } else if (hasCode(IR_COMMAND_VERY_SLOW_CODES, results.value)) {
      // globalPhase koreksi untuk menyimpan posisi servo ketika mengubah periode.
      globalPhase = globalPhase * STEP_PERIOD_VERY_SLOW / stepPeriod;
      stepPeriod = STEP_PERIOD_VERY_SLOW;
    } else if (hasCode(IR_COMMAND_SLOW_CODES, results.value)) {
      globalPhase = globalPhase * STEP_PERIOD_SLOW / stepPeriod;
      stepPeriod = STEP_PERIOD_SLOW;
    } else if (hasCode(IR_COMMAND_FAST_CODES, results.value)) {
      globalPhase = globalPhase * STEP_PERIOD_FAST / stepPeriod;
      stepPeriod = STEP_PERIOD_FAST;
    } else if (hasCode(IR_COMMAND_VERY_FAST_CODES, results.value)) {
      globalPhase = globalPhase * STEP_PERIOD_VERY_FAST / stepPeriod;
      stepPeriod = STEP_PERIOD_VERY_FAST;
    }
    // Setelah kode tersebut, metode resume() harus dipanggil untuk melanjutkan menerima kode.
    irrecv.resume();
  }
  
  if (isAttached) {
    LeftServo.write(LEFT_SERVO_ZERO_VALUE + getAngle(amplitudeLeftServo, globalPhase, angleShiftLeftServo));
    RightServo.write(RIGHT_SERVO_ZERO_VALUE + getAngle(amplitudeRightServo, globalPhase, angleShiftRightServo));
    CentralServo.write(CENTRAL_SERVO_ZERO_VALUE + getAngle(CENTRAL_SERVO_AMPLITUDE, globalPhase, angleShiftCentralServo));
  }
}

