#include <Arduino.h>

#ifdef ARDUINO_BLUEPILL_F103C8

#define PAPER_FEED_INPUT_A PB9
#define PAPER_FEED_INPUT_B PB8
#define PAPER_FEED_MOTOR_A PB7
#define PAPER_FEED_MOTOR_B PB6

#define CARRIAGE_INPUT_A PB5
#define CARRIAGE_INPUT_B PB4
#define CARRIAGE_MOTOR_A PB3
#define CARRIAGE_MOTOR_B PA15

#define PAPER_FEED_STEP PB15
#define PAPER_FEED_DIR PB14
#define CARRIAGE_STEP PB13
#define CARRIAGE_DIR PB12

#define PAPER_FEED_ENABLE PA0
#define CARRIAGE_ENABLE PA1
#endif

#ifdef ARDUINO_AVR_NANO

#define PAPER_FEED_INPUT_A 2
#define PAPER_FEED_INPUT_B 3
#define PAPER_FEED_MOTOR_A 4
#define PAPER_FEED_MOTOR_B 5

#define CARRIAGE_INPUT_A 6
#define CARRIAGE_INPUT_B 7
#define CARRIAGE_MOTOR_A 8
#define CARRIAGE_MOTOR_B 9

#define PAPER_FEED_STEP 0
#define PAPER_FEED_DIR 0
#define CARRIAGE_STEP 0
#define CARRIAGE_DIR 0

#endif

#define PAPER_FEED_LPI 200
#define CARRIAGE_LPI 150
#define PAPER_FEED_DEAD_STEPS 30
#define CARRIAGE_DEAD_STEPS 30

uint8_t paperPWM = 255;
uint8_t carriagePWM = 255;

int rotDirection = 0;
int pressed = false;
int running = 1;
volatile int64_t paperFeedCurrent = 0;
int64_t paperFeedTarget = 1000;
volatile int64_t carriageCurrent = 0;
int64_t carriageTarget = 100;
// uint8_t rollSensorA = 0;
volatile uint8_t lastRollsensor = 0;
volatile uint8_t lastLinearsensor = 0;

uint8_t paperFeedReached = 0;
uint8_t carriageReached = 0;

void changeRotary() {
  uint8_t rollSensorA = digitalRead(PAPER_FEED_INPUT_A);
  uint8_t rollSensorB = digitalRead(PAPER_FEED_INPUT_B);
  switch (lastRollsensor) {
    case 0x0:
      if (rollSensorA) {
        paperFeedCurrent--;
      }
      if (rollSensorB) {
        paperFeedCurrent++;
      }
      break;
    case 0x1:
      if (rollSensorA) {
        paperFeedCurrent++;
      }
      if (!rollSensorB) {
        paperFeedCurrent--;
      }
      break;
    case 0x2:
      if (!rollSensorA) {
        paperFeedCurrent++;
      }
      if (rollSensorB) {
        paperFeedCurrent--;
      }
      break;
    case 0x3:
      if (!rollSensorA) {
        paperFeedCurrent--;
      }
      if (!rollSensorB) {
        paperFeedCurrent++;
      }
      break;
    default:
      break;
  }
  lastRollsensor = rollSensorA << 1 | rollSensorB;
}

void changeLinear() {
  uint8_t linearSensorA = digitalRead(CARRIAGE_INPUT_A);
  uint8_t linearSensorB = digitalRead(CARRIAGE_INPUT_B);
  switch (lastLinearsensor) {
    case 0x0:
      if (linearSensorA) {
        carriageCurrent--;
      }
      if (linearSensorB) {
        carriageCurrent++;
      }
      break;
    case 0x1:
      if (linearSensorA) {
        carriageCurrent++;
      }
      if (!linearSensorB) {
        carriageCurrent--;
      }
      break;
    case 0x2:
      if (!linearSensorA) {
        carriageCurrent++;
      }
      if (linearSensorB) {
        carriageCurrent--;
      }
      break;
    case 0x3:
      if (!linearSensorA) {
        carriageCurrent--;
      }
      if (!linearSensorB) {
        carriageCurrent++;
      }
      break;
    default:
      break;
  }
  lastLinearsensor = linearSensorA << 1 | linearSensorB;
}

void feedStep() {
  if (digitalRead(PAPER_FEED_DIR)) {
    paperFeedTarget++;
  } else {
    paperFeedTarget--;
  }
}

void carriageStep() {
  if (digitalRead(CARRIAGE_DIR)) {
    carriageTarget++;
  } else {
    carriageTarget--;
  }
}

void setup() {
  pinMode(PAPER_FEED_INPUT_A, INPUT);
  pinMode(PAPER_FEED_INPUT_B, INPUT);
  pinMode(CARRIAGE_INPUT_A, INPUT);
  pinMode(CARRIAGE_INPUT_B, INPUT);

  pinMode(PAPER_FEED_STEP, INPUT);
  pinMode(PAPER_FEED_DIR, INPUT);
  pinMode(CARRIAGE_STEP, INPUT);
  pinMode(CARRIAGE_DIR, INPUT);

  pinMode(PAPER_FEED_MOTOR_A, OUTPUT);
  pinMode(PAPER_FEED_MOTOR_B, OUTPUT);
  pinMode(CARRIAGE_MOTOR_A, OUTPUT);
  pinMode(CARRIAGE_MOTOR_B, OUTPUT);

#ifdef ARDUINO_BLUEPILL_F103C8
  // attach the interrupts
  attachInterrupt(PAPER_FEED_INPUT_A, changeRotary, CHANGE);
  attachInterrupt(PAPER_FEED_INPUT_B, changeRotary, CHANGE);
  attachInterrupt(CARRIAGE_INPUT_A, changeLinear, CHANGE);
  attachInterrupt(CARRIAGE_INPUT_B, changeLinear, CHANGE);
  attachInterrupt(PAPER_FEED_STEP, feedStep, CHANGE);
  attachInterrupt(CARRIAGE_STEP, carriageStep, CHANGE);

  // PA0 2MHz alternate func pushpull2
  MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF0, 0b10 << GPIO_CRL_CNF0_Pos);
  MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE0, 0b01 << GPIO_CRL_MODE0_Pos);
  // PA1 2MHz alternate func pushpull
  MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF1, 0b10 << GPIO_CRL_CNF1_Pos);
  MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE1, 0b01 << GPIO_CRL_MODE1_Pos);
  // Enable timer 2
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);
  // Timer 2 channel 1
  MODIFY_REG(TIM2->CCMR1, TIM_CCMR1_OC1M, 0b110 << TIM_CCMR1_OC1M_Pos);
  SET_BIT(TIM2->CCMR1, TIM_CCMR1_OC1PE);
  // Timer 2 channel 2
  MODIFY_REG(TIM2->CCMR1, TIM_CCMR1_OC2M, 0b110 << TIM_CCMR1_OC2M_Pos);
  SET_BIT(TIM2->CCMR1, TIM_CCMR1_OC2PE);
  // enable outputs
  SET_BIT(TIM2->CCER, TIM_CCER_CC1E + TIM_CCER_CC2E);
  // inverse outputs
  // SET_BIT(TIM2->CCER, TIM_CCER_CC1P + TIM_CCER_CC2P);
  // Timer 2 auto reload register, defines the maximum value of the counter in
  // PWM mode.
  TIM2->ARR = 400;
  // Timer 2 clock prescaler, the APB2 clock is divided by this value +1.
  TIM2->PSC = 0;
  // Timer 2 enable counter and auto-preload
  SET_BIT(TIM2->CR1, TIM_CR1_CEN + TIM_CR1_ARPE);
  // set to 50%
  TIM2->CCR1 = 200;
  TIM2->CCR2 = 200;
#endif
#ifdef ARDUINO_AVR_NANO
  pinMode(PAPER_FEED_ENABLE, OUTPUT);
  pinMode(CARRIAGE_ENABLE, OUTPUT);
  analogWrite(PAPER_FEED_ENABLE, 255);
  analogWrite(CARRIAGE_ENABLE, 200);
#endif
}

void loop() {
#ifdef ARDUINO_BLUEPILL_F103C8
// update duty cycle based on positon
// TIM2->CCR1 = 400;
// TIM2->CCR2 = 0;
#endif

#ifdef ARDUINO_AVR_NANO
  analogWrite(PAPER_FEED_ENABLE, paperPWM);
  analogWrite(CARRIAGE_ENABLE, carriagePWM);
#endif

  if (paperFeedReached &&
      abs(paperFeedCurrent - paperFeedTarget) >= PAPER_FEED_DEAD_STEPS) {
    paperFeedReached = 0;
  }

  if (paperFeedTarget > paperFeedCurrent && !paperFeedReached) {
    digitalWrite(PAPER_FEED_MOTOR_A, HIGH);
    digitalWrite(PAPER_FEED_MOTOR_B, LOW);
  } else if (paperFeedTarget < paperFeedCurrent && !paperFeedReached) {
    digitalWrite(PAPER_FEED_MOTOR_A, LOW);
    digitalWrite(PAPER_FEED_MOTOR_B, HIGH);
  } else {
    digitalWrite(PAPER_FEED_MOTOR_A, LOW);
    digitalWrite(PAPER_FEED_MOTOR_B, LOW);
    paperFeedReached = 1;
  }

  if (carriageReached &&
      abs(carriageCurrent - carriageTarget) >= CARRIAGE_DEAD_STEPS) {
    carriageReached = 0;
  }

  if (carriageTarget > carriageCurrent && !carriageReached) {
    digitalWrite(CARRIAGE_MOTOR_A, HIGH);
    digitalWrite(CARRIAGE_MOTOR_B, LOW);
  } else if (carriageTarget < carriageCurrent && !carriageReached) {
    digitalWrite(CARRIAGE_MOTOR_A, LOW);
    digitalWrite(CARRIAGE_MOTOR_B, HIGH);
  } else {
    digitalWrite(CARRIAGE_MOTOR_A, LOW);
    digitalWrite(CARRIAGE_MOTOR_B, LOW);
    carriageReached = 1;
  }
}