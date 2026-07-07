#define A_ENA 2
#define A_DIR 3
#define A_PUL 4
#define A_RUNSW A0
#define A_DIRSW A1

#define B_ENA 5
#define B_DIR 6
#define B_PUL 7
#define B_RUNSW A2
#define B_DIRSW A3

#define C_ENA 8
#define C_DIR 9
#define C_PUL 10
#define C_RUNSW 11
#define C_DIRSW 12

#define SERIAL_BAUD 115200

enum ControlMode {
  MODE_MIXED = 0,
  MODE_MANUAL = 1,
  MODE_ROS = 2
};

struct Motor {
  char name;
  byte enaPin;
  byte dirPin;
  byte pulPin;
  byte runSwitchPin;
  byte dirSwitchPin;
  bool runState;
  bool dirState;
  int lastRunButton;
  int lastDirButton;
  unsigned long lastRunDebounce;
  unsigned long lastDirDebounce;
  unsigned long lastStepTime;
  unsigned long intervalMicros;
};

const unsigned long DEBOUNCE_MS = 50;
const unsigned int PULSE_WIDTH_US = 10;
const unsigned long DEFAULT_A_INTERVAL_US = 50;
const unsigned long DEFAULT_B_INTERVAL_US = 3000;
const unsigned long DEFAULT_C_INTERVAL_US = 50;

const byte SERIAL_BUFFER_SIZE = 64;
char serialBuffer[SERIAL_BUFFER_SIZE];
byte serialIndex = 0;
ControlMode controlMode = MODE_MIXED;

Motor motorA = {'A', A_ENA, A_DIR, A_PUL, A_RUNSW, A_DIRSW, false, false, HIGH, HIGH, 0, 0, 0, DEFAULT_A_INTERVAL_US};
Motor motorB = {'B', B_ENA, B_DIR, B_PUL, B_RUNSW, B_DIRSW, false, false, HIGH, HIGH, 0, 0, 0, DEFAULT_B_INTERVAL_US};
Motor motorC = {'C', C_ENA, C_DIR, C_PUL, C_RUNSW, C_DIRSW, false, false, HIGH, HIGH, 0, 0, 0, DEFAULT_C_INTERVAL_US};

void setupMotor(Motor &motor) {
  pinMode(motor.enaPin, OUTPUT);
  pinMode(motor.dirPin, OUTPUT);
  pinMode(motor.pulPin, OUTPUT);
  pinMode(motor.runSwitchPin, INPUT_PULLUP);
  pinMode(motor.dirSwitchPin, INPUT_PULLUP);

  digitalWrite(motor.enaPin, HIGH);
  digitalWrite(motor.dirPin, motor.dirState);
  digitalWrite(motor.pulPin, LOW);
}

void updateButtons(Motor &motor, unsigned long nowMillis) {
  if (controlMode == MODE_ROS) {
    return;
  }

  int runButton = digitalRead(motor.runSwitchPin);
  if (runButton == LOW && motor.lastRunButton == HIGH && nowMillis - motor.lastRunDebounce >= DEBOUNCE_MS) {
    motor.runState = !motor.runState;
    motor.lastRunDebounce = nowMillis;
  }
  motor.lastRunButton = runButton;

  int dirButton = digitalRead(motor.dirSwitchPin);
  if (dirButton == LOW && motor.lastDirButton == HIGH && nowMillis - motor.lastDirDebounce >= DEBOUNCE_MS) {
    motor.dirState = !motor.dirState;
    digitalWrite(motor.dirPin, motor.dirState);
    motor.lastDirDebounce = nowMillis;
  }
  motor.lastDirButton = dirButton;
}

void updateStep(Motor &motor, unsigned long nowMicros) {
  if (!motor.runState) {
    return;
  }

  if (nowMicros - motor.lastStepTime >= motor.intervalMicros) {
    digitalWrite(motor.pulPin, HIGH);
    delayMicroseconds(PULSE_WIDTH_US);
    digitalWrite(motor.pulPin, LOW);
    motor.lastStepTime = nowMicros;
  }
}

Motor *findMotor(const char *name) {
  if (!name) {
    return NULL;
  }

  if (name[0] == 'A' || name[0] == 'a') {
    return &motorA;
  }
  if (name[0] == 'B' || name[0] == 'b') {
    return &motorB;
  }
  if (name[0] == 'C' || name[0] == 'c') {
    return &motorC;
  }

  return NULL;
}

void printMotorStatus(const Motor &motor) {
  Serial.print(motor.name);
  Serial.print(" RUN=");
  Serial.print(motor.runState ? 1 : 0);
  Serial.print(" DIR=");
  Serial.print(motor.dirState ? 1 : 0);
  Serial.print(" INTERVAL_US=");
  Serial.println(motor.intervalMicros);
}

void printStatus() {
  Serial.print("MODE=");
  if (controlMode == MODE_MANUAL) {
    Serial.println("MANUAL");
  } else if (controlMode == MODE_ROS) {
    Serial.println("ROS");
  } else {
    Serial.println("MIXED");
  }

  printMotorStatus(motorA);
  printMotorStatus(motorB);
  printMotorStatus(motorC);
}

bool setControlMode(const char *value) {
  if (!value) {
    return false;
  }

  if (strcmp(value, "MIXED") == 0 || strcmp(value, "0") == 0) {
    controlMode = MODE_MIXED;
    return true;
  }
  if (strcmp(value, "MANUAL") == 0 || strcmp(value, "1") == 0) {
    controlMode = MODE_MANUAL;
    return true;
  }
  if (strcmp(value, "ROS") == 0 || strcmp(value, "2") == 0) {
    controlMode = MODE_ROS;
    return true;
  }

  return false;
}

void handleSerialCommand(char *line) {
  char *command = strtok(line, " ");
  char *motorName = strtok(NULL, " ");
  char *value = strtok(NULL, " ");

  if (!command) {
    return;
  }

  if (strcmp(command, "STATUS") == 0) {
    printStatus();
    return;
  }

  if (strcmp(command, "MODE") == 0) {
    if (setControlMode(motorName)) {
      Serial.println("OK");
    } else {
      Serial.println("ERR MODE");
    }
    return;
  }

  if (controlMode == MODE_MANUAL) {
    if (strcmp(command, "INTERVAL_US") != 0) {
      Serial.println("ERR MANUAL_MODE");
      return;
    }
  }

  Motor *motor = findMotor(motorName);
  if (!motor) {
    Serial.println("ERR MOTOR");
    return;
  }

  if (strcmp(command, "START") == 0) {
    motor->runState = true;
    Serial.println("OK");
    return;
  }

  if (strcmp(command, "STOP") == 0) {
    motor->runState = false;
    Serial.println("OK");
    return;
  }

  if (strcmp(command, "RUN") == 0 && value) {
    motor->runState = atol(value) != 0;
    Serial.println("OK");
    return;
  }

  if (strcmp(command, "DIR") == 0 && value) {
    motor->dirState = atol(value) != 0;
    digitalWrite(motor->dirPin, motor->dirState);
    Serial.println("OK");
    return;
  }

  if (strcmp(command, "INTERVAL_US") == 0 && value) {
    unsigned long interval = atol(value);
    if (interval < PULSE_WIDTH_US + 5) {
      Serial.println("ERR INTERVAL");
      return;
    }
    motor->intervalMicros = interval;
    Serial.println("OK");
    return;
  }

  Serial.println("ERR COMMAND");
}

void updateSerial() {
  while (Serial.available() > 0) {
    char incoming = Serial.read();

    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      serialBuffer[serialIndex] = '\0';
      handleSerialCommand(serialBuffer);
      serialIndex = 0;
      continue;
    }

    if (serialIndex < SERIAL_BUFFER_SIZE - 1) {
      serialBuffer[serialIndex] = incoming;
      serialIndex++;
    } else {
      serialIndex = 0;
      Serial.println("ERR BUFFER");
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  setupMotor(motorA);
  setupMotor(motorB);
  setupMotor(motorC);
  Serial.println("READY");
}

void loop() {
  updateSerial();

  unsigned long nowMillis = millis();
  updateButtons(motorA, nowMillis);
  updateButtons(motorB, nowMillis);
  updateButtons(motorC, nowMillis);

  unsigned long nowMicros = micros();
  updateStep(motorA, nowMicros);
  updateStep(motorB, nowMicros);
  updateStep(motorC, nowMicros);
}
