// Hall sensor Arduino firmware
// Pins 2~7: Joint 1~6 hall sensors
// Output: "0,1,0,1,0,0\n" format (1=triggered/LOW, 0=not triggered/HIGH)
// Baud: 115200
// Used by: /hall_states topic via my_robot_hall_bridge node

int hallPins[6] = {2,3,4,5,6,7};

void setup() {
  Serial.begin(115200);
  for(int i=0;i<6;i++){
    pinMode(hallPins[i], INPUT_PULLUP);
  }
}

void loop(){
  for(int i=0;i<6;i++){
    int state = digitalRead(hallPins[i]);
    if(state == LOW){
      Serial.print(1);
    }
    else{
      Serial.print(0);
    }
    if(i<5){
      Serial.print(",");
    }
  }
  Serial.println();
  delay(50);
}
