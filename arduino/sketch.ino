#include <List.hpp>
#include <Servo.h>

Servo myservo;  
List<int> listqued;
List<int> listelindr;


int cord1 = 100;
int cord12 = 128;
int cord13 = 143;
int cord14 = 168;

int cord21 = 58;
int cord22 = 40;
int cord23 = 20;
int cord24 = 10;


String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);
  inputString.reserve(200);
  listqued.add(cord1);
  listqued.add(cord12);
  listqued.add(cord13);
  listqued.add(cord14);
  listelindr.add(cord21);
  listelindr.add(cord22);
  listelindr.add(cord23);
  listelindr.add(cord24);
  myservo.attach(5);
  myservo.write(90);
   
}

void loop() {
  if (stringComplete) {
    Serial.println(inputString);
    
    stringComplete = false;
    if (inputString == "1\n"){
      myservo.write(listqued.getValue(0));
      listqued.removeFirst();
    }
    else if (inputString == "2\n"){
      myservo.write(listelindr.getValue(0));
      listelindr.removeFirst();
    }
    inputString = "";
  }
}


void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}