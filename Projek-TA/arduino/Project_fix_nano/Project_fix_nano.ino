
#include <Servo.h> 
 
Servo myservo;  
int pos1 = 300;
int pos2 = 0;  
int Buzz = 7;
int r = 0;
int status1;

void setup() {    

  Serial.begin(9600);
  myservo.attach(9);
  pinMode(Buzz, OUTPUT); 
  while (! Serial); 
  Serial.println("Connected!");
}


void loop() {
   
  if(Serial.available()){        
    char c = Serial.read();
    r = (Serial.read() - '0');    

      Serial.println("Aktif");
      
      myservo.write(pos1); 
      delay(2500);
      myservo.write(pos2); 
      digitalWrite(Buzz, HIGH);     
      delay(200);   
      digitalWrite(Buzz, LOW);     
    
      
   }
}
