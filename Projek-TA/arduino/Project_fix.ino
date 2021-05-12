
int Relay = 6;
int Buzz = 7;
int r = 0;
int hitung = 0;
int status1;

void setup() {    
  
  // initialize the digital pin as an output.
  Serial.begin(9600);
  pinMode(Relay, OUTPUT);
  pinMode(Buzz, OUTPUT);  
  digitalWrite(Relay, HIGH);
  
}


void loop() {
   
  if(Serial.available()){                 //From RPi to Arduino
    char c = Serial.read();
    //if (c == '1')
    //  hitung++;
    r = (Serial.read() - '0');            //conveting the value of chars to integer
    r = r + 1;
    //Serial.println(hitung);
    
    //if (hitung == 2){
      
      digitalWrite(Relay, LOW);  
      digitalWrite(Buzz, HIGH);                // turn the LED OFF
      delay(1000);              
      digitalWrite(Relay, HIGH);
      digitalWrite(Buzz, LOW);                  // turn the LED ON 
      //hitung = 0;
      
      
   }
}
