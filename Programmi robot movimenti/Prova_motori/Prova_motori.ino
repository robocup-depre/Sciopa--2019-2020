void setup(){
  for(short k=45;k<53;k++){
    pinMode(k,OUTPUT);
  }
  pinMode(12,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(9,OUTPUT);
}
void indietro(){
  digitalWrite(47,LOW);
  digitalWrite(48,HIGH);
  digitalWrite(50,HIGH);
  digitalWrite(52,LOW);
  digitalWrite(45,LOW);
  digitalWrite(49,HIGH);
  digitalWrite(51,HIGH);
  digitalWrite(53,LOW);
}
void avanti(){
  digitalWrite(47,LOW);
  digitalWrite(48,HIGH);
  digitalWrite(50,HIGH);
  digitalWrite(52,LOW);
  digitalWrite(45,LOW);
  digitalWrite(49,HIGH);
  digitalWrite(51,HIGH);
  digitalWrite(53,LOW);
}
void sinistra(){
  digitalWrite(47,LOW);
  digitalWrite(48,HIGH);
  digitalWrite(50,LOW);
  digitalWrite(52,HIGH);
  digitalWrite(45,HIGH);
  digitalWrite(49,LOW);
  digitalWrite(51,HIGH);
  digitalWrite(53,LOW);
}
void destra(){
  digitalWrite(47,HIGH);
  digitalWrite(48,LOW);
  digitalWrite(50,HIGH);
  digitalWrite(52,LOW);
  digitalWrite(45,LOW);
  digitalWrite(49,HIGH);
  digitalWrite(51,LOW);
  digitalWrite(53,HIGH);
}
void loop(){
  digitalWrite(47,HIGH);
  digitalWrite(48,LOW);
  digitalWrite(50,HIGH);
  digitalWrite(52,LOW);
  digitalWrite(45,LOW);
  digitalWrite(49,HIGH);
  digitalWrite(51,LOW);
  digitalWrite(53,HIGH);
  analogWrite(12,60);
  analogWrite(11,60);
  analogWrite(10,60);
  analogWrite(9,60);  
}

//9 AD  10 AS    11 PD    12 PS
