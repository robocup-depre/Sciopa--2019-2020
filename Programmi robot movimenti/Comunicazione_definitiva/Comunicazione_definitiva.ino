void setup(){
  Serial.begin(115200);
  for(short k=45;k<53;k++){
    pinMode(k,OUTPUT);
  }
  pinMode(12,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(9,OUTPUT);
}

//9/3 AD  10/2 AS    11/1 PD    12/0 PS
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
  digitalWrite(47,HIGH);
  digitalWrite(48,LOW);
  digitalWrite(50,LOW);
  digitalWrite(52,HIGH);
  digitalWrite(45,HIGH);
  digitalWrite(49,LOW);
  digitalWrite(51,LOW);
  digitalWrite(53,HIGH);
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
char x[2];
unsigned short dato;
void loop(){
  if(Serial.available()>0)
  {
    Serial.readBytes(x,2);
    dato=((x[0]&0b00001111)<<4)|(x[1]&0b00001111);
    /*Serial.print(x[1]>>4);
    Serial.print(x[0]>>4);*/
    //Serial.println(dato);
    if((x[1]>>4)==0){
      avanti();
    }
    if((x[1]>>4)==1){
      indietro();
    }
    if((x[1]>>4)==2){
      sinistra();
    }
    if((x[1]>>4)==3){
      destra();
    }
    if((x[0]>>4)==0){
      analogWrite(12,dato);
    }
    if((x[0]>>4)==1){
      analogWrite(11,dato);
    }
    if((x[0]>>4)==2){
      analogWrite(10,dato);
    }
    if((x[0]>>4)==3){
      analogWrite(9,dato);
    }
  }
}
