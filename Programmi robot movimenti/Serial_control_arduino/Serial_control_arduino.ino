void setup() {
  Serial.begin(115200);
  pinMode(13,OUTPUT);
  analogWrite(13,150);
}

unsigned short val;

void loop() {
  if(Serial.available()>0){
    Serial.println(val);
    val=Serial.read()*3;
    analogWrite(13,val);
  }
}
