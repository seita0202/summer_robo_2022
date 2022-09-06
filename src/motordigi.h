void motordigi(int pin1_, int pin2_ , int power_){
  if(power_ == 100){
    digitalWrite(pin1_,HIGH);
    digitalWrite(pin2_,LOW);
  }
  else if(power_ == 0){
    digitalWrite(pin1_, LOW);
    digitalWrite(pin2_, LOW);
  }
  else if(power_ == -100){
    digitalWrite(pin1_, LOW);
    digitalWrite(pin2_, HIGH);
  }
}
