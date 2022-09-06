void motorrun(int pin1_, int pin2_, int power_){
    if (power_ >= 0){
        analogWrite(pin1_, power_);
        analogWrite(pin2_, 0);
    }
    else {
        power_ *= -1;
        analogWrite(pin1_, 0);
        analogWrite(pin2_, power_);
    }
}