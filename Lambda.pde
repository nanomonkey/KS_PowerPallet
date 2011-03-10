// Lambda
void DoLambda() {
    switch(lambda_state) {
      case LAMBDA_CLOSEDLOOP:
        lambda_input = GetLambda();
        //don't reset changed PID values
        //lambda_PID.SetTunings(lambda_P[0], lambda_I[0], lambda_D[0]);
        lambda_PID.Compute();
        SetPremixServoAngle(lambda_output);
        if (engine_state == ENGINE_OFF) {
          TransitionLambda(LAMBDA_SEALED);
        }
        if (serial_last_input == 'o') {
          TransitionLambda(LAMBDA_STEPTEST);
          serial_last_input = '\0';
        }
        if (serial_last_input == 'O') {
          TransitionLambda(LAMBDA_SPSTEPTEST);
          serial_last_input = '\0';
        }
        break;
      case LAMBDA_SEALED:
        lambda_input = GetLambda();
        if (engine_state == ENGINE_STARTING) {
          TransitionLambda(LAMBDA_CLOSEDLOOP);
        }
        if (serial_last_input == 'o') {
          TransitionLambda(LAMBDA_STEPTEST);
          serial_last_input = '\0';
        }
        if (serial_last_input == 'O') {
          TransitionLambda(LAMBDA_SPSTEPTEST);
          serial_last_input = '\0';
        }
        SetPremixServoAngle(0);
        break;
      case LAMBDA_STEPTEST: //used for PID tuning
        if (millis()-lambda_state_entered > 15000) { //change output every 5 seconds
          TransitionLambda(LAMBDA_STEPTEST);
        }
        if (serial_last_input == 'o') {
          TransitionLambda(LAMBDA_CLOSEDLOOP);
          serial_last_input = '\0';
        }
        SetPremixServoAngle(lambda_output);
        break;
      case LAMBDA_SPSTEPTEST:
        lambda_input = GetLambda();
        lambda_PID.SetTunings(lambda_P[0], lambda_I[0], lambda_D[0]);
        lambda_PID.Compute();
        SetPremixServoAngle(lambda_output);
        if (millis()-lambda_state_entered > 15000) { //change output every 5 seconds
          TransitionLambda(LAMBDA_SPSTEPTEST);
        }
        if (serial_last_input == 'o') {
          TransitionLambda(LAMBDA_CLOSEDLOOP);
          serial_last_input = '\0';
        }
        SetPremixServoAngle(lambda_output);
        break;
     }
}

void TransitionLambda(int new_state) {
  //Exit
  switch (lambda_state) {
    case LAMBDA_CLOSEDLOOP:
      break;
    case LAMBDA_SEALED:
      break;
    case LAMBDA_STEPTEST:
      loopPeriod1 = loopPeriod1*4; //return to normal datalogging rate
      break;
     case LAMBDA_SPSTEPTEST:
       loopPeriod1 = loopPeriod1*4; //return to normal datalogging rate
       break;
  }
  Serial.print("# Lambda switching from ");
  Serial.print(lambda_state_name);
  
  //Enter
  lambda_state=new_state;
  lambda_state_entered = millis();
  switch (new_state) {
    case LAMBDA_CLOSEDLOOP:
      lambda_state_name = "Closed Loop";
      lambda_setpoint = lambda_setpoint_mode[0];
      lambda_PID.SetMode(AUTO);
      lambda_PID.SetSampleTime(20);
      lambda_PID.SetInputLimits(0.5,1.5);
      lambda_PID.SetOutputLimits(premix_valve_min,premix_valve_max);
      SetPremixServoAngle(premix_valve_center);
      break;
    case LAMBDA_SEALED:
      lambda_state_name = "Sealed";
      SetPremixServoAngle(premix_valve_closed);
      lambda_PID.SetMode(MANUAL);
      break;
    case LAMBDA_STEPTEST:
      lambda_state_name = "Step Test";
      lambda_PID.SetMode(AUTO);
      lambda_output = (random(2,4)/10.0)*(lambda_PID.GetOUTMax()-lambda_PID.GetOUTMin()); //steps in random 10% increments of control output limits
      loopPeriod1 = loopPeriod1/4; //fast datalogging
      break;
    case LAMBDA_SPSTEPTEST:
      lambda_state_name = "Setpoint Step Test";
      lambda_PID.SetMode(AUTO);
      lambda_setpoint = random(8,12)/10.0; //steps in random 10% increments of control output limits
      loopPeriod1 = loopPeriod1/4; //fast datalogging
      break;
  }
  Serial.print(" to ");  
  Serial.println(lambda_state_name);
}

    //this doesn't need to be checked that often.....
    //if (millis() - lamba_updated_time > 60000 & write_lambda) {
    // WriteLambda(); //values for engine on stored in flash
    // write_lambda = false;
    // Serial.print("Lambda PID values saved");
    //}
    
double GetLambda() {
  return analogRead(ANA_LAMBDA)/1024.0+0.5; //0-5V = 0.5 - 1.5 L;
}

void SetPremixServoAngle(double percent) {
 Servo_Mixture.write(premix_valve_closed + percent*(premix_valve_open-premix_valve_closed));
}

void WriteLambda() {
  //0-8 for P_calib
  EEPROM.write(9, lambda_P[0]*20+128); // stores -6.4 -> 6.4 at 0.05 resolution
  EEPROM.write(10, lambda_I[0]*20+128);
  EEPROM.write(11, lambda_D[0]*20+128);
}

void LoadLambda() {
  //0-8 for P_calib
  lambda_P[0] = EEPROM.read(9)/20-6.4; // stores -6.4 -> 6.4 at 0.05 resolution
  lambda_I[0] = EEPROM.read(10)/20-6.4;
  lambda_D[0] = EEPROM.read(11)/20-6.4;
}


