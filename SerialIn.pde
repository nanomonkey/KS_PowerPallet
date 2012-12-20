// SerialIn
void DoSerialIn() {
  int incomingByte = 0;
   PID &v_PID = lambda_PID;
   double p,i,d;
    double p_d =0.02;
    double i_d = 0.02;
    double d_d = 0.02;
  // Serial input
  if (Serial.available() > 0) {
    p=v_PID.GetP_Param();
    i=v_PID.GetI_Param();
    d=v_PID.GetD_Param();
    serial_last_input = Serial.read();
    switch (serial_last_input) {
    case 'p':
      PrintLambdaUpdate(p,i,d,p+p_d,i,d);
      p=p+p_d;
      v_PID.SetTunings(p,i,d);
      break;
    case 'P':
      PrintLambdaUpdate(p,i,d,p-p_d,i,d);
      p=p-p_d;
      v_PID.SetTunings(p,i,d);
      break;
    case 'i':
      PrintLambdaUpdate(p,i,d,p,i+i_d,d);
      i=i+i_d;
      v_PID.SetTunings(p,i,d);
      break;
    case 'I':
      PrintLambdaUpdate(p,i,d,p,i-i_d,d);
      i=i-i_d;
      v_PID.SetTunings(p,i,d);
      break;
    case 'd':
      PrintLambdaUpdate(p,i,d,p,i,d+d_d);
      d=d+d_d;
      v_PID.SetTunings(p,i,d);
      break;
    case 'D':
      PrintLambdaUpdate(p,i,d,p,i,d-d_d);
      d=d-d_d;
      v_PID.SetTunings(p,i,d);
      break;
    case 'c':
      CalibratePressureSensors();
      LoadPressureSensorCalibration();
      break;
    case 's':
      Servo_Calib.write(Servo_Calib.read()+10);
      putstring("#Servo1 (degrees) now:");
      Serial.println(Servo_Calib.read());
      break;
    case 'S':
      Servo_Calib.write(Servo_Calib.read()-10);
      putstring("#Servo1 (degrees) now:");
      Serial.println(Servo_Calib.read());
      break;
    case 'l':
      lambda_setpoint += 0.01;
      putstring("#Lambda Setpoint now:");
      Serial.println(lambda_setpoint);
      WriteLambda();
      break;
    case 'L':
      lambda_setpoint -= 0.01;
      putstring("#Lambda Setpoint now:");
      Serial.println(lambda_setpoint);
      WriteLambda();
      break;
    case 't':
      loopPeriod1 = max(loopPeriod1-100,100);
      putstring("#Sample Period now:");
      Serial.println(loopPeriod1);
      break;
    case 'T':
      loopPeriod1 = min(loopPeriod1+100,10000);
      putstring("#Sample Period now:");
      Serial.println(loopPeriod1);
      break;
    case 'g':  
      grate_val = GRATE_SHAKE_CROSS; //set grate val to shake for grate_on_interval
      Serial.println("#Grate Shaken");
      break;
    case 'G':  
      switch (grateMode) {
      case GRATE_SHAKE_OFF:
        grateMode = GRATE_SHAKE_ON;
        Serial.println("#Grate Mode: On");
        break;
      case GRATE_SHAKE_ON:
        grateMode = GRATE_SHAKE_PRATIO;
        Serial.println("#Grate Mode: Pressure Ratio");
        break;
      case GRATE_SHAKE_PRATIO:
        grateMode = GRATE_SHAKE_OFF;
        Serial.println("#Grate Mode: Off");
        break;
      }
      break;  
    case 'm':
      grate_max_interval += 5;
      grate_min_interval = grate_max_interval*0.5;
      putstring("#Grate Max Interval now:");
      Serial.println(grate_max_interval);
      putstring("#Grate Min Interval now:");
      Serial.println(grate_min_interval);
      break;
    case 'M':
      grate_max_interval -= 5;
      grate_min_interval = grate_max_interval*0.5;
      putstring("#Grate Max Interval now:");
      Serial.println(grate_max_interval);
      putstring("#Grate Min Interval now:");
      Serial.println(grate_min_interval);
      break;   
    case 'e':
      TransitionEngine(ENGINE_GOV_TUNING);
      break; 
    case 'x':
      testSD();
      break;
    case '!': //Clear EEPROM memory address
    { //extra bracket necessary for scope of memorySpace declaration:
      int memorySpace = SerialReadInt();
      if (memorySpace<4000){
        EEPROM.write(255, memorySpace);
      }
    }
      break;
    case '#':
      if (serial_num[0] == '/0') {
        Serial.println("# No serial saved, set line ending to 'Newline' and enter one now: ");
        SerialReadString(';');
        EEPROMWriteAlpha(40, 10, serial_buffer);
      }
      putstring("# Serial number: ");
      Serial.println(serial_num);
      break;
//   case 'h' || 'H':
//      Serial.println(HELP);
//      break; 
//   case '$':
//      SerialReadString(';');
//      Serial.println(serial_buffer);
//      break;
//   case 'W':  //write to config.ini
//      Serial.read
    //used values: $HheMmGgTtLlSscDdIiPp
    }
  }
  
}

int SerialReadInt(){
  byte incomingByte;
  int integerValue = 0;  
  while(1) {            
    incomingByte = Serial.read();
    if (incomingByte == '\n') break;   // exit the while(1), we're done receiving
    if (incomingByte == -1) continue;  // if no characters are in the buffer read() returns -1
    integerValue *= 10;  // shift left 1 decimal place
    // convert ASCII to integer, add, and shift left 1 decimal place
    integerValue = ((incomingByte - 48) + integerValue);
  }
  return integerValue;
}

void SerialReadString(char endString){
  byte incomingByte;
  int charCount = 0;
  unsigned long serial_time = millis();
  while(charCount <= 20){
    if (millis() - serial_time > 300) break;
    incomingByte = Serial.read();
    if (incomingByte == '\n' || incomingByte == endString) break;
    if (incomingByte == -1) continue;
    serial_buffer[charCount] = incomingByte;
    incomingByte++;
    serial_buffer[charCount] = '\0';
  }
}
    


void PrintLambdaUpdate(double P, double I, double D, double nP, double nI, double nD) {
  putstring("#Updating PID from [");
  Serial.print(P);
  putstring(",");
  Serial.print(I);
  putstring(",");
  Serial.print(D);
  putstring("] to [");
  Serial.print(nP);
  putstring(",");
  Serial.print(nI);
  putstring(",");
  Serial.print(nD);
  Serial.println("]");
}

