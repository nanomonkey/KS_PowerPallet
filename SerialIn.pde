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
//    case 's':
//      Servo_Calib.write(Servo_Calib.read()+10);
//      Log_p("Servo1 (degrees) now:");
//      Logln(Servo_Calib.read());
//      break;
//    case 'S':
//      Servo_Calib.write(Servo_Calib.read()-10);
//      Log_p("Servo1 (degrees) now:");
//      Logln(Servo_Calib.read());
//      break;
    case 'l':
      lambda_setpoint += 0.01;
      Log_p("Lambda Setpoint now:");
      Logln(lambda_setpoint);
      WriteLambda();
      break;
    case 'L':
      lambda_setpoint -= 0.01;
      Log_p("Lambda Setpoint now:");
      Logln(lambda_setpoint);
      WriteLambda();
      break;
    case 't':
      loopPeriod1 = max(loopPeriod1-100,100);
      Log_p("Sample Period now:");
      Logln(loopPeriod1);
      break;
    case 'T':
      loopPeriod1 = min(loopPeriod1+100,10000);
      Log_p("Sample Period now:");
      Logln(loopPeriod1);
      break;
    case 'g':  
      grate_val = GRATE_SHAKE_CROSS; //set grate val to shake for grate_on_interval
      Logln("Grate Shaken");
      break;
    case 'G':  
      switch (grateMode) {
      case GRATE_SHAKE_OFF:
        grateMode = GRATE_SHAKE_ON;
        Logln("Grate Mode: On");
        break;
      case GRATE_SHAKE_ON:
        grateMode = GRATE_SHAKE_PRATIO;
        Logln("Grate Mode: Pressure Ratio");
        break;
      case GRATE_SHAKE_PRATIO:
        grateMode = GRATE_SHAKE_OFF;
        Logln("Grate Mode: Off");
        break;
      }
      break;  
    case 'm':
      grate_max_interval += 5;
      grate_min_interval = grate_max_interval*0.5;
      Log_p("Grate Max Interval now:");
      Logln(grate_max_interval);
      Log_p("Grate Min Interval now:");
      Logln(grate_min_interval);
      break;
    case 'M':
      grate_max_interval -= 5;
      grate_min_interval = grate_max_interval*0.5;
      Log_p("Grate Max Interval now:");
      Logln(grate_max_interval);
      Log_p("Grate Min Interval now:");
      Logln(grate_min_interval);
      break;   
    case 'e':
      TransitionEngine(ENGINE_GOV_TUNING);
      break; 
//    case 'x':
//      testSD();
//      break;
    case '!': //Clear EEPROM memory address
    { //extra bracket necessary for scope of memorySpace declaration:
      int memorySpace = SerialReadInt();
      if (memorySpace<4000){
        EEPROM.write(memorySpace, 255);
      }
      Log_p("Rewriting eeprom space "); Logln(memorySpace);
      Log_p("Changed to "); Logln(int(EEPROM.read(memorySpace)));
    }
      break;
    case '-':
      loopPeriod2 = max(loopPeriod2-100,100);
      Log_p("Display Period (ms):");
      Logln(loopPeriod2);
      break;
    case '+':
      loopPeriod2 = min(loopPeriod2+100,1000);
      Log_p("Display Period (ms):");
      Logln(loopPeriod2);
      break;
    case '#':
      serial_buffer[0] = '\0';
      SerialReadString(';');
      if (serial_buffer[0] != '\0'){
        EEPROMWriteAlpha(40, 10, serial_buffer);
      }
      EEPROMReadAlpha(40, 10, p_buffer);
      Log_p("Serial number: ");
      Logln(p_buffer);
      break;
//   case 'h' || 'H':
//      Logln(P(help));
//      break; 
//   case '$':
//      SerialReadString(';');
//      Logln(serial_buffer);
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
    //if (millis() - serial_time > 300) break;
    incomingByte = Serial.read();
    if (incomingByte == '\n' || incomingByte == endString) break;
    if (incomingByte == -1) continue;
    serial_buffer[charCount] = incomingByte;
    incomingByte++;
    serial_buffer[charCount+1] = '\0';
    charCount += 1;
  }
}
    


void PrintLambdaUpdate(double P, double I, double D, double nP, double nI, double nD) {
  Log_p("Updating PID from [");
  Serial.print(P);
  Log_p(",");
  Serial.print(I);
  Log_p(",");
  Serial.print(D);
  Log_p("] to [");
  Serial.print(nP);
  Log_p(",");
  Serial.print(nI);
  Log_p(",");
  Serial.print(nD);
  Logln(P("]"));
}

