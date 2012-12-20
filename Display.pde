void DoDisplay() {
  boolean disp_alt; // Var for alternating value display
  char choice[5] = "    ";
  char buf[20];
  if (millis() % 2000 > 1000) {
    disp_alt = false;
  } 
  else {
    disp_alt = true;
  }
  switch (display_state) {
  case DISPLAY_SPLASH:
    //Row 0
    Disp_RC(0,0);
    Disp_PutStr("    Power Pallet    ");
    //Row 1
    Disp_RC(1,0);
    Disp_PutStr("www.allpowerlabs.org");
    //Row 2
    Disp_RC(2,0);
    sprintf(buf, "       %s        ", CODE_VERSION);
    Disp_PutStr(buf);
    //Row 3
//    Disp_RC(3,0);
//    sprintf(buf, "       %s%i        ", serial_num, unique_number);
//    Disp_PutStr(buf);
//    Disp_CursOff();
    //Transition out after delay
    if (millis()-display_state_entered>2000) {
      TransitionDisplay(DISPLAY_REACTOR);
    }
    break;
  case DISPLAY_REACTOR:
    Disp_CursOff();
    alarm_shown = alarm_queue[cur_item - 1];
    if (millis() % 4000 > 2000 && (alarm_count > 0)) {
      item_count = alarm_count;
      if (cur_item>item_count) {  //if an alarm is removed while displaying start over at beginning
        cur_item = 1;
      } 
      //Row 0
      Disp_RC(0, 0);
      if (shutdown[alarm_shown]> 0){
        sprintf(buf, "SHUTDOWN ALARM %2i/%2i", cur_item, alarm_count); 
      } else {
        sprintf(buf, "      ALARM   %2i/%2i ", cur_item, alarm_count); 
      }
      Disp_PutStr(buf);
      //Row 1
      Disp_RC(1, 0);
      Disp_PutStr(display_alarm[alarm_shown]);
      //Row 2
      Disp_RC(2, 0);
      Disp_PutStr(display_alarm2[alarm_shown]);
      if (shutdown[alarm_shown] > 999 && engine_state == ENGINE_ON){     //anything less than 999 is probably a count and not a shutdown time in millisecond so don't show. 
        Disp_RC(2, 13);
        sprintf(buf, "OFF:%3i", (shutdown[alarm_shown] - alarm_start[alarm_shown] - (millis() - alarm_on[alarm_shown]))/1000);
        Disp_PutStr(buf);
      }
      //Row 3
      Disp_RC(3, 0);
      Disp_PutStr("NEXT ADV QUIET      ");
      if (millis() - alarm_on[alarm_shown] > 4000){ //Wait to show RESET button in case new alarm state has taken over screen.
        Disp_RC(3, 15);
        Disp_PutStr("RESET");
      }
    } else {
      //Row 0
      Disp_RC(0, 0);
      if (millis()-transition_entered<2000) {
      transition_message.toCharArray(buf,21);
      Disp_PutStr(buf);
      } else {
        if (disp_alt) {
          sprintf(buf, "Ttred%4i  ", Temp_Data[T_TRED]);
        } else {
          sprintf(buf, "Ttred%s", T_tredLevel[TempLevelName]);
        }
        Disp_PutStr(buf);
        Disp_RC(0, 11);
        sprintf(buf, "Pcomb%4i", Press[P_COMB] / 25);
        Disp_PutStr(buf);
      }

      //Row 1
      Disp_RC(1, 0);
      if (disp_alt) {
        sprintf(buf, "Tbred%4i  ", Temp_Data[T_BRED]);
      } 
      else {
        sprintf(buf, "Tbred%s", T_bredLevel[TempLevelName]);
      }
      Disp_PutStr(buf);
      Disp_RC(1, 11);
      sprintf(buf, "Preac%4i", Press[P_REACTOR] / 25);
      Disp_PutStr(buf);
      //Row 2
      Disp_RC(2,0);
      if (P_reactorLevel != OFF) {
        //the value only means anything if the pressures are high enough, otherwise it is just noise
        sprintf(buf, "Pratio%3i  ", int(pRatioReactor*100)); //pressure ratio
        Disp_PutStr(buf);
      } else {
        Disp_PutStr("Pratio --  ");
      }
      Disp_RC(2, 11);
      if (true) {
        sprintf(buf, "Pfilt%4i", Press[P_FILTER] / 25);
      } 
      else {
        //TO DO: Implement filter warning
        if (pRatioFilterHigh) {
          sprintf(buf, "Pfilt Bad");
        } 
        else {
          sprintf(buf, "PfiltGood");
        }
      }
      Disp_PutStr(buf);
      //Row 3
      Disp_RC(3,0);
      switch(auger_state){ 
      case AUGER_FORWARD:
        sprintf(buf, "AugFwd%3i  ", (millis() - auger_state_entered)/1000);
        break;
      case AUGER_OFF:
        if (P_reactorLevel == OFF) {
          sprintf(buf, "AugOff%s  ", " --"); 
        } else {
          sprintf(buf, "AugOff%3i  ", (millis() - auger_state_entered)/1000);  
        }  
        break;
      case AUGER_REVERSE:
        sprintf(buf, "AugRev%3i  ", (millis() - auger_state_entered)/1000); 
        break;
      case AUGER_HIGH:
        sprintf(buf, "AugHi %3i  ", (millis() - auger_state_entered)/1000); 
        break;
      case AUGER_CURRENT_LOW:
        sprintf(buf, "AugLow%3i  ", (millis() - auger_state_entered)/1000);
        break;
      case AUGER_REVERSE_HIGH:
        sprintf(buf, "AugRHi%3i  ", (millis() - auger_state_entered)/1000); 
        break;
      case AUGER_ALARM:
        sprintf(buf, "AugALRM%3i ", (millis() - auger_state_entered)/1000);
        break;
      default:
        sprintf(buf, "Aug   %3i  ", (millis() - auger_state_entered)/1000);
        break;
      }
      Disp_PutStr(buf);
      Disp_RC(3, 11);
      sprintf(buf, "   %6i", millis()/1000);
      //if (disp_alt) {
      //  sprintf(buf, "Hz   %4i", int(CalculatePeriodHertz()));
      //} else {
      //  sprintf(buf, "Batt%5i", int(battery_voltage*10));
      //  //sprintf(buf, "Pow %5i", int(CalculatePulsePower()));
      //}
      Disp_PutStr(buf);
    } 
    if (alarm_count > 0){ //keypresses for alarms only
      if (key == 2) {
        alarm = false;
      }
      if (millis() - alarm_on[alarm_shown] > 4000){ //wait until RESET button shows up, a wait of 4 seconds is given so that 
       if (key == 3) {
          removeAlarm(alarm_shown);
          resetAlarm(alarm_shown);
          cur_item = 1; //start at beginning of alarm queue
        }
      }
    }
    break;
  case DISPLAY_ENGINE:
    Disp_CursOff();
    Disp_RC(0,0);
#if T_ENG_COOLANT != ABSENT
    sprintf(buf, "Tcool%4i  ", Temp_Data[T_ENG_COOLANT]);
#else
    sprintf(buf, "Tcool  NA  ");
#endif
    Disp_PutStr(buf);
    Disp_RC(0,11); 
    Disp_PutStr("           ");
    //Row 1
    Disp_RC(1,0); 
    Disp_PutStr("                    ");
    //Row 2
    Disp_RC(2,0); 
    Disp_PutStr("                    ");
    //Row 3
    Disp_RC(3,0);
    Disp_PutStr("                    ");
    Disp_CursOff();
    break;
  case DISPLAY_TESTING:
    Disp_CursOff();
    item_count = 1;
    //Row 0
    Disp_RC(0,0);
    Disp_PutStr("Testing             "); 
    //Row 1			
    Disp_RC(1,0);
    sprintf(buf, "Test:%-15s", TestingStateName[testing_state]);
    Disp_PutStr(buf);
    //Row 2
    Disp_RC(2,0);
    switch (testing_state) {
    case TESTING_ANA_LAMBDA:
      sprintf(buf, "Value: %4i         ", int(analogRead(ANA_LAMBDA)));
      break;
    case TESTING_ANA_ENGINE_SWITCH:
      sprintf(buf, "Value: %4i         ", int(analogRead(ANA_ENGINE_SWITCH)));
      break;
    case TESTING_ANA_FUEL_SWITCH:
      sprintf(buf, "Value: %4i         ", int(analogRead(ANA_FUEL_SWITCH)));
      break;
    case TESTING_ANA_OIL_PRESSURE:
      sprintf(buf, "Value: %4i         ", int(analogRead(ANA_OIL_PRESSURE)));
      break;
    case TESTING_GOV_TUNING:
      break;
    default:
      sprintf(buf,"                   ");
    }
    Disp_PutStr(buf);
    //Row 3
    switch (cur_item) {
    case 1: // Testing 
      if (key == 2) {
        GoToNextTestingState(); //first testing state
      }
      Disp_RC(3,0);
      Disp_PutStr("NEXT       TEST     ");
      break;
    default:
      Disp_RC(3,0);
      Disp_PutStr("NEXT                ");
    }
    break;
  case DISPLAY_LAMBDA:
    Disp_CursOff();
    double P,I;
    item_count = 4;
    P=lambda_PID.GetP_Param();
    I=lambda_PID.GetI_Param();
    Disp_RC(0,0);
    sprintf(buf, "LamSet%3i  ", int(ceil(lambda_setpoint*100.0)));
    Disp_PutStr(buf);
    Disp_RC(0,11);
    sprintf(buf, "Lambda%3i", int(lambda_input*100.0));
    Disp_PutStr(buf);
    //Row 1
    Disp_RC(1,0);
    sprintf(buf, "P     %3i  ", int(ceil(lambda_PID.GetP_Param()*100.0)));
    Disp_PutStr(buf);
    Disp_RC(1,11);
    sprintf(buf, "I     %3i", int(ceil(lambda_PID.GetI_Param()*100.0)));
    Disp_PutStr(buf);
    Disp_RC(2,0);
    Disp_PutStr("                    ");
    switch (cur_item) {
    case 1: // Lambda setpoint
      if (key == 2) {
        lambda_setpoint += 0.01;
        WriteLambda();
      }
      if (key == 3) {
        lambda_setpoint -= 0.01;
        WriteLambda();
      }          
      Disp_RC(0,0);
      Disp_CursOn();
      Disp_RC(3,0);
      strcpy_P(buf, menu1);
      Disp_PutStr(buf);
      break;
    case 2: //Lambda reading
      Disp_RC(3,0);
      Disp_PutStr("NEXT  ADV           ");
      Disp_RC(0,11);
      Disp_CursOn();
      break;
    case 3: //Lambda P
      if (key == 2) {
        P += 0.01;
        lambda_P[0] = P;
        WriteLambda();
      }
      if (key == 3) {
        P -= 0.01;
        lambda_P[0] = P;
        WriteLambda();
      }
      lambda_PID.SetTunings(P,I,0);
      Disp_RC(3,0);
      strcpy_P(buf, menu1);
      Disp_PutStr(buf);
      Disp_RC(1,0);
      Disp_CursOn();
      break;
    case 4: //Lambda I
      if (key == 2) {
        I += 0.1;
        lambda_I[0] = I;
        WriteLambda();
      }
      if (key == 3) {
        I -= 0.1;
        lambda_I[0] = I;
        WriteLambda();
      }
      lambda_PID.SetTunings(P,I,0);
      Disp_RC(3,0);
      strcpy_P(buf, menu1);
      Disp_PutStr(buf);
      Disp_RC(1,11);
      Disp_CursOn();
      break;
    }
    break;
  case DISPLAY_GRATE: 
    int vmin,vmax;
    item_count = 4;
    Disp_RC(0,0);
    sprintf(buf, "GraMin%3i  ", grate_min_interval);
    Disp_PutStr(buf);
    Disp_RC(0,11);
    sprintf(buf, "GraMax%3i", grate_max_interval);
    Disp_PutStr(buf);
    //Row 1
    Disp_RC(1,0);
    sprintf(buf, "GraLen%3i  ", grate_on_interval);
    Disp_PutStr(buf);
    Disp_RC(1,11);
    if (grate_motor_state == GRATE_MOTOR_OFF) {  
      Disp_PutStr("Grate Off");
    } 
    else {
      Disp_PutStr("Grate  On");
    }
    Disp_RC(2,0);
    Disp_PutStr("                    ");
    switch (cur_item) {
    case 1: // Grate Min Interval
      vmin = max(0,grate_on_interval);
      vmax = grate_max_interval;
      if (key == 2) {

        grate_min_interval += 3;
        grate_min_interval = constrain(grate_min_interval,vmin,vmax);
        CalculateGrate();
        WriteGrate();
      }
      if (key == 3) {
        grate_min_interval -= 3;
        grate_min_interval = constrain(grate_min_interval,vmin,vmax);
        CalculateGrate();
        WriteGrate();
      }
      Disp_RC(3,0);
      strcpy_P(buf, menu1);
      Disp_PutStr(buf);
      Disp_RC(0,0);
      Disp_CursOn();
      break;
    case 2: //Grate Interval
      vmin = max(grate_on_interval,grate_min_interval);
      vmax = 255*3;
      if (key == 2) {
        grate_max_interval += 3;
        grate_max_interval = constrain(grate_max_interval,vmin,vmax);
        CalculateGrate();
        WriteGrate();
      }
      if (key == 3) {
        grate_max_interval -= 3;
        grate_max_interval = constrain(grate_max_interval,vmin,vmax);
        CalculateGrate();
        WriteGrate();
      }
      Disp_RC(3,0);
      strcpy_P(buf, menu1);
      Disp_PutStr(buf);
      Disp_RC(0,11);
      Disp_CursOn();
      break;
    case 3: //Grate On Interval
      vmin = 0;
      vmax = min(grate_min_interval,255);
      if (key == 2) {
        grate_on_interval += 1;
        grate_on_interval = constrain(grate_on_interval,vmin,vmax);
        CalculateGrate();
        WriteGrate();
      }
      if (key == 3) {
        grate_on_interval -= 1;
        grate_on_interval = constrain(grate_on_interval,vmin,vmax);
        CalculateGrate();
        WriteGrate();
      }
      Disp_RC(3,0);
      strcpy_P(buf, menu1);
      Disp_PutStr(buf);
      Disp_RC(1,0);
      Disp_CursOn();
      break;
    case 4: //Grate
      if (key == 2) {
        grate_motor_state = GRATE_MOTOR_OFF;
      }
      if (key == 3) {
      grate_val = GRATE_SHAKE_CROSS;
      }
      Disp_RC(3,0);
      Disp_PutStr("NEXT  ADV  OFF   ON ");
      Disp_RC(1,11);
      Disp_CursOn();
      break; 
    }
    break;
  case DISPLAY_SERVO:   //need to add constraints for min and max?
    item_count = 2;
    testing_state = TESTING_SERVO;  //necessary so that there isn't any conflicting servo writes
    Disp_RC(0,0);
    sprintf(buf, "ServoMin%3i", int(premix_valve_closed));
    Disp_PutStr(buf);
    Disp_RC(0,11);
    sprintf(buf, " Max %3i", int(premix_valve_open));
    Disp_PutStr(buf);
    //Row 1
    Disp_RC(1,0);
    Disp_PutStr(" Careful of Sides!  "); 
    Disp_RC(2,0);
    Disp_PutStr("                    ");
    switch (cur_item) {
    case 1: // Servo Min
      Servo_Mixture.write(premix_valve_closed);
      if (key == 2) {
        if (premix_valve_closed + 1 < premix_valve_open){
          premix_valve_closed += 1;
        }
      }
      if (key == 3) {
        premix_valve_closed -= 1;
      }
      Disp_RC(3,0);
      strcpy_P(buf, menu1);
      Disp_PutStr(buf);
      Disp_RC(0,0);
      Disp_CursOn();
      break;
    case 2: //Servo Max 
      Servo_Mixture.write(premix_valve_open);
      if (key == 2) {
        premix_valve_open += 1;
      }
      if (key == 3) {
        if (premix_valve_open - 1 > premix_valve_closed) {
          premix_valve_open -= 1;
        }
      }
      Disp_RC(3,0);
      strcpy_P(buf, menu1);
      Disp_PutStr(buf);
      Disp_RC(0,11);
      Disp_CursOn();
      break;
    }
    break;
  case DISPLAY_CALIBRATE_PRESSURE:
    Disp_CursOff();
    item_count = 1;
    Disp_RC(0,0);
    Disp_PutStr("Calibrate Pressure  ");
    Disp_RC(1,0);
    Disp_PutStr("Sensors to zero?    ");
    Disp_RC(2,0);
    Disp_PutStr("                    ");
    Disp_RC(3,0);
    Disp_PutStr("NEXT       YES      ");
    if (key == 2) {
      CalibratePressureSensors();
      LoadPressureSensorCalibration();
      Disp_RC(2,0);
      Disp_PutStr("   CALIBRATED!      ");
    }
    break;
//  case DISPLAY_RELAY:
//    Disp_CursOff();
//    item_count = 8;
//    testing_state = TESTING_SERVO;
//    Disp_RC(0,0);
//    sprintf(buf, "Test Relay: %1i       ", cur_item);
//    Disp_PutStr(buf);
//    Disp_RC(1,0);
//    Disp_PutStr("                    ");
//    Disp_RC(2,0);
//    Disp_PutStr("                    ");
//    Disp_RC(3,0);
//    Disp_PutStr("NEXT  ADV   ON   OFF");
//    if (key == 2) {
//      relayOn(cur_item);
//      Disp_RC(1,0);
//      Disp_PutStr("           ON       ");
//    }
//    if (key == 3) {
//      relayOff(cur_item);
//      Disp_RC(1,0);
//    Disp_PutStr("           OFF        ");
//    }
//    break;
  case DISPLAY_CONFIG:
    Disp_CursOff();
    item_count = CONFIG_COUNT - 1; //sizeof(defaults)/sizeof(int);
    if (config_changed == false){
      config_var = getConfig(cur_item);
    }
    if (config_var == 255){  //EEPROM default state, not a valid choice.  Loops choice back to zero.
      config_var = 0;
    }
    if (config_var == -1){  //keeps values from being negative
      config_var = 254;
    }
    Disp_RC(0,0);
    Disp_PutStr("   Configurations   ");
    Disp_RC(1,0);
    if (Config_Choices[cur_item] == "+    -  "){
      sprintf(buf, "%s:%3i", Configuration[cur_item], config_var);
    } else if (Config_Choices[cur_item] == "+5  -5  "){
      sprintf(buf, "%s:%3i", Configuration[cur_item], 5*config_var);
    } else {
      if (config_var == 0){
      choice[0] = Config_Choices[cur_item][0];
      choice[1] = Config_Choices[cur_item][1];
      choice[2] = Config_Choices[cur_item][2];
      choice[3] = Config_Choices[cur_item][3];
      } else {
        choice[0] = Config_Choices[cur_item][4];
        choice[1] = Config_Choices[cur_item][5];
        choice[2] = Config_Choices[cur_item][6];
        choice[3] = Config_Choices[cur_item][7];
      }
      sprintf(buf, "%s:%s", Configuration[cur_item], choice);
    }
    Disp_PutStr(buf);
    Disp_RC(2,0);
    Disp_PutStr("ADV to save choice  ");
    Disp_RC(3,0);
    sprintf(buf, "NEXT  ADV   %s", Config_Choices[cur_item]);
    Disp_PutStr(buf);
    if (Config_Choices[cur_item] == "+    -  "){
      if (key == 2) {
        if (config_max[cur_item] >= config_var + 1){
          config_var += 1;
          config_changed = true;
        }
      }
      if (key == 3) {
        if (config_min[cur_item] <= config_var - 1){
          config_var -= 1;
          config_changed = true;
        }
      }
    } else if (Config_Choices[cur_item] == "+5  -5  "){
      if (key == 2) {
        if (config_max[cur_item] >= config_var + 1){
          config_var += 1;
          config_changed = true;
        }
      }
      if (key == 3) {
        if (config_min[cur_item] <= config_var - 1){
          config_var -= 1;
          config_changed = true;
        }
      }
    } else {
      if (key == 2) {  
        config_var = 0;
        config_changed = true;
      }
      if (key == 3) {
        config_var = 1;
        config_changed = true;
      }
    }
    break;
//  case DISPLAY_SD:
//    Disp_CursOff();
//    Disp_RC(0,0);
//    Disp_PutStr("   Test SD Card     ");
//    Disp_RC(1,0);
//    Disp_PutStr("See Serial Monitor  ");
//    Disp_RC(2,0);
//    Disp_PutStr("for more information");
//    Disp_RC(3,0);
//    Disp_PutStr("NEXT            TEST");
//    if (key == 3 and cur_item == 1) {  //only allow the button to be pressed once
//      testSD();
//      cur_item++;  
//    }
//    break;
//  case DISPLAY_PHIDGET:
//      Disp_CursOff();
////    Disp_RC(0,0);
////    sprintf(buf, "O2%4i Fu%4iKey%4i", analogRead(ANA0),analogRead(ANA1),analogRead(ANA2));
////    Disp_PutStr(buf);
////    Disp_RC(1,0);
////    sprintf(buf, "Oil%4iAug%4iTh%4i", analogRead(ANA3),analogRead(ANA4),analogRead(ANA5));
////    Disp_PutStr(buf);
////    Disp_RC(2,0);
////    sprintf(buf, "CoolT%4i Aux%4i   ", analogRead(ANA6),analogRead(ANA7));
////    Disp_PutStr(buf);  
////    Disp_RC(3,0);
////    Disp_PutStr("NEXT                ");
//    Disp_RC(0,0);
//    sprintf(buf, "Phidgits:     0:%4i", analogRead(ANA0));
//    Disp_PutStr(buf);
//    Disp_RC(1,0);
//    sprintf(buf, "1:%4i 2:%4i 3:%4i", analogRead(ANA1),analogRead(ANA2),analogRead(ANA3));
//    Disp_PutStr(buf);
//    Disp_RC(2,0);
//    sprintf(buf, "4:%4i 5:%4i 6:%4i", analogRead(ANA4),analogRead(ANA5),analogRead(ANA6));
//    Disp_PutStr(buf);  
//    Disp_RC(3,0);
//    sprintf(buf, "NEXT          7:%4i", analogRead(ANA7));
//    Disp_PutStr(buf);
//    break;
    //    case DISPLAY_TEMP2:
    //      break;
    //    case DISPLAY_FETS:
    //      break;
  }
  key = -1; //important, must clear key to read new input
}

void TransitionDisplay(int new_state) {
  //Enter
  display_state_entered = millis();
  switch (new_state) {
  case DISPLAY_SPLASH:
    break;
  case DISPLAY_REACTOR:
    cur_item = 1;
    break;
  case DISPLAY_ENGINE:
    break;
  case DISPLAY_LAMBDA:
    cur_item = 1;
    break;
  case DISPLAY_GRATE:
    cur_item = 1;
    break;
  case DISPLAY_TESTING:
    cur_item = 1;
    break;
  case DISPLAY_SERVO:
    cur_item = 1;
    break;
  case DISPLAY_CALIBRATE_PRESSURE:
    cur_item = 1;
    break;
//  case DISPLAY_RELAY:
//    cur_item = 1;
//    break; 
  case DISPLAY_CONFIG: 
    cur_item = 0;
    config_changed = false;
    break;
//  case DISPLAY_PHIDGET: 
//    break;
  case DISPLAY_SD:
    cur_item = 1;
    break;
  }
  display_state=new_state;
}

void DoKeyInput() {
  int k;
  k =  Kpd_GetKeyAsync();
  if (key == -1) { //only update key if it has been cleared
    key = k;
  }
  if (key == 0) {
    switch (display_state) {
    case DISPLAY_SPLASH:
      TransitionDisplay(DISPLAY_REACTOR);
      break;
    case DISPLAY_REACTOR:
      TransitionDisplay(DISPLAY_LAMBDA);
      break;
    case DISPLAY_ENGINE:
      TransitionDisplay(DISPLAY_REACTOR);
      break;
    case DISPLAY_LAMBDA:
      TransitionDisplay(DISPLAY_GRATE);
      break;
    case DISPLAY_GRATE:
      if (engine_state == ENGINE_OFF) {
        TransitionDisplay(DISPLAY_TESTING);
      } 
      else {
        TransitionDisplay(DISPLAY_REACTOR);
      }
      break;
    case DISPLAY_TESTING:
      if (engine_state == ENGINE_OFF){
        TransitionDisplay(DISPLAY_SERVO);
      } else {
      TransitionDisplay(DISPLAY_REACTOR);
      TransitionTesting(TESTING_OFF);
      }
      break;
    case DISPLAY_SERVO:
      WriteServo();
      TransitionDisplay(DISPLAY_CALIBRATE_PRESSURE);  //assume that engine state is off because we are already in DISPLAY_SERVO
      TransitionTesting(TESTING_OFF);
      break;
    case DISPLAY_CALIBRATE_PRESSURE:                 //assume that engine state is off
//      TransitionDisplay(DISPLAY_RELAY);
//      break;
//    case DISPLAY_RELAY:
      TransitionDisplay(DISPLAY_CONFIG);
      TransitionTesting(TESTING_OFF);
      break;
    case DISPLAY_CONFIG:
//      TransitionDisplay(DISPLAY_PHIDGET);
//      break;
//    case DISPLAY_PHIDGET:
//      TransitionDisplay(DISPLAY_SD);
//      break;
//    case DISPLAY_SD:
      TransitionDisplay(DISPLAY_REACTOR);
      break;
    }
    key = -1; //key caught
  }
  if (key == 1) {
    if (display_state == DISPLAY_CONFIG and config_changed == true){
      saveConfig(cur_item, config_var);
      update_config_var(cur_item);
      config_changed = false;
    }
    cur_item += 1;
    if (cur_item > item_count) {
      if (display_state == DISPLAY_CONFIG){
        cur_item = 0;
      } else {
      cur_item = 1;
      }
    } 
    key = -1; //key caught
  }
}

void DoHeartBeat() {
  if (millis() % 50 > 5) {
    bitSet(PORTJ, 7);
  } 
  else {
    bitClear(PORTJ, 7);
  }
  //PORTJ ^= 0x80;    // toggle the heartbeat LED
}

void TransitionMessage(String t_message) {
  transition_message = t_message;
  transition_entered = millis();
}

void saveConfig(int item, int state){  //EEPROM:  0-499 for internal states, 500-999 for configurable states, 1000-4000 for data logging configurations.
  if (item == 0  and state == 1){
    resetConfig();
  }
  if (item > 0){  //skip first config 
    int old_state = EEPROM.read(499+item);
    if(state != old_state){
      EEPROM.write(499+item, state);
      delay(5); //ensure that value is not read until EERPROM has been fully written (~3.3ms)
    }
  }
}

int getConfig(int item){
  int value = 0;
  if (item > 0){  //Config item zero is 'Reset to Defaults?' so skip
    value = int(EEPROM.read(499+item));
    if (value == 255){  //values hasn't been saved yet to EEPROM, go with default value saved in defaults[]
      value = defaults[item];
      EEPROM.write(499+item, value);
    }
  }
  config_changed = true;
  return value;
}

void update_config_var(int var_num){
  switch (var_num) {
    case 0:
      putstring("# Updating Configurations to Defaults\n");
      for (int i=1; i<CONFIG_COUNT; i++){
        update_config_var(i);
      }
      break;
    case 1:
      engine_type = getConfig(1);
      //Serial.println("Updating engine_type");
      break;
    case 2:
      relay_board = getConfig(2);
      //Serial.println("Updating relay_board");
      break;
    case 3:
      aug_rev_time = getConfig(3)*100;
      //Serial.println("Updating aug_rev_time");
      break;
    case 4:
      current_low_boundary = getConfig(4); 
      //{ { -140, 10}, {10, current_low_boundary}, {current_low_boundary+5, current_high_boundary-5}, {current_high_boundary, 750} }
      AugerCurrentLevelBoundary[CURRENT_LOW][1] = current_low_boundary; 
      AugerCurrentLevelBoundary[CURRENT_ON][0] = current_low_boundary+5;
      //Serial.print("#Updating current_low_boundary: "); 
      //Serial.println(current_low_boundary); 
      break;
    case 5:
      current_high_boundary = getConfig(5);
      AugerCurrentLevelBoundary[CURRENT_ON][1] = current_high_boundary - 5; 
      AugerCurrentLevelBoundary[CURRENT_HIGH][0] = current_high_boundary;
      //Serial.print("# Updating current_high_boundary: ");
      //Serial.println(current_high_boundary);
      break;
    case 6:
      low_oil_psi = getConfig(6);
      break;
    case 7:
      save_datalog_to_sd = getConfig(7);
      break;
    case 8:
      pratio_max = getConfig(8)*5;
      break;
    case 9:
      high_coolant_temp = getConfig(9)*5;
      break;
    case 10:
      display_per = getConfig(10)*5;
      loopPeriod2 = display_per;
      break;
  }
}

void resetConfig() {  //sets EEPROM configs back to untouched state...
  int default_count = sizeof(defaults)/sizeof(int);
  for (int i=1; i < default_count; i++){
    EEPROM.write(499+i, defaults[i]);
  }
}
