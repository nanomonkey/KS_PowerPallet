void InitGrate() {
  //setup grate slopes
  //LoadGrate();
  CalculateGrate();
}

void CalculateGrate() {
  m_grate_bad = (GRATE_SHAKE_INIT-GRATE_SHAKE_CROSS)/grate_min_interval;
  m_grate_good = (GRATE_SHAKE_INIT-GRATE_SHAKE_CROSS)/grate_max_interval;
  m_grate_on = GRATE_SHAKE_CROSS/grate_on_interval;
}

void DoGrate() { // call once per second
  pRatioReactor = (float)Press[P_COMB]/(float)Press[P_REACTOR];
  if (pRatioReactor > pRatioReactorLevelBoundary[PR_LOW][0] && pRatioReactor < pRatioReactorLevelBoundary[PR_LOW][1]) {
    pRatioReactorLevel = PR_LOW;
  }
  if (pRatioReactor > pRatioReactorLevelBoundary[PR_CORRECT][0] && pRatioReactor < pRatioReactorLevelBoundary[PR_CORRECT][1]) {
    pRatioReactorLevel = PR_CORRECT;
  }
  if (pRatioReactor > pRatioReactorLevelBoundary[PR_HIGH][0] && pRatioReactor < pRatioReactorLevelBoundary[PR_HIGH][1]) {
    pRatioReactorLevel = PR_HIGH;
  }
  
  // if pressure ratio is bad for a long time, shake harder
  if (pRatioReactorLevel == PR_LOW && Press[P_REACTOR] < -50 && Press[P_COMB] < -50) {
    grate_pratio_accumulator++;
  } else {
    grate_pratio_accumulator -= 5;
  }
  grate_pratio_accumulator = max(0,grate_pratio_accumulator); // don't let it go below 0
  
  // handle different shaking modes
  switch (grateMode) {
  case GRATE_SHAKE_ON:
    digitalWrite(FET_GRATE,HIGH);
    grate_motor_state = GRATE_MOTOR_LOW;
    break;
  case GRATE_SHAKE_OFF:
    digitalWrite(FET_GRATE,LOW);
    grate_motor_state = GRATE_MOTOR_OFF;
    break;
  case GRATE_SHAKE_PRATIO:
    if (engine_state == ENGINE_ON || engine_state == ENGINE_STARTING || P_reactorLevel != OFF) { //shake only if reactor is on and/or engine is on
      //condition above will leave grate_val in the last state until conditions are met (not continuing to cycle)
      if (grate_val >= GRATE_SHAKE_CROSS) { // not time to shake
        if (pRatioReactorLevel == PR_LOW) {
          grate_val -= m_grate_bad;
        } else {
          grate_val -= m_grate_good;
        }
        digitalWrite(FET_GRATE,LOW);
        grate_motor_state = GRATE_MOTOR_OFF;
      }
    }
    if (grate_val >= 0 && grate_val <= GRATE_SHAKE_CROSS) { //time to shake or reset
      grate_motor_state = GRATE_MOTOR_LOW;
      digitalWrite(FET_GRATE,HIGH); 
      grate_val -= m_grate_on;
    } else if (grate_val <= 0) {
      grate_val = GRATE_SHAKE_INIT;
      grate_motor_state = GRATE_MOTOR_OFF;
      digitalWrite(FET_GRATE,LOW);
    } else {
      digitalWrite(FET_GRATE, LOW);
      grate_motor_state = GRATE_MOTOR_OFF;
    }
    break;
  }
}

//
//void LoadGrate() {
//  byte check;
//  double ming,maxg,gon;
//  check = EEPROM.read(16); 
//  ming = EEPROM.read(17)*3;
//  maxg = EEPROM.read(19)*3;
//  gon= EEPROM.read(21);
//  if (check == 128) { //check to see if grate has been set
//    Logln_p("Loading grate from EEPROM");
//    grate_min_interval = ming;
//    grate_max_interval = maxg;
//    grate_on_interval = gon;
//  } else {
//    WriteGrate();
//  }
//}
//
//void WriteGrate() {
//  Logln_p("Writing grate to EEPROM");
//  EEPROM.write(16,128);
//  EEPROM.write(17,constrain(grate_min_interval/3,0,255));
//  EEPROM.write(19,constrain(grate_max_interval/3,0,255));
//  EEPROM.write(21,constrain(grate_on_interval,0,255));
//}
