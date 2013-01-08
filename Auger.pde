void DoAuger() {
  checkAuger();
  switch (auger_state) {
  case AUGER_OFF:
    if (FuelDemand == SWITCH_ON) {
      if (relay_board == 1){
        TransitionAuger(AUGER_STARTING);
      } else {
        TransitionAuger(AUGER_FORWARD);
      }
    }
    if (P_reactorLevel == OFF) {
      auger_state_entered = millis(); //reset to zero if no vacuum and auger off
    }
    if (millis() - auger_state_entered > shutdown[ALARM_AUGER_OFF_LONG] && engine_state == ENGINE_ON){
      putstring("# Auger off too long, Engine Shutdown at: ");
      Serial.println((millis() - auger_state_entered));
      TransitionEngine(ENGINE_SHUTDOWN);
      TransitionAuger(AUGER_ALARM);
    } 
    if ((millis() - auger_state_entered) % 60000 > 59000) {  //pulse every minute of auger off
      putstring("# Pulsing Auger\n");
      TransitionAuger(AUGER_PULSE);
    } 
    break;
  case AUGER_CURRENT_LOW:
    if (FuelDemand == SWITCH_OFF) {
      TransitionAuger(AUGER_OFF);
    }
    if (AugerCurrentLevel != CURRENT_LOW && AugerCurrentLevel != CURRENT_OFF && millis() - auger_state_entered > 500){ //switch forward instead?
      TransitionAuger(AUGER_FORWARD);
    } 
    if ((millis() - auger_state_entered) > shutdown[ALARM_AUGER_LOW_CURRENT]){  //turn engine and auger off if auger current low for 3 minutes
      TransitionAuger(AUGER_ALARM);
      if (engine_state == ENGINE_ON){
        putstring("# Low Auger Current for too long, Engine Shutdown at");
        Serial.println((millis() - auger_state_entered));
        TransitionEngine(ENGINE_SHUTDOWN);
      }
    }
    break;
  case AUGER_STARTING:  //disregard all current readings while starting, pulse in reverse for a moment
    if (millis() - auger_state_entered > 500){
      TransitionAuger(AUGER_FORWARD);
    }
    break;
  case AUGER_FORWARD:
    if (FuelDemand == SWITCH_OFF) {
      TransitionAuger(AUGER_OFF);
    }
    if (AugerCurrentLevel == CURRENT_HIGH  && millis() - auger_state_entered > 500){
      TransitionAuger(AUGER_HIGH);
    } 
    if (AugerCurrentLevel == CURRENT_LOW or AugerCurrentLevel == CURRENT_OFF && millis() - auger_state_entered > 500){
      TransitionAuger(AUGER_CURRENT_LOW);
    } 
    if ((millis() - auger_state_entered) > shutdown[ALARM_AUGER_ON_LONG]){  //turn engine and auger off if auger runs none stop for too long
      TransitionAuger(AUGER_ALARM);
      if (engine_state == ENGINE_ON){
        putstring("# Auger on too long, Engine Shutdown at:");
        Serial.println((millis() - auger_state_entered));
        TransitionEngine(ENGINE_SHUTDOWN);
      }
    }
    break;
  case AUGER_HIGH:
    if (FuelDemand == SWITCH_OFF) {
      TransitionAuger(AUGER_OFF);
    }
    if (AugerCurrentLevel != CURRENT_HIGH){
      TransitionAuger(AUGER_FORWARD);
    }
    if (millis() - auger_state_entered > 500){ 
      TransitionAuger(AUGER_REVERSE);
    }
    break;
  case AUGER_REVERSE:
    if (FuelDemand == SWITCH_OFF) {
       TransitionAuger(AUGER_OFF);
     }
    if (millis() - auger_state_entered > 500  && AugerCurrentLevel == CURRENT_HIGH){
      TransitionAuger(AUGER_REVERSE_HIGH);
    }
    if (millis() - auger_reverse_entered > aug_rev_time){
      TransitionAuger(AUGER_FORWARD);
    }
    if (auger_rev_count > 20){  //catch oscillating auger from broken Fuel Switch
      putstring("# Auger Bound or broken Fuel Switch, stopping Auger\n");
      TransitionAuger(AUGER_ALARM);
      if (engine_state == ENGINE_ON){
        putstring("# Auger Oscillations, Engine Shutdown at: ");
        Serial.println((millis() - auger_state_entered));
        TransitionEngine(ENGINE_SHUTDOWN);
      }
    }
    break;
  case AUGER_REVERSE_HIGH:
    if (FuelDemand == SWITCH_OFF) {
       TransitionAuger(AUGER_OFF);
     }
    if (AugerCurrentLevel != CURRENT_HIGH){
      TransitionAuger(AUGER_REVERSE);
    }
    if (millis() - auger_state_entered > 500){ 
      TransitionAuger(AUGER_FORWARD);  //skip Auger starting as it has an initial reverse pulse
    }
    break; 
  case AUGER_ALARM:  //Auger will remain off until rebooted with a reset from front panel display
    break; 
  case AUGER_PULSE:
    if (millis() - auger_pulse_entered > auger_pulse_time){
      if (auger_pulse_state == 1){
        TransitionAuger(AUGER_PULSE);
      } else {
        TransitionAuger(AUGER_OFF);
      }
    }
    if (AugerCurrentLevel == CURRENT_HIGH && millis() - auger_pulse_entered > 500){
      if (auger_pulse_state == 1){  //if in reverse...try going forward
        TransitionAuger(AUGER_PULSE);
      } else {
      TransitionAuger(AUGER_OFF);
      }
    }  
    break;
  case AUGER_MANUAL_FORWARD:
    if (AugerCurrentLevel == CURRENT_HIGH  && millis() - auger_state_entered > 500){
      TransitionAuger(AUGER_HIGH);
    }
    break;
  }
}


void TransitionAuger(int new_state) {
  //can look at auger_state for "old" state before transitioning at the end of this method
  if (new_state != AUGER_PULSE && auger_state != AUGER_PULSE) {
    auger_state_entered = millis();
  }
  switch (new_state) {
  case AUGER_OFF:
    AugerOff();
    putstring("# New Auger State: Off\n");
    //TransitionMessage("Auger: Off         ");
    auger_rev_count = 0;
    auger_pulse_state = 0;
    break;
  case AUGER_STARTING:
    AugerReverse(); //start in reverse for a few moments to reduce bridging 
    putstring("# New Auger State: Starting Forward\n");  
    //TransitionMessage("Auger: Starting      "); 
    break;
  case AUGER_FORWARD:
    AugerForward();
    putstring("# New Auger State: Forward\n");
    //TransitionMessage("Auger: Forward      ");
    break;
  case AUGER_HIGH:
    //Serial.print("Current:");
    //Serial.print(AugerCurrentValue);
    //Serial.print(" current_low_boundary:");
    //Serial.print(current_low_boundary);
    //Serial.print(" current_high_boundary:");
    //Serial.println(current_high_boundary);
    putstring("# New Auger State: Forward, Current High\n");
    //TransitionMessage("Auger: Current High ");
    break;
  case AUGER_REVERSE:
    auger_reverse_entered = millis();
    putstring("# New Auger State: Reverse\n");
    AugerReverse();
    auger_rev_count++;
    putstring("# Auger Rev Count Incremented to ");
    Serial.println(auger_rev_count);
    //TransitionMessage("Auger: Reverse      ");
    break;
  case AUGER_REVERSE_HIGH:
    putstring("# New Auger State: Reverse High Current\n"); 
    //TransitionMessage("Auger: Reverse High"); 
    break; 
  case AUGER_CURRENT_LOW:
    //Serial.print("Current:");
    //Serial.print(AugerCurrentValue);
    //Serial.print(" current_low_boundary:");
    //Serial.print(current_low_boundary);
    //Serial.print(" current_high_boundary:");
    //Serial.println(current_high_boundary);
    putstring("# New Auger State: Current Low\n");
    //TransitionMessage("Auger: Low Current");
    break;
  case AUGER_ALARM:
    AugerOff();
    putstring("# New Auger State: Alarmed, Off\n");
    //TransitionMessage("Auger: Off          ");
    break; 
  case AUGER_PULSE:
    putstring("# New Auger State: Pulse\n");
    if (auger_pulse_state == 0){
      AugerReverse();
    } else {
      AugerForward();
    }
    auger_pulse_entered = millis();
    auger_pulse_state++;
    break;
  }
  auger_state=new_state;
}

void checkAuger(){
  FuelSwitchValue = analogRead(ANA_FUEL_SWITCH); // switch voltage, 1024 if on, 0 if off
  if (FuelSwitchValue > 600){
    if (FuelDemand == SWITCH_OFF){
      fuel_state_entered = millis();
    }
    FuelDemand = SWITCH_ON;
  } 
  else {
    FuelDemand = SWITCH_OFF;
    if (FuelDemand == SWITCH_ON){
      fuel_state_entered = millis();
    }
  }
  if (relay_board == 1){     //when relay board is present auger current sensing is enabled
    AugerCurrentValue = (10*(analogRead(ANA_AUGER_CURRENT)-120))/12;  //convert from analog values to current (.1A) values
    if (AugerCurrentValue > AugerCurrentLevelBoundary[CURRENT_OFF][0] && AugerCurrentValue < AugerCurrentLevelBoundary[CURRENT_OFF][1]) {
      AugerCurrentLevel = CURRENT_OFF;
    }
    if (AugerCurrentValue > AugerCurrentLevelBoundary[CURRENT_LOW][0] && AugerCurrentValue < AugerCurrentLevelBoundary[CURRENT_LOW][1]) {
      AugerCurrentLevel = CURRENT_LOW;
    }
    if (AugerCurrentValue > AugerCurrentLevelBoundary[CURRENT_ON][0] && AugerCurrentValue < AugerCurrentLevelBoundary[CURRENT_ON][1]) {
      AugerCurrentLevel = CURRENT_ON;
    }
    if (AugerCurrentValue > AugerCurrentLevelBoundary[CURRENT_HIGH][0] && AugerCurrentValue < AugerCurrentLevelBoundary[CURRENT_HIGH][1]) {
      AugerCurrentLevel = CURRENT_HIGH;
    }
  }
}

void AugerReverse(){
  if (relay_board == 1){ 
    digitalWrite(FET_AUGER,LOW);
    digitalWrite(FET_AUGER_REV, HIGH);
  }
}

void AugerForward(){
  digitalWrite(FET_AUGER, HIGH);
  if (relay_board == 1){ 
    digitalWrite(FET_AUGER_REV, LOW);
  }
}

void AugerOff(){
  digitalWrite(FET_AUGER,LOW);
  if (relay_board == 1){ 
    digitalWrite(FET_AUGER_REV, LOW);
  }
}


