void InitModbusSlave(){  //include in Setup() loop
    
    regs[MB_CONFIG1]=getConfig(1); //engine_type;
    regs[MB_CONFIG2]=getConfig(2); //relay_board;
    regs[MB_CONFIG3]=getConfig(3); //aug_rev_time;
    regs[MB_CONFIG4]=getConfig(4); //current_low_boundary;
    regs[MB_CONFIG5]=getConfig(5); //current_high_boundary;
    regs[MB_CONFIG6]=getConfig(6); //low_oil_psi;
    regs[MB_CONFIG7]=getConfig(7); //save_datalog_to_sd;
    regs[MB_CONFIG8]=getConfig(8); //pratio_max;
    regs[MB_CONFIG9]=getConfig(9); //high_coolant_temp;
    regs[MB_CONFIG10]=getConfig(10); //display_per;
    regs[MB_CONFIG11]=getConfig(11); //tred_low_temp;
    regs[MB_CONFIG12]=getConfig(12); //ttred_high;
    regs[MB_CONFIG13]=getConfig(13); //tbred_high;
    regs[MB_CONFIG14]=getConfig(14); //pfilter_alarm;
    regs[MB_CONFIG15]=getConfig(15); //grate_max_interval;
    regs[MB_CONFIG16]=getConfig(16); //grate_min_interval;
    regs[MB_CONFIG17]=getConfig(17); //grate_on_interval;
    regs[MB_CONFIG18]=getConfig(18); //servo_start;
    regs[MB_CONFIG19]=getConfig(19); //lambda_rich;
    regs[MB_CONFIG20]=getConfig(20); //use_modbus;
    regs[MB_CONFIG21]=getConfig(21); //m_baud;
    regs[MB_CONFIG22]=getConfig(22); //m_parity;
    regs[MB_CONFIG23]=getConfig(23); //m_address;
    
    init_mb_slave(baud_rates[m_baud], parity[m_parity], 16);  //baud, parity, tx_en_pin
}


void DoModbus() {
    /* This is all for the Modbus slave */
    start_mb_slave(m_address, regs, MB_REGS);
    
    regs[MB_ALARMS] = getAlarmBin();
    regs[MB_T_TRED] = Temp_Data[T_TRED]; 	 
    regs[MB_T_BRED] = Temp_Data[T_BRED];	 
    regs[MB_T_REACTOR_GAS_OUT] = Temp_Data[T_REACTOR_GAS_OUT];  //(ABSENT?)
    regs[MB_T_ENG_COOLANT] = Temp_Data[T_ENG_COOLANT];
    regs[MB_P_REACTOR] = Press[P_REACTOR];
    regs[MB_P_FILTER] = Press[P_FILTER];
    regs[MB_P_Q_AIR_ENG] = Press[P_Q_AIR_ENG]; //ABSENT;
    regs[MB_P_COMB] = Press[P_COMB]; 	 
    regs[MB_P_Q_AIR_RCT] = Press[P_Q_AIR_RCT];   //(ABSENT?)
    regs[MB_P_Q_GAS_ENG] = Press[P_Q_GAS_ENG];  //(ABSENT?)
    regs[MB_GRATE_STATE] = grate_motor_state;
    regs[MB_ENGINE_STATE] = engine_state;
    regs[MB_AUGER_STATE] = auger_state;
    regs[MB_FLARE_STATE] = flare_state;
    regs[MB_BLOWER_STATE] = ABSENT; //(NOT SURE IF THIS CAN BE COMBINED WITH FLARE)
    regs[MB_ENGINE_STATE] = engine_state;
    regs[MB_P_RATIO_REACTOR] = pRatioReactor;	 
    regs[MB_P_RATIO_STATE_REACTOR] = pRatioReactorLevel;	 //pRatioReactorLevel[pRatioReactorLevelName]
    regs[MB_GRATE_VAL] = grate_val;	 
    regs[MB_P_RATIO_FILTER] = pRatioFilter;	 
    regs[MB_P_RATIO_FILTER_STATE] = pRatioFilterHigh;	 
    regs[MB_LAMBDA_IN] = lambda_input;	 
    regs[MB_LAMBDA_OUT] = lambda_output;	 
    regs[MB_LAMBDA_SETPOINT] = lambda_setpoint;	 
    regs[MB_LAMBDA_P] = lambda_PID.GetP_Param();	 
    regs[MB_LAMBDA_I] = lambda_PID.GetI_Param();	 
    regs[MB_LAMBDA_D] = lambda_PID.GetD_Param();	 //Not used
    regs[MB_FUELSWITCHLEVEL] = FuelSwitchLevel; //FuelSwitchLevel[FuelSwitchLevelName]	 
    regs[MB_P_REACTORLEVEL] = pRatioReactorLevel;	 //pRatioReactorLevel[pRatioReactorLevelName]
    regs[MB_T_TREDLEVEL] = T_tredLevel;//T_tredLevel[TempLevelName]
    regs[MB_T_BREDLEVEL] = T_bredLevel;//T_bredLevel[TempLevelName]	 
    regs[MB_T_COMB] = Temp_Data[T_COMB]; //ABSENT;
    regs[MB_T_DRYING_GAS_OUT] = Temp_Data[T_COMB]; //ABSENT;
    regs[MB_T_LOW_FUEL] = Temp_Data[T_LOW_FUEL]; //ABSENT;
    regs[MB_T_PYRO_IN] = Temp_Data[T_PYRO_IN]; //ABSENT;
    regs[MB_T_PYRO_OUT] = Temp_Data[T_PYRO_OUT]; //ABSENT;


    if (written.num_regs) {
        Log_p("Modbus recieved Register:");Logln(written.num_regs);
        
        switch (written.num_regs) {
        case MB_ENGINE_STATE:
          TransitionEngine(regs[MB_ENGINE_STATE]);
          break;
        case MB_AUGER_STATE:
          TransitionAuger(regs[MB_AUGER_STATE]);
          break;
        case MB_FLARE_STATE:
          flare_state = regs[MB_FLARE_STATE]; //???
          break;
        case  MB_BLOWER_STATE:  
          break;
        case MB_LAMBDA_OUT:
          lambda_output = regs[MB_LAMBDA_OUT];
          break;
        case MB_LAMBDA_SETPOINT:
          lambda_setpoint = regs[MB_LAMBDA_SETPOINT];	
          break;
        case MB_LAMBDA_P:
        case MB_LAMBDA_I:
        case MB_LAMBDA_D:  //the following catches all three:
          lambda_P[0] = regs[MB_LAMBDA_P];
          lambda_I[0] = regs[MB_LAMBDA_I];
          lambda_PID.SetTunings(regs[MB_LAMBDA_P],regs[MB_LAMBDA_I],0);
          WriteLambda();
          break;
//        case MB_CONFIG1:
//          saveConfig(1,regs[MB_CONFIG1]);
//          updateConfig(1);
//          break;
        default:  //catch all configs
          if(written.num_regs >= MB_CONFIG1 && written.num_regs < MB_CONFIG1+CONFIG_COUNT-1){
            saveConfig(written.num_regs-MB_CONFIG1+1,regs[written.num_regs]);
            update_config_var(written.num_regs);
          }
          break;
        }
        written.num_regs=0;
    }
}


