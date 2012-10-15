void InitSD() {
  pinMode(SS_PIN, OUTPUT); 
  pinMode(MOSI_PIN, OUTPUT); 
  pinMode(MISO_PIN, INPUT); 
  pinMode(SCK_PIN, OUTPUT); 
  Serial.print("#Initializing SD card...");
  if(!SD.begin(SS_PIN)){
    Serial.println("initialization failed. ");
      sd_loaded = false;
  } else {
      Serial.println("card initialized.");
      sd_loaded = true;
      int data_log_num = EEPROM.read(18);
      if (data_log_num == 255){
        data_log_num = 1;
      }
      sprintf(sd_data_file_name, "log%5i.csv", data_log_num); //99,999 possible logs...is that enough?
      EEPROM.write(18, data_log_num + 1); //update the value for the next time
  }
}

void DatalogSD(String dataString, char file_name[13]) {    //file_name should be 8.3 format names
  //SD.begin(SS_PIN);
  File dataFile = SD.open(file_name, FILE_WRITE);  //if file doesn't exist it will be created
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    Serial.println(dataString);
  }  
  else {
    Serial.print("# Error loading ");
    Serial.println(file_name);
  } 
}

String readSDline(char file_name[13], int line_num = 0){ //pass a filename in the root directory
  char c;
  String SD_line = "";
//  SD.begin(SS_PIN);
  int line_count = 0;
  File file = SD.open(file_name);
  while((c=file.read())>0 && line_count <= line_num){
    if (c == '/n'){
      line_count++;
    }
    if (line_count == line_num && c != '\n'){
      SD_line += c;
    }
  }
  file.close();
  return SD_line;
}

String readSDline(File file, int line_num = 0){ //pass an open file
  char c;
  String SD_line = "";
  int line_count = 0;
  while((c=file.read())>0 && line_count <= line_num){
    if (c == '/n'){
      line_count++;
    }
    if (line_count == line_num && c != '\n'){
      SD_line += c;
    }
  }
  return SD_line;
}

void checkSDconfig(){
  int line = 0;  
  if (SD.exists("config.ini")){
    File config = SD.open("config.ini");
    config_count = config.size() / sizeof(config_entry);
    String SD_config_entry[config_count];
    while (line <= config_count){
      SD_config_entry[line] = readSDline(config, line);
      Serial.print("# ");
      Serial.println(SD_config_entry[line]);
      line++;
    }
  } else {
    Serial.println("# config.ini doesn't exist on SD card");
  }
}

//config_entry Config2Struct(String config_line){
//  string name
//  config_entry config;
//  name = config_line.substring(0,7);
//  name.toCharArray(config.name, 8)
//  //config.name = name.toArray();
//  config.sensor_num = int(config_line.charAt(9));
//  config.flag = int(config_line.charAt(11));
//  config.show = int(config_line.charAt(13));
//  return config;
//}

//void ConfigSD2Array(char file_name[12] = "config.ini", int line_num = 0){  //loads all configurations saved on SD card to sensor_config array
//  char c;
//  char entry[];
//  //char sensor_config[][][3];
//  int line_count = 0;
//  if(SD.begin() != 0){
//    Serial.print("Problem loading SD card");
//    break;
//  }
//  file = SD.open(file_name)
//  while((c = file.read())>0){
//    if (c == '/n'){
//      sensor_config[line_count][index] = entry;
//      entry = "";
//      line_count++;
//      index = 0;
//    } else {
//      if (c == ','){
//        sensor_config[line_count][index] = entry;
//        entry = "";
//        index++;
//      } else {
//        entry += c;
//      }
//    }
//  }
//  file.close();
//}  
      
void testSD() {
  InitSD();
  DatalogSD("test data", "datalog.txt");
  checkSDconfig();
  if (!sd_card.init(SPI_HALF_SPEED, SS_PIN)) {
    Serial.println("initialization failed. ");
    sd_loaded = false;
    return;
  } else {
    Serial.println("card initialized.");
    sd_loaded = true;
  }
  switch(sd_card.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println("SD1");
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println("SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("Unknown");
    }
    if (!sd_volume.init(sd_card)) {
      Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
      return;
    }
    uint32_t volumesize;
    Serial.print("\nVolume type is FAT");
    Serial.println(sd_volume.fatType(), DEC);
    Serial.println();
    
    volumesize = sd_volume.blocksPerCluster();    // clusters are collections of blocks
    volumesize *= sd_volume.clusterCount();       // we'll have a lot of clusters
    volumesize *= 512;                            // SD card blocks are always 512 bytes
    Serial.print("Volume size (bytes): ");
    Serial.println(volumesize);
    Serial.print("Volume size (Kbytes): ");
    volumesize /= 1024;
    Serial.println(volumesize);
    Serial.print("Volume size (Mbytes): ");
    volumesize /= 1024;
    Serial.println(volumesize);
    Serial.println("\nFiles found on the card (name, date and size in bytes): ");
    sd_root.openRoot(sd_volume);
    sd_root.ls(LS_R | LS_DATE | LS_SIZE);  // list all files in the card with date and size
  
  // print the type of card
  Serial.print("#Card type: ");
  switch(sd_card.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println("SD1");
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println("SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("Unknown");
  }
  
  if (!sd_volume.init(sd_card)) {
    Serial.println("# Could not find FAT16/FAT32 partition.  Make sure you've formatted the card");
    return;
  }
}
    
  
//void readJSON(String line){
//  //{key:value, key2:[0,1,2,3],{nested_object_key:nested_object_value}}  //allow nested objects??
//  while (open_bracket > close_bracket){
//    ...
//    if (character == "{"){
//      open_bracket++
//    }
//    if (character == "}"){
//      close_bracket++
//    }
//    ...
//  }
