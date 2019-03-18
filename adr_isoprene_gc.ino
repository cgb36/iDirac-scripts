/*
 gc code 20160712
 12-Jul-2016 adding setup submenu, dual sampling, monthly folders, sync error handling
 
 updated handling of mybuffer
 added manual control of valves
 removed load_time, now equivalent to vol_timeout
 adding trap temperature sensor
 adding trap heating
 added staged standby sequence
 added jamjar control
 added ads1115 code
 added pi shutdown button and column power on/off
*/

/*
  SD card read/write
 * SD card attached to SPI bus as follows:
 ** UNO:  MOSI - pin 11, MISO - pin 12, CLK - pin 13, CS - pin 4 (CS pin can be changed)
  and pin #10 (SS) must be an output
 ** Mega:  MOSI - pin 51, MISO - pin 50, CLK - pin 52, CS - pin 4 (CS pin can be changed)
  and pin #52 (SS) must be an output
 ** Leonardo: Connect to hardware SPI via the ICSP header
 
 created   Nov 2010  by David A. Mellis
 modified 9 Apr 2012  by Tom Igoe
*/
 
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include <Adafruit_ADS1015.h>

//Adafruit_ADS1015 ads1015;
Adafruit_ADS1115 ads1115(0x48); // construct an ads1115 at address 0x48
RTC_DS1307 rtc;

byte last_sam = 0; // toggle for dual sampling mode, 0 = do sam1, 1 = do sam2
char run_type = 'S', batt_OK = 1; // assume battery is OK at the start
char str[25];
char c_code[4] = "", code_data[4] = "", date_folder[6] = "";
char end_sequence = 0;
int chrom_count = 0, resp_cals[4], sam_count = 0;
long current_fileno = 0;

// config defaults
int max_cycles = 999;       // ka:, max number of chroms to run (999 = forever)
//int load_time = 60;         // kb:, seconds of trap load time (now replaced by vol_timeout)
int inject_time = 30;      // kc:, seconds of inject time
int backflush_time = 60;   // kd:, seconds of backflush time
int target_s_vol = 100;       // ke:, target sample volume (scc)
int vol_timeout = 600;      // kf:, volume integration time out (seconds)
int jamjar_kPa_target = 25; // kg:, jamjar target pressure differential (kPa)
int sams_between_cals = 999; // kh:, number of samples to run between cals
int target_c_vol = 20;       // ki:, target calibration volume (scc)
int dual_sampling = 0;       // kj:, dual sampling mode, sam1 and sam2 (0 = sam1 only)

int last_target_c_vol = 0;  // need this to get alternate random response cal sequence

File myFile;

// change this to match your SD shield or module;
//     Arduino Ethernet shield: pin 4
//     Adafruit SD shields and modules: pin 10
//     Sparkfun SD shield: pin 8
const int chipSelect = 4;

//TMP36 Pin variables
// int sensorPin = 0;
// resolution is 10 mV / degree centigrade with 500 mV offset
// to allow for -ve temps

float col_temperatureC = 0, trap_temperatureC = 0, input_voltage = 0, actual_jamjar_P_kPa = 0.0;
int buttonState4 = 0, buttonState5 = 0, buttonState6 = 0; // to store digital input state for switches)

void setup()
{
    // Open serial communications and wait for port to open:
    Serial.begin(9600);
    Serial1.begin(9600);
    delay(5000);
    Serial.println(F("Arduino Mega code: adr_isoprene_gc"));
    Serial.println(F("version date: 12-Jul-2016"));
    /*
    init pin 4 as input for pi shutdown switch
    init pin 5 as input for pi reboot switch  
    pin 6 as input for arduino pause switch
    */
    pinMode(4,INPUT); // init pin 4 as input for pi reboot switch
    pinMode(5,INPUT); // init pin 5 as input for pi shutdown switch  
    pinMode(6,INPUT); // pin 6 as input for arduino pause switch
    /*
    init pin 22 as output for column power switch
    init pin 24 as output for PID power switch
    init pin 26 as output for spare power switch
    init pin 28 as output for pump
    init pin 30 as output for bypass valve
    init pin 34 as output for sample 1 pneumax1
    init pin 36 as output for sample 2 pneumax2
    init pin 38 as output for cal pneumax3
    init pin 40 as output for blank pneumax4
    init pin 42 as output for valco actuation pneumax5
    init pin 44 as output for pre-column backflush flow pneumax6
    init pin 46 as output for main column flow pneumax7
    init pin 47 as output for red LED status
    init pin 48 as output for spare pneumax8
    init pin 49 as output for green LED status
    */
    for(char thisPin = 22; thisPin <= 48; thisPin+=2)
    {
        if((thisPin > 30) && (thisPin < 34))
        {
            //Serial.print("ignoring pin: ");
            //Serial.println(thisPin, DEC);
        }
        else
        {
            //Serial.print("pin OUTPUT (default LO): ");
            //Serial.println(thisPin, DEC);
            pinMode(thisPin,OUTPUT); // init thisPin as output
            digitalWrite(thisPin,LOW); // default is off
        }
    }
    pinMode(47,OUTPUT); // init pin 47 as output for red LED status
    digitalWrite(47,HIGH); // on for start up amber
    pinMode(49,OUTPUT); // init pin 49 as output for green LED status
    digitalWrite(49,HIGH); // on for start up amber
    do_SDcard_init();
    setup_menu();
    //datetime_edit();
    //config_edit();    
    //manual_control();
    digitalWrite(47,LOW); // amber off
    digitalWrite(49,LOW); // amber off
    //powerup_test();
    Serial.println(F("aw:,main column flow on"));
    digitalWrite(46,HIGH); // main column flow on command
    Serial.println(F("ay:,pre-column backflush flow on"));
    digitalWrite(44,HIGH); // pre-column backflush flow on command
    Serial.println(F("aj:,column oven on"));
    digitalWrite(22,HIGH); // column oven on command
    //Serial.println("al:,PID power on");
    //digitalWrite(24,HIGH); // PID power on command  
    Serial.println(F("Getting differential reading from AIN0 (P) and AIN1 (N)"));
    //ads1015.begin();
    ads1115.begin(); // initialise ads1115
    //ads1115.setGain(GAIN_TWOTHIRDS); // DEFAULT for an input range of +- 6.144V
    //Serial.println("ADC gain TWOTHIRDS: +/- 6.144V (1 bit = 0.1875mV)");
    //ads1115.setGain(GAIN_ONE);     // for an input range of +- 4.096V
    //Serial.println("ADC gain ONE: +/- 4.096V (1 bit = 0.125mV)");
    //ads1115.setGain(GAIN_TWO);     // for an input range of +- 2.048V
    //Serial.println("ADC gain TWO: +/- 2.048V (1 bit = 0.0625mV)");
    ads1115.setGain(GAIN_FOUR);    // for an input range of +- 1.024V
    Serial.println(F("ADC gain FOUR: +/- 1.024V (1 bit = 0.03125mV)"));
    //ads1115.setGain(GAIN_EIGHT);   // for an input range of +- 0.512V
    //Serial.println("ADC gain EIGHT: +/- 0.512V (1 bit = 0.015625mV)");
    //ads1115.setGain(GAIN_SIXTEEN); // for an input range of +- 0.256V
    //Serial.println("ADC gain SIXTEEN: +/- 0.256V (1 bit = 0.0078125mV)");
    
    //delete_configfile();
    if(SD.exists("config.txt"))
    {
      Serial.println(F("config.txt found!"));
    }
    else
    {
      Serial.println(F("creating new config.txt file from new values and/or defaults"));
      create_configfile();
    }
    delay(1000);
    read_configfile();
    set_cal_resp_vols();
    //goto_pause();
}


void loop()
{
  //end_sequence = 0;
  do
  {
    //check_pause();
    if(!batt_OK)
    {
      low_batt_standby();
    }
    check_pause();
    chk_index_exists();
    delay(1000);
    read_indexfile();
    col_temperatureC = read_10bit_analog(0); // get column temperature on pin 0, ignore first reading
    input_voltage = read_10bit_analog(3); // get input DC voltage on pin 3
    //goto_pause();
    decide_runtype();
    create_chromfile();
    delay(1000);
    delete_indexfile();
    delay(1000);
    create_indexfile();
    //check_stop();
    //check_pause();
    //goto_pause();
    //delay(60000);
    if(max_cycles != 999) // allow 999 to mean repeat forever
    {
      chrom_count++;
    }
    Serial.print(F("chrom count = "));Serial.println(chrom_count);
    Serial.print(F("sam count = "));Serial.println(sam_count, DEC);
  }
  while(chrom_count < max_cycles);
  end_sequence = 1;
  //check_max_cycles();
  check_pause();
}


void set_cal_resp_vols()
{
  resp_cals[0] = target_c_vol * 0.25;
  resp_cals[1] = target_c_vol * 0.5;
  resp_cals[2] = target_c_vol * 2;
  resp_cals[3] = target_c_vol * 4;   
  //get random response cal volume (response cal)
  randomSeed(analogRead(2));
  for( char myloop = 0; myloop < 4; myloop++)
  {
    //rand_no = random(4);
    //Serial.print("random no between 0 and 3 = ");Serial.println(rand_no, DEC);
    Serial.print(F("random target resp_cals = "));Serial.println(resp_cals[myloop], DEC);
  }
  //goto_pause();*/
}


void do_SDcard_init()
{
    Serial.print(F("Initialising SD card..."));
    // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
    // Note that even if it's not used as the CS pin, the hardware SS pin 
    // (10 on most Arduino boards, 53 on the Mega) must be left as an output 
    // or the SD library functions will not work. 
    pinMode(SS, OUTPUT);
    if(!SD.begin(10,11,12,13))
    {
      Serial.println(F("initialisation failed!"));
      return;
    }
    Serial.println(F("initialisation done."));
    #ifdef AVR
      Wire.begin();
    #else
      Wire1.begin(); // Shield I2C pins connect to alt I2C bus on Arduino Due
    #endif
    rtc.begin();
    if (! rtc.isrunning())
    {
      Serial.println(F("RTC is NOT running!"));
      // following line sets the RTC to the date & time this sketch was compiled
      // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      // This line sets the RTC with an explicit date & time, for example to set
      // January 21, 2014 at 3am you would call:
      // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }
    // to manually set date & time change line below then compiled and run
    // then rem out again otherwise same date and time will keep being set
    //rtc.adjust(DateTime(2015, 3, 28, 20, 46, 0));
    //Serial.println("Date and time updated ...");
    Serial.println(F("Date/time check ..."));
    DateTime now = rtc.now();    
    Serial.print(now.year(),DEC); Serial.print(','); Serial.print(now.month(),DEC); Serial.print(','); Serial.print(now.day(),DEC); Serial.print(',');
    Serial.print(now.hour(),DEC); Serial.print(','); Serial.print(now.minute(),DEC); Serial.print(','); Serial.println(now.second(),DEC);    
}


void chk_index_exists()
{
  if(SD.exists("index.txt"))
  {
    Serial.println(F("index.txt already exists, now reading ..."));
    //create_indexfile();
  }
  else
  {
    Serial.println(F("creating index.txt"));
    create_indexfile();
  }
}


void read_indexfile()
{
  char ind = 0, inchar = 0, indata[25] = "";
  // open the file for reading:
  myFile = SD.open("index.txt");
  if (myFile)
  {
    Serial.print(F("Reading from index.txt ... "));
    // read from the file until there's nothing else in it:
    while (myFile.available())
    {
        //current_fileno = myFile.parseInt();
        //Serial.println(current_fileno);
        inchar = myFile.read();
        indata[ind] = inchar;
        ind++;
    }
    current_fileno = atol(indata);
    //Serial.print(indata);
    Serial.println(current_fileno);
    // close the file:
    myFile.close();
  }
  else
  {
     // if the file didn't open, print an error:
     Serial.print(F("error opening index.txt")); Serial.println(F(" ... halting code ..."));
     while(1){} // holds the loop
  }
  Serial.print(F("previous file no: ")); Serial.println(current_fileno);
  current_fileno += 1;
  Serial.print(F("this file no: ")); Serial.println(current_fileno);
}


void delete_indexfile()
{
  Serial.println(F("deleting index.txt"));
  SD.remove("index.txt");
}
 
 
void create_indexfile()
{
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("index.txt", FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile)
  {
    Serial.print(F("Writing to index.txt ..."));
    ltoa(current_fileno, str,10);
    myFile.write(str);
    // close the file:
    myFile.close();
    Serial.println(F("done."));
  }
  else
  {
    // if the file didn't open, print an error:
    Serial.println(F("error opening index.txt"));
  }
}


void decide_runtype()
{
  if(sam_count < sams_between_cals)
  {
    run_type = 'S';
    if(dual_sampling)
    {
      if(last_sam == 0)
      {
        run_type = 'S';
        last_sam = 1;
      }
      else
      {
        run_type = 'X';
        last_sam = 0;
      }
    }
  }
  if(sam_count == sams_between_cals)
  {
    run_type = 'B';
  }
  if(sam_count == (sams_between_cals+1))
  {
    run_type = 'C';
  }
  if(sam_count == (sams_between_cals+2))
  {
    run_type = 'B';
    sam_count = 0;
  }
  else
  {
    if(sam_count >= 32000) // reset sam_count to prevent integer overflow
    {
      sam_count = 0;
    }
    else
    {
      sam_count++;
    }
  }
}


void create_chromfile()
{
  char fullpath_str[25] = "", folder_str[15] = "/", chromfile_str[15] = "", chromfile_OK = 0;
  char outstr1[17] = "", outstr2[17] = "", asc_time[6] = "";
  
  determine_monthly_folder();
  Serial.print(F("date_folder = ")); Serial.println(date_folder);
  strcat(folder_str, date_folder);
  strcat(folder_str, "/");
  Serial.println(folder_str);
  //goto_pause(); 
  if(!SD.exists(folder_str))
  {
    Serial.print(F("making new folder: ")); Serial.println(folder_str);
    SD.mkdir(folder_str);
  }
  else
  {
    Serial.print(F("folder already exists: "));Serial.println(folder_str);
  }
  //goto_pause();
  strcat(fullpath_str, folder_str);
  ltoa(current_fileno, chromfile_str,10);
  strcat(chromfile_str, ".csv");
  strcat(fullpath_str, chromfile_str);
  Serial.println(fullpath_str);
  //goto_pause();
  // open the file. note that only one file can be open at a time,
  if(SD.exists(fullpath_str))
  {
    Serial.print(F("Problem! Chrom file already exists KO ... ")); Serial.println(fullpath_str);
    Serial.println(F("Now doing it the long way! ..."));
    chromfile_OK = 0;
    while(SD.exists(fullpath_str))
    {
        strcpy(fullpath_str, folder_str);
        ltoa(current_fileno, chromfile_str,10);      
        strcat(chromfile_str, ".csv");
        strcat(fullpath_str, chromfile_str);
        current_fileno++;
        Serial.println(fullpath_str);
    }
  }
  else
  { 
    Serial.print(F("Chrom file name new OK ... ")); Serial.println(fullpath_str);
    chromfile_OK = 1;
  }  
  myFile = SD.open(fullpath_str, FILE_WRITE);
  // if the file opened okay, write to it:
  if(myFile)
  {
    Serial.println(F("Writing to chromfile ... "));
    // send data folder to RPi
    //ltoa(folder_str, str,10);
    Serial.print(F("cq:,")); Serial.println(folder_str); // cq: year and month folder
    // send file index number to RPi      
    ltoa(current_fileno, str,10);
    Serial.print(F("aa:,")); Serial.println(str); //aa: file index no.
    myFile.print("aa:,"); myFile.println(str); //aa: file index no.
    DateTime now = rtc.now();
    Serial.print(F("ab:,")); // ab: date & time stamp
    //Serial.print(now.year(),DEC); Serial.print('/'); Serial.print(now.month(),DEC); Serial.print('/'); Serial.print(now.day(),DEC); Serial.print(' ');
    //Serial.print(now.hour(),DEC); Serial.print(':'); Serial.print(now.minute(),DEC); Serial.print(':'); Serial.println(now.second(),DEC);
    Serial.print(now.year(),DEC); Serial.print(','); Serial.print(now.month(),DEC); Serial.print(','); Serial.print(now.day(),DEC); Serial.print(',');
    Serial.print(now.hour(),DEC); Serial.print(','); Serial.print(now.minute(),DEC); Serial.print(','); Serial.println(now.second(),DEC);
    Serial.print(F("run type = ")); Serial.println(run_type);
    myFile.print("ab:,"); // ab: date & time stamp
    myFile.print(now.year(),DEC); myFile.print(","); myFile.print(now.month(),DEC); myFile.print(","); myFile.print(now.day(),DEC); myFile.print(",");
    myFile.print(now.hour(),DEC); myFile.print(","); myFile.print(now.minute(),DEC); myFile.print(","); myFile.println(now.second(),DEC);
    Serial.print(F("ac:,")); Serial.println(run_type); //ac: run type
    myFile.print("ac:,"); myFile.println(run_type); //ac: run type
    Serial.print(F("ad:,")); Serial.println(col_temperatureC,1); //ad: column temperature
    myFile.print("ad:,"); myFile.println(col_temperatureC,1); //ad: column temperature
    Serial.println(F("ca:,999")); // ca: dummy R-Pi system temperature
    myFile.println("ca:,999"); // ca: dummy R-Pi system temperature
    Serial.print(F("cb:,")); Serial.println(input_voltage,1); //cb: input DC voltage
    myFile.print("cb:,"); myFile.println(input_voltage,1); //cb: input DC voltage    
    //delay(1000);
    get_altimeter_pressure();
    get_altimeter_temperature();
    if(chromfile_OK)
    {
      Serial.println(F("an:,Chrom file name new OK"));
      myFile.println("an:,Chrom file name new OK"); //an: index was correctly read and the file is new
    }
    else
    {
      Serial.println(F("ao:,Chrom file already existed KO!"));
      myFile.println("ao:,Chrom file already existed KO!"); //an: Chrom file already existed so new filendame found the long way!
    }  
    do_chrom_cycle();
    myFile.close();// close the file:
    Serial.println(F("av:, chromatogram done!")); Serial.println(); Serial.println();      
  }
  else
  {
    // if the file didn't open, print an error:
    Serial.print(F("error opening chromfile ... "));
    Serial.println(fullpath_str);
  }
}


void determine_monthly_folder()
{
  char year_ascii[8] = "", month_ascii[8]= "", day_ascii[8]= "", hour_ascii[8]= "";
  int year_folder = 0, month_folder = 0, day_folder = 0, hour_folder = 0; 
  
  Serial.print(F("Determining monthly data folder ..."));
  
  DateTime now = rtc.now();
  Serial.print(now.year(),DEC); Serial.print('/'); Serial.print(now.month(),DEC); Serial.print('/'); Serial.print(now.day(),DEC); Serial.print(' ');
  Serial.print(now.hour(),DEC); Serial.print(':'); Serial.print(now.minute(),DEC); Serial.print(':'); Serial.println(now.second(),DEC);
  year_folder = now.year(); month_folder = now.month(); day_folder = now.day(); hour_folder = now.hour();
  itoa(year_folder, year_ascii, DEC); 
  itoa(month_folder, month_ascii, DEC); 
  itoa(day_folder, day_ascii, DEC); 
  itoa(hour_folder, hour_ascii, DEC);  
  if(month_folder < 10)
  {
    strcat(year_ascii, "_0");
  }
  else
  {
    strcat(year_ascii, "_");
  }
  /*
  if(day_folder < 10)
  {
    strcat(month_ascii, "_0");
  }
  else
  {
    strcat(month_ascii, "_");
  }*/
  /*
  if(hour_folder < 10)
  {
    strcat(day_ascii, "_0");
  }
  else
  {
    strcat(day_ascii, "_");
  }*/
  strcpy(date_folder, year_ascii); strcat(date_folder, month_ascii);
  //strcpy(buf, month_ascii); strcat(buf, day_ascii);  
  //strcpy(buf, "160"); strcat(buf, month_ascii); strcat(buf, day_ascii); strcat(buf, hour_ascii); 
  //Serial.println(buf);
  //goto_pause();
}


float read_10bit_analog(int sensorPin) // sensorPin 0 = column temp', sensorPin 1 = trap temp', sensorPin 2 = p_diff', sensorPin 3 = input voltage'
{
  float reading_mult = 0, converted_value = 0;
  for (char loop1 = 0; loop1 < 10; loop1 ++)
  {
    reading_mult += analogRead(sensorPin);
  }
  reading_mult /= 10;   
  float voltage = reading_mult * 5.0; // convert ADC reading to voltage
  voltage /= 1024.0;
  //Serial.print("ADCreading 10 av = "); Serial.print(reading_mult,0); Serial.print(", Voltage = "); Serial.println(voltage);
  switch(sensorPin)
  {
    case 0:
      converted_value = (voltage - 0.5) * 100; // convert voltage to temperature
      // converting from 10 mV per degree with 500 mV offset
      Serial.print(F("Column T sensor = ")); Serial.print(converted_value,1); Serial.println(F(" degrees C"));
      break;
    case 1:
      converted_value = (voltage - 0.5) * 100; // convert voltage to temperature
      // converting from 10 mV per degree with 500 mV offset       
      Serial.print(F("Trap T sensor = ")); Serial.print(converted_value,1); Serial.println(F(" degrees C"));
      break;
    case 2:     
      // counts seem to max out on vacuum port (-ve) at 883 counts (4.31 volts, 45.68 kPa, 6.62 psi)
      converted_value = ((float)(voltage * 200)/18) - 2.222;
      //Serial.print("pressure reading kPa = "); Serial.println(converted_value);; Serial.println("");
      break;              
    case 3:
      converted_value = voltage * 60/10; // convert 5V voltage to real input voltage      
      //Serial.print("input DC voltage = "); Serial.print(converted_value,1); Serial.println(" VDC");
      if (converted_value < 10)
      {
        Serial.println(F("low input voltage, battery almost flat!!"));
        batt_OK = 0;
      }      
      break;      
  }
  delay(1000);
  return converted_value;
}


/*
void check_max_cycles()
{
    if(end_sequence == 1) 
    {
        Serial.println("ap:,maximum number of chrom cycles reached");
        Serial.println("am:,PID power off");
        digitalWrite(24,LOW); // PID power off command        
        delay(1000);
        while(end_sequence == 1)
        { 
            buttonState4 = digitalRead(4); // check for pi reboot switch
            delay(1000);
            buttonState5 = digitalRead(5); // check for pi shutdown switch
            delay(1000);
            if(buttonState4 == HIGH)
            {
                Serial.println("ak:,column oven off");
                digitalWrite(22,LOW); // column oven off command              
                Serial.println("ah:,pi reboot command (input 4 HIGH): ");
                while(1){} // holds the loop
                //Serial.println(buttonState4);
                //Serial.println("");
                //delay(1000);
            }
            if(buttonState5 == HIGH)
            {
                Serial.println("ak:,column oven off");
                digitalWrite(22,LOW); // column oven off command 
                Serial.println("ai:,pi shutdown command (input 5 HIGH): ");
                while(1){} // holds the loop
                //Serial.println(buttonState5);
                //Serial.println("");
                //delay(1000);
            }
        }
    }
}*/


void check_pause()
{
    char timeout = 0, standby_stage = 0;
    unsigned long start_time = 0, end_time = 0;
    digitalWrite(47,LOW); // red LED off (goto green only)
    buttonState6 = digitalRead(6); // check for arduino pause status
    //Serial.print("button 6 state: ");
    //Serial.println(buttonState6);
    if((buttonState6 == HIGH) || (end_sequence) ) // check arduino pause switch
    {      
        if(buttonState6 == HIGH)
        {
          Serial.print(F("af:,pause arduino command (input 6 HIGH): ")); Serial.println(buttonState6);
          Serial.println(F(""));
        }
        if(end_sequence)
        {
          Serial.println(F("ap:,maximum number of chrom cycles reached"));
        }
        Serial.println(F("Staged standby, 5 minute intervals ..."));
        delay(1000);
        start_time = millis();
        while((buttonState6) || (end_sequence) )
        { 
            low_batt_standby();  
            digitalWrite(49,HIGH); // green LED on  
            buttonState4 = digitalRead(4); // check for pi reboot switch
            delay(1000);
            buttonState5 = digitalRead(5); // check for pi shutdown switch
            delay(1000);
            //digitalWrite(49,LOW); // green LED off
            buttonState6 = digitalRead(6); // check for arduino pause status
            delay(1000);
            if((buttonState6 == LOW) && (!end_sequence) )
            {
                Serial.print(F("ag:,resume arduino command (input 6 LOW): ")); Serial.println(buttonState6);
                //Serial.println("");
                Serial.println(F("ay:,pre-column backflush flow on"));
                digitalWrite(44,HIGH); // pre-column backflush flow on command
                Serial.println(F("aw:,main column flow on"));
                digitalWrite(46,HIGH); // main column flow on command
                Serial.println(F("aj:,column oven on"));
                digitalWrite(22,HIGH); // column oven on command
                //Serial.println("al:,PID power on");
                //digitalWrite(24,HIGH); // PID power on command
                end_sequence = 0;
                digitalWrite(47,HIGH); // red LED on (goto red only)
                digitalWrite(49,LOW); // green LED off (goto red only)
                delay(1000);
            }
            if(buttonState4 == HIGH)
            {
                Serial.println(F("am:,PID power off"));
                digitalWrite(24,LOW); // PID power off command
                Serial.println(F("ak:,column oven off"));
                digitalWrite(22,LOW); // column oven off command                
                Serial.println(F("ax:,main column flow off"));
                digitalWrite(46,LOW); // main column off command                
                Serial.println(F("az:,pre-column backflush flow off"));
                digitalWrite(44,LOW); // pre-column backflush flow off command                        
                Serial.println(F("ah:,pi reboot command (input 4 HIGH): "));
                while(1){}; // holds the loop
                //Serial.println(buttonState4);
                //Serial.println("");
                //delay(1000);
            }
            if(buttonState5 == HIGH)
            {
                Serial.println(F("am:,PID power off"));
                digitalWrite(24,LOW); // PID power off command
                Serial.println(F("ak:,column oven off"));
                digitalWrite(22,LOW); // column oven off command                
                Serial.println(F("ax:,main column flow off"));
                digitalWrite(46,LOW); // main column off command                
                Serial.println(F("az:,pre-column backflush flow off"));
                digitalWrite(44,LOW); // pre-column backflush flow off command  
                Serial.println(F("ai:,pi shutdown command (input 5 HIGH): "));
                while(1){}; // holds the loop
                //Serial.println(buttonState5);
                //Serial.println("");
                //delay(1000);
            }
            end_time = millis();
            if(((end_time-start_time) > 60000) && ((end_time-start_time) < 63000))
            {
              Serial.println(F("1 min ..."));
            }
            if(((end_time-start_time) > 120000) && ((end_time-start_time) < 123000))
            {
              Serial.println(F(" 2 min ..."));
            }
            if(((end_time-start_time) > 180000) && ((end_time-start_time) < 183000))
            {
              Serial.println(F("  3 min ..."));
            }         
            if(((end_time-start_time) > 240000) && ((end_time-start_time) < 243000))
            {
              Serial.println(F("   4 min ..."));
            }            
            if(((end_time-start_time) > 300000) && (standby_stage == 0))
            {
                timeout = 1;
                start_time = millis();
            }
            if(((end_time-start_time) > 300000) && (standby_stage == 1))
            {
                timeout = 2;
                start_time = millis();
            }
            if(((end_time-start_time) > 300000) && (standby_stage == 2))
            {
                timeout = 3;
                //start_time = millis();
            }
            switch(timeout)
            {
              case 1:
                Serial.println(F("am:,PID power off"));
                digitalWrite(24,LOW); // PID power off command
                standby_stage = 1;
                timeout = 0;
                break;
              case 2:
                Serial.println(F("ak:,column oven off"));
                digitalWrite(22,LOW); // column oven off command 
                standby_stage = 2;
                timeout = 0;
                break;  
              case 3:
                Serial.println(F("ax:,main column flow off"));
                digitalWrite(46,LOW); // main column off command                
                Serial.println(F("az:,pre-column backflush flow off"));
                digitalWrite(44,LOW); // pre-column backflush flow off command  
                Serial.println(F("Now in standby mode: PID, column oven, carrier flows OFF"));
                standby_stage = 3;
                timeout = 0;
                break;              
            }
            if(standby_stage)
            {
              digitalWrite(49,LOW); // green LED off (flashing stage)
              delay(1000);
            }
        }
    }
    digitalWrite(47,HIGH); // red LED on
    digitalWrite(49,LOW); // green LED off
}


void low_batt_standby()
{
  // read battery analog channel again to double check!
  input_voltage = read_10bit_analog(3); // get input DC voltage on pin 3
  if(!batt_OK)
  {
    Serial.println(F("Low input voltage!! Battery almost flat!!")); 
    Serial.println(F("am:,PID power off"));
    digitalWrite(24,LOW); // PID power off command
    Serial.println(F("ak:,column oven off"));
    digitalWrite(22,LOW); // column oven off command                
    Serial.println(F("ax:,main column flow off"));
    digitalWrite(46,LOW); // main column off command                
    Serial.println(F("az:,pre-column backflush flow off"));
    digitalWrite(44,LOW); // pre-column backflush flow off command  
    Serial.println(F("ai:,pi shutdown command (input 5 HIGH): "));
    digitalWrite(49,LOW); // green LED off
    while(1)
    {
        digitalWrite(47,HIGH); // red LED on
        delay(1000);
        digitalWrite(47,LOW); // red LED off
        delay(1000);
    }      
  } 
}


void goto_pause()
{
      Serial.println(F("Arduino forced pause!"));
      while(1){} // holds the loop
}


void powerup_test()
{
      for(char thisPin = 28; thisPin <= 48; thisPin+=2)    
      {
          if(thisPin != 32) // miss out pin 32 as this is still unused
          { 
            Serial.print(F("Digital pin hi: ")); Serial.println(thisPin, DEC);
            digitalWrite(thisPin,HIGH);
            delay(200);
            Serial.print(F("Digital pin lo: ")); Serial.println(thisPin, DEC);
            digitalWrite(thisPin,LOW);
            delay(200);
          }
      }
      Serial.println(F("LED status test ..."));
      Serial.println(F("red LED on"));
      digitalWrite(47,HIGH); // red LED on
      delay(1500);
      digitalWrite(47,LOW); // red LED off
      delay(1500);
      Serial.println(F("green LED on"));
      digitalWrite(49,HIGH); // green LED on
      delay(1500);
      digitalWrite(49,LOW); // green LED off
      delay(1500);
      Serial.println(F("amber on (red & green LEDs on)"));
      digitalWrite(47,HIGH); // amber LED status test (need both red and green together)
      digitalWrite(49,HIGH);
      delay(1500);
      digitalWrite(47,LOW);
      digitalWrite(49,LOW);
      Serial.println(F("Trap heating test ..."));
      trap_temperatureC = read_10bit_analog(1); // get trap cold temperature on pin 1
      digitalWrite(26,HIGH); // trap heater on
      Serial.println(F("Trap now heating ..."));
      delay(10000);
      digitalWrite(26,LOW); // trap heater off
      Serial.println(F("Trap heating now off"));      
      delay(5000);
      trap_temperatureC = read_10bit_analog(1); // get trap hot temperature on pin 1  
      input_voltage = read_10bit_analog(3); // get input DC voltage on pin 3
}


void read_ads1115(uint16_t log_secs)
{
    int16_t ADC_result = 0;
    uint16_t start_time = 0, end_time = 0;
    unsigned long ADC_sum = 0;
    
    for(uint16_t pid_count = 0; pid_count <= (log_secs * 5); pid_count++) // log_secs * 5 for 5 Hz sampling
    {
        start_time = millis();
        ADC_sum = 0;
        for( char ADC_readings = 0; ADC_readings < 20; ADC_readings++) // average 20 readings from ADC
        {
            ADC_sum += ads1115.readADC_Differential_0_1();
            //Serial.print("ADC_sum: "); Serial.println(ADC_sum);
        }
        ADC_result = ADC_sum / 20;
        Serial.print(F("zz:,")); Serial.println(ADC_result);
        myFile.print("zz:,"); myFile.println(ADC_result); //zz: PID data */
        //Serial.print("Differential: "); Serial.print(ADC_result); Serial.print("("); Serial.print((float)ADC_result * 0.188); Serial.println("mV)");
        do
        {
            end_time = millis();
        } 
        while((end_time-start_time) < 200);
        //Serial.print("milli's elapsed: "); Serial.println(end_time-start_time);
    }
}


void create_configfile()
{
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("config.txt", FILE_WRITE);
  // if the file opened okay, write to it:
  if(myFile)
  {
    Serial.print(F("Writing to config.txt ..."));
    itoa(max_cycles, str,10);
    myFile.print("ka:,");myFile.print(str);myFile.println(",max number of chroms to run (999 = forever)");
    //itoa(load_time, str,10);
    //myFile.print("kb:,");myFile.print(str);myFile.println(",trap load time (seconds)");
    itoa(inject_time, str,10);
    myFile.print("kc:,");myFile.print(str);myFile.println(",inject time (seconds)");
    itoa(backflush_time, str,10);
    myFile.print("kd:,");myFile.print(str);myFile.println(",backflush time (seconds)");     
    itoa(target_s_vol, str,10);
    myFile.print("ke:,");myFile.print(str);myFile.println(",target sample volume (scc)");    
    itoa(vol_timeout, str,10);
    myFile.print("kf:,");myFile.print(str);myFile.println(",volume integration timeout (seconds)"); 
    itoa(jamjar_kPa_target, str,10);
    myFile.print("kg:,");myFile.print(str);myFile.println(",jamjar target pressure differential (kPa)");    
    itoa(sams_between_cals, str,10);
    myFile.print("kh:,");myFile.print(str);myFile.println(",number of samples to run between cals");    
    itoa(target_c_vol, str,10);
    myFile.print("ki:,");myFile.print(str);myFile.println(",target calibration volume (scc)");
    itoa(dual_sampling, str,10);
    myFile.print("kj:,");myFile.print(str);myFile.println(F(",dual sampling mode (0-1)"));    
    myFile.close(); // close the file:
    Serial.println(F("new config file creation done!"));
  }
  else
  {
    // if the file didn't open, print an error:
    Serial.println(F("error opening config.txt"));
  }
}


void read_configfile()
{
  char line_read_count = 0;
  // open the file for reading:
  myFile = SD.open("config.txt");
  if(myFile)
  {
    while(line_read_count < 9)
    {
      read_config_line();
      mine_config_line();
      line_read_count++;
    }
  }
  else
  {
     // if the file didn't open, print an error:
     Serial.println(F("error opening config.txt halting code ..."));
     goto_pause();
  }
  myFile.close(); // close the file:
  //goto_pause();
}


void read_config_line()
{
  unsigned char char_count = 0, code_count = 0;
  char inchar;
  for(char count = 0; count < 4; count ++)
  {
    c_code[count] = (char)'\0'; // reset code array
    code_data[count] = (char)'\0'; // reset code data array
  }
  Serial.println(F("Reading from config.txt ... "));
  // read config code part first
  while((myFile.available()) && (char_count < 3) )
  {
    inchar = myFile.read();
    //Serial.print(inchar);
    c_code[char_count] = inchar;
    char_count++;
  }
  //Serial.print(c_code);Serial.print(" ");
  // check for 1st comma
  while((myFile.available()) && (char_count < 4) )
  {
    inchar = myFile.read();
    char_count++;
    if(inchar !=',')
    {
      Serial.println(F("error reading config.txt halting code ..."));
      goto_pause();
    }
  }
  // read numerical part
  while((myFile.available()) && (code_count < 3) )
  {
    inchar = myFile.read();
    //Serial.print(inchar);
    if(inchar == ',')
    {
      break; // break out from while loop if comma found
    }     
    //Serial.print(inchar);
    code_data[code_count] = inchar;
    code_count++;
  }
  //Serial.println(code_data);   
  //goto_pause();
  // now read comment text to the end of the line
  while(myFile.available())
  {
    inchar = myFile.read();
    //Serial.print(inchar);
    /*if(inchar == 13)
    {
        Serial.println(""); Serial.println("carriage return found");
    }*/
    if(inchar == 10)
    {
        //Serial.println(""); Serial.println("line feed found");
        break; // break out from while loop if line feed found
        //goto_pause();
    }
  }
}


void mine_config_line()
{
  if(strcmp(c_code,"ka:") == 0)
  {
    Serial.println(F("found max no of chroms"));
    max_cycles = atoi(code_data);
    Serial.print(F("max_cycles = "));Serial.println(max_cycles);
  }
/*  if(strcmp(c_code,"kb:") == 0)
  {
    Serial.println("found trap load time (seconds)");
    load_time = atoi(code_data);
    Serial.print("load_time = ");Serial.println(load_time);
  }*/
  if(strcmp(c_code,"kc:") == 0)
  {
    Serial.println(F("found inject time (seconds)"));
    inject_time = atoi(code_data);
    Serial.print(F("inject_time = "));Serial.println(inject_time);
  }
  if(strcmp(c_code,"kd:") == 0)
  {
    Serial.println(F("found backflush time (seconds)"));
    backflush_time = atoi(code_data);
    Serial.print(F("backflush_time = "));Serial.println(backflush_time);
  }
  if(strcmp(c_code,"ke:") == 0)
  {
    Serial.println(F("found target sample volume (scc)"));
    target_s_vol = atoi(code_data);
    Serial.print(F("target_s_vol = "));Serial.println(target_s_vol);
  }
  if(strcmp(c_code,"kf:") == 0)
  {
    Serial.println(F("found volume integration timeout (seconds)"));
    vol_timeout = atoi(code_data);
    Serial.print(F("vol_timeout = "));Serial.println(vol_timeout);
  }
  if(strcmp(c_code,"kg:") == 0)
  {
    Serial.println(F("found jamjar target pressure differential (kPa)"));
    jamjar_kPa_target = atoi(code_data);
    Serial.print(F("jamjar_kPa_target = "));Serial.println(jamjar_kPa_target);
  } 
  if(strcmp(c_code,"kh:") == 0)
  {
    Serial.println(F("found number of samples to run between cals"));
    sams_between_cals = atoi(code_data);
    Serial.print(F("sams_between_cals = "));Serial.println(sams_between_cals);
  } 
  if(strcmp(c_code,"ki:") == 0)
  {
    Serial.println(F("found target calibration volume (scc)"));
    target_c_vol = atoi(code_data);
    Serial.print(F("target_c_vol = "));Serial.println(target_c_vol);
  }
  if(strcmp(c_code,"kj:") == 0)
  {
    Serial.println(F("found dual sampling mode (0 or 1)"));
    dual_sampling = atoi(code_data);
    Serial.print(F("dual_sampling = "));Serial.println(dual_sampling);
  }    
}


void delete_configfile()
{
  delay(5000);
  Serial.println(F("deleting old config.txt"));
  SD.remove("config.txt");
}


void setup_menu()
{
  char incoming_byte = 0, time_out = 0;
  unsigned long st_time = 0, en_time = 0;
  
  Serial.println(F("co:,setup mode"));
  st_time = millis();
  while( (incoming_byte != 's') && (!time_out) )
  {
    if(Serial.available())
    {
      incoming_byte = Serial.read();
      if(incoming_byte == 's')
      {
        Serial.println(F("Entering setup ..."));
        datetime_edit();
        config_edit();    
        manual_control();
      }
    }
    en_time = millis();
    if( (en_time-st_time) > 60000 || (incoming_byte =='q'))
    {
      time_out = 1;
      Serial.println(F("Time-out on setup mode!"));
      Serial.println(F("cp:,ending setup mode"));
    }
  } 
}


void datetime_edit()
{
  char incoming_byte = 0, time_out = 0;
  int new_year, new_mon, new_mday, new_hour, new_min, new_sec;
  unsigned long st_time = 0, en_time = 0;
  
  DateTime now = rtc.now();    
  //serial_flush('0', 1); // flush main serial port (1= quiet mode)
  Serial.println(F("cc:,datetime edit mode"));
  //Serial.println("Press 't' + enter to edit date & time (30 second time-out)");
  st_time = millis();
  while( (incoming_byte != 't') && (!time_out) )
  {
    if(Serial.available())
    {
      incoming_byte = Serial.read();
      if(incoming_byte == 't')
      {
        Serial.println(F("Running date & time editor..."));
        //serial_flush('0'); // flush main serial port (1= quiet mode)
      }
    }   
    en_time = millis();
    if( (en_time-st_time) > 60000 || (incoming_byte == 'q'))
    {
      time_out = 1;
      Serial.println(F("Time-out on date & time edit!"));
      Serial.println(F("cd:,ending date & time edit"));
    }
  }
  // YEAR
  if(!time_out)
  {
    st_time = millis();
    //Serial.print(F("ce:, enter year (yy) CURRENTLY: ")); Serial.println(now.year(),DEC);
    Serial.print(F("ce:, enter year (yyyy) CURRENTLY: ")); Serial.println(now.year(),DEC);    
  }
  while( (!Serial.available()) && (!time_out) )
  {
    en_time = millis();
    if((en_time-st_time) > 60000)
    {
      time_out = 1;
      Serial.println(F("Time-out on date & time edit!"));
      Serial.println(F("cd:,ending date & time edit"));
    }
  }
  if(!time_out)
  {
    new_year = Serial.parseInt();
    Serial.print(F("new year = ")); Serial.println(new_year);
  }
  // MONTH
  if(!time_out)
  {
    st_time = millis();
    Serial.print(F("cf:, enter month (mm) CURRENTLY: ")); Serial.println(now.month(),DEC);
  }
  while( (!Serial.available()) && (!time_out) )
  {
    en_time = millis();
    if((en_time-st_time) > 60000)
    {
      time_out = 1;
      Serial.println(F("Time-out on date & time edit!"));
      Serial.println(F("cd:,ending date & time edit"));
    }
  }
  if(!time_out)
  {
    new_mon = Serial.parseInt();
    Serial.print(F("new month = ")); Serial.println(new_mon);
  }
  // DAY
  if(!time_out)
  {
    st_time = millis();
    Serial.print(F("cg:, enter day (dd) CURRENTLY: ")); Serial.println(now.day(),DEC);
  }
  while( (!Serial.available()) && (!time_out) )
  {
    en_time = millis();
    if((en_time-st_time) > 60000)
    {
      time_out = 1;
      Serial.println(F("Time-out on date & time edit!"));
      Serial.println(F("cd:,ending date & time edit"));
    }
  }
  if(!time_out)
  {
    new_mday = Serial.parseInt();
    Serial.print(F("new day = ")); Serial.println(new_mday);
  }  
  // HOUR
  if(!time_out)
  {
    st_time = millis();
    Serial.print(F("ch:, enter hour (hh) CURRENTLY: ")); Serial.println(now.hour(),DEC);
  }
  while( (!Serial.available()) && (!time_out) )
  {
    en_time = millis();
    if((en_time-st_time) > 60000)
    {
      time_out = 1;
      Serial.println(F("Time-out on date & time edit!"));
      Serial.println(F("cd:,ending date & time edit"));
    }
  }
  if(!time_out)
  {
    new_hour = Serial.parseInt();
    Serial.print(F("new hour = ")); Serial.println(new_hour);
  } 
  // MINUTE
  if(!time_out)
  {
    st_time = millis();
    Serial.print(F("ci:, enter minute (mm) CURRENTLY: ")); Serial.println(now.minute(),DEC);
  }
  while( (!Serial.available()) && (!time_out) )
  {
    en_time = millis();
    if((en_time-st_time) > 60000)
    {
      time_out = 1;
      Serial.println(F("Time-out on date & time edit!"));
      Serial.println(F("cd:,ending date & time edit"));
    }
  }
  if(!time_out)
  {
    new_min = Serial.parseInt();
    Serial.print(F("new minute = ")); Serial.println(new_min);
    Serial.println(F("new seconds = 00"));
  }   
  // SECONDS
  if(!time_out)
  {
    st_time = millis();
    Serial.print(F("cj:, enter seconds (ss) CURRENTLY: ")); Serial.println(now.second(),DEC);
  }
  while( (!Serial.available()) && (!time_out) )
  {
    en_time = millis();
    if((en_time-st_time) > 60000)
    {
      time_out = 1;
      Serial.println(F("Time-out on date & time edit!"));
      Serial.println(F("cd:,ending date & time edit"));
    }
  }
  if(!time_out)
  {
    new_sec = Serial.parseInt();
    Serial.print(F("new seconds = ")); Serial.println(new_sec);
    //rtc.setTime(t); // send the new date n time
    rtc.adjust(DateTime(new_year, new_mon, new_mday, new_hour, new_min, new_sec));    
    delay(1000);
    //Serial.print(F("Date: "));Serial.print(new_mday);Serial.print("-");Serial.print(new_mon);Serial.print("-20");Serial.print(new_year);
    Serial.print(F("Date: "));Serial.print(new_mday);Serial.print("-");Serial.print(new_mon);Serial.print("-");Serial.print(new_year);
    Serial.print(F("   Time: "));Serial.print(new_hour);Serial.print(":");Serial.print(new_min);Serial.print(":");Serial.println(new_sec);
  } 
  if(time_out)
  {
    Serial.println(F("Time out! Date and time NOT updated!"));
  }
}


void config_edit()
{
  char incoming_byte = 0, time_out = 0;
  unsigned long st_time = 0, en_time = 0;
  
  Serial.println(F("bh:,config edit mode"));
  //Serial.println("Press 'c' + enter to edit config file (60 second time-out)");
  st_time = millis();
  while( (incoming_byte != 'c') && (!time_out) )
  {
    if(Serial.available())
    {
      incoming_byte = Serial.read();
      if(incoming_byte == 'c')
      {
        Serial.println(F("Running config editor..."));
      }
    }
    en_time = millis();
    if( (en_time-st_time) > 60000 || (incoming_byte == 'q') )
    {
      time_out = 1;
      Serial.println(F("Time-out on config edit!"));
      Serial.println(F("bi:,ending config edit"));
    }
  }
  // MAX NUMBER OF CHROMS TO RUN
  if(!time_out)
  {
    st_time = millis();
    Serial.print(F("bj:, enter no of chroms (1-999, 999 = forever) DEFAULT: ")); Serial.println(max_cycles);
  }
  while( (!Serial.available()) && (!time_out) )
  {
    en_time = millis();
    if((en_time-st_time) > 60000)
    {
      time_out = 1;
      Serial.println(F("Time-out on config editor!"));
      Serial.println(F("bi:, ending config edit"));
    }
  }
  if(!time_out)
  {
    max_cycles = Serial.parseInt();
    Serial.print(F("new max_cycles = ")); Serial.println(max_cycles);
  }
  // TRAP INJECT TIME
  if(!time_out)
  {
    st_time = millis();
    Serial.print(F("bl:, enter inject time, seconds (30-300) DEFAULT: ")); Serial.println(inject_time);
  }
  while( (!Serial.available()) && (!time_out) )
  {
    en_time = millis();
    if((en_time-st_time) > 60000)
    {
      time_out = 1;
      Serial.println(F("Time-out on config editor!"));
      Serial.println(F("bi:, ending config edit"));
    }
  }
  if(!time_out)
  {
    inject_time = Serial.parseInt();
    Serial.print(F("new inject_time = ")); Serial.println(inject_time);
  } 
  // BACKFLUSH TIME
  if(!time_out)
  {
    st_time = millis();
    Serial.print(F("bm:, enter backflush time, seconds (30-300) DEFAULT: ")); Serial.println(backflush_time);
  }
  while( (!Serial.available()) && (!time_out) )
  {
    en_time = millis();
    if((en_time-st_time) > 60000)
    {
      time_out = 1;
      Serial.println(F("Time-out on config editor!"));
      Serial.println(F("bi:, ending config edit"));
    }
  }
  if(!time_out)
  {
    backflush_time = Serial.parseInt();
    Serial.print(F("new backflush_time = ")); Serial.println(backflush_time);
  }
  // SAMPLE VOLUME
  if(!time_out)
  {
    st_time = millis();
    Serial.print(F("bn:, enter target sample volume, scc (2-200) DEFAULT: ")); Serial.println(target_s_vol);
  }
  while( (!Serial.available()) && (!time_out) )
  {
    en_time = millis();
    if((en_time-st_time) > 60000)
    {
      time_out = 1;
      Serial.println(F("Time-out on config editor!"));
      Serial.println(F("bi:, ending config edit"));
    }
  }
  if(!time_out)
  {
    target_s_vol = Serial.parseInt();
    Serial.print(F("new target_s_vol = ")); Serial.println(target_s_vol);
  }
  // VOLUME TIMEOUT
  if(!time_out)
  {
    st_time = millis();
    Serial.print(F("bo:, enter volume integration time-out, seconds (60-600) DEFAULT: ")); Serial.println(vol_timeout);
  }
  while( (!Serial.available()) && (!time_out) )
  {
    en_time = millis();
    if((en_time-st_time) > 60000)
    {
      time_out = 1;
      Serial.println(F("Time-out on config editor!"));
      Serial.println(F("bi:, ending config edit"));
    }
  }
  if(!time_out)
  {
    vol_timeout = Serial.parseInt();
    Serial.print(F("new vol_timeout = ")); Serial.println(vol_timeout);
  } 
  // TARGET PUMP PRESSURE DIFFERENTIAL
  if(!time_out)
  {
    st_time = millis();
    Serial.print(F("bp:, enter target pump pressure differential, kPa (5-20) DEFAULT: ")); Serial.println(jamjar_kPa_target);
  }
  while( (!Serial.available()) && (!time_out) )
  {
    en_time = millis();
    if((en_time-st_time) > 60000)
    {
      time_out = 1;
      Serial.println(F("Time-out on config editor!"));
      Serial.println(F("bi:, ending config edit"));
    }
  }
  if(!time_out)
  {
    jamjar_kPa_target = Serial.parseInt();
    Serial.print(F("new jamjar_kPa_target = ")); Serial.println(jamjar_kPa_target);
  }  
  // NUMBER OF SAMPLES TO RUN BETWEEN CALS
  if(!time_out)
  {
    st_time = millis();
    Serial.print(F("bq:, enter number of samples to run between cals (0-999) DEFAULT: ")); Serial.println(sams_between_cals);
  }
  while( (!Serial.available()) && (!time_out) )
  {
    en_time = millis();
    if((en_time-st_time) > 60000)
    {
      time_out = 1;
      Serial.println(F("Time-out on config editor!"));
      Serial.println(F("bi:, ending config edit"));
    }
  }
  if(!time_out)
  {
    sams_between_cals = Serial.parseInt();
    Serial.print(F("new sams_between_cals = ")); Serial.println(sams_between_cals);
  }
  // TARGET CALIBRATION VOL
  if(!time_out)
  {
    st_time = millis();
    Serial.print(F("br:, enter target calibration volume (2-200) DEFAULT: ")); Serial.println(target_c_vol);
  }
  while( (!Serial.available()) && (!time_out) )
  {
    en_time = millis();
    if((en_time-st_time) > 60000)
    {
      time_out = 1;
      Serial.println(F("Time-out on config editor!"));
      Serial.println(F("bi:, ending config edit"));
    }
  }
  if(!time_out)
  {
    target_c_vol = Serial.parseInt();
    Serial.print(F("new target_c_vol = ")); Serial.println(target_c_vol);
  }  
  // DUAL SAMPLING
  if(!time_out)
  {
    st_time = millis();
    Serial.print(F("ck:, enter dual sampling mode, 0 = sam1 only (0-1) DEFAULT: ")); Serial.println(dual_sampling);
  }
  while( (!Serial.available()) && (!time_out) )
  {
    en_time = millis();
    if((en_time-st_time) > 60000)
    {
      time_out = 1;
      Serial.println(F("Time-out on config editor!"));
      Serial.println(F("bi:, ending config edit"));
    }
  }
  if(!time_out)
  {
    dual_sampling = Serial.parseInt();
    Serial.print(F("new dual_sampling = ")); Serial.println(dual_sampling);
  }  
  
  if(incoming_byte == 'c') // only delete existing config file if user has interacted
  {
    delete_configfile();
  }
  //goto_pause();
}


void manual_control()
{
/*  init pin 34 as output for sample 1 pneumax1
    init pin 36 as output for sample 2 pneumax2
    init pin 38 as output for cal pneumax3
    init pin 40 as output for blank pneumax4
    init pin 42 as output for valco actuation pneumax5
    init pin 44 as output for pre-column backflush flow pneumax6
    init pin 46 as output for main column flow pneumax7  */
  char incoming_byte = 0, time_out = 0;
  unsigned long st_time = 0, en_time = 0;
  
  Serial.println(F("cl:,manual control mode"));
  //Serial.println("Press 't' + enter to edit date & time (30 second time-out)");
  st_time = millis();
  while( (incoming_byte != 'm') && (!time_out) )
  {
    if(Serial.available())
    {
      incoming_byte = Serial.read();
      if(incoming_byte == 'm')
      {
        Serial.println(F("Running manual control mode..."));
        while( (incoming_byte != 'q') && (!time_out) )
        {
          if(Serial.available())
          {
            incoming_byte = Serial.read();
            switch(incoming_byte)
            {
              case '1':
                digitalWrite(34,!digitalRead(34));
                Serial.print(F("sample1 valve (pneumax1) "));
                if(digitalRead(34))
                {
                  Serial.println(F("ON!"));
                }
                else
                {
                  Serial.println(F("OFF!"));
                }
                st_time = millis(); // reset the time-out
                break;
             case '2':
                digitalWrite(36,!digitalRead(36));
                Serial.print(F("sample2 valve (pneumax2) "));
                if(digitalRead(36))
                {
                  Serial.println(F("ON!"));
                }
                else
                {
                  Serial.println(F("OFF!"));
                }
                st_time = millis(); // reset the time-out
                break;
             case '3':
                digitalWrite(38,!digitalRead(38));
                Serial.print(F("cal valve (pneumax3) "));
                if(digitalRead(38))
                {
                  Serial.println(F("ON!"));
                }
                else
                {
                  Serial.println(F("OFF!"));
                }
                st_time = millis(); // reset the time-out              
                break;
             case '4':
                digitalWrite(40,!digitalRead(40));
                Serial.print(F("blank valve (pneumax4) "));
                if(digitalRead(40))
                {
                  Serial.println(F("ON!"));
                }
                else
                {
                  Serial.println(F("OFF!"));
                }
                st_time = millis(); // reset the time-out              
                break;
             case '5':
                digitalWrite(42,!digitalRead(42));
                Serial.print(F("valco actuation valve (pneumax5) "));
                if(digitalRead(42))
                {
                  Serial.println(F("ON!"));
                }
                else
                {
                  Serial.println(F("OFF!"));
                }
                st_time = millis(); // reset the time-out              
                break;
             case '6':
                digitalWrite(44,!digitalRead(44));
                Serial.print(F("pre-column backflush flow valve (pneumax6) "));
                if(digitalRead(44))
                {
                  Serial.println(F("ON!"));
                }
                else
                {
                  Serial.println(F("OFF!"));
                }
                st_time = millis(); // reset the time-out              
                break;
             case '7':
                digitalWrite(46,!digitalRead(46));
                Serial.print(F("main column flow valve (pneumax7) "));
                if(digitalRead(46))
                {
                  Serial.println(F("ON!"));
                }
                else
                {
                  Serial.println(F("OFF!"));
                }    
                st_time = millis(); // reset the time-out  
                break;
             case '8':
                digitalWrite(48,!digitalRead(48));
                Serial.print(F("spare (pneumax8) "));
                if(digitalRead(48))
                {
                  Serial.println(F("ON!"));
                }
                else
                {
                  Serial.println(F("OFF!"));
                }    
                st_time = millis(); // reset the time-out  
                break; 
             case 'p': // solenoid valve 8, spare
                Serial.println(F("pump and flowmeter test ..."));
                jamjar_flowmeter_test(); 
                break;                 
             default:
                Serial.println(F("Invalid input - Try harder next time!")); 
            }           
          }
          //incoming_byte = '/0';     
          en_time = millis();
          if( (en_time-st_time) > 3600000 || (incoming_byte == 'q')) // 1 hour timeout once in manual mode
          {
            time_out = 2;
            //Serial.println(F("Time-out on manual control mode!"));
            Serial.println(F("cm:,ending manual control mode"));
          }     
        incoming_byte = '/0';
        }
      }  
    }   
    en_time = millis();
    if( ( ((en_time-st_time) > 60000)  && (time_out !=2) ) || (incoming_byte == 'q') )
    {
      time_out = 1;
      //Serial.println(F("Time-out on manual control mode!"));
      Serial.println(F("cm:,ending manual control mode"));
    }
    for(char thisPin = 34; thisPin <= 46; thisPin+=2)    
    {
      //Serial.print(F("Digital pin lo: ")); Serial.println(thisPin, DEC);
      digitalWrite(thisPin,LOW); // ensure all valves are off
    }
  }
}

void jamjar_flowmeter_test()
{
  byte check_done = 0, loop_limit = 0;
  
  Serial.print(F("jamjar_kPa_target = "));Serial.println(jamjar_kPa_target);
  actual_jamjar_P_kPa = read_10bit_analog(2); // get jamjar P_diff on pin 2
  //Serial.print("Pump on phase ... "); Serial.print("differential pressure (kPa) = "); Serial.println(actual_jamjar_P_kPa);   
  //while( (actual_jamjar_P_kPa < jamjar_kPa_target) && (!check_done) && (loop_limit < 8) )
  while( (!check_done) && (loop_limit < 8) )
  {
    //Serial.print(F("Pump on phase... diff' P (kPa) = ")); Serial.println(actual_jamjar_P_kPa);
    if( actual_jamjar_P_kPa < jamjar_kPa_target )
    {
      Serial.print(F("Pump on phase... diff' P (kPa) = ")); Serial.println(actual_jamjar_P_kPa);
      do_pump_cycle(); // start pump
    }
    else
    {
      Serial.print(F("Pump off phase... diff' P (kPa) = ")); Serial.println(actual_jamjar_P_kPa);
    }
    get_single_flow_reading();
    if(Serial1.available()) // check for when volume integration is ready
    {
       check_done = 1;
    }
    actual_jamjar_P_kPa = read_10bit_analog(2); // get jamjar P_diff on pin 2
    //Serial.print(F("loop_limit "));Serial.println(loop_limit);
    loop_limit++;
  }
  Serial.println(F("cn:,pump flowmeter test done!"));
}              



void get_single_flow_reading(void)
{
  unsigned long st_time = 0, en_time = 0;
  char timeout = 0;
  String F_data = "";
  serial_flush('1', 1); // flush serial1 port (1= quiet mode)
  Serial1.write('f');
  st_time = millis();
  while(!Serial1.available() && (!timeout) ) // wait for data to come back
  { 
     en_time = millis();
     if((en_time-st_time) > (5*1000))
     {
       timeout = 1;
       Serial.println(F("Flowmeter timeout :("));
       F_data = "999";
     }
  } 
  delay(100);  
  while(Serial1.available())
  {
    char inByte = Serial1.read();
    //Serial.write(inByte);
    F_data.concat(inByte);
  }
  Serial.print(F("flow (sccm) ")); Serial.println(F_data);
}

void do_chrom_cycle()
{
  char done = 0, rand_no = 0;
  int target_vol = 0;
  unsigned long st_time = 0, en_time = 0; 
  //digitalWrite(42,HIGH);
  //Serial.println(F("be:, valco switched to inject: purging inlet manifold (pneumax5 on)"));
  switch (run_type)
  {
    case 'S':
      digitalWrite(34,HIGH);
      Serial.println(F("bs:, sample1 valve on (pneumax1 on)"));
      target_vol = target_s_vol;
      break;
    case 'X': // sample 2 when implemented
      digitalWrite(36,HIGH);
      Serial.println(F("bu:, sample2 valve on (pneumax2 on)"));
      target_vol = target_s_vol;
      break;
    case 'C':
      digitalWrite(38,HIGH);
      Serial.println(F("bw:, cal valve on (pneumax3 on)"));
      if(last_target_c_vol == target_c_vol)
      {
        rand_no = random(4);
        target_vol = resp_cals[rand_no];
      }
      else
      {
        // use target_c_vol (precision cal)
        target_vol = target_c_vol;
      }
      last_target_c_vol = target_vol;      
      break;
    case 'B':
      digitalWrite(40,HIGH);
      Serial.println(F("by:, blank valve on (pneumax4 on)"));
      target_vol = target_c_vol;
      break;      
  }
  //Serial.println(F("starting jamjar regulation ..."));
  //jamjar_pump_phase();
  Serial.println(F("ax:,main column flow off"));
  digitalWrite(46,LOW); // main column off command  
  //delay(15000); // wait 15 seconds for inlet manifold purging
  trap_temperatureC = read_10bit_analog(1); // get trap temperature on pin 1
  Serial.print(F("bc:,")); Serial.println(trap_temperatureC,1); //bc: trap cold temperature
  myFile.print("bc:,"); myFile.println(trap_temperatureC,1); //bc: trap cold temperature
  //digitalWrite(42,LOW);
  //Serial.println(F("bf:, valco valve switched to load (pneumax5 off)"));
  start_volume_integration(target_vol);
  st_time = millis();
  do
  {
     en_time = millis(); 
     if(Serial1.available()) // check for when volume integration is ready
     { 
       switch (run_type) // switch off relevant valve first to minimise volume integration error
       {
         case 'S':
           digitalWrite(34,LOW);
           Serial.println(F("bt:, sample1 valve off (pneumax1 off)"));
           break;
         case 'X': // sample 2 when implemented
           digitalWrite(36,LOW);
           Serial.println(F("bv:, sample2 valve off (pneumax2 off)"));
           break;
         case 'C':
           digitalWrite(38,LOW);
           Serial.println(F("bx:, cal valve off (pneumax3 off)"));
           break;
         case 'B':
           digitalWrite(40,LOW);
           Serial.println(F("bz:, blank valve off (pneumax4 off)"));
           break;      
       }                     
       read_volume_integration();
       //goto_pause();
       done = 1;
     }
     if((!Serial1.available()) && (!done) ) // only if nothing incoming from the flowmeter
     {
       jamjar_pump_phase();
     }
     if((!Serial1.available()) && (!done) ) // only if nothing incoming from the flowmeter 
     {
       jamjar_rest_phase();
     }
  } 
  //while((en_time-st_time) < (load_time*1000) && (!done) );
  while((en_time-st_time) < ((unsigned long)vol_timeout*1000) && (!done) );
  if(!done)
  {
    Serial.println(F("Check flowmeter time out!!")); 
    Serial.println(F("as:,999")); // as: integrated volume
    myFile.println("as:,999"); // as: integrated volume
    Serial.println(F("at:,999")); // at: average flowrate
    myFile.println("at:,999"); // at: average flowrate
    Serial.println(F("au:,999")); // au: volume integration time
    myFile.println("au:,999"); // as: volume integration time
  }
  // read jamjar kPa here for housekeeping
  actual_jamjar_P_kPa = read_10bit_analog(2); // get jamjar P_diff on pin 2 
  Serial.print(F("bg:,")); Serial.println(actual_jamjar_P_kPa,1); //bg: sample pump pressure differential
  myFile.print("bg:,"); myFile.println(actual_jamjar_P_kPa,1); //bg: sample pump pressure differential  
  digitalWrite(49,HIGH); // green LED on (goto amber LED, red already on)
  Serial.println(F("al:,PID power on"));
  digitalWrite(24,HIGH); // PID power on command
  //trap_n2_purge();
  Serial.println(F("be:, valco to inject (pneumax5 on)"));  
  digitalWrite(42,HIGH); // pneumax 5, valco to inject  
  Serial.println(F("ba:,trap heat on..."));
  digitalWrite(26,HIGH); // trap heater on!
  delay(9000); // 9 second delay for trap heating
  Serial.println(F("aw:,main flow on"));
  digitalWrite(46,HIGH); // main column flow on command
  delay(9000); // 9 second delay for trap heating
  Serial.println(F("bb:,trap heat off"));
  digitalWrite(26,LOW); // trap heater off
  /*Serial.println(F("be:, valco valve switched to inject (pneumax5 on)"));  
  //digitalWrite(42,HIGH); // pneumax 5, valco to inject  
  Serial.println(F("ba:,trap power on"));
  digitalWrite(26,HIGH); // trap heater on!
  delay(10000); // 10 second delay for trap heating
  Serial.println(F("bb:,trap power off"));
  digitalWrite(26,LOW); // trap heater off
  Serial.println(F("aw:,main column flow on"));
  digitalWrite(46,HIGH); // main column flow on command*/
  trap_temperatureC = read_10bit_analog(1); // get trap temperature on pin 1
  Serial.print(F("bd:,")); Serial.println(trap_temperatureC,1); //bd: trap hot temperature
  myFile.print("bd:,"); myFile.println(trap_temperatureC,1); //bd: trap hot temperature  
  Serial.println(F("az:,pre-column backflush flow off"));
  digitalWrite(44,LOW); // pre-column backflush flow off command   
  //Serial.println("al:,PID power on");
  //digitalWrite(24,HIGH); // PID power on command
  //delay(2000); // wait for PID power on spike  
  read_ads1115(inject_time);
  //Serial.println("al:,PID power on");
  //digitalWrite(24,HIGH); // PID power on command 
  Serial.println(F("ay:,pre-column backflush flow on"));
  digitalWrite(44,HIGH); // pre-column backflush flow on command  
  Serial.println(F("bf:, valco valve switched to load (pneumax5 off"));
  digitalWrite(42,LOW);
  read_ads1115(backflush_time);
  Serial.println(F("am:,PID power off"));
  digitalWrite(24,LOW); // PID power off command
  //Serial.println("chrom finished");
  //goto_pause();
}


void trap_n2_purge()
{
  digitalWrite(40,HIGH);
  Serial.println(F("by:, blank valve on, N2 purge (pneumax4 on)"));
  delay(10000);
  digitalWrite(40,LOW);
  Serial.println(F("bz:, blank valve off (pneumax4 off)"));
}


void get_altimeter_pressure()
{
  unsigned long st_time = 0, en_time = 0;
  char timeout = 0;
  String P_data = "";
  serial_flush('1', 0); // flush serial1 port (1= quiet mode)
  Serial1.write('p');
  st_time = millis();
  while(!Serial1.available() && (!timeout) ) // wait for data to come back
  { 
     en_time = millis();
     if((en_time-st_time) > (5*1000))
     {
       timeout = 1;
       Serial.println(F("Altimeter pressure timeout :("));
       P_data = "999";
     }
  } 
  delay(100);  
  while(Serial1.available())
  {
    char inByte = Serial1.read();
    //Serial.write(inByte);
    P_data.concat(inByte);
  }
  Serial.print(F("aq:,")); Serial.println(P_data); // aq: altimeter pressure
  myFile.print("aq:,"); myFile.println(P_data); // aq: altimeter pressure
  //goto_pause();
}


void get_altimeter_temperature()
{
  unsigned long st_time = 0, en_time = 0;
  char timeout = 0;
  String T_data = "";
  serial_flush('1', 0); // flush serial1 port (1= quiet mode)
  Serial1.write('t');
  st_time = millis();
  while(!Serial1.available() && (!timeout) ) // wait for data to come back
  { 
     en_time = millis();
     if((en_time-st_time) > (5*1000))
     {
       timeout = 1;
       Serial.println(F("Altimeter temperature timeout :("));
       T_data = "999";
     }
  } 
  delay(100);  
  while(Serial1.available())
  {
    char inByte = Serial1.read();
    //Serial.write(inByte);
    T_data.concat(inByte);
  }
  Serial.print(F("ar:,")); Serial.println(T_data); // ar: altimeter temperature
  myFile.print("ar:,"); myFile.println(T_data); // ar: altimeter temperature
  //goto_pause();
}


void start_volume_integration(int t_vol)
{
  unsigned long st_time = 0, en_time = 0;
  char inByte, timeout = 0;
  Serial.println(F("Volume integration requested ..."));
  Serial.print(F("Target volume (scc) = ")); Serial.println(t_vol);
  serial_flush('1', 0); // flush serial1 port (1= quiet mode)
  Serial1.write('v');
  st_time = millis();
  while(!Serial1.available() && (!timeout)) // wait for 1st acc character to come back
  {
    en_time = millis();
    if((en_time-st_time) > (5*1000))
    {
      timeout = 1;
      Serial.println(F("Volume integration request timeout :("));
    }
  }
  inByte = Serial1.read();
  if(inByte == 'v')
  {
    Serial1.print(t_vol);
    //goto_pause();
  }
  delay(1500); // let the uno serial port timeout (default 1000 millseconds)
  Serial.print(F("Volume integration timeout (seconds) = ")); Serial.println(vol_timeout);
  serial_flush('1', 0); // flush serial1 port (1= quiet mode)
  Serial1.write('w');
  while(!Serial1.available()); // wait for 2nd acc character to come back
  inByte = Serial1.read();
  if(inByte == 'w')
  {
    Serial1.print(vol_timeout);
    //goto_pause();
  }
  Serial.println(F("Now integrating flow ..."));
}


void read_volume_integration()
{
  unsigned long st_time = 0, en_time = 0;
  char timeout = 0;  
  String V_data = "", F_data = "", T_data = "", S_data = "";
  char inByte, mybuffer[32], pos = 0;
  
  //for( char a = 0; a < sizeof((mybuffer)-1); a++ )
  for( char a = 0; a < 31; a++ )
  {
    mybuffer[a] = '.'; 
  }
  mybuffer[31]='\0';
  st_time = millis();
  while(!Serial1.available() && (!timeout) ) // wait for data to come back
  { 
     en_time = millis();
     if((en_time-st_time) > (5*1000))
     {
       timeout = 1;
       Serial.println("Flowmeter timeout :(");
       //P_data = "999";
     }
  }
  delay(100);                               // short delay to ensure entire packet has arrived in buffer    
  while(Serial1.available() && (pos < 30) ) // read bytes from serial1 as long as buffer pos is < 30
  {
    inByte = Serial1.read();
    mybuffer[pos++] = inByte;
    //delay(10);
    //Serial.print("pos "); Serial.println(pos, DEC);
  }
  mybuffer[31]='\0';
  Serial.print(F("input mybuffer = "));Serial.println(mybuffer);
  Serial.println(pos,DEC);
  // now data mine the input mybuffer for volume, flow rate and integration time
  for(char ind = 0; ind < pos; ind++)
  {
    switch(mybuffer[ind])
    {
      case 'V':
        ind++;
        while(( mybuffer[ind] != 'F' ) && ( mybuffer[ind] != '\0'))
        {
          V_data.concat(mybuffer[ind++]);
        }
        ind--;
        Serial.println(V_data);
        break;
      case 'F':
        ind++;
        while(( mybuffer[ind] != 'T' ) && ( mybuffer[ind] != '\0'))
        {
          F_data.concat(mybuffer[ind++]);
        }
        ind--;
        Serial.println(F_data);
        break; 
     case 'T':
        ind++;
        while(( mybuffer[ind] != 'S' ) && ( mybuffer[ind] != '\0'))
        {
          T_data.concat(mybuffer[ind++]);
        }
        ind--;
        Serial.println(T_data);
        break;
      case 'S':
        ind++;
        while(( mybuffer[ind] != 'E' ) && ( mybuffer[ind] != '\0'))
        {
          S_data.concat(mybuffer[ind++]);
        }
        ind--;
        Serial.println(S_data);
        break;       
    }
  }
  Serial.print(F("as:,")); Serial.println(V_data); // as: integrated volume
  myFile.print("as:,"); myFile.println(V_data); // as: integrated volume

  Serial.print(F("at:,")); Serial.println(F_data); // at: average flowrate
  myFile.print("at:,"); myFile.println(F_data); // at: average flowrate
 
  Serial.print(F("au:,")); Serial.println(T_data); // au: integration time
  myFile.print("au:,"); myFile.println(T_data); // au: integration time
  
  Serial.print(F("cr:,")); Serial.println(S_data); // cr: sync_error count
  myFile.print("cr:,"); myFile.println(S_data); // cr: sync_error count  
}

/*
void old_read_volume_integration()
{
  unsigned long st_time = 0, en_time = 0;
  char timeout = 0;
  
  String V_data = "", F_data = "", T_data = "";
  char inByte;
  st_time = millis();
  while(!Serial1.available() && (!timeout) ) // wait for data to come back
  { 
     en_time = millis();
     if((en_time-st_time) > (5*1000))
     {
       timeout = 1;
       Serial.println("Flowmeter timeout :(");
       //P_data = "999";
     }
  }
  inByte = Serial1.read();
  Serial.println(inByte);
  while(!Serial1.available() && (!timeout) ) // wait for data to come back
  { 
     en_time = millis();
     if((en_time-st_time) > (15*1000))
     {
       timeout = 1;
       Serial.println("Flowmeter timeout :(");
       //P_data = "999";
     }
  }
  delay(100);
  while(Serial1.available())
  {
    char V_Byte = Serial1.read();
    //Serial.print(V_Byte);
    V_data.concat(V_Byte);
  }
  Serial.print("as:,"); Serial.println(V_data); // as: integrated volume
  myFile.print("as:,"); myFile.println(V_data); // as: integrated volume
  //goto_pause();
  while(!Serial1.available() && (!timeout) ) // wait for data to come back
  { 
     en_time = millis();
     if((en_time-st_time) > (20*1000))
     {
       timeout = 1;
       Serial.println("Flowmeter timeout :(");
       //P_data = "999";
     }
  }
  inByte = Serial1.read();
  Serial.println(inByte);
  while(!Serial1.available() && (!timeout) ) // wait for data to come back
  { 
     en_time = millis();
     if((en_time-st_time) > (25*1000))
     {
       timeout = 1;
       Serial.println("Flowmeter timeout :(");
       //P_data = "999";
     }
  }
  delay(100);
  while(Serial1.available())
  {
    char F_Byte = Serial1.read();
    //Serial.print(F_Byte);
    F_data.concat(F_Byte);
  }
  Serial.print("at:,"); Serial.println(F_data); // at: average flowrate
  myFile.print("at:,"); myFile.println(F_data); // at: average flowrate
  //goto_pause();
  while(!Serial1.available() && (!timeout) ) // wait for data to come back
  { 
     en_time = millis();
     if((en_time-st_time) > (30*1000))
     {
       timeout = 1;
       Serial.println("Flowmeter timeout :(");
       //P_data = "999";
     }
  }
  inByte = Serial1.read();
  Serial.println(inByte);
  while(!Serial1.available() && (!timeout) ) // wait for data to come back
  { 
     en_time = millis();
     if((en_time-st_time) > (35*1000))
     {
       timeout = 1;
       Serial.println("Flowmeter timeout :(");
       //P_data = "999";
     }
  }
  delay(100);
  while(Serial1.available())
  {
    char T_Byte = Serial1.read();
    //Serial.print(T_Byte);
    T_data.concat(T_Byte);
  }
  Serial.print("au:,"); Serial.println(T_data); // au: integration time
  myFile.print("au:,"); myFile.println(T_data); // au: integration time 
}*/



/*
void jamjar_pump_phase()
{
  char check_done = 0;
  float actual_P_kPa = 0.0;
  
  do
  {
    Serial.println("Pump on phase ...");
    actual_P_kPa = read_jamjar_P_sensor();
    Serial.print("Jamjar pressure (kPa) = ");Serial.println(actual_P_kPa);
    do_pump_cycle();
    if(Serial1.available()) // check for when volume integration is ready
    {
       check_done = 1;
    }
    //delay(5000); // 10 seconds loop delay
  }
  while((actual_P_kPa < ((float)jamjar_kPa_target)) && (!check_done));
}


void jamjar_rest_phase()
{
  char check_done = 0;
  float actual_P_kPa = 0.0;
  
  do
  {
    Serial.println("Pump off phase ...");
    actual_P_kPa = read_10bit_analog(2); // read jamjar P_diff on channel 1
    Serial.print("Jamjar differential pressure (kPa) = ");Serial.println(actual_P_kPa);
    delay(250); // 1 seconds loop delay
    if(Serial1.available()) // check for when volume integration is ready
    {
       check_done = 1;
    }
  }
  while((actual_P_kPa > ((float)jamjar_kPa_target)) && (!check_done));
}
*/



void jamjar_pump_phase()
{
  byte check_done = 0, loop_limit = 0; 
  actual_jamjar_P_kPa = read_10bit_analog(2); // get jamjar P_diff on pin 2
  //Serial.print("Pump on phase ... "); Serial.print("differential pressure (kPa) = "); Serial.println(actual_jamjar_P_kPa);   
  while( (actual_jamjar_P_kPa < jamjar_kPa_target) && (!check_done) && (loop_limit < 8))
  {
    Serial.print(F("Pump on phase ... ")); Serial.print(F("diff' pressure (kPa) = ")); Serial.println(actual_jamjar_P_kPa);
    do_pump_cycle();
    if(Serial1.available()) // check for when volume integration is ready
    {
       check_done = 1;
    }
    actual_jamjar_P_kPa = read_10bit_analog(2); // get jamjar P_diff on pin 2
    Serial.print(F("loop_limit "));Serial.println(loop_limit);
    loop_limit++;
   }
}


void jamjar_rest_phase()
{
  unsigned long pump_st_time = 0, pump_en_time = 0;
  char timeout = 0;
  char check_done = 0; 
  actual_jamjar_P_kPa = read_10bit_analog(2); // get jamjar P_diff on pin 2
  while( (actual_jamjar_P_kPa > jamjar_kPa_target) && (!check_done) )
  {
    Serial.print(F("Pump off phase ... ")); Serial.print(F("diff' pressure (kPa) = "));Serial.println(actual_jamjar_P_kPa);
    pump_st_time = millis();
    while(!Serial1.available() && (!timeout)) // wait for 1st acc character to come back
    {
      pump_en_time = millis();
      if((pump_en_time-pump_st_time) > (2000))
      {
        timeout = 1;
      }
    }
    timeout = 0;
    //delay(250); // 1 seconds loop delay
    if(Serial1.available()) // check for when volume integration is ready
    {
       check_done = 1;
    }
    actual_jamjar_P_kPa = read_10bit_analog(2); // get jamjar P_diff on pin 2
  }
}

/*
float read_jamjar_P_sensor()
{
  int sensorPin = 2;      // input pin for the pressure sensor
  int sensorValue = 0;  // variable to store the value coming from the sensor
  float voltage_Value = 0.0, kPa_Value = 0.0;
  
  sensorValue = analogRead(sensorPin);// read the value from the sensor:
  //Serial.print("pressure reading counts = "); Serial.println(sensorValue);
  // seems to max out on vacuum port (-ve) at 883 counts (4.31 volts, 45.68 kPa, 6.62 psi)
  
  voltage_Value = ((float)sensorValue / 1024) * 5;
  //Serial.print("sensor voltage = "); Serial.println(voltage_Value); 
  
  kPa_Value = ((float)(voltage_Value * 200)/18) - 2.222;
  //Serial.print("pressure reading kPa = "); Serial.println(kPa_Value);; Serial.println("");
  return kPa_Value;
}*/


void do_pump_cycle()
{
  unsigned long pump_st_time = 0, pump_en_time = 0;
  char timeout = 0;
  
  digitalWrite(28, HIGH);   // turn pump on  
  pump_st_time = millis();
  while(!Serial1.available() && (!timeout)) // wait for 1st acc character to come back
  {
    pump_en_time = millis();
    if((pump_en_time-pump_st_time) > (1000))
    {
      timeout = 1;
    }
  }
  timeout = 0;
  digitalWrite(30, HIGH);   // turn bypass valve on
  pump_st_time = millis();
  while(!Serial1.available() && (!timeout)) // wait for 1st acc character to come back
  {
    pump_en_time = millis();
    if((pump_en_time-pump_st_time) > (1500))
    {
      timeout = 1;
    }
  }
  timeout = 0;
  digitalWrite(30, LOW);    // turn bypass valve off
  pump_st_time = millis();
  while(!Serial1.available() && (!timeout)) // wait for 1st acc character to come back
  {
    pump_en_time = millis();
    if((pump_en_time-pump_st_time) > (500))
    {
      timeout = 1;
    }
  }
  timeout = 0;
  digitalWrite(28, LOW);    // turn pump off  
  pump_st_time = millis();
  while(!Serial1.available() && (!timeout)) // wait for 1st acc character to come back
  {
    pump_en_time = millis();
    if((pump_en_time-pump_st_time) > (1000))
    {
      timeout = 1;
    }
  } 
}


void serial_flush(char port, char quiet)
{
  char flush_c;
  switch(port)
  {
    case '0': //main serial port on pins 0 & 1
      while(Serial.available() >0)
      {
        flush_c = Serial.read();
      }
      if(!quiet)
      {
        Serial.println(F("Flushed main Serial port"));
      }
      break;
    case '1': // serial port1 on pins 18 & 19
      while(Serial1.available() >0)
      {
        flush_c = Serial1.read();
      }
      if(!quiet)
      {
        Serial.println(F("Flushed Serial1 port"));
      }
      break;
  }
}
