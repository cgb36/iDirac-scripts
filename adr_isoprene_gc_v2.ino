/*
 adr_isoprene_gc_v2 code 20160708
 version date: 03-Oct-2016
 written for version 2 of the iDirac (motor shields for valve control)
 adding awm3200 flowmeter on analog pin 8

 14-Jun-2016 debugged problem with file already existing
 23-May-2016 added monthly data folders
 20-May-2016 added setup submenu
 18-May-2016 adding manual control of valves and pump/flowmeter readout
 implemented serial_flush subroutine
 updated buffer declaration for mybuffer etc
*/
 
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <WireRtcLib.h>
#include <Adafruit_ADS1015.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

//Adafruit_ADS1015 ads1015;
Adafruit_ADS1115 ads1115(0x48); // construct an ads1115 at address 0x48
WireRtcLib rtc;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS_A = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
Adafruit_MotorShield AFMS_B = Adafruit_MotorShield(0x61); 

// Select which 'motor' M1, M2, M3 or M4
Adafruit_DCMotor *motorA_ch1 = AFMS_A.getMotor(1); // FORWARD pneumax 1 sample 1,       BACKWARD pneumax 2 sample 2
Adafruit_DCMotor *motorA_ch2 = AFMS_A.getMotor(2); // FORWARD pneumax 3 cal,            BACKWARD pneumax 4 blank
Adafruit_DCMotor *motorA_ch3 = AFMS_A.getMotor(3); // FORWARD pneumax 5 inject,         BACKWARD pneumax 8 spare
Adafruit_DCMotor *motorA_ch4 = AFMS_A.getMotor(4); // FORWARD pneumax 6 backflush flow, BACKWARD spare

Adafruit_DCMotor *motorB_ch1 = AFMS_B.getMotor(1); // FORWARD pneumax 7 main flow,      BACKWARD spare
Adafruit_DCMotor *motorB_ch2 = AFMS_B.getMotor(2); // FORWARD pump,                     BACKWARD spare
Adafruit_DCMotor *motorB_ch3 = AFMS_B.getMotor(3); // FORWARD dynamco,                  BACKWARD spare
Adafruit_DCMotor *motorB_ch4 = AFMS_B.getMotor(4); // FOWARD spare,                     BACKWARD spare

/*
PORT MAP:
port 0:  motorA_ch1 FORWARD:  solenoid valve 1 (sample 1)
port 1:  motorA_ch1 BACKWARD: solenoid valve 2 (sample 2)
port 2:  motorA_ch2 FORWARD:  solenoid valve 3 (cal)
port 3:  motorA_ch2 BACKWARD: solenoid valve 4 (blank)
port 4:  motorA_ch3 FORWARD:  solenoid valve 5 (inject)
port 5:  motorA_ch3 BACKWARD: solenoid valve 8 (spare)
port 6:  motorA_ch4 FORWARD:  solenoid valve 6 (backflush flow)
port 7:  motorA_ch4 BACKWARD: spare
port 8:  motorB_ch1 FORWARD:  solenoid valve 7 (main flow)
port 9:  motorB_ch1 BACKWARD: spare
port 10: motorB_ch2 FORWARD:  pump
port 11: motorB_ch2 BACKWARD: spare
port 12: motorB_ch3 FORWARD:  dynamco
port 13: motorB_ch3 BACKWARD: spare
port 14: motorB_ch4 FORWARD:  spare
port 15: motorB_ch4 BACKWARD: spare
*/

byte reboot_switch = 42;   // pin 42 as input for rpi reboot switch
byte shutdown_switch = 43; // pin 43 as input for rpi shutdown switch
byte pause_switch = 44;    // pin 44 as input for arduino pause switch
byte ssr_1_oven = 47, ssr_2_trap = 48, ssr_3_pid = 49; // pin assignments for ssr's
byte last_sam = 0; // toggle for dual sampling mode, 0 = do sam1, 1 = do sam2
char run_type = 'S', batt_OK = 1; // assume battery is OK at the start
char flow_OK = 1; // assume flow is OK at the start
char str[25] = "";
char c_code[4] = "";
//char code_data[4] = "";
char code_data[5] = "";
char date_folder[6] = "";
char end_sequence = 0, sam_count = 0;
String ver = "ver 8-Jul-2016";
int chrom_count = 0, resp_cals[4];
long current_fileno = 0;

// config defaults
int max_cycles = 999;        // ka:, max number of chroms to run (999 = forever)
//int load_time = 60;        // kb:, seconds of trap load time (now replaced by vol_timeout)
int inject_time = 30;        // kc:, seconds of inject time
int backflush_time = 60;     // kd:, seconds of backflush time
int target_s_vol = 100;      // ke:, target sample volume (scc)
int vol_timeout = 300;       // kf:, volume integration time out (seconds)
int jamjar_kPa_target = 20;  // kg:, jamjar target pressure differential (kPa)
int sams_between_cals = 100; // kh:, number of samples to run between cals
int target_c_vol = 20;       // ki:, target calibration volume (scc)
int dual_sampling = 0;       // kj:, dual sampling mode, sam1 and sam2 (0 = sam1 only)

int last_target_c_vol = 0;  // need this to get alternate random response cal sequence

File myFile;

// change this to match your SD shield or module;
//     Arduino Ethernet shield: pin 4
//     Adafruit SD shields and modules: pin 10
//     Sparkfun SD shield: pin 8
const int chipSelect = 53; // on iDirac MEGA

float col_temperatureC = 0, trap_temperatureC = 0, input_voltage = 0; 
float actual_jamjar_P_kPa = 0.0, awm3200_voltage = 0;
int buttonState4 = 0, buttonState5 = 0, buttonState6 = 0; // to store digital input state for switches)

void setup()
{
    Wire.begin();
    //rtc.begin();
    // Open serial communications, do not wait for port to open ...
    Serial.begin(9600);  // comms with RPi
    Serial1.begin(9600); // comms with arduino micro (flowmeter)
    Serial2.begin(9600); // comms with LCD
    // initialise the motor shields   
    //AFMS_A.begin();  // create with the default frequency 1.6 KHz
    AFMS_A.begin(1000);  // OR with a different frequency, say 1.0 KHz
    //AFMS_B.begin();  // create with the default frequency 1.6 KHz
    AFMS_B.begin(1000);  // OR with a different frequency, say 1.0 KHz
    Serial.println(F("adr_isoprene_gc_v2"));
    Serial.println(ver);
    // initialise LCD
    backlight_off();
    lcd_print("Welcome folks!","");
    delay(4000);
    lcd_print("iDirac v2", ver);
    delay(4000);
    rtc_init(); // init real-time clock
    // set up switch and LED D I/O pins ...
    pinMode(reboot_switch,INPUT);   // input for rpi reboot switch
    pinMode(shutdown_switch,INPUT); // input for rpi shutdown switch
    pinMode(pause_switch,INPUT);    // input for arduino pause switch
    pinMode(45,OUTPUT);             // init green_led pin 45 as output
    led_drive('g',0);               // green LED off
    pinMode(46,OUTPUT);             // init red_led pin 46 as output
    led_drive('r',0);               // red LED off    
    // set up the ssr's and default to off ...
    pinMode(ssr_1_oven,OUTPUT);   // init as output
    digitalWrite(ssr_1_oven,LOW); // default is off
    pinMode(ssr_2_trap,OUTPUT);   // init as output
    digitalWrite(ssr_2_trap,LOW); // default is off
    pinMode(ssr_3_pid,OUTPUT);    // init as output
    digitalWrite(ssr_3_pid,LOW);  // default is off
    led_drive('a', 1); // amber LED on    
    SDcard_init();
    setup_menu();
    led_drive('a', 0); // amber LED off
    powerup_test();    
    Serial.println(F("aw:,main flow on"));
    motor_drive(8, 1, 0); // port 8 ON non-quiet (solenoid valve 7, main flow)
    Serial.println(F("ay:,backflush flow on"));
    motor_drive(6, 1, 0); // port 6 ON non-quiet (solenoid valve 6, backflush flow)
    Serial.println(F("aj:,oven on"));
    digitalWrite(ssr_1_oven,HIGH); // column oven on command
    //Serial.println("al:,PID on");
    //digitalWrite(ssr_3_pid,HIGH); // PID power on command  
    //Serial.println("Getting differential reading from AIN0 (P) and AIN1 (N)");
    ads1115.begin(); // initialise ads1115
    //ads1115.setGain(GAIN_TWOTHIRDS); // DEFAULT for an input range of +- 6.144V
    //Serial.println("ADC gain TWOTHIRDS: +/- 6.144V (1 bit = 0.1875mV)");
    //ads1115.setGain(GAIN_ONE);     // for an input range of +- 4.096V
    //Serial.println("ADC gain ONE: +/- 4.096V (1 bit = 0.125mV)");
    //ads1115.setGain(GAIN_TWO);     // for an input range of +- 2.048V
    //Serial.println("ADC gain TWO: +/- 2.048V (1 bit = 0.0625mV)");
    ads1115.setGain(GAIN_FOUR);    // for an input range of +- 1.024V
    //Serial.println("ADC gain FOUR: +/- 1.024V (1 bit = 0.03125mV)");
    //ads1115.setGain(GAIN_EIGHT);   // for an input range of +- 0.512V
    //Serial.println("ADC gain EIGHT: +/- 0.512V (1 bit = 0.015625mV)");
    //ads1115.setGain(GAIN_SIXTEEN); // for an input range of +- 0.256V
    //Serial.println("ADC gain SIXTEEN: +/- 0.256V (1 bit = 0.0078125mV)");    
    //delete_configfile();
    if(SD.exists("config.txt"))
    {
      Serial.println(F("config.txt found"));
    }
    else
    {
      Serial.println(F("making new config.txt from new/default values"));
      create_configfile();
    }
    delay(1000);
    read_configfile();
    set_cal_resp_vols();
    //goto_pause();
}


void loop()
{
  char outstr[10];
  
  //end_sequence = 0;
  do
  {
    //check_pause();
    if(!batt_OK)
    {
      low_batt_standby();
    }
    check_overheat();
    check_pause();
    awm3200_voltage = read_10bit_analog(8,1); // get awm3200 output voltage on pin 8
    dtostrf(awm3200_voltage, 4, 2, outstr);
    lcd_print("awm3200 voltage", outstr);
    if(!flow_OK)
    {
      low_flow_standby();
    }
    chk_index_exists();
    delay(1000);
    read_indexfile();
    col_temperatureC = read_10bit_analog(0,1); // get column temperature on pin 0, ignore first reading
    input_voltage = read_10bit_analog(3,1); // get input DC voltage on pin 3
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
  check_overheat();
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
    Serial.print(F("random resp_cals = "));Serial.println(resp_cals[myloop], DEC);
  }
  //goto_pause();*/
}


void SDcard_init()
{
    Serial.print(F("Init SD card..."));
    // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
    // Note that even if it's not used as the CS pin, the hardware SS pin 
    // (10 on most Arduino boards, 53 on the Mega) must be left as an output 
    // or the SD library functions will not work. 
    pinMode(SS, OUTPUT);
    if(!SD.begin(chipSelect))
    {
      Serial.println(F("init failed"));
      return;
    }
    Serial.println(F("init OK"));
}


void rtc_init()
{
    rtc.begin();
    /*if(!rtc.isrunning())
    {
      Serial.println("RTC is NOT running!");
      // following line sets the RTC to the date & time this sketch was compiled
      // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      // This line sets the RTC with an explicit date & time, for example to set
      // January 21, 2014 at 3am you would call:
      // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }*/
    if(rtc.isDS1307())
    {
      Serial.println(F("Found DS1307"));
    }
    else if (rtc.isDS3231())
    {
      Serial.println(F("Found DS3231"));
    }
    else
    {
      Serial.println(F("RTC not found"));
    } 
    WireRtcLib::tm* t = rtc.getTime(); // get date and time
    Serial.print(F("Date: "));Serial.print(t->mday);Serial.print(":");Serial.print(t->mon);Serial.print(":20");Serial.print(t->year);
    Serial.print(F("   Time: "));Serial.print(t->hour);Serial.print(":");Serial.print(t->min);Serial.print(":");Serial.println(t->sec);
    // check rtc temperature 
    if (rtc.isDS3231())
    {
      int8_t i;
      uint8_t f;
      rtc.getTemp(&i, &f);
      Serial.print(F("Temp: "));Serial.print(i);Serial.print(".");Serial.print(f);Serial.println(" C");
    }
}


void chk_index_exists()
{
  if(SD.exists("index.txt"))
  {
    Serial.println(F("index.txt exists, reading ..."));
    //create_indexfile();
  }
  else
  {
    Serial.println(F("making index.txt"));
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
    Serial.print(F("Reading index.txt... "));
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
     Serial.print(F("error opening index.txt")); Serial.println(F(" ...halting code..."));
     while(1){} // holds the loop
  }
  Serial.print(F("last file no: ")); Serial.println(current_fileno);
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
    Serial.print(F("Writing to index.txt..."));
    ltoa(current_fileno, str,10);
    myFile.write(str);
    // close the file:
    myFile.close();
    Serial.println(F("done"));
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
    sam_count++;
  }
}


void create_chromfile()
{
  char fullpath_str[25] = "", folder_str[15] = "/20", chromfile_str[15] = "", chromfile_OK = 0;
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
    Serial.println(F("Writing to chromfile... "));
    // send data folder to RPi
    //ltoa(folder_str, str,10);
    Serial.print(F("cq:,")); Serial.println(folder_str); // cq: year and month folder
    // send file index number to RPi  
    ltoa(current_fileno, str,10);
    Serial.print(F("aa:,")); Serial.println(str); //aa: file index no.
    myFile.print("aa:,"); myFile.println(str); //aa: file index no.
    // get current date and time
    WireRtcLib::tm* t = rtc.getTime();
    // print data time to RPi
    Serial.print(F("ab:,")); // ab: date & time stamp
    Serial.print("20");Serial.print(t->year);Serial.print(",");Serial.print(t->mon);Serial.print(",");Serial.print(t->mday);Serial.print(",");
    Serial.print(t->hour);Serial.print(",");Serial.print(t->min);Serial.print(",");Serial.println(t->sec);       
    Serial.print(F("run type = ")); Serial.println(run_type);
    myFile.print("ab:,"); // ab: date & time stamp
    // write out date and time to microSD
    myFile.print("20");myFile.print(t->year);myFile.print(",");myFile.print(t->mon);myFile.print(",");myFile.print(t->mday);myFile.print(",");
    myFile.print(t->hour);myFile.print(",");myFile.print(t->min);myFile.print(",");myFile.println(t->sec);  
    Serial.print(F("ac:,")); Serial.println(run_type); //ac: run type
    myFile.print("ac:,"); myFile.println(run_type); //ac: run type
    Serial.print(F("ad:,")); Serial.println(col_temperatureC,1); //ad: column temperature
    myFile.print("ad:,"); myFile.println(col_temperatureC,1); //ad: column temperature
    Serial.println(F("ca:,999")); // ca: dummy R-Pi system temperature
    myFile.println("ca:,999"); // ca: dummy R-Pi system temperature
    Serial.print(F("cb:,")); Serial.println(input_voltage,1); //cb: input DC voltage
    myFile.print("cb:,"); myFile.println(input_voltage,1); //cb: input DC voltage    
    // now form info strings to write to lcd
    itoa(t->mday,asc_time,10);
    strcat(outstr1,asc_time);
    strcat(outstr1,"."); 
    itoa(t->mon,asc_time,10);
    strcat(outstr1,asc_time);  
    strcat(outstr1,"  ");
    itoa(t->hour,asc_time,10);
    strcat(outstr1,asc_time);
    strcat(outstr1,":");  
    itoa(t->min,asc_time,10);
    strcat(outstr1,asc_time);
    strcat(outstr1,":"); 
    itoa(t->sec,asc_time,10);
    strcat(outstr1,asc_time);
    switch(run_type)
    {
      case 'S':
        strcat(outstr2, "S");
        break;
      case 'X':
        strcat(outstr2, "X");
        break;  
     case 'C':
        strcat(outstr2, "C");
        break;
     case 'B':
        strcat(outstr2, "B");
        break;     
    }
    strcat(outstr2," "); strcat(outstr2,chromfile_str);
    lcd_print(outstr1, outstr2);
    get_altimeter_pressure();
    get_altimeter_temperature(1);
    if(chromfile_OK)
    {
      Serial.println(F("an:,Chrom file name new OK"));
      myFile.println(F("an:,Chrom file name new OK")); //an: index was correctly read and the file is new
    }
    else
    {
      Serial.println(F("ao:,Chrom file already existed KO!"));
      myFile.println(F("ao:,Chrom file already existed KO!")); //an: Chrom file already existed so new filendame found the long way!
    }  
    do_chrom_cycle();
    myFile.close();// close the file:
    Serial.println(F("av:,Chrom done")); Serial.println(); Serial.println();      
  }
  else
  {
    // if the file didn't open, print an error:
    Serial.print(F("error opening chromfile... "));
    Serial.println(fullpath_str);
    goto_pause();
  }
}


void determine_monthly_folder()
{
  char year_ascii[8] = "", month_ascii[8]= "", day_ascii[8]= "", hour_ascii[8]= "";
  int year_folder = 0, month_folder = 0, day_folder = 0, hour_folder = 0; 
  
  Serial.print(F("Determining monthly data folder ..."));
  WireRtcLib::tm* t = rtc.getTime(); // get date and time
  Serial.print(F("Date: "));Serial.print(t->mday);Serial.print(":");Serial.print(t->mon);Serial.print(":20");Serial.print(t->year);
  Serial.print(F("   Time: "));Serial.print(t->hour);Serial.print(":");Serial.print(t->min);Serial.print(":");Serial.println(t->sec);
  year_folder = t->year; month_folder = t->mon; day_folder = t->mday; hour_folder = t->hour;
  //DateTime now = rtc.now();
  //Serial.print(now.year(),DEC); Serial.print(','); Serial.print(now.month(),DEC); Serial.print(','); Serial.print(now.day(),DEC); Serial.print(',');
  //Serial.print(now.hour(),DEC); Serial.print(','); Serial.print(now.minute(),DEC); Serial.print(','); Serial.println(now.second(),DEC);
  //year_folder = now.year(); month_folder = now.month(); day_folder = now.day(); hour_folder = now.hour();
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


float read_10bit_analog(int sensorPin, byte message) // sensorPin 0 = column temp', sensorPin 1 = trap temp', sensorPin 2 = p_diff', sensorPin 3 = input voltage, sensorPin 8 = awm3200 voltage
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
      // converting from 10 mV per degree with 500 mV offset to allow for -ve temps
      if(message)
      {
        Serial.print(F("Col' temp = ")); Serial.print(converted_value,1); Serial.println(" deg C");
      }
      break;
    case 1:
      converted_value = (voltage - 0.5) * 100; // convert voltage to temperature
      // converting from 10 mV per degree with 500 mV offset to allow for -ve temps
      if(message)
      {      
        Serial.print(F("Trap temp = ")); Serial.print(converted_value,1); Serial.println(" deg C");
      }
      break;
    case 2:     
      // counts seem to max out on vacuum port (-ve) at 883 counts (4.31 volts, 45.68 kPa, 6.62 psi)
      converted_value = ((float)(voltage * 200)/18) - 2.222;
      if(message)
      {
        Serial.print(F("pressure reading kPa = ")); Serial.println(converted_value);; Serial.println("");
      }
      break;              
    case 3:
      converted_value = voltage * 60/10; // convert 5V voltage to real input voltage
      if(message)
      {    
        Serial.print(F("input voltage = ")); Serial.print(converted_value,1); Serial.println(F(" VDC"));
      }
      if (converted_value < 10)
      {
        Serial.println(F("voltage low, battery flat?"));
        batt_OK = 0;
      }      
      break;
    case 8:
      converted_value = voltage; // converted voltage is same as ADC voltage
      if(message)
      {
        Serial.print(F("awm3200 voltage = ")); Serial.print(converted_value,2); Serial.println(F(" VDC"));
      }
      if(converted_value < 1)
      {
        Serial.println(F("flow too low!!"));
        flow_OK = 0;
      }
      else
      {
        flow_OK = 1;
      }
      break;
  }
  delay(1000);
  return converted_value;
}


void check_pause()
{
    char timeout = 0, standby_stage = 0;
    unsigned long start_time = 0, end_time = 0;
    
    led_drive('r',0); // red LED off
    buttonState6 = digitalRead(pause_switch); // check for arduino pause status
    //Serial.print("button 6 state: ");
    //Serial.println(buttonState6);
    if((buttonState6 == LOW) || (end_sequence) ) // check arduino pause switch
    {      
        if(buttonState6 == LOW)
        {
          Serial.print(F("af:,pause request: ")); Serial.println(buttonState6);
          Serial.println("");
          lcd_print("pause switched","");
        }
        if(end_sequence)
        {
          Serial.println(F("ap:,max no of chroms reached"));
        }
        Serial.println(F("Staged standby, 5 min interval..."));
        delay(1000);
        start_time = millis();
        led_drive('g',1); // green LED on
        while((!buttonState6) || (end_sequence) )
        { 
            low_batt_standby();  
            led_drive('g',1); // green LED on  
            buttonState4 = digitalRead(reboot_switch); // check for pi reboot switch
            delay(1000);
            buttonState5 = digitalRead(shutdown_switch); // check for pi shutdown switch
            delay(1000);
            buttonState6 = digitalRead(pause_switch); // check for arduino pause status
            delay(1000);
            if((buttonState6 == HIGH) && (!end_sequence) )
            {
                Serial.print(F("ag:,resume request: ")); Serial.println(buttonState6);
                //Serial.println("");
                lcd_print("resuming...","");
                Serial.println(F("ay:,backflush flow on"));
                motor_drive(6, 1, 0); // port 6 ON non-quiet (solenoid valve 6, backflush flow)
                Serial.println(F("aw:,main flow on"));
                motor_drive(8, 1, 0); // port 8 ON non-quiet (solenoid valve 7, main flow)
                Serial.println(F("aj:,oven on"));
                digitalWrite(ssr_1_oven,HIGH); // column oven on command
                //Serial.println("al:,PID power on");
                //digitalWrite(ssr_3_pid,HIGH); // PID power on command
                end_sequence = 0;
                led_drive('r',1); // red LED on
                led_drive('g',0); // green LED off
                delay(1000);
            }
            if(buttonState4 == HIGH)
            {
                Serial.println(F("am:,PID off"));
                digitalWrite(ssr_3_pid,LOW); // PID power off command
                Serial.println(F("ak:,oven off"));
                digitalWrite(ssr_1_oven,LOW); // column oven off command                
                Serial.println(F("ax:,main flow off"));
                motor_drive(8, 0, 0); // port 8 OFF non-quiet (solenoid valve 7, main flow OFF)                
                Serial.println(F("az:,backflush flow off"));
                motor_drive(6, 0, 0); // port 6 OFF non-quiet (solenoid valve 6, backflush flow OFF)                        
                Serial.println(F("ah:,pi reboot request"));
                lcd_print("RPi rebooting","");
                while(1){}; // holds the loop
                //Serial.println(buttonState4);
                //Serial.println("");
                //delay(1000);
            }
            if(buttonState5 == HIGH)
            {
                Serial.println(F("am:,PID off"));
                digitalWrite(ssr_3_pid,LOW); // PID power off command
                Serial.println(F("ak:,oven off"));
                digitalWrite(ssr_1_oven,LOW); // column oven off command                
                Serial.println(F("ax:,main flow off"));
                motor_drive(8, 0, 0); // port 8 OFF non-quiet (solenoid valve 7, main flow OFF)                
                Serial.println(F("az:,backflush flow off"));
                motor_drive(6, 0, 0); // port 6 OFF non-quiet (solenoid valve 6, backflush flow OFF)  
                Serial.println(F("ai:,pi shutdown request"));
                lcd_print("RPi shutdown...","");
                while(1){}; // holds the loop
                //Serial.println(buttonState5);
                //Serial.println("");
                //delay(1000);
            }
            end_time = millis();
            if(((end_time-start_time) > 60000) && ((end_time-start_time) < 63000))
            {
              Serial.println(F("1 min..."));
              lcd_print("1 min...","");
            }
            if(((end_time-start_time) > 120000) && ((end_time-start_time) < 123000))
            {
              Serial.println(F(" 2 min..."));
              lcd_print("2 min...","");
            }
            if(((end_time-start_time) > 180000) && ((end_time-start_time) < 183000))
            {
              Serial.println(F("  3 min..."));
              lcd_print("3 min...","");
            }         
            if(((end_time-start_time) > 240000) && ((end_time-start_time) < 243000))
            {
              Serial.println(F("   4 min..."));
              lcd_print("4 min...","");
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
                Serial.println(F("am:,PID off"));
                lcd_print("PID off","");
                digitalWrite(ssr_3_pid,LOW); // PID power off command
                standby_stage = 1;
                timeout = 0;
                break;
              case 2:
                Serial.println(F("ak:,oven off"));
                lcd_print("Oven off","");
                digitalWrite(ssr_1_oven,LOW); // column oven off command 
                standby_stage = 2;
                timeout = 0;
                break;  
              case 3:
                Serial.println(F("ax:,main flow off"));
                motor_drive(8, 0, 0); // port 8 OFF non-quiet (solenoid valve 7, main flow OFF)                
                Serial.println(F("az:,backflush flow off"));
                motor_drive(6, 0, 0); // port 6 OFF non-quiet (solenoid valve 6, backflush flow)  
                Serial.println(F("Standing by: PID, oven, flows OFF"));
                lcd_print("flows off","Standyby!");
                standby_stage = 3;
                timeout = 0;
                break;              
            }
            if(standby_stage)
            {
              led_drive('g',0); // green LED off (flashing stage)
              delay(1000);
            }
        }
    }
    led_drive('r',1); // red LED on
    led_drive('g',0); // green LED off
}


void check_overheat() // reads PID temperature and if too high switches oven off and waits
{
  byte overheat_level = 48;
  byte okheat_level = 46;
  byte too_hot = 0;
  float PID_temperature = get_altimeter_temperature(0);
  Serial.print(F("PID temperature C = ")); Serial.println(PID_temperature); // print altimeter temperature
  if( (PID_temperature > overheat_level) && (PID_temperature != 999) )
  {
    too_hot = 1;
    Serial.print(F("PID temperature too high! > "));Serial.println(overheat_level);
    Serial.println(F("ak:,oven off"));
    lcd_print("Oven off","too hot!");
    digitalWrite(ssr_1_oven,LOW); // column oven off command
    while(too_hot)
    {
      delay(60000);
      PID_temperature = get_altimeter_temperature(0);
      Serial.print(F("PID temperature C = ")); Serial.println(PID_temperature, 1); // print altimeter temperature
      if(PID_temperature < okheat_level)
      {
        too_hot = 0;
        Serial.print(F("PID temperature now OK! < "));Serial.println(okheat_level,1);
        Serial.println(F("aj:,oven on"));
        digitalWrite(ssr_1_oven,HIGH); // column oven on command        
      }
      if(PID_temperature == 999) break;
    }
  }  
  //goto_pause();
}


void low_batt_standby()
{
  // read battery analog channel again to double check!
  input_voltage = read_10bit_analog(3,1); // get input DC voltage on pin 3
  if(!batt_OK)
  {
    Serial.println(F("voltage low! Battery flat?")); 
    Serial.println(F("am:,PID off"));
    digitalWrite(ssr_3_pid,LOW); // PID power off command
    delay(1000);
    Serial.println(F("ak:,oven off"));
    digitalWrite(ssr_1_oven,LOW); // column oven off command                
    Serial.println(F("ax:,main flow off"));
    delay(1000);    
    motor_drive(8, 0, 0); // port 8 OFF non-quiet (solenoid valve 7, main flow OFF)                
    Serial.println(F("az:,backflush flow off"));
    delay(1000);    
    motor_drive(6, 0, 0); // port 6 OFF non-quiet (solenoid valve 6, backflush flow OFF)
    Serial.println(F("ai:,pi shutdown request"));
    led_drive('g',0); // green LED off
    lcd_print("RPi shutdown","");
    while(1)
    {
        led_drive('r',1); // red LED on
        delay(1000);
        led_drive('r',0); // red LED off
        delay(1000);
        lcd_print("low battery","");
    }      
  } 
}

void low_flow_standby()
{
  if(!flow_OK)
  {
    while(!flow_OK)
    {
      motor_drive(6, 1, 0); //port 6 ON non-quiet (solenoid valve 6, backflush flow)
      delay(10000); // wait for flow to stabilise
      // read flow analog channel again to double check!
      awm3200_voltage = read_10bit_analog(8,1); //get awm3200 voltage on pin 8
      Serial.println(F("Carrier flow too low!"));
      lcd_print("Carrier flow", "too low!");
      Serial.println(F("am:,PID off"));
      digitalWrite(ssr_3_pid, LOW); //PID power off command
      delay(1000);
      Serial.println(F("ak:,oven off"));
      digitalWrite(ssr_1_oven,LOW); //column oven off command
      Serial.println(F("ax:,main flow off"));
      delay(1000);
      motor_drive(8, 0, 0); // port 8 OFF non-quiet (solenoid valve 7, main flow OFF)
      Serial.println(F("az:,backflush flow off"));
      delay(1000);
      motor_drive(6, 0, 0); // port 6 OFF non-quiet (solenoid valve 6, backflush flow OFF)
      delay(10000);
      check_pause();
    }
  }
}

void goto_pause()
{
      Serial.println(F("Arduino paused!"));
      lcd_print("Arduino paused!", "");
      while(1){} // holds the loop
}


void powerup_test()
{
  char outstr[10];
  int delay_on = 1000;
  int delay_off = 500;
  
  lcd_print("valve test", "");
  motor_drive(0, 1, 0); // port 0 ON non-quiet (solenoid valve 1, sample 1)
  delay(delay_on);
  motor_drive(0, 0, 0); // port 0 OFF non-quiet (solenoid valve 1, sample 1)
  delay(delay_off);
  
  motor_drive(1, 1, 0); // port 1 ON non-quiet (solenoid valve 2, sample 2)
  delay(delay_on);
  motor_drive(1, 0, 0); // port 1 OFF non-quiet (solenoid valve 2, sample 2)
  delay(delay_off);
  
  motor_drive(2, 1, 0); // port 2 ON non-quiet (solenoid valve 3, cal)
  delay(delay_on);
  motor_drive(2, 0, 0); // port 2 OFF non-quiet (solenoid valve 3, cal)
  delay(delay_off);
  
  motor_drive(3, 1, 0); // port 3 ON non-quiet (solenoid valve 4, blank)
  delay(delay_on);
  //goto_pause(); 
  motor_drive(3, 0, 0); // port 3 OFF non-quiet (solenoid valve 4, blank)
  delay(delay_off); 
   
  motor_drive(4, 1, 0); // port 4 ON non-quiet (solenoid valve 5, inject)
  delay(delay_on);
  motor_drive(4, 0, 0); // port 4 OFF non-quiet (solenoid valve 5, inject)
  delay(delay_off); 
  
  motor_drive(6, 1, 0); // port 6 ON non-quiet (solenoid valve 6, backflush flow)
  //delay(delay_on);
  delay(10000); // wait for flow to stabilise
  // now check backflush flow
  awm3200_voltage = read_10bit_analog(8,1); // get awm3200 voltage on pin 8
  dtostrf(awm3200_voltage, 4, 2, outstr);
  lcd_print("awm3200 voltage", outstr);
  motor_drive(6, 0, 0); // port 6 OFF non-quiet (solenoid valve 6, backflush flow)
  delay(delay_off);  
  
  motor_drive(8, 1, 0); // port 8 ON non-quiet (solenoid valve 7, main flow)
  delay(delay_on);
  motor_drive(8, 0, 0); // port 8 OFF non-quiet (solenoid valve 7, main flow)
  delay(delay_off);   
 
  motor_drive(5, 1, 0); // port 5 ON non-quiet (solenoid valve 8, spare)
  delay(delay_on);
  motor_drive(5, 0, 0); // port 5 OFF non-quiet (solenoid valve 8, spare)
  delay(delay_off);   
  
  motor_drive(10, 1, 0); // port 10 ON non-quiet (pump)
  delay(delay_on);
  motor_drive(10, 0, 0); // port 10 OFF non-quiet (pump)
  delay(delay_off);  
  
  motor_drive(12, 1, 0); // port 12 ON non-quiet (dynamco)
  delay(delay_on);
  motor_drive(12, 0, 0); // port 12 OFF non-quiet (dynamco)
  delay(delay_off);  
  
  Serial.println(F("LED status test..."));
  lcd_print("red LED on", "");
  led_drive('r',1); // red LED on
  delay(2000);
  led_drive('r',0); // red LED off
  delay(1000);
  lcd_print("green LED on", "");
  led_drive('g',1); // green LED on
  delay(2000);
  led_drive('g',0); // green LED off
  delay(1000);
  lcd_print("amber LED on", "");
  led_drive('a',1); // amber LED on
  delay(2000);
  led_drive('a',0); // amber LED off
  delay(1000);
  
  Serial.println(F("Trap heat test..."));
  trap_temperatureC = read_10bit_analog(1,1); // get trap cold temperature on pin 1
  dtostrf(trap_temperatureC, 4, 1, outstr); 
  lcd_print("Trap T deg C", outstr);
  if(flow_OK)
  { 
    digitalWrite(ssr_2_trap,HIGH); // trap heater on
    Serial.println(F("Trap heat on..."));
    delay(10000);
    digitalWrite(ssr_2_trap,LOW); // trap heater off
    Serial.println(F("Trap heat off"));      
    delay(5000);
    trap_temperatureC = read_10bit_analog(1,1); // get trap hot temperature on pin 1  
    dtostrf(trap_temperatureC, 4, 1, outstr); 
    lcd_print("Trap T deg C", outstr);
  }
  else
  {
    Serial.println(F("Not heating trap, carrier flow too low!"));
  }
  input_voltage = read_10bit_analog(3,1); // get input DC voltage on pin 3
  dtostrf(input_voltage, 4, 1, outstr); 
  lcd_print("input voltage", outstr);
  delay(3000);
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
    Serial.print(F("Writing config.txt..."));
    itoa(max_cycles, str,10);
    myFile.print("ka:,");myFile.print(str);myFile.println(F(",max number of chroms to run (999 = forever)"));
    //itoa(load_time, str,10);
    //myFile.print("kb:,");myFile.print(str);myFile.println(",trap load time (seconds)");
    itoa(inject_time, str,10);
    myFile.print("kc:,");myFile.print(str);myFile.println(F(",inject time (seconds)"));
    itoa(backflush_time, str,10);
    myFile.print("kd:,");myFile.print(str);myFile.println(F(",backflush time (seconds)"));     
    itoa(target_s_vol, str,10);
    myFile.print("ke:,");myFile.print(str);myFile.println(F(",target sample volume (scc)"));    
    itoa(vol_timeout, str,10);
    myFile.print("kf:,");myFile.print(str);myFile.println(F(",volume integration timeout (seconds)")); 
    itoa(jamjar_kPa_target, str,10);
    myFile.print("kg:,");myFile.print(str);myFile.println(F(",jamjar target pressure differential (kPa)"));    
    itoa(sams_between_cals, str,10);
    myFile.print("kh:,");myFile.print(str);myFile.println(F(",number of samples to run between cals"));    
    itoa(target_c_vol, str,10);
    myFile.print("ki:,");myFile.print(str);myFile.println(F(",target calibration volume (scc)")); 
    itoa(dual_sampling, str,10);
    myFile.print("kj:,");myFile.print(str);myFile.println(F(",dual sampling mode (0-1)"));     
    myFile.close(); // close the file:
    Serial.println(F("new config file made!"));
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
    lcd_print("config file OK","");
  }
  else
  {
     // if the file didn't open, print an error:
     Serial.println(F("error opening config.txt!"));
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
  Serial.println(F("Reading config.txt..."));
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
      Serial.println(F("error reading config.txt!"));
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
  lcd_print("Setup mode","waiting ...");
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
  unsigned long st_time = 0, en_time = 0;
  
  WireRtcLib::tm* t;
  t = rtc.getTime();
  
  Serial.println(F("cc:,datetime edit mode"));
  lcd_print("datetime edit", "60 sec time-out");
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
      }
    }
    en_time = millis();
    if( (en_time-st_time) > 60000 || (incoming_byte =='q'))
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
    Serial.print(F("ce:, enter year (yy) CURRENTLY: ")); Serial.println(t->year);
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
    t->year = Serial.parseInt();
    Serial.print(F("new year = ")); Serial.println(t->year);
  }
  // MONTH
  if(!time_out)
  {
    st_time = millis();
    Serial.print(F("cf:, enter month (mm) CURRENTLY: ")); Serial.println(t->mon);
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
    t->mon = Serial.parseInt();
    Serial.print(F("new month = ")); Serial.println(t->mon);
  }
  // DAY
  if(!time_out)
  {
    st_time = millis();
    Serial.print(F("cg:, enter day (dd) CURRENTLY: ")); Serial.println(t->mday);
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
    t->mday = Serial.parseInt();
    Serial.print(F("new day = ")); Serial.println(t->mday);
  }  
  // HOUR
  if(!time_out)
  {
    st_time = millis();
    Serial.print(F("ch:, enter hour (hh) CURRENTLY: ")); Serial.println(t->hour);
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
    t->hour = Serial.parseInt();
    Serial.print(F("new hour = ")); Serial.println(t->hour);
  } 
  // MINUTE
  if(!time_out)
  {
    st_time = millis();
    Serial.print(F("ci:, enter minute (mm) CURRENTLY: ")); Serial.println(t->min);
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
    t->min = Serial.parseInt();
    Serial.print(F("new minute = ")); Serial.println(t->min);
    Serial.println(F("new seconds = 00"));
  }   
  // SECONDS
  if(!time_out)
  {
    st_time = millis();
    Serial.print(F("cj:, enter seconds (ss) CURRENTLY: ")); Serial.println(t->sec);
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
    t->sec = Serial.parseInt();
    Serial.print(F("new seconds = ")); Serial.println(t->sec);
    rtc.setTime(t); // send the new date n time
    delay(1000);
    Serial.print(F("Date: "));Serial.print(t->mday);Serial.print(":");Serial.print(t->mon);Serial.print(":20");Serial.print(t->year);
    Serial.print(F("   Time: "));Serial.print(t->hour);Serial.print(":");Serial.print(t->min);Serial.print(":");Serial.println(t->sec);
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
  lcd_print("config edit", "60 sec time-out");
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
    Serial.print(F("bl:, enter inject time, seconds (30-1200) DEFAULT: ")); Serial.println(inject_time);
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
    Serial.print(F("bm:, enter backflush time, seconds (30-1200) DEFAULT: ")); Serial.println(backflush_time);
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
  /*
  PORT MAP:
  port 0:  motorA_ch1 FORWARD:  solenoid valve 1 (sample 1)
  port 1:  motorA_ch1 BACKWARD: solenoid valve 2 (sample 2)
  port 2:  motorA_ch2 FORWARD:  solenoid valve 3 (cal)
  port 3:  motorA_ch2 BACKWARD: solenoid valve 4 (blank)
  port 4:  motorA_ch3 FORWARD:  solenoid valve 5 (inject)
  port 5:  motorA_ch3 BACKWARD: solenoid valve 8 (spare)
  port 6:  motorA_ch4 FORWARD:  solenoid valve 6 (backflush flow)
  port 7:  motorA_ch4 BACKWARD: spare
  port 8:  motorB_ch1 FORWARD:  solenoid valve 7 (main flow)
  port 9:  motorB_ch1 BACKWARD: spare
  port 10: motorB_ch2 FORWARD:  pump
  port 11: motorB_ch2 BACKWARD: spare
  port 12: motorB_ch3 FORWARD:  dynamco
  port 13: motorB_ch3 BACKWARD: spare
  port 14: motorB_ch4 FORWARD:  spare
  port 15: motorB_ch4 BACKWARD: spare
  */
  
  char incoming_byte = 0, time_out = 0;
  unsigned long st_time = 0, en_time = 0;
  byte port_state[17] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  
  Serial.println(F("cl:,manual control mode"));
  //lcd_print("datetime edit", "60 sec time-out");
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
              case '1': // solenoid valve 1, sample 1
                if(port_state[0]) // is valve 1 already on!
                {
                  // don't need to do anything with valve 2 first
                  motor_drive(0, 0, 0); // port 0 OFF non-quiet (solenoid valve 1, sample 1)
                  port_state[0] = 0; // update port 0 state register
                }
                else
                {
                  // make sure valve 2 is off first
                  motor_drive(1, 0, 1); // port 1 OFF quiet (solenoid valve 2, sample 2)
                  port_state[1] = 0; // update port 1 state register
                  // now switch on valve 1
                  motor_drive(0, 1, 0); // port 0 ON non-quiet (solenoid valve 1, sample 1)
                  port_state[0] = 1; // update port 0 state register
                }
                break;
              case '2': // solenoid valve 2, sample 2
                if (port_state[1]) // is valve 2 already on!
                {
                  // don't need to do anything with valve 1 first
                  motor_drive(1, 0, 0); // port 0 OFF non-quiet (solenoid valve 2, sample 2)
                  port_state[1] = 0; // update port 1 state register
                }
                else
                {
                  // make sure valve 1 is off first
                  motor_drive(0, 0, 1); // port 0 OFF quiet (solenoid valve 1, sample 1)
                  port_state[0] = 0; // update port 0 state register
                  // now switch on valve 2
                  motor_drive(1, 1, 0); // port 1 ON non-quiet (solenoid valve 2, sample 2)
                  port_state[1] = 1; // update port 1 state register
                }
                break;    
              case '3': // solenoid valve 3, cal
                if(port_state[2]) // is valve 3 already on!
                {
                  // don't need to do anything with valve 2 first
                  motor_drive(2, 0, 0); // port 2 OFF non-quiet (solenoid valve 3, cal)
                  port_state[2] = 0; // update port 2 state register
                }
                else
                {
                  // make sure valve 4 is off first
                  motor_drive(3, 0, 1); // port 1 OFF quiet (solenoid valve 4, blank)
                  port_state[3] = 0; // update port 3 state register
                  // now switch on valve 3
                  motor_drive(2, 1, 0); // port 2 ON non-quiet (solenoid valve 3, cal)
                  port_state[2] = 1; // update port 2 state register
                }
                break; 
              case '4': // solenoid valve 4, blank
                if(port_state[3]) // is valve 4 already on!
                {
                  // don't need to do anything with valve 3 first
                  motor_drive(3, 0, 0); // port 3 OFF non-quiet (solenoid valve 4, blank)
                  port_state[3] = 0; // update port 3 state register
                }
                else
                {
                  // make sure valve 3 is off first
                  motor_drive(2, 0, 1); // port 3 OFF quiet (solenoid valve 3, cal)
                  port_state[2] = 0; // update port 2 state register
                  // now switch on valve 4
                  motor_drive(3, 1, 0); // port 3 ON non-quiet (solenoid valve 4, blank)
                  port_state[3] = 1; // update port 3 state register
                }
                break;   
              case '5': // solenoid valve 5, inject
                if(port_state[4]) // is valve 5 already on!
                {
                  // don't need to do anything with valve 8 first
                  motor_drive(4, 0, 0); // port 4 OFF non-quiet (solenoid valve 5, inject)
                  port_state[4] = 0; // update port 4 state register
                }
                else
                {
                  // make sure valve 8 is off first
                  motor_drive(5, 0, 1); // port 5 OFF quiet (solenoid valve 8, spare)
                  port_state[5] = 0; // update port 5 state register
                  // now switch on valve 5
                  motor_drive(4, 1, 0); // port 4 ON non-quiet (solenoid valve 5, inject)
                  port_state[4] = 1; // update port 4 state register
                }
                break;   
              case '6': // solenoid valve 6, backflush
                if(port_state[6]) // is valve 6 already on!
                {
                  // don't need to do anything with port 7 first (not used)
                  motor_drive(6, 0, 0); // port 6 OFF non-quiet (solenoid valve 6, backflush)
                  port_state[6] = 0; // update port 6 state register
                }
                else
                {
                  // don't need to do anything with port 7 first (not used)
                  //motor_drive(7, 0, 1); // port 7 OFF quiet (not used)
                  //port_state[7] = 0; // update port 5 state register
                  // now switch on valve 6
                  motor_drive(6, 1, 0); // port 6 ON non-quiet (solenoid valve 6, backflush)
                  port_state[6] = 1; // update port 6 state register
                }
                break;  
              case '7': // solenoid valve 7, main
                if(port_state[8]) // is valve 7 already on!
                {
                  // don't need to do anything with backward port (not used)
                  motor_drive(8, 0, 0); // port 8 OFF non-quiet (solenoid valve 7, main)
                  port_state[8] = 0; // update port 8 state register
                }
                else
                {
                  // don't need to do anything with backward port (not used)
                  //motor_drive(9, 0, 1); // port 9 OFF quiet (not used)
                  //port_state[9] = 0; // update port 9 state register
                  // now switch on valve 7
                  motor_drive(8, 1, 0); // port 8 ON non-quiet (solenoid valve 7, main)
                  port_state[8] = 1; // update port 8 state register
                }
                break;                
              case '8': // solenoid valve 8, spare
                if(port_state[5]) // is valve 8 already on!
                {
                  // don't need to do anything with valve 5 first
                  motor_drive(5, 0, 0); // port 5 OFF non-quiet (solenoid valve 8, spare)
                  port_state[5] = 0; // update port 5 state register
                }
                else
                {
                  // make sure valve 5 is off first
                  motor_drive(4, 0, 1); // port 4 OFF quiet (solenoid valve 5, inject)
                  port_state[4] = 0; // update port 4 state register
                  // now switch on valve 8
                  motor_drive(5, 1, 0); // port 5 ON non-quiet (solenoid valve 8, spare)
                  port_state[5] = 1; // update port 5 state register
                }
                break;  
              case 'p': // solenoid valve 8, spare
                Serial.println(F("pump and flowmeter test ..."));
                jamjar_flowmeter_test(); 
                break;                 
              default:
                Serial.println(F("invalid input, try harder next time ..."));                          
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
  }
  // switch all valves off
  motor_drive(0, 0, 1); // port 0 OFF quiet (solenoid valve 1, sample 1)
  motor_drive(1, 0, 1); // port 1 OFF quiet (solenoid valve 2, sample 2)
  motor_drive(2, 0, 1); // port 3 OFF quiet (solenoid valve 3, cal)
  motor_drive(3, 0, 1); // port 1 OFF quiet (solenoid valve 4, blank)
  motor_drive(4, 0, 1); // port 4 OFF quiet (solenoid valve 5, inject)
  motor_drive(6, 0, 1); // port 6 OFF quiet (solenoid valve 6, backflush)  
  motor_drive(8, 0, 1); // port 8 OFF quiet (solenoid valve 7, main)
  motor_drive(5, 0, 1); // port 5 OFF quiet (solenoid valve 8, spare)
}


void do_chrom_cycle()
{
  char done = 0, rand_no = 0;
  int target_vol = 0;
  unsigned long st_time = 0, en_time = 0; 
  motor_drive(4, 1, 0); // port 4 ON non-quiet (solenoid valve 5, inject)
  Serial.println(F("be:, valco on inject: purging inlet manifold (pneumax5 on)"));
  switch(run_type)
  {
    case 'S':
      motor_drive(0, 1, 0); // port 0 ON non-quiet (solenoid valve 1, sample 1 ON)
      Serial.println(F("bs:, sample1 valve on (pneumax1)"));
      target_vol = target_s_vol;
      break;
    case 'X': // sample 2
      motor_drive(1, 1, 0); // port 1 ON non-quiet (solenoid valve 2, sample 2 ON)
      Serial.println(F("bu:, sample2 valve on (pneumax2)"));
      target_vol = target_s_vol;
      break;
    case 'C':
      motor_drive(2, 1, 0); // port 2 ON non-quiet (solenoid valve 3, cal ON)
      Serial.println(F("bw:, cal valve on (pneumax3)"));
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
      motor_drive(3, 1, 0); // port 3 ON non-quiet (solenoid valve 4, blank ON)
      Serial.println(F("by:, blank valve on (pneumax4)"));
      target_vol = target_c_vol;
      break;      
  }
  Serial.println(F("starting jamjar control..."));
  jamjar_pump_phase();
  Serial.println(F("ax:,main flow off"));
  motor_drive(8, 0, 0); // port 8 OFF non-quiet (solenoid valve 7, main flow OFF)  
  delay(15000); // wait 15 seconds for inlet manifold purging
  trap_temperatureC = read_10bit_analog(1,1); // get trap temperature on pin 1
  Serial.print(F("bc:,")); Serial.println(trap_temperatureC,1); //bc: trap cold temperature
  myFile.print("bc:,"); myFile.println(trap_temperatureC,1); //bc: trap cold temperature
  motor_drive(4, 0, 0); // port 4 OFF non-quiet (solenoid valve 5, load)
  Serial.println(F("bf:, valco on load (pneumax5 off)"));
  start_volume_integration(target_vol);
  st_time = millis();
  do
  {
     en_time = millis(); 
     if(Serial1.available()) // check for when volume integration is ready
     { 
       switch(run_type) // switch off relevant valve first to minimise volume integration error
       {
         case 'S':
           motor_drive(0, 0, 0); // port 0 OFF non-quiet (solenoid valve 1, sample 1 OFF)
           Serial.println(F("bt:, sample1 valve off (pneumax1)"));
           break;
         case 'X': // sample 2
           motor_drive(1, 0, 0); // port 1 OFF non-quiet (solenoid valve 2, sample 2 OFF)
           Serial.println(F("bv:, sample2 valve off (pneumax2)"));
           break;
         case 'C':
           motor_drive(2, 0, 0); // port 2 OFF non-quiet (solenoid valve 3, cal OFF)
           Serial.println(F("bx:, cal valve off (pneumax3)"));
           break;
         case 'B':
           motor_drive(3, 0, 0); // port 3 OFF non-quiet (solenoid valve 4, blank OFF)
           Serial.println(F("bz:, blank valve off (pneumax4)"));
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
  actual_jamjar_P_kPa = read_10bit_analog(2,1); // get jamjar P_diff on pin 2 
  Serial.print(F("bg:,")); Serial.println(actual_jamjar_P_kPa,1); //bg: sample pump pressure differential
  myFile.print("bg:,"); myFile.println(actual_jamjar_P_kPa,1); //bg: sample pump pressure differential  
  led_drive('a',1); // amber LED on
  Serial.println(F("al:,PID on"));
  digitalWrite(ssr_3_pid,HIGH); // PID power on command
  trap_n2_purge();
  Serial.println(F("be:, valco to inject (pneumax5 on)"));  
  motor_drive(4, 1, 0); // port 4 ON non-quiet (solenoid valve 5, inject)
  if(flow_OK)
  {  
    Serial.println(F("ba:,trap heat on..."));
    digitalWrite(ssr_2_trap,HIGH); // trap heater on!
    delay(9000); // 9 second delay for trap heating
  }
  else
  {
    Serial.println(F("Not heating trap, carrier flow too low!"));
  }
  Serial.println(F("aw:,main flow on"));
  motor_drive(8, 1, 0); // port 8 ON non-quiet (solenoid valve 7, main flow ON)
  delay(9000); // 9 second delay for trap heating
  Serial.println(F("bb:,trap heat off"));
  digitalWrite(ssr_2_trap,LOW); // trap heater off
  //Serial.println(F("aw:,main flow on"));
  //motor_drive(8, 1, 0); // port 8 ON non-quiet (solenoid valve 7, main flow ON)
  trap_temperatureC = read_10bit_analog(1,1); // get trap temperature on pin 1
  Serial.print(F("bd:,")); Serial.println(trap_temperatureC,1); //bd: trap hot temperature
  myFile.print("bd:,"); myFile.println(trap_temperatureC,1); //bd: trap hot temperature  
  Serial.println(F("az:,backflush flow off"));
  motor_drive(6, 0, 0); // port 6 OFF non-quiet (solenoid valve 6, backflush flow OFF)   
  //Serial.println("al:,PID power on");
  //digitalWrite(ssr_3_pid,HIGH); // PID power on command
  //delay(2000); // wait for PID power on spike  
  read_ads1115(inject_time);
  //Serial.println("al:,PID power on");
  //digitalWrite(ssr_3_pid,HIGH); // PID power on command 
  Serial.println(F("ay:,backflush flow on"));
  motor_drive(6, 1, 0); // port 6 ON non-quiet (solenoid valve 6, backflush flow ON)  
  Serial.println(F("bf:, valco to load (pneumax5 off"));
  motor_drive(4, 0, 0); // port 4 OFF non-quiet (solenoid valve 5, load)
  read_ads1115(backflush_time);
  Serial.println(F("am:,PID off"));
  digitalWrite(ssr_3_pid,LOW); // PID power off command
  //Serial.println("chrom finished");
  //goto_pause();
}


void trap_n2_purge()
{
  motor_drive(3, 1, 0); // port 3 ON non-quiet (solenoid valve 4, purge ON)
  Serial.println(F("by:, blank valve on, N2 purge (pneumax4)"));
  //delay(5000);
  delay(10000);
  motor_drive(3, 0, 0); // port 3 OFF non-quiet (solenoid valve 4, purge OFF)
  Serial.println(F("bz:, blank valve off (pneumax4)"));
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
       Serial.println(F("Altimeter P timeout :("));
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


float get_altimeter_temperature(char file_write)
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
       Serial.println(F("Altimeter T timeout :("));
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
  if(file_write)
  {
    Serial.print(F("ar:,")); Serial.println(T_data); // ar: altimeter temperature
    myFile.print("ar:,"); myFile.println(T_data); // ar: altimeter temperature
  }
  float PID_temp = T_data.toFloat();
  //Serial.print(F("PID temperature C = ")); Serial.println(PID_temp, 1); // print altimeter temperature
  //goto_pause();
  return PID_temp;
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


void start_volume_integration(int t_vol)
{
  unsigned long st_time = 0, en_time = 0;
  char inByte, timeout = 0;
  Serial.println(F("Volume integration requested..."));
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
  Serial.println(F("Integrating flow..."));
}


void read_volume_integration()
{
  unsigned long st_time = 0, en_time = 0;
  char timeout = 0;  
  String V_data = "", F_data = "", T_data = "";
  char inByte, mybuffer[32], pos = 0;
  
  //for( char a = 0; a < sizeof(mybuffer); a++ )
  for( char a = 0; a < 32; a++ )
  {
    mybuffer[a] = '.';
  }
  mybuffer[31] = '\0';
  st_time = millis();
  while(!Serial1.available() && (!timeout) ) // wait for data to come back
  { 
     en_time = millis();
     if((en_time-st_time) > (5*1000))
     {
       timeout = 1;
       Serial.println(F("Flowmeter timeout :("));
       //P_data = "999";
     }
  }
  while( (Serial1.available()) && (pos < 30) ) // read bytes from serial1 as log as buffer pos is < 30
  {
    inByte = Serial1.read();
    mybuffer[pos++] = inByte;
    delay(10);
  }
  Serial.print(F("input buffer = "));Serial.println(mybuffer);
  Serial.println(pos,DEC);
  // now data mine the input buffer for volume, flow rate and integration time
  for(char ind = 0; ind < pos; ind++)
  {
    switch(mybuffer[ind])
    {
      case 'V':
        ind++;
        while(mybuffer[ind] != 'F')
        {
          V_data.concat(mybuffer[ind++]);
        }
        ind--;
        Serial.println(V_data);
        break;
      case 'F':
        ind++;
        while(mybuffer[ind] != 'T')
        {
          F_data.concat(mybuffer[ind++]);
        }
        ind--;
        Serial.println(F_data);
        break; 
     case 'T':
        ind++;
        while(mybuffer[ind] != 'E')
        {
          T_data.concat(mybuffer[ind++]);
        }
        ind--;
        Serial.println(T_data);
        break;        
    }
  }
  Serial.print(F("as:,")); Serial.println(V_data); // as: integrated volume
  myFile.print("as:,"); myFile.println(V_data);    // as: integrated volume
  Serial.print(F("at:,")); Serial.println(F_data); // at: average flowrate
  myFile.print("at:,"); myFile.println(F_data);    // at: average flowrate
  Serial.print(F("au:,")); Serial.println(T_data); // au: integration time
  myFile.print("au:,"); myFile.println(T_data);    // au: integration time 
}


void jamjar_pump_phase()
{
  byte check_done = 0, loop_limit = 0;
  actual_jamjar_P_kPa = read_10bit_analog(2,0); // get jamjar P_diff on pin 2
  //Serial.print("Pump on phase ... "); Serial.print("differential pressure (kPa) = "); Serial.println(actual_jamjar_P_kPa);   
  while( (actual_jamjar_P_kPa < jamjar_kPa_target) && (!check_done) && (loop_limit < 8) )
  {
    Serial.print(F("Pump on phase... diff' P (kPa) = ")); Serial.println(actual_jamjar_P_kPa);
    do_pump_cycle(0); // non-quiet mode
    if(Serial1.available()) // check for when volume integration is ready
    {
       check_done = 1;
    }
    actual_jamjar_P_kPa = read_10bit_analog(2,0); // get jamjar P_diff on pin 2
    Serial.print(F("loop_limit "));Serial.println(loop_limit);
    loop_limit++;
  }
}


void jamjar_rest_phase()
{
  unsigned long pump_st_time = 0, pump_en_time = 0;
  char timeout = 0;
  char check_done = 0; 
  actual_jamjar_P_kPa = read_10bit_analog(2,0); // get jamjar P_diff on pin 2
  while( (actual_jamjar_P_kPa > jamjar_kPa_target) && (!check_done) )
  {
    Serial.print(F("Pump off phase... diff' P (kPa) = ")); Serial.println(actual_jamjar_P_kPa);
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
    actual_jamjar_P_kPa = read_10bit_analog(2,0); // get jamjar P_diff on pin 2
  }
}


void jamjar_flowmeter_test()
{
  byte check_done = 0, loop_limit = 0;
  
  Serial.print(F("jamjar_kPa_target = "));Serial.println(jamjar_kPa_target);
  actual_jamjar_P_kPa = read_10bit_analog(2,0); // get jamjar P_diff on pin 2
  //Serial.print("Pump on phase ... "); Serial.print("differential pressure (kPa) = "); Serial.println(actual_jamjar_P_kPa);   
  //while( (actual_jamjar_P_kPa < jamjar_kPa_target) && (!check_done) && (loop_limit < 8) )
  while( (!check_done) && (loop_limit < 8) )
  {
    //Serial.print(F("Pump on phase... diff' P (kPa) = ")); Serial.println(actual_jamjar_P_kPa);
    if( actual_jamjar_P_kPa < jamjar_kPa_target )
    {
      Serial.print(F("Pump on phase... diff' P (kPa) = ")); Serial.println(actual_jamjar_P_kPa);
      do_pump_cycle(1); // quiet mode
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
    actual_jamjar_P_kPa = read_10bit_analog(2,0); // get jamjar P_diff on pin 2
    //Serial.print(F("loop_limit "));Serial.println(loop_limit);
    loop_limit++;
  }
  Serial.println(F("cn:,pump flowmeter test done!"));
}


void do_pump_cycle(char quiet) // quiet = 1 reduced printing
{
  unsigned long pump_st_time = 0, pump_en_time = 0;
  char timeout = 0;
  
  if(!quiet)
  {   
    motor_drive(10, 1, 0); // port 10 ON non-quiet (pump ON) 
  }
  else
  {
    motor_drive(10, 1, 1); // port 10 ON quiet (pump ON) 
  }
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
  if(!quiet)
  {
    motor_drive(12, 1, 0); // port 12 ON non-quiet (dynamco ON)
  }
  else
  {
    motor_drive(12, 1, 1); // port 12 ON quiet (dynamco ON)
  }
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
  if(!quiet)
  {
    motor_drive(12, 0, 0); // port 12 OFF non-quiet (dynamco OFF)
  }
  else
  {
    motor_drive(12, 0, 1); // port 12 OFF quiet (dynamco OFF)
  }
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
  if(!quiet)
  {
    motor_drive(10, 0, 0); // port 10 OFF non-quiet (pump OFF) 
  }
  else
  {
    motor_drive(10, 0, 1); // port 10 OFF quiet (pump OFF) 
  }
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


void motor_drive(char port_no, char on_off, char quiet)
{
  // call with quiet=1 for no status message or quiet=0 otherwise
  int hit_delay = 100;
  uint8_t top_speed = 255;
  String invalid_para = "Invalid motor parameter!";
  
  if(port_no == 0)
  {
    switch(on_off)
    {
      case 0:
        motorA_ch1->run(RELEASE);
        if(!quiet)
        {
          Serial.println(F("Solenoid 1: Sample 1 OFF!"));
        }
        break;
      case 1:
        motorA_ch1->run(RELEASE);
        motorA_ch1->run(FORWARD);
        motorA_ch1->setSpeed(top_speed);
        delay(hit_delay);
        motorA_ch1->setSpeed(top_speed/3); 
        if(!quiet)
        {
          Serial.println(F("Solenoid 1: Sample 1 ON!"));
        }
        break;
      default:
        Serial.println(invalid_para);          
    }
  }
  if(port_no == 1)
  {
    switch(on_off)
    {
      case 0:
        motorA_ch1->run(RELEASE);
        if(!quiet)
        {        
          Serial.println(F("Solenoid 2: Sample 2 OFF!"));
        }
        break;
      case 1:
        motorA_ch1->run(RELEASE);
        motorA_ch1->run(BACKWARD);
        motorA_ch1->setSpeed(top_speed);
        delay(hit_delay);
        motorA_ch1->setSpeed(top_speed/3);
        if(!quiet)
        { 
          Serial.println(F("Solenoid 2: Sample 2 ON!"));
        }
        break;
      default:
        Serial.println(invalid_para); 
    }       
  }
  if(port_no == 2)
  {
    switch(on_off)
    {
      case 0:
        motorA_ch2->run(RELEASE);
        if(!quiet)
        {
          Serial.println(F("Solenoid 3: Cal OFF!"));
        }
        break;
      case 1:
        motorA_ch2->run(RELEASE);
        motorA_ch2->run(FORWARD);
        motorA_ch2->setSpeed(top_speed);
        delay(hit_delay);
        motorA_ch2->setSpeed(top_speed/3);
        if(!quiet)
        { 
          Serial.println(F("Solenoid 3: Cal ON!"));
        }
        break;
      default:
        Serial.println(invalid_para);   
    }       
  }      
  if(port_no == 3)
  {  
    switch(on_off)
    {
      case 0:
        motorA_ch2->run(RELEASE);
        if(!quiet)
        {
          Serial.println(F("Solenoid 4: Blank OFF!"));
        }
        break;
      case 1:
        motorA_ch2->run(RELEASE);
        motorA_ch2->run(BACKWARD);
        motorA_ch2->setSpeed(top_speed);
        delay(hit_delay);
        motorA_ch2->setSpeed(top_speed/3);
        if(!quiet)
        { 
          Serial.println(F("Solenoid 4: Blank ON!"));
        }
        break;
      default:
        Serial.println(invalid_para);
    }
  }    
  if(port_no == 4)
  {  
    switch(on_off)
    {
      case 0:
        motorA_ch3->run(RELEASE);
        if(!quiet)
        {
          Serial.println(F("Solenoid 5: Inject OFF!"));
        }
        break;
      case 1:
        motorA_ch3->run(RELEASE);
        motorA_ch3->run(FORWARD);
        motorA_ch3->setSpeed(top_speed);
        delay(hit_delay);
        motorA_ch3->setSpeed(top_speed/3);
        if(!quiet)
        { 
          Serial.println(F("Solenoid 5: Inject ON!"));
        }
        break;
      default:
        Serial.println(invalid_para);
    }
  }   
  if(port_no == 5)
  {  
    switch(on_off)
    {
      case 0:
        motorA_ch3->run(RELEASE);
        if(!quiet)
        {
          Serial.println(F("Solenoid 8: Spare OFF!"));
        }
        break;
      case 1:
        motorA_ch3->run(RELEASE);
        motorA_ch3->run(BACKWARD);
        motorA_ch3->setSpeed(top_speed);
        delay(hit_delay);
        motorA_ch3->setSpeed(top_speed/3);
        if(!quiet)
        { 
          Serial.println(F("Solenoid 8: Spare ON!"));
        }
        break;
      default:
        Serial.println(invalid_para);
    }
  }   
  if(port_no == 6)
  {  
    switch(on_off)
    {
      case 0:
        motorA_ch4->run(RELEASE);
        if(!quiet)
        {
          Serial.println(F("Solenoid 6: Backflush OFF!"));
        }
        break;
      case 1:
        motorA_ch4->run(RELEASE);
        motorA_ch4->run(FORWARD);
        motorA_ch4->setSpeed(top_speed);
        delay(hit_delay);
        motorA_ch4->setSpeed(top_speed/3); 
        if(!quiet)
        {
          Serial.println(F("Solenoid 6: Backflush ON!"));
        }
        break;
      default:
        Serial.println(invalid_para);
    }
  }     
  if(port_no == 8)
  {  
    switch(on_off)
    {
      case 0:
        motorB_ch1->run(RELEASE);
        if(!quiet)
        {
          Serial.println(F("Solenoid 7: Main flow OFF!"));
        }
        break;
      case 1:
        motorB_ch1->run(RELEASE);
        motorB_ch1->run(FORWARD);
        motorB_ch1->setSpeed(top_speed);
        delay(hit_delay);
        motorB_ch1->setSpeed(top_speed/3); 
        if(!quiet)
        {
          Serial.println(F("Solenoid 7: Main flow ON!"));
        }
        break;
      default:
        Serial.println(invalid_para);
    }
  }      
  if(port_no == 10)
  {  
    switch(on_off)
    {
      case 0:
        motorB_ch2->run(RELEASE);
        if(!quiet)
        {
          Serial.println(F("Pump OFF!"));
        }
        break;
      case 1:
        motorB_ch2->run(RELEASE);
        motorB_ch2->run(FORWARD);
        motorB_ch2->setSpeed(top_speed);
        if(!quiet)
        {
          Serial.println(F("Pump ON!"));
        }
        break;
      default:
        Serial.println(invalid_para);
    }
  }      
  if(port_no == 12)
  {  
    switch(on_off)
    {
      case 0:
        motorB_ch3->run(RELEASE);
        if(!quiet)
        {
          Serial.println(F("Dynamco valve OFF!"));
        }
        break;
      case 1:
        motorB_ch3->run(RELEASE);
        motorB_ch3->run(FORWARD);
        motorB_ch3->setSpeed(top_speed);
        delay(hit_delay);
        motorB_ch1->setSpeed(top_speed/3); 
        if(!quiet)
        {
          Serial.println(F("Dynamco valve ON!"));
        }
        break;
      default:
        Serial.println(invalid_para);
    }
  }        
}


void backlight_off()
{
  Serial2.write(124); // put LCD into instruction mode
  delay(5);
  //Serial2.write(128); // backlight off
  //Serial2.write(129); // backlight slightly on (~5 mA)
  Serial2.write(130); // backlight slightly on (~10 mA)
  //Serial2.write(157); // backlight fully on (~165 mA)
  delay(5);
}


void lcd_print(String line_1, String line_2)
{
  Serial2.write(254); // put LCD into instruction mode
  delay(5);
  Serial2.write(1); // clear screen
  delay(5);
  Serial2.write(254); // put LCD into instruction mode
  delay(5);
  Serial2.write(128); // goto 1st char on 1st line
  delay(5);
  Serial2.print(line_1);
  delay(55);
  Serial2.write(254); // put LCD into instruction mode
  delay(5);
  Serial2.write(192); // goto 1st char on 2nd line
  delay(5);
  Serial2.write(254); // put LCD into instruction mode
  delay(5);
  Serial2.write(12); // turn off cursor
  delay(5);
  Serial2.print(line_2); 
}


void led_drive(char colour, char on_off)
{
  if(colour == 'r')
  {
    switch(on_off)
    {
      case 0:
        digitalWrite(46,LOW); // red LED off
        //Serial.println(F("Red LED OFF!"));
        break;
      case 1:
        digitalWrite(46,HIGH); // red LED on
        //Serial.println(F("Red LED ON!"));
        break;
      default:
        Serial.println(F("Invalid LED parameter!"));
    }
  }
  if(colour == 'g')
  {
    switch(on_off)
    {
      case 0:
        digitalWrite(45,LOW); // green LED off
        //Serial.println(F("Green LED OFF!"));
        break;
      case 1:
        digitalWrite(45,HIGH); // green LED on
        //Serial.println(F("Green LED ON!"));
        break;
      default:
        Serial.println(F("Invalid LED parameter!"));
    }
  }
  if(colour == 'a')
  {
    switch(on_off)
    {
      case 0:
        digitalWrite(46,LOW); // red LED off
        //Serial.println("Red LED OFF!");
        digitalWrite(45,LOW); // green LED off
        //Serial.println("Green LED OFF!");
        //Serial.println(F("Amber LED OFF!"));
        break;
      case 1:
        digitalWrite(46,HIGH); // red LED on
        //Serial.println("Red LED ON!");
        digitalWrite(45,HIGH); // green LED on
        //Serial.println("Green LED ON!");
        //Serial.println(F("Amber LED ON!"));
        break;
      default:
        Serial.println(F("Invalid LED parameter!"));
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
