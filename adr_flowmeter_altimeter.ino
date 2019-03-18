/*
  adr_flowmeter_altimeter
  revision 20171211
  adding sync error trapping
  
  uses software serial on pins 10 (RX) and 11 (TX) for flowmeter comms
  uses software serial on pins 8 (RX) and 9 (TX) for mega comms
  uses SDA (A4) and SCL (A5) for altimeter i2c comms
  20171211 added line 201 serial flush the flowmeter for every reading
*/

#include <SoftwareSerial.h>
#include <Wire.h>
//#include "IntersemaBaro.h"
#include <BaroSensor.h>

//Intersema::BaroPressure_MS5607B baro(true);

SoftwareSerial megaSerial(8, 9);          // RX, TX
SoftwareSerial flowmeterSerial(10, 11);   // RX, TX
unsigned long time_now = 0, last_time = 0;
float flow = 0.0, vol_sum = 0.0, flow_sum = 0.0;
byte myData[4] = {0,0,0,0};               // array to store the 4 flowmeter packet bytes
byte sync_err_count = 0;                  // counter to store number of sync errors
int target_vol = 2;                       // default target volume (scc)
int vol_timeout = 60;                     // default volume integration time out (seconds)

void setup()
{ 
  delay(10000); //wait for serial port
  Serial.begin(9600);
  Serial.println(F("Starting comms ..."));
  Serial.println(F("version date: 20171211"));
  flowmeterSerial.begin(19200);  // set data rate for flowmeterSerial port
  serial_flush('2', 0); // flush flowmeterSerial port (1= quiet mode)
  megaSerial.begin(9600);  // set data rate for megaSerial port
  serial_flush('1', 0); // flush megaSerial port (1= quiet mode)
  BaroSensor.begin();
  //BaroSensor.init();
  //flowmeterSerial.listen();
  //while(flowmeterSerial.available()) // clear incoming buffer
  //{
    //flowmeterSerial.read();
    //Serial.print('c');
  //}
  //while (Serial.read() >= 0); // clear buffer
  //flowmeterSerial.println("help");
  //flowmeterSerial.write(45);
  //flowmeterSerial.write(10);
  //flowmeterSerial.write('s');
  //flowmeterSerial.println("go\r");
  //flowmeterSerial.print('s'); // works!
  //flowmeterSerial.write(10); //works!
  //flowmeterSerial.println("help"); 
  //flowmeterSerial.println("get"); 
  //flowmeterSerial.println("ver");
  //flowmeterSerial.println("go"); 
}

void loop()
{
  //pressure_reading();
  //Serial.println("Paused ...");
  //while(1){};
  //String target_vol = "";
  delay(100);
  megaSerial.listen();  // listen to megaSerial port
  //while(megaSerial.available() > 0)
  if(megaSerial.available())
  {
    char inByte = megaSerial.read();
    switch(inByte)
    {
      case 'p':  // altimeter pressure request
        Serial.println(inByte);
        pressure_reading();
        break;
      case 't':  // altimeter temperature request
        Serial.println(inByte);
        temperature_reading();
        break;
      case 'f':  // single flowmeter reading  
        Serial.println(inByte);
        single_flow_reading();
        break;
      case 'v':  // volume integration request
        Serial.println(inByte);
        megaSerial.write('v'); // send confirmation
        delay(100);
        while(megaSerial.available())
        {
          target_vol =  megaSerial.parseInt();
        }
        Serial.print(F("Target vol = ")); Serial.println(target_vol);
        break; 
      case 'w':  // volume timeout request
        Serial.println(inByte);
        megaSerial.write('w'); // send confirmation
        delay(100);
        while(megaSerial.available())
        {
          vol_timeout =  megaSerial.parseInt();
        }
        Serial.print(F("Volume integration timeout = ")); Serial.println(vol_timeout);
        //while(1){};
        flowmeter_loop();
        break;        
    }
  }
  /*pressure_reading();
  delay(5000);
  temperature_reading();
  delay(5000);
  //flowmeter_loop();
  Serial.println("Done!");
  while(1){};*/
}

void pressure_reading()
{
    //int32_t p_Pa = BaroSensor.getPressure_Pa();
    /*int32_t p_Pa = BaroSensor.getPressure();
    float mbar = 0.0;
    mbar = (float) p_Pa / 100; //convert to mbar*/
    float mbar = BaroSensor.getPressure(); //changed this on 20171212 to reflect new library
    Serial.print(F("Pressure, mbar: "));  Serial.println(mbar, 2);
    megaSerial.print(mbar, 2);
}

void temperature_reading()
{
    //int32_t t_Cx100 = baro.getTemp_Cx100();
    /*int32_t t_Cx100 = BaroSensor.getTemperature();
    float t_degrees = 0.0;
    t_degrees = (float) t_Cx100 / 100; //convert to degrees C*/
    float t_degrees = BaroSensor.getTemperature(); //changed this on 20171212 to reflect new library
    Serial.print(F("Temperature, C: ")); Serial.println(t_degrees, 2); Serial.println(""); //changed println(t_degrees, 2) to println(t_degrees)
    megaSerial.print(t_degrees, 2);
}


void single_flow_reading(void)
{
  unsigned int my_count1 = 0, my_count2 = 0;
  
  serial_flush('2', 0); // flush flowmeterSerial port (1= quiet mode)
  flow = 0.0;
  flowmeterSerial.listen();
  my_count2 = 0;
  while(my_count2 < 4)
  {
    while(!flowmeterSerial.available()){}; // wait until data reaches input buffer
    if(flowmeterSerial.available()) // read a byte from buffer
    {
      myData[my_count2] = flowmeterSerial.read();
      my_count2++;
      //Print_millis();
    }
  }
  //Print_millis();
  //Print_packet();
  Do_flow_display();
}


void Do_flow_display()
{
    if((myData[0] == 0X7f) && (myData[1] == 0X7f) && (myData[2] != 0X7f))
    {
        Serial.println(F("correct sync bytes found"));
        flow = (float)((myData[2]*256)+(myData[3]))/70;
        Serial.print(F("flow (sccm): "));
        Serial.println(flow,2);
        megaSerial.print(flow, 2);
    }
    else
    {
        Serial.println(F("sync error!"));
     }
    Serial.println("");
}

void flowmeter_loop() 
{
  unsigned int my_count1 = 0, my_count2 = 0;
  unsigned long loop_limit = (unsigned long) vol_timeout * 1000 / 640; // 938 ~ 10 minutes
  unsigned long start_time = 0, end_time = 0, integration_time = 0;

  sync_err_count = 0;   // reset sync error counter for each run
  serial_flush('2', 0); // flush flowmeterSerial port (1= quiet mode)
  Serial.print(F("loop limit = ")); Serial.println(loop_limit);
  flow = 0.0; vol_sum = 0.0; flow_sum = 0.0; time_now = 0; last_time = 0;
  flowmeterSerial.listen();
  start_time = millis(); 
  Print_millis();
  Serial.println("");
  //while(my_count1 < loop_limit)
  while((vol_sum < target_vol) && (my_count1 < loop_limit))
  {
    Serial.print(F("count no: "));
    Serial.println(my_count1);
    Serial.println("this is before the flowmeter serial is read: ");
    serial_flush('2', 0); // flush flowmeterSerial port (1= quiet mode) // Added this 20171212
    my_count2 = 0;
    while(my_count2 < 4)
    {
      while(!flowmeterSerial.available()){}; // wait until data reaches input buffer
      if(flowmeterSerial.available()) // read a byte from buffer
      {
        //putting a delay 100 here causes a sync error on every cycle
        myData[my_count2] = flowmeterSerial.read();
        my_count2++;
        //Print_millis();
      }
    }
    Print_millis();
    Print_packet();
    Do_flow_calc();
    //if (flowmeterSerial.overflow())
    //{
    //Serial.println("SoftwareSerial overflow!");
    //}
    my_count1 ++;
    // clear data array
    for(my_count2 = 0; my_count2 < 4; my_count2++)
    {
      myData[my_count2] = 0;
    }
  }
  end_time = millis();
  integration_time = (end_time - start_time) / 1000;
  if( my_count1 >= loop_limit)
  {
    Serial.println(F("integration timed out!"));
  }
  flow_sum = (float) flow_sum / my_count1;
  Serial.print(F("total integrated volume (scc): ")); Serial.println(vol_sum, 2);
  megaSerial.print('V');
  //delay(100); 
  megaSerial.print(vol_sum, 2);
  //delay(1500); //wait for mega serial port to timeout
  Serial.print(F("average flow (sccm): ")); Serial.println(flow_sum, 2);
  megaSerial.print('F');
  //delay(100); 
  megaSerial.print(flow_sum, 2);
  //delay(1500); //wait for mega serial port to timeout
  Serial.print(F("integration time (s): ")); Serial.println(integration_time);
  megaSerial.print('T');
  //delay(100); //wait for mega serial port to timeout
  megaSerial.print(integration_time);

  Serial.print(F("sync errors: ")); Serial.println(sync_err_count, DEC);
  megaSerial.print('S');
  //delay(100); //wait for mega serial port to timeout
  megaSerial.print(sync_err_count, DEC);
  
  megaSerial.print('E'); // 'end' flag for the mega to pick up on
  Serial.println(F("stopped!"));
  //while(1){};
}


void Print_millis()
{
  last_time = last_time + time_now;
  //Serial.print("Last time: "); Serial.println(last_time);
  Serial.print(F("Time elapsed: "));
  time_now = millis();
  time_now = time_now - last_time;
  //prints time since program started
  Serial.println(time_now);
}


void Print_packet()
{
    for(char pack_count = 0; pack_count < 4; pack_count++)
    {
      if (myData[pack_count] < 16) {Serial.print("0");}
      Serial.print(myData[pack_count],HEX);
      Serial.print(' ');
      //Serial.println("");
    }
    Serial.println("");
}


void Do_flow_calc()
{
    if((myData[0] == 0X7f) && (myData[1] == 0X7f) && (myData[2] != 0X7f))
    {
        Serial.println(F("correct sync bytes found"));
        flow = (float)((myData[2]*256)+(myData[3]))/70;
        Serial.print(F("flow (sccm): "));
        Serial.println(flow,2);
        if(flow < 0.02) // count low or negative flow as zero flow
        {
          flow = 0.0;
        }
        flow_sum = (float) flow_sum + flow;
        //Serial.println(time);
        vol_sum += (float) flow * time_now/60000;
        Serial.print(F("running volume (scc): "));
        Serial.println(vol_sum, 2);
        serial_flush('2', 0); // flush flowmeterSerial port (1= quiet mode) // Added this 20171212
    }
    else
    {
        serial_flush('2', 0); // flush flowmeterSerial port (1= quiet mode)
        if(sync_err_count < 255)
        {
          sync_err_count++;
        }
        Serial.print(F("sync error! "));Serial.println(sync_err_count, DEC);
        //flow = 0;
        //Serial.print("flow default zero (sccm): ");
        //Serial.println(flow,2);
        serial_flush('2', 0); // flush flowmeterSerial port (1= quiet mode) // Added this 20171212
     }
    Serial.println("");
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
    case '1': // megaSerial on pins 8 & 9
      while(megaSerial.available() >0)
      {
        flush_c = megaSerial.read();
      }
      if(!quiet)
      {
        Serial.println(F("Flushed megaSerial port"));
      }
      break;       
    case '2': // flowmeterSerial on pins 10 & 11
      while(flowmeterSerial.available() >0)
      {
        flush_c = flowmeterSerial.read();
      }
      if(!quiet)
      {
        Serial.println(F("Flushed flowmeterSerial port"));
      }
      break;   
  }
}
