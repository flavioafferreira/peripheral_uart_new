#include <stdint.h>
#include <string.h>
#include <stdio.h>


#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>


#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/timing/timing.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>

//Special Routines
#include "includes/accessories/special.h"

//Circular Buffer Structure
#include "includes/accessories/variables.h"

//Encoder & Decoder Protobuf

#include "includes/Protobuf/pb.h"
#include "includes/Protobuf/pb_common.h"
#include "includes/Protobuf/pb_decode.h"
#include "includes/Protobuf/pb_encode.h"

#include "includes/Protobuf/data_def_v0_pb.h" 

#include <time.h>
#include <date_time.h>
#include <zephyr/fs/nvs.h>

//Lorawan
#include <zephyr/lorawan/lorawan.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/random/rand32.h>
/*
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/services/nus.h>

extern struct bt_conn *current_conn;
extern struct bt_conn *auth_conn;
extern const struct bt_data ad[];
extern const struct bt_data sd[];
*/

//Circular Buffer
uint32_t C_Buffer_Free_Position=0;
uint32_t C_Buffer_Current_Position=0;
uint32_t C_Buffer_Alarm_Free_Position=0;
uint32_t C_Buffer_Alarm_Current_Position=0;

_Circular_Buffer C_Buffer[CIRCULAR_BUFFER_ELEMENTS];
_Alarm C_Buffer_Alarm[ALARM_EVENT_QTY_MAX];

#define ARRAY_TEST 12

#include <zephyr/drivers/adc.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

extern int16_t adc_value[8]; //0=channel 1
extern int16_t digital_value[8];//value for digital inputs
extern struct k_mutex ad_ready;
extern const struct adc_dt_spec adc_channels[];

extern Gnss position;
  
//Probe Digital
extern uint8_t dig_probe;

//NVS
extern struct flash_pages_info info;
extern struct nvs_fs fs;
extern uint32_t button2_counter;
void save_memory(uint32_t Pos);
_Circular_Buffer read_memory(uint32_t Pos);

//SEMAPHORE
extern uint8_t lorawan_reconnect;
//extern uint32_t lorawan_reconnect_cnt;
//extern struct k_sem lorawan_tx;
extern uint32_t data_sent_cnt;
//MUTEX
K_MUTEX_DEFINE(c_buffer_busy);

//SETUP
_Setup Initial_Setup;


//LEDS
extern struct gpio_dt_spec pin_test_led0;
extern struct gpio_dt_spec pin_test_led1;
extern struct gpio_dt_spec pin_test_led2;
extern struct gpio_dt_spec pin_test_led3;


//
extern Sensor_Status_ sensor_status;

//Alarm ON/Off




void flash_button2_counter(void){
	int rc = 0;
    button2_counter++;
	(void)nvs_write(
	&fs, BOOT_POSITION, &button2_counter,
	sizeof(button2_counter));
    rc = nvs_read(&fs, BOOT_POSITION, &button2_counter, sizeof(button2_counter));
	if (rc > 0) { /* item was found, show it */
		printk("Id: %d, button2_counter: %d\n",
			BOOT_POSITION, button2_counter);

	}	
}

void flash_write_setup(void){
    uint8_t err=0;
    uint16_t size_setup=sizeof(_Setup);
    err=nvs_write(&fs, SETUP_POSITION, &Initial_Setup,size_setup);
}

void flash_read_setup(void){
	(void)nvs_read(&fs, SETUP_POSITION, &Initial_Setup, sizeof(Initial_Setup));
}

void fill_date(uint8_t *field_time,uint8_t *field_date ){

	         uint8_t part[2];
				   //day
				   part[0]=field_date[0];
           part[1]=field_date[1];
           position.t.tm_mday=atoi(part);
				   //month
				   part[0]=field_date[2];
           part[1]=field_date[3];
				   position.t.tm_mon=(atoi(part)-1); 	// Month, where 0 = jan
				   //year
				   part[0]=field_date[4];
           part[1]=field_date[5];
				   position.t.tm_year=atoi(part);	
                  
				   //hour
				   part[0]=field_time[0];
           part[1]=field_time[1];
				   position.t.tm_hour=atoi(part);	
				   //min
				   part[0]=field_time[2];
           part[1]=field_time[3];
				   position.t.tm_min=atoi(part);	
				   //sec
				   part[0]=field_time[4];
           part[1]=field_time[5];
				   position.t.tm_sec=atoi(part);	
           //latitude_time_print();				   				   
           
}

void latitude_time_print(void){
    printf("TimeStamp  :%02d/%02d/%02d %02d:%02d:%02d \r\n",
				  position.t.tm_mday,
					(position.t.tm_mon+1),
					position.t.tm_year,
					position.t.tm_hour,
					position.t.tm_min,
					position.t.tm_sec );

    printf("Latitude  N:%s\r\n",position.latitude);
    printf("Longitude E:%s\r\n",position.longitude);

}

uint64_t time_stamp_function(void) {
    struct tm t;
    time_t t_of_day;
  
    t.tm_year = (2000+position.t.tm_year)-1900;  // Year - 1900
    t.tm_mon = position.t.tm_mon;         // Month, where 0 = jan
    t.tm_mday = position.t.tm_mday;       // Day of the month
    t.tm_hour = position.t.tm_hour;
    t.tm_min = position.t.tm_min;
    t.tm_sec = position.t.tm_sec;
    t.tm_isdst = -1;        // Is DST on? 1 = yes, 0 = no, -1 = unknown
    t_of_day = mktime(&t);
    //printf("TimeStamp UTC  :%02d/%02d/%02d %02d:%02d:%02d \r\n",
	  //			  position.t.tm_mday,position.t.tm_mon,position.t.tm_year,
    //        position.t.tm_hour,position.t.tm_min,position.t.tm_sec );
    //printf("seconds since the Epoch: %ld\n", (long) t_of_day);

    if (position.gps_fixed !=1 ){t_of_day=k_uptime_get()/1000;}

    return t_of_day;
}

void time_print (void){
 
    struct tm  *ts;
    char       buf[80];
        
    // Get current time
    ts = gmtime(time_stamp_function());

   

    // Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
    
    strftime(buf, sizeof(buf), "%a %Y-%m-%d %H:%M:%S %Z", &ts);
    printk("%s\n", buf);

}

Gnss values_of_gnss_module(void){
  Gnss gnss_return_value;
  gnss_return_value.latitude=position.latitude;
  gnss_return_value.longitude=position.longitude;
  gnss_return_value.timestamp=time_stamp_function();
  //implement here module routine to read value
  return gnss_return_value;
}

Analog values_of_analog_sensor(uint8_t channel){
  Analog analog_return_value;
  analog_return_value.timestamp=time_stamp_function();
  analog_return_value.value=adc_value[channel];
  return analog_return_value;
}

Ntc values_of_ntc_sensor(uint8_t sensor_number){
  Ntc ntc_return;
  ntc_return.timestamp=time_stamp_function();

  switch (sensor_number){
    case NTC_1:ntc_return.value=adc_value[NTC_1_CH];
              break;
    case NTC_2:ntc_return.value=adc_value[NTC_2_CH];
              break;
    case NTC_3:ntc_return.value=adc_value[NTC_3_CH];
              break;
  }
  return ntc_return;
}

Digital values_of_digital_sensor(uint8_t sensor_number){
  Digital digital_return_value;
  digital_return_value.timestamp=time_stamp_function();
  digital_return_value.value=digital_value[sensor_number];
  digital_value[sensor_number]=0; //resets the counter
  return digital_return_value;
}

// PRINT EVERY MINUTE THE VALUES SAVED OF SENSORS
// ACCESS THE CURRENT POSITION OF CIRCULAR BUFFER
void print_current_position_cb(uint32_t pos){
   int32_t val_mv;
   char buf_lati[14];
   char buf_long[14];


   uint8_t i=0;
    
    k_mutex_lock(&c_buffer_busy,K_FOREVER);
    color(13);
    printf("\n\n####Position %d #####\n",pos);

    if (position.gps_fixed==1) {color(2);printf("GPS Fixed  :Yes\n");}
      else {color(2); printf("GPS Fixed  :No\n");}
    color(13);
    
    sprintf(buf_lati, "%f", C_Buffer[pos].gnss_module.latitude);
    sprintf(buf_long, "%f", C_Buffer[pos].gnss_module.longitude);

    printf("https://www.google.com/maps/place/%c%c ",buf_lati[0],buf_lati[1]);
    i=2;
    while (buf_lati[i]){buf_lati[i-2]=buf_lati[i];i++;}
    buf_lati[i]=0x00;
    printf("%s,",buf_lati);
    

    printf("%c%c ",buf_long[0],buf_long[1]);    
    i=2;
    while (buf_long[i]){buf_long[i-2]=buf_long[i];i++;}
    buf_long[i]=0x00;
    printf("%s \n",buf_long);
    

    printf("GNSS Position Lat=%f Long=%f UTC Epoch Unix Timestamp=%d \n",
      C_Buffer[pos].gnss_module.latitude,
      C_Buffer[pos].gnss_module.longitude,
      C_Buffer[pos].gnss_module.timestamp);


    val_mv = C_Buffer[pos].analog.value;
    adc_raw_to_millivolts_dt(&adc_channels[ANALOG_SENSOR],&val_mv);
 
    printf("Analog  Value=%d  %"PRId32"mV \n",
      C_Buffer[pos].analog.value,
      val_mv);
    
    i=0;
    while (i<3){
      printf("NTC %d Value=%d %3.1f C\n",
      i,      
      C_Buffer[pos].ntc[i].value,
      ntc_temperature(C_Buffer[pos].ntc[i].value,(i+1)));
      i++;
    }

    i=0;
    while (i<2){
      printf("Digital%d  Value=%d\n",
      i,      
      C_Buffer[pos].digital[i].value);
      i++;
    }
    color(255);
  k_mutex_unlock(&c_buffer_busy);
}

void print_current_position_cb_new(uint32_t pos){
   int32_t val_mv;
   _Circular_Buffer *C_Buffer;
   uint16_t size=sizeof(_Circular_Buffer);

   C_Buffer = k_malloc(size);
   *C_Buffer=read_memory(pos);
    
    k_mutex_lock(&c_buffer_busy,K_FOREVER);
    color(13);
    printf("\n\n####Position %d #####\n",pos);

    printf("GNSS Position Lat=%d Long=%d TimeStamp=%d \n",
      C_Buffer->gnss_module.latitude,
      C_Buffer->gnss_module.longitude,
      C_Buffer->gnss_module.timestamp);


    val_mv = C_Buffer->analog.value;
    adc_raw_to_millivolts_dt(&adc_channels[ANALOG_SENSOR],&val_mv);
 
    printf("Analog  TimeStamp=%d Value=%d  %"PRId32"mV \n",
      C_Buffer->analog.timestamp,
      C_Buffer->analog.value,
      val_mv);
    
    int i=0;
    while (i<3){
      printf("NTC %d TimeStamp=%d Value=%d %3.1f C\n",
      i,
      C_Buffer->ntc[i].timestamp,
      C_Buffer->ntc[i].value,
      ntc_temperature(C_Buffer->ntc[i].value,(i+1)));
      i++;
    }

    i=0;
    while (i<2){
      printf("Digital%d  TimeStamp=%d Value=%d\n",
      i,
      C_Buffer->digital[i].timestamp,
      C_Buffer->digital[i].value);
      i++;
    }
 color(255);
 k_free(C_Buffer);
 k_mutex_unlock(&c_buffer_busy);
}

void init_circular_buffer(void){
   C_Buffer_Free_Position=0;
}

void feed_circular_buffer(void){

    k_mutex_lock(&c_buffer_busy,K_FOREVER);
      
     
    if (C_Buffer_Free_Position < CIRCULAR_BUFFER_ELEMENTS){

     C_Buffer_Current_Position=C_Buffer_Free_Position;
     // This funcion must be called each 1 minute
     // FIRST FREE POSITION IS 0 AND THE LAST IS (CIRCULAR_BUFFER_ELEMENTS-1)   

     C_Buffer[C_Buffer_Free_Position].gnss_module=values_of_gnss_module();
     C_Buffer[C_Buffer_Free_Position].analog=values_of_analog_sensor(ANALOG_SENSOR);

    // here must deploy call to convert the values into temperature
     C_Buffer[C_Buffer_Free_Position].ntc[0].value=values_of_ntc_sensor(NTC_1).value;
     C_Buffer[C_Buffer_Free_Position].ntc[1].value=values_of_ntc_sensor(NTC_2).value;
     C_Buffer[C_Buffer_Free_Position].ntc[2].value=values_of_ntc_sensor(NTC_3).value;
    
     C_Buffer[C_Buffer_Free_Position].ntc[0].timestamp=values_of_ntc_sensor(NTC_1).timestamp;
     C_Buffer[C_Buffer_Free_Position].ntc[1].timestamp=values_of_ntc_sensor(NTC_2).timestamp;
     C_Buffer[C_Buffer_Free_Position].ntc[2].timestamp=values_of_ntc_sensor(NTC_3).timestamp;
   
     C_Buffer[C_Buffer_Free_Position].digital[0]=values_of_digital_sensor(0);
     C_Buffer[C_Buffer_Free_Position].digital[1]=values_of_digital_sensor(1);
     save_memory(C_Buffer_Free_Position);
     C_Buffer_Free_Position++;
    }else{
      C_Buffer_Free_Position=0;
      }
    k_mutex_unlock(&c_buffer_busy);  
}

_Circular_Buffer read_memory(uint32_t Pos){
    k_mutex_lock(&c_buffer_busy,K_FOREVER);
    _Circular_Buffer *buf;
    uint16_t size=sizeof(_Circular_Buffer),err=0;
    buf = k_malloc(size);
    uint16_t Id= Pos + BASE_DATA_BUFFER;
    err=nvs_read(&fs, Id, buf, size);
    printf("Result read=%d bytes\n",err);
    return *buf;
    k_free(buf);
    k_mutex_unlock(&c_buffer_busy); 
}

void save_memory(uint32_t Pos){
    color(6);
    _Circular_Buffer *buf;
    uint16_t size=sizeof(_Circular_Buffer),err=0;
    printf("Size of structure=%d bytes\n",size);
    buf = k_malloc(size);
    *buf=C_Buffer[Pos];
    uint16_t id= Pos + BASE_DATA_BUFFER;
    printf("Position %d\n",Pos); 

    err=nvs_write(&fs, id, buf,size);
    printf("Result=%d bytes saved\n",err);
    (void)nvs_write(&fs, LOG_POSITION, &C_Buffer_Current_Position,sizeof(C_Buffer_Current_Position));
    color(255);
    k_free(buf);
    
}

void init_alarm_circular_buffer(void){
    C_Buffer_Alarm_Free_Position=0;
}

void feed_alarm_circular_buffer(void){

    // This funcion must be called each alarm event
    // FIRST FREE POSITION IS 0 AND THE LAST IS (ALARM_EVENT_QTY_MAX-1)     
     if (C_Buffer_Alarm_Free_Position >= ALARM_EVENT_QTY_MAX){
       C_Buffer_Alarm_Free_Position=0;
     }

     C_Buffer_Alarm[C_Buffer_Alarm_Free_Position].event_code=0;
     C_Buffer_Alarm[C_Buffer_Alarm_Free_Position].id=0;
     C_Buffer_Alarm[C_Buffer_Alarm_Free_Position].timestamp=0;
     C_Buffer_Alarm[C_Buffer_Alarm_Free_Position].value=0;

    C_Buffer_Alarm_Free_Position++;
}

History_st *fill_fields_to_test(){
   static History_st msg;
   char tag[30]="10203";
   int sensor_id=0;
 
    msg.timestamp=0;

     //here are 1440 measures
     msg.positions.timestamp=1;
     msg.positions.latitude=2;
     msg.positions.longitude=3;

    //here are 1440 measures
     msg.device_internal_temperatures.timestamp=11;
     msg.device_internal_temperatures.value=12;
    
   while (sensor_id<6){
    msg.input_data[sensor_id].input_id=sensor_id+111;
    msg.input_data[sensor_id].input_id=sensor_id+112;
    msg.input_data[sensor_id].enable=sensor_id+113;
    msg.input_data[sensor_id].has_label=0xFF; // if false the label will not show
    strcpy(msg.input_data[sensor_id].label,tag);
    msg.input_data[sensor_id].has_phy_dimension=0xFF; // if false the dimension will not show

    msg.input_data[sensor_id].phy_dimension=PhysicalDimension_AREA; 
    //here are 1440 measures    
    msg.input_data[sensor_id].values.timestamp=sensor_id+1111;
    msg.input_data[sensor_id].values.value=sensor_id+1112;
   
   sensor_id++;
   }
  return &msg;
}

buf_data send_array_dd_v0(void){
 
   static buf_data function_return;

   static uint8_t buffer[History_size];
   uint32_t sensor_data=0;
   uint32_t total_bytes_encoded = 0;

   k_mutex_lock(&c_buffer_busy,K_FOREVER);

   History_st *data = fill_fields_to_test();
   pb_ostream_t ostream;
   UplinkMessage msg_all;

   msg_all.type=MessageType_HISTORY; // tipo history
   msg_all.which_Data=MessageType_HISTORY;

   msg_all.Data.history.timestamp=1234567;

   sensor_data=0;  //CORRECT 1440
    while (sensor_data<ARRAY_TEST){
     msg_all.Data.history.positions[sensor_data].timestamp=data->positions.timestamp;
     msg_all.Data.history.positions[sensor_data].latitude=data->positions.latitude;
     msg_all.Data.history.positions[sensor_data].longitude=data->positions.longitude;
    sensor_data++;
    }
 
   sensor_data=0;  //CORRECT 1440
    while (sensor_data<ARRAY_TEST){
    msg_all.Data.history.device_internal_temperatures[sensor_data].timestamp=data->device_internal_temperatures.timestamp;
    msg_all.Data.history.device_internal_temperatures[sensor_data].value=data->device_internal_temperatures.value;
   sensor_data++;
    }

   sensor_data=0;
   int sensor_id=0;
   while (sensor_id<6){
    msg_all.Data.history.input_data[sensor_id].input_id=data->input_data[sensor_id].input_id;
    msg_all.Data.history.input_data[sensor_id].enable=data->input_data[sensor_id].enable;
    msg_all.Data.history.input_data[sensor_id].has_label=data->input_data[sensor_id].has_label;
    strcpy(msg_all.Data.history.input_data[sensor_id].label,data->input_data[sensor_id].label);
    msg_all.Data.history.input_data[sensor_id].has_phy_dimension=data->input_data[sensor_id].has_phy_dimension;
    msg_all.Data.history.input_data[sensor_id].phy_dimension=data->input_data[sensor_id].phy_dimension;
    
    sensor_data=0;  //CORRECT 1440
    while (sensor_data<ARRAY_TEST){
    msg_all.Data.history.input_data[sensor_id].values[sensor_data].timestamp=data->input_data[sensor_id].values.timestamp;
    msg_all.Data.history.input_data[sensor_id].values[sensor_data].value=data->input_data[sensor_id].values.value;
     sensor_data++;
    }
   
   sensor_id++;
   }

   ostream = pb_ostream_from_buffer(buffer, UplinkMessage_size);
   pb_encode(&ostream, UplinkMessage_fields, &msg_all);
 
   total_bytes_encoded = ostream.bytes_written;
     

   int j=0;
   while(j < total_bytes_encoded ){
   function_return.data[j]=buffer[j];
   j++;
   }
   
   function_return.len=total_bytes_encoded;
   return function_return;
   k_mutex_unlock(&c_buffer_busy);
}

void test_calendar(void){

   uint64_t actual_time = 0;
   signed int  h, m, s,last_minute;
    h = (actual_time/3600); 
	m = (actual_time -(3600*h))/60;
	s = (actual_time -(3600*h)-(m*60));
	last_minute=m;

  while(1){   
    actual_time++;
	k_sleep(K_MSEC(1));
	h = (actual_time/3600); 
	m = (actual_time -(3600*h))/60;
	s = (actual_time -(3600*h)-(m*60));

   
   if (m==(last_minute+1)){
		last_minute=m;
        if (m==59){last_minute=-1;}
		if (h==24){h=0;} // only up to 23:59:59h
		
	    //RUN THE MINUTE ROUTINE
	    printk("%02d:%02d:%02d\n",h,m,s);
	    printk("%lld\n",actual_time);
   }
   
  }

}

float BetaTermistor(void) {
  //USE ONLY ONCE TO CALCULATE BETA
  float beta;
  float T1=(-6 + 273.15); // -6 celsius
  float T2=(56 + 273.15); // 56 celsius
  float RT1=54200/1000; //54.2k
  float RT2=2480/1000;  //2.48k
  beta = (log(RT1 / RT2)) / ((1 / T1) - (1 / T2));  // cálculo de beta.
  printf("Beta=%f\n",beta);
  return beta;
 
}
  
float ntc_temperature(uint16_t conversao,uint8_t sensor_number){
  // ELECTRIC WIRE DIAGRAM
  //  +3V --- RESISTOR_SERIE_NTC ----AD--- NTC --- GND


  //sources:  https://blog.eletrogate.com/termistor-ntc-para-controle-de-temperatura/
  //          https://elcereza.com/termistor/
  float voltageUc = conversao*(ADC_VOLTAGE_REF/(ADC_RESOLUTION-1));
  //printf("voltageUC=%f\n",voltageUc);

  float resistor=0;
  switch (sensor_number){
    case NTC_1: resistor=RESISTOR_SERIE_NTC1;break;
    case NTC_2: resistor=RESISTOR_SERIE_NTC2;break;
    case NTC_3: resistor=RESISTOR_SERIE_NTC3;break;
  }

  float Rt =  (voltageUc*resistor)/(VOLTAGE_ALIM-voltageUc);
  //printf("Rt=%f\n",Rt);
  float T = 1 /( 1 / TERMISTOR_KELVIN_25 + log(Rt / TERMISTOR_RES_25) / TERMISTOR_BETA ); 
  //printf("T=%f\n",T);
  float Tc = T - 273.15; 
  //printf("Tc=%f\n",Tc);
  return Tc;
}

//gnss

int debug_print_fields(int numfields, char **fields)
{
	printf("Parsed %d fields\r\n",numfields);

	for (int i = 0; i <= numfields; i++) {
		printf("Field %02d: [%s]\r\n",i,fields[i]);
	}
}

int parse_comma_delimited_str(char *string, char **fields, int max_fields)
{
	int i = 0;
	fields[i++] = string;

	while ((i < max_fields) && NULL != (string = strchr(string, ','))) {
		*string = '\0';
		fields[i++] = ++string;
	}

	return --i;
}


//lorawan
// https://www.youtube.com/watch?v=M5VGos3YTpI&t=150s
// https://playcode.io/javascript
//https://www.exploringbinary.com/displaying-the-raw-fields-of-a-floating-point-number/



void lorawan_tx_data(void){

  char data_test[] =  { 0X00 , 0X00 , 0X00 , 0X00 , //LATITUDE
                        0X00 , 0X00 , 0X00 , 0X00 , //LONGITUDE
					              0X00 , 0X00 , 0X00 , 0X00 , //TIMESTAMP
					              0X00 , 0X00 , 0X00 , 0X00 , //ANALOG
                        0X00 ,                      //DIGITAL
                        0X00 ,                      //DIGITAL
					              0X00 , 0X00 ,               //NTC0
                        0X00 , 0X00 ,               //NTC1 
                        0X00 , 0X00                 //NTC2 
                      };
  int ret=0,nt=0,k=0;
  uint64_t j=0;

  k_mutex_lock(&c_buffer_busy, K_FOREVER);

  uint32_t pos=C_Buffer_Current_Position;
  float a=C_Buffer[pos].gnss_module.latitude;  //4 bytes 0..3
  float b=C_Buffer[pos].gnss_module.longitude; //4 bytes 4..7
  float c=C_Buffer[pos].gnss_module.timestamp; //4 bytes 8 
  float d=C_Buffer[pos].analog.value;          //4 bytes 12..17
  uint8_t e=C_Buffer[pos].digital[0].value;      //1 byte 16
  uint8_t f=C_Buffer[pos].digital[1].value;      //1 byte 17
  uint16_t g=C_Buffer[pos].ntc[0].value;          //2 bytes 18..19
  uint16_t h=C_Buffer[pos].ntc[1].value;          //2 bytes 20..21
  uint16_t i=C_Buffer[pos].ntc[2].value;          //2 bytes 22..23
                                             //total 30 bytes
  k_mutex_unlock(&c_buffer_busy);
  unsigned char *ptr_lati         = (unsigned char *) &a; //TAKE THE ADDRESS OF VARIABLES
  unsigned char *ptr_long         = (unsigned char *) &b;
  unsigned char *ptr_timestamp    = (unsigned char *) &c;
  unsigned char *ptr_analog       = (unsigned char *) &d;
  unsigned char *ptr_digi0        = (unsigned char *) &e;
  unsigned char *ptr_digi1        = (unsigned char *) &f;
  unsigned char *ptr_ntc0         = (unsigned char *) &g;
  unsigned char *ptr_ntc1         = (unsigned char *) &h;
  unsigned char *ptr_ntc2         = (unsigned char *) &i;

  
  for (int i = 0; i < sizeof(float); i++) {
     data_test[i]    =*(ptr_lati      + i);
     data_test[i+4]  =*(ptr_long      + i);
     data_test[i+8]  =*(ptr_timestamp + i);
     data_test[i+12] =*(ptr_analog    + i);
  }

     //data_test[16] =*(ptr_digi0);
     data_test[16] = sensor_status.number[SENSOR_DIG_4]; //ALARM COUNTER
     data_test[17] =*(ptr_digi1);


     data_test[18]    =*(ptr_ntc0 + 0); //first LSB and after MSB - little endian
     data_test[19]    =*(ptr_ntc0 + 1); //first LSB and after MSB - little endian
     data_test[20]    =*(ptr_ntc1 + 0); //first LSB and after MSB - little endian
     data_test[21]    =*(ptr_ntc1 + 1); //first LSB and after MSB - little endian
     data_test[22]    =*(ptr_ntc2 + 0); //first LSB and after MSB - little endian
     data_test[23]    =*(ptr_ntc2 + 1); //first LSB and after MSB - little endian



 color(12);
 printk("HELIUM PAYLOAD: ");
 for (int h = 0; h < sizeof(data_test); h++) {
     printk("%02X ",data_test[h]);
  }
  color(10);
  printk("\nSending payload...\n");
  color(255);
  data_sent_cnt++;

  ret = lorawan_send(2, data_test, sizeof(data_test),LORAWAN_MSG_UNCONFIRMED);

		if (ret < 0) {
			printk("lorawan_send confirm failed -trying again : %d\n\n", ret);

      while(ret<0 && nt<=RETRY){ 
       ret = lorawan_send(2, data_test, sizeof(data_test),LORAWAN_MSG_UNCONFIRMED);
       nt++;
       //lorawan_reconnect_cnt++;
       //if(lorawan_reconnect_cnt==LIMIT_RECONNECT_CNT){lorawan_reconnect_cnt=0;lorawan_reconnect=1;}
       if (ret==0){
        printk("Payload Data sent %d\n",data_sent_cnt);
        
        //lorawan_reconnect_cnt=0;
        }else{printk("Data send failed-trying again ret=%d \n ",ret);
              k_sleep(DELAY_RTY);
            }
      }
      nt=0;
			//return;
		}else{  color(10);
		        printk("Payload Data sent %d\n\n",data_sent_cnt);
            color(255);
            //lorawan_reconnect_cnt=0;
		     }
    if(data_sent_cnt>=DATA_SENT_JOIN_AGAIN){lorawan_reconnect=1;}

  

}

void setup_initialize(void){

  uint8_t i;
  uint8_t dev[8] = LORAWAN_DEV_EUI_HELIUM;
  uint8_t join[8] = LORAWAN_JOIN_EUI_HELIUM;
  uint8_t key[16] = LORAWAN_APP_KEY_HELIUM;
  for(i=0;i<=7;i++){Initial_Setup.dev[i] = dev[i];} 
  for(i=0;i<=7;i++){Initial_Setup.join[i] = join[i];} 
  for(i=0;i<=15;i++){Initial_Setup.key[i] = key[i];} 
  for(i=0;i<=15;i++){Initial_Setup.nwk_key[i] = 0;} 
  Initial_Setup.joined=OFF;
  Initial_Setup.dev_nonce=0;
  
  Initial_Setup.led_blink_time=RUN_LED_BLINK_INTERVAL;
  Initial_Setup.interval_uplink=LORAWAN_INTERVAL_NORMAL;
  Initial_Setup.output_port=0;
  Initial_Setup.turn_angle[0]=0;
  Initial_Setup.turn_angle[1]=0;
  Initial_Setup.turn_angle[2]=0;
  Initial_Setup.turn_angle[3]=0;
  Initial_Setup.turn_speed[0]=0;
  Initial_Setup.turn_speed[1]=0;
  Initial_Setup.turn_speed[2]=0;
  Initial_Setup.turn_speed[3]=0;
  
  
}

void print_setup(void){
  
	  printk("Led Blink Time      : %d ms\n",Initial_Setup.led_blink_time);
	  printk("Interval UpLink Time: %d minutes\n",Initial_Setup.interval_uplink);
    printk("DEV     : ");
    for(int i=0;i<=7;i++){printk("%02X ",Initial_Setup.dev[i]);}
    printk("\n");
    printk("JOIN    : ");
    for(int i=0;i<=7;i++){printk("%02X ",Initial_Setup.join[i]);}
    printk("\n");
    printk("KEY     : ");
    for(int i=0;i<=15;i++){printk("%02X ",Initial_Setup.key[i]);}
    printk("\n");
    printk("NWK_KEY : ");
    for(int i=0;i<=15;i++){printk("%02X ",Initial_Setup.nwk_key[i]);}
    printk("\n");
    printk("DEV_NOUNCE: %08X\n",Initial_Setup.dev_nonce);
    if(Initial_Setup.joined==1){printk("JOIN = ON");}else{printk("JOIN = OFF");}
    printk("\n");


}

void color(uint8_t color) {
    switch (color) {
        case 0: printk("\033[0m");        // Preto
                break;
        case 1: printk("\033[31m");       // Vermelho
                break;
        case 2: printk("\033[32m");       // Verde
                break;
        case 3: printk("\033[33m");       // Amarelo
                break;
        case 4: printk("\033[34m");       // Azul
                break;
        case 5: printk("\033[35m");       // Magenta
                break;
        case 6: printk("\033[36m");       // Ciano
                break;
        case 7: printk("\033[37m");       // Branco
                break;
        case 8: printk("\033[90m");       // Cinza claro
                break;
        case 9: printk("\033[91m");       // Vermelho claro
                break;
        case 10: printk("\033[92m");      // Verde claro
                break;
        case 11: printk("\033[93m");      // Amarelo claro
                break;
        case 12: printk("\033[94m");      // Azul claro
                break;
        case 13: printk("\033[95m");      // Magenta claro
                break;
        case 14: printk("\033[96m");      // Ciano claro
                break;
        case 15: printk("\033[97m");      // Branco claro
                break;
        case 255: printk("\033[0m");       // Padrão (branco)
                break;
    }
}



Data_Return cmd_interpreter(uint8_t *data,uint8_t len){
  
  static Data_Return buf;
  buf.len=0;
  
  //printk("testing command \n");
  color(4);
  	switch(data[0]){
			case CMD_RESET_ALARM_FLAG: //RESET ALARM SIGNAL
			   color(1);
         sensor_status.number[SENSOR_DIG_4]=0;
         Initial_Setup.interval_uplink=LORAWAN_INTERVAL_NORMAL;
			   printk("ALARM FLAG RESET 4\n");
         buf.len=sprintf(buf.data, "ALARM FLAG RESET");
         
		     
			break;
			case CMD_LED4_ON: // TURN ON LED 4
			   color(1);
			   gpio_pin_set_dt(LED4, ON);
			   printk("TURNED ON LED 4\n");
         buf.len=sprintf(buf.data, "TURNED ON LED 4");
         
		     
			break;
			
			case CMD_LED4_OFF: //TURN OFF LED 4
			   color(1);
			   gpio_pin_set_dt(LED4, OFF);
			   printk("TURNED OFF LED 4\n");
         buf.len=sprintf(buf.data, "TURNED OFF LED 4");
         
			break;


      case CMD_RESET: //R
			    color(2);
			    setup_initialize();
				  flash_write_setup();
				  print_setup();
				  printk("Setup Reset\n");
          buf.len=sprintf(buf.data, "SETUP RESET");
              
			break;

      case CMD_ALARM_OFF:
           color(1);
           sensor_status.active[SENSOR_DIG_4]=0;
           sensor_status.busy[SENSOR_DIG_4]=ON;
           printk("ALARM OFF\n");
           buf.len=sprintf(buf.data, "ALARM OFF");
      break;

      case CMD_ALARM_ON:
           color(1);
           sensor_status.active[SENSOR_DIG_4]=1;
           sensor_status.busy[SENSOR_DIG_4]=OFF;
           printk("ALARM ON\n");
           buf.len=sprintf(buf.data, "ALARM ON");
      break;


      case CMD_READ: //P
			    color(3);
			    flash_read_setup();
			    print_setup();
          buf.len=sprintf(buf.data, "COMMAND READ"); 
			break;

      case CMD_WRITE: //Q
			     color(3);
			     
			     print_setup();
           buf.len=sprintf(buf.data, "COMMAND WRITE");             
			break;
			
      case CMD_TEST: //Q
			     color(3);
			     test_command();
           //k_sem_give(&lorawan_tx);
			break;


		}
       color(0);
          
	  return buf;
    
}

/*below CPI under test*/


void commandFunc1(uint16_t param) {
    printk("Command 1 ok: Param:%u\n", param);
}

void commandFunc2(uint16_t param) {
    printk("Command 2 ok: Param:%u\n", param);
}

void commandFunc3(uint16_t param) {
    printk("Command 3 ok: Param:%u\n", param);
}

Command commands[MAX_COMMANDS] = {
    {"1", commandFunc1},
    {"2", commandFunc2},
    {"3", commandFunc3}
};

void executeCommand(char *commandString, uint16_t param) {
    int i;
    for (i = 0; i < MAX_COMMANDS; i++) {
        if (strncmp(commands[i].command, commandString, MAX_COMMAND_LENGTH) == 0) {
            commands[i].function(param);
            return;
        }
    }
    
    printf("Command invalid\n");
}
void cmd_interpreter_pwd(uint8_t *data, uint8_t len, uint8_t *password) {
    char password_definition[] = {'1', '2', '3', '4', '5', '6'};

    
    if (memcmp(password, password_definition, sizeof(password_definition)) != 0) {
        printk("Wrong Password\n");
        return;
    }

    
    char command[MAX_COMMAND_LENGTH];
    strncpy(command, (char *)data, MAX_COMMAND_LENGTH - 1);
    command[MAX_COMMAND_LENGTH - 1] = '\0';

    uint16_t param = data[MAX_COMMAND_LENGTH];

    executeCommand(command, param);
}


void test_command(void) {
    uint8_t data[] = {'1', 0x00,0x0A};
    uint8_t len = sizeof(data) / sizeof(data[0]);

    uint8_t password_sent[] = {'1', '2', '3', '4', '5', '6'};
    cmd_interpreter_pwd(data, len, password_sent);
}



