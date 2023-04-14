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

//NVS
extern struct flash_pages_info info;
extern struct nvs_fs fs;
extern uint32_t button2_counter;
void save_memory(uint32_t Pos);
_Circular_Buffer read_memory(uint32_t Pos);



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


void time_print (void){
 
    struct tm  *ts;
    char       buf[80];
    
    uint64_t actual_time_seconds = k_uptime_get()/1000;
    
    // Get current time
    ts = gmtime(actual_time_seconds);

   

    // Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
    
    strftime(buf, sizeof(buf), "%a %Y-%m-%d %H:%M:%S %Z", &ts);
    printk("%s\n", buf);

}



uint32_t time_stamp_function(void){

  // needs to configure the Zephyr date/time using some source
  uint32_t time_st=0;
  uint32_t base_january_2022 =1640995200;
  uint64_t actual_time_seconds = k_uptime_get()/1000;

  time_st = base_january_2022 + actual_time_seconds;
  
  return time_st;
}

Gnss values_of_gnss_module(void){
  Gnss gnss_return_value;
  gnss_return_value.latitude=0;
  gnss_return_value.longitude=0;
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
 
    printf("\n\n####Position %d #####\n",pos);

    printf("GNSS Position Lat=%d Long=%d TimeStamp=%d \n",
      C_Buffer[pos].gnss_module.latitude,
      C_Buffer[pos].gnss_module.longitude,
      C_Buffer[pos].gnss_module.timestamp);


    val_mv = C_Buffer[pos].analog.value;
    adc_raw_to_millivolts_dt(&adc_channels[ANALOG_SENSOR],&val_mv);
 
    printf("Analog  TimeStamp=%d Value=%d  %"PRId32"mV \n",
      C_Buffer[pos].analog.timestamp,
      C_Buffer[pos].analog.value,
      val_mv);
    
    int i=0;
    while (i<3){
      printf("NTC %d TimeStamp=%d Value=%d %3.1f C\n",
      i,
      C_Buffer[pos].ntc[i].timestamp,
      C_Buffer[pos].ntc[i].value,
      ntc_temperature(C_Buffer[pos].ntc[i].value,(i+1)));
      i++;
    }

    i=0;
    while (i<2){
      printf("Digital%d  TimeStamp=%d Value=%d\n",
      i,
      C_Buffer[pos].digital[i].timestamp,
      C_Buffer[pos].digital[i].value);
      i++;
    }

}

void print_current_position_cb_new(uint32_t pos){
   int32_t val_mv;
   _Circular_Buffer *C_Buffer;
   uint16_t size=sizeof(_Circular_Buffer);

   C_Buffer = k_malloc(size);
   *C_Buffer=read_memory(pos);

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

 k_free(C_Buffer);
}


void init_circular_buffer(void){
   C_Buffer_Free_Position=0;
}

void feed_circular_buffer(void){

    
    C_Buffer_Current_Position=C_Buffer_Free_Position;
    // This funcion must be called each 1 minute
    // FIRST FREE POSITION IS 0 AND THE LAST IS (CIRCULAR_BUFFER_ELEMENTS-1)     
     
    if (C_Buffer_Free_Position < CIRCULAR_BUFFER_ELEMENTS){
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
}


_Circular_Buffer read_memory(uint32_t Pos){
    _Circular_Buffer *buf;
    uint16_t size=sizeof(_Circular_Buffer),err=0;
    buf = k_malloc(size);
    uint16_t Id= Pos + BASE_DATA_BUFFER;
    err=nvs_read(&fs, Id, buf, size);
    printf("Result read=%d bytes\n",err);
    return *buf;
    k_free(buf);
}

void save_memory(uint32_t Pos){
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

