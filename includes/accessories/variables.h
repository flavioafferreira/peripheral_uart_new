#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>


#include "includes/Protobuf/pb.h"
#include "includes/Protobuf/data_def_v0_pb.h"

#define ON  1
#define OFF 0

#define DEBUG OFF

//      sensor_name   sensor_number
#define ANALOG_SENSOR 0
#define NTC_1         1
#define NTC_2         2
#define NTC_3         3

// the channel number can be different- defined on app.overlay
#define ANALOG_SENSOR_CH 0 //sensor's number  - channel
#define NTC_1_CH         1
#define NTC_2_CH         2
#define NTC_3_CH         3

//digital inputs limit - after the limit, will not increase each pulse
#define DIGITAL_0_LIMIT 0xFFFF // maximum for uint16_t
#define DIGITAL_1_LIMIT 0xFFFF
#define DIGITAL_2_LIMIT 0xFFFF


#define NUMBER_OF_TEMPERATURE_SENSORS 3
#define NUMBER_OF_TIMER_SENSORS 2
#define CIRCULAR_BUFFER_ELEMENTS 1440
#define PROTOBUF_ELEMENTS 77760
#define ALARM_EVENT_QTY_MAX 100

#define ADC_RESOLUTION          16383 
#define ADC_VOLTAGE_REF         3.6 //volts
#define VOLTAGE_ALIM            3.0 //volts
#define RESISTOR_SERIE_NTC1     9860 //ohms THERMISTOR SHOULD BE GROUNDED ONE SIDE AND IN SERIES WITH TERMISTOR_SERIE
#define RESISTOR_SERIE_NTC2     9980 //ohms
#define RESISTOR_SERIE_NTC3     9890 //ohms

#define TERMISTOR_KELVIN_25   298.15 // KELVIN TEMPERATURE 25C
#define TERMISTOR_RES_25       10000 // RESISTANCE AT 25C
#define TERMISTOR_BETA          3969 // BETA

#define BOOT_POSITION    1     //THIS IS AN ID TO IDENTIFY THE POSITION ON NVS
#define LOG_POSITION     2     //THIS IS AN ID TO IDENTIFY THE POSITION ON NVS
#define BASE_DATA_BUFFER 1000  //THIS IS AN ID TO IDENTIFY THE POSITION ON NVS

#define BUFF_SIZE 480  //BUFFER TO JOIN UART BYTES RECEIVED

//STRUCTURE FOR HISTORY

typedef struct _InputValue_ { 
    int32_t timestamp; 
    float value; 
} InputValue_st;

typedef struct _InputData_ { 
    uint32_t input_id; /* Source ID table */
    bool enable; 
    bool has_label;
    char label[20]; 
    bool has_phy_dimension;
    PhysicalDimension phy_dimension; 
    InputValue_st values; // 800 fields 
} InputData_st;

typedef struct _Position_ { 
    int32_t timestamp; 
    float latitude; 
    float longitude; 
} Position_st;

typedef struct _History_ { 
    int32_t timestamp; 
    Position_st positions; // 800 fields
    InputValue_st device_internal_temperatures; // 800 fields 
    InputData_st input_data[6]; /* one for each input */
} History_st;


typedef struct _UplinkMessage_ { 
    MessageType type; 
    uint8_t which_Data;
    union {
        History_st history;
        //Statistics_st statistics;
        //Events_st events;
    } Data_st; 
} UplinkMessage_st;

// The structure for Circular Buffer is minor than structure for create
// and send the Protobuf. The Protobuf requires different size of fields
// Because of this, was created two different structures. 

typedef struct _Gnss_ { 
    int32_t timestamp; 
    float latitude; 
    float longitude;
    uint8_t  gps_fixed;  
} Gnss;

typedef struct _Analog_ { 
    int32_t timestamp; 
    int32_t value; 
} Analog;

typedef struct _Ntc_ { 
    int32_t timestamp; 
    int16_t value; 
} Ntc;

typedef struct _Digital_ { 
    int32_t timestamp; 
    int32_t value; 
} Digital;

//Circular Buffer Structure

typedef struct _Circular_Buffer_ { 
    uint16_t indice;
    Gnss     gnss_module;
    Analog   analog;
    Ntc      ntc[3];
    Digital  digital[2];
} _Circular_Buffer;



//Circular Buffer for Alarm System

typedef struct _Alarm_ { 
    int32_t timestamp; 
    int8_t  id;
    int8_t  event_code;
    int32_t value;     
} _Alarm;

//Alarm C_Buffer_Alarm[ALARM_EVENT_QTY_MAX];
//next position 

