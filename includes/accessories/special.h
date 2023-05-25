#include "includes/Protobuf/data_def_v0_pb.h"
//#include "includes/accessories/variables.h"

//#define MaxBuf 164 para STACKSIZE 1024 'E O MAXIMO

#define MaxBuf 164

#define MAX_COMMANDS 10
#define MAX_COMMAND_LENGTH 2
#define MAX_PASSWORD_LENGTH 6

typedef struct buf_data_t {
	   void *fifo_reserved;
	   uint8_t  data[History_size];
	   uint16_t len;
}buf_data;

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
typedef struct _Data_Return_{
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
}Data_Return;

typedef struct {
    char command[MAX_COMMAND_LENGTH];
    void (*function)(uint16_t);
} Command;



buf_data send_array_dd_v0(void);
Data_Return cmd_interpreter(uint8_t *data,uint8_t len);

void feed_circular_buffer(void);
void print_current_position_cb(uint32_t pos);
void print_current_position_cb_new(uint32_t pos);
void time_print (void);

float ntc_temperature(uint16_t conversao,uint8_t sensor_number);
void flash_button2_counter(void);

int hex2int(char *c);
int checksum_valid(char *string);
int parse_comma_delimited_str(char *string, char **fields, int max_fields);
int debug_print_fields(int numfields, char **fields);
int SetTime(char *date, char *time);
void gps_main(uint8_t *buffer,uint32_t nbytes);
void fill_date(uint8_t *field_time,uint8_t *field_date );
void latitude_time_print(void);
void lorawan_tx_data(void);
float ntc_temperature(uint16_t conversao,uint8_t sensor_number);
void setup_initialize(void);
void print_setup(void);
void flash_write_setup(void);
void flash_read_setup(void);
void color(uint8_t color);
void test_command(void); 






/*

//JAVASCRIPT DECODER FUNCTION USED IN HELIUM PLATAFORM
// https://console.helium.com
//HELIUM PAYLOAD: 38 78 8D 45 E8 43 97 44 AF B6 C8 4E 00 C0 D1 45 00 00 56 1A 24 1A 9A 19
//RESULTS:
//"entry.1359999784=45.4504557&entry.1873862209=12.1687012&entry.81100085=2023-05-10%2008%3A36%3A16%20UTC&entry.393908294=1.475&entry.1609835373=0&entry.732934055=0&entry.378011195=25.9&entry.219951820=25.9&entry.1303722275=27.0"


const ADC_RESOLUTION = 16383;
const ADC_VOLTAGE_REF = 3.6; // volts
const VOLTAGE_ALIM = 3.0; // volts
const RESISTOR_SERIE_NTC1 = 9860; // ohms THERMISTOR SHOULD BE GROUNDED ONE SIDE AND IN SERIES WITH TERMISTOR_SERIE
const RESISTOR_SERIE_NTC2 = 9980; // ohms
const RESISTOR_SERIE_NTC3 = 9890; // ohms
const TERMISTOR_KELVIN_25 = 298.15; // KELVIN TEMPERATURE 25C
const TERMISTOR_RES_25 = 10000; // RESISTANCE AT 25C
const TERMISTOR_BETA = 3969; // BETA

const NTC_1 = 0; // BETA
const NTC_2 = 1; // BETA
const NTC_3 = 2; // BETA

function ntc_temperature(conversao, sensor_number) {
  const voltageUc = conversao * (ADC_VOLTAGE_REF / (ADC_RESOLUTION - 1));
  let resistor = 0;
  switch (sensor_number) {
    case NTC_1:
      resistor = RESISTOR_SERIE_NTC1;
      break;
    case NTC_2:
      resistor = RESISTOR_SERIE_NTC2;
      break;
    case NTC_3:
      resistor = RESISTOR_SERIE_NTC3;
      break;
      default:break;
  }
  const Rt = (voltageUc * resistor) / (VOLTAGE_ALIM - voltageUc);
  const T = 1 / (1 / TERMISTOR_KELVIN_25 + Math.log(Rt / TERMISTOR_RES_25) / TERMISTOR_BETA);
  const Tc = T - 273.15;
  return Tc;
}

function convertUnixTimestamp(unixTimestamp) {
  const date = new Date(unixTimestamp * 1000);
  const year = date.getUTCFullYear();
  const month = ('0' + (date.getUTCMonth() + 1)).slice(-2);
  const day = ('0' + date.getUTCDate()).slice(-2);
  const hours = ('0' + date.getUTCHours()).slice(-2);
  const minutes = ('0' + date.getUTCMinutes()).slice(-2);
  const seconds = ('0' + date.getUTCSeconds()).slice(-2);
  return `${year}-${month}-${day} ${hours}:${minutes}:${seconds} UTC`;
}

function littleEndianBytesToFloat(bytes) {
  let u = (bytes[3] << 24) | (bytes[2] << 16) | (bytes[1] << 8) | bytes[0];
  let result = 0;
  let bytesResult = [];
  for (let i = 0; i < 4; i++) {
    bytesResult[i] = String.fromCharCode((u >> (i * 8)) & 0xff);
  }
  let buffer = new ArrayBuffer(4);
  let view = new DataView(buffer);
  bytesResult.forEach(function (b, i) {
    view.setUint8(i, b.charCodeAt(0));
  });
  result = view.getFloat32(0, true);
  return result;
}

function littleEndianBytesToUint16(bytes) {
   return (bytes[1] << 8) | bytes[0];
}

function convertToGNSSPosition(latitude, longitude) {
  const lat = Math.floor(latitude / 100) + (latitude % 100) / 60;
  const lon = Math.floor(longitude / 100) + (longitude % 100) / 60;
  return `${lat.toFixed(7)},${lon.toFixed(7)}`;
}

function Decoder(bytes,port){

   var decoded={};
   var nro = [bytes[0],bytes[1],bytes[2],bytes[3]];
      
  const dt_lat = littleEndianBytesToFloat([bytes[0],bytes[1],bytes[2],bytes[3]]);
  const lat =Math.floor(dt_lat / 100) + (dt_lat % 100) / 60;
  decoded.longitude=lat.toFixed(7);
  
  const dt_lon  = littleEndianBytesToFloat([bytes[4],bytes[5],bytes[6],bytes[7]]);
  const lon=Math.floor(dt_lon / 100) + (dt_lon % 100) / 60;
  decoded.latitude=lon.toFixed(7);
   
  unixTimestamp = littleEndianBytesToFloat([bytes[8],bytes[9],bytes[10],bytes[11]]);
  
  decoded.timestamp = convertUnixTimestamp(unixTimestamp);
  
  const conversao    = littleEndianBytesToFloat([bytes[12],bytes[13],bytes[14],bytes[15]]);
  
  const voltageUc = conversao * (ADC_VOLTAGE_REF / (ADC_RESOLUTION - 1));
  decoded.analog=voltageUc.toFixed(3)
  
  decoded.digi0     = bytes[16];
  decoded.digi1     = bytes[17];
  
  const ntc0_vl           = littleEndianBytesToUint16([bytes[18],bytes[19]]);
  decoded.ntc0      = ntc_temperature(ntc0_vl,0).toFixed(1);
  
  const ntc1_vl           = littleEndianBytesToUint16([bytes[20],bytes[21]]);
  decoded.ntc1      = ntc_temperature(ntc1_vl,1).toFixed(1);
  
  const ntc2_vl      = littleEndianBytesToUint16([bytes[22],bytes[23]]);
  decoded.ntc2      = ntc_temperature(ntc2_vl,2).toFixed(1);
  
var decodedPayload = {
    "longitude"   : decoded.longitude,
    "latitude"    : decoded.latitude,
    "time-stamp"   : decoded.timestamp,
    "analog"      : decoded.analog,
    "dig-0"       : decoded.digi0,
    "dig-1"       : decoded.digi1,
    "ntc-0"        : decoded.ntc0,
    "ntc-1"        : decoded.ntc1,
    "ntc-2"        : decoded.ntc2,
};
  return Serialize(decodedPayload)
}
// Generated: do not touch unless your Google Form fields have changed
var field_mapping = {
  "longitude": "entry.1359999784",
  "latitude": "entry.1873862209",
  "time-stamp": "entry.81100085",
  "analog": "entry.393908294",
  "dig-0": "entry.1609835373",
  "dig-1": "entry.732934055",
  "ntc-0": "entry.378011195",
  "ntc-1": "entry.219951820",
  "ntc-2": "entry.1303722275"
};
// End Generated

function Serialize(payload) {
  var str = [];
  for (var key in payload) {
    if (payload.hasOwnProperty(key)) {
      var name = encodeURIComponent(field_mapping[key]);
      var value = encodeURIComponent(payload[key]);
      str.push(name + "=" + value);
    }
  }
  return str.join("&");
}
// DO NOT REMOVE: Google Form Function
  
*/