#include "includes/Protobuf/data_def_v0_pb.h"

//#define MaxBuf 164 para STACKSIZE 1024 'E O MAXIMO

#define MaxBuf 164


typedef struct buf_data_t {
	   void *fifo_reserved;
	   uint8_t  data[History_size];
	   uint16_t len;
}buf_data;



buf_data send_array_dd_v0(void);

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


/*

//JAVASCRIPT DECODER FUNCTION USED IN HELIUM PLATAFORM
//PAYLOAD 38 78 8D 45 F2 43 97 44 91 B4 C8 4E 00 E0 D1 45 00 00 00 00 00 00 00 00 00 80 00 D0 00 68
//RESULTS:
//"entry.1359999784=4127.02880859375&entry.1873862209=1310.1185302734375&entry.81100085=1683638400&entry.393908294=6716&entry.1609835373=0&entry.732934055=0&entry.378011195=10240&entry.219951820=0&entry.1303722275=0"


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

function Decoder(bytes,port){

   var decoded={};
   var nro = [bytes[0],bytes[1],bytes[2],bytes[3]];
      
  decoded.longitude = littleEndianBytesToFloat([bytes[0],bytes[1],bytes[2],bytes[3]]);
  decoded.latitude  = littleEndianBytesToFloat([bytes[4],bytes[5],bytes[6],bytes[7]]);
  decoded.timestamp = littleEndianBytesToFloat([bytes[8],bytes[9],bytes[10],bytes[11]]);
  decoded.analog    = littleEndianBytesToFloat([bytes[12],bytes[13],bytes[14],bytes[15]]);
  decoded.digi0     = littleEndianBytesToFloat([bytes[16],bytes[17],bytes[18],bytes[19]]);
  decoded.digi1     = littleEndianBytesToFloat([bytes[20],bytes[21],bytes[22],bytes[23]]);
  decoded.ntc0      = littleEndianBytesToUint16([bytes[24],bytes[25]]);
  decoded.ntc1 = littleEndianBytesToUint16((bytes[27] | (bytes[28] << 8)));
  decoded.ntc2 = littleEndianBytesToUint16((bytes[29] | (bytes[30] << 8)));
  
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