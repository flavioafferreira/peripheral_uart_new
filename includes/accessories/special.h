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
void time_print (void);

float ntc_temperature(uint16_t conversao,uint8_t sensor_number);

