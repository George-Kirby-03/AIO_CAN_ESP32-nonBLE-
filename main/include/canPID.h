#ifndef canPID_h
#define canPID_h
#include <common.h>

#define BOOL int8_t

#define CAN_PID_SENSOR_SETUP_STANDARD  \
    .extd = 0, \
    .rtr = 0, \
    .ss = 0,  \
    .self = 0, \
    .dlc_non_comp = 0, \
    .identifier = 0x7DF, \
    .data_length_code = 8,
    

 #define CAN_PID_SENSOR_SETUP_EXT  \
    .extd = 1, \
    .rtr = 0, \
    .ss = 0,  \
    .self = 0, \
    .dlc_non_comp = 0, \
    .identifier = 0x18DB33F1, \
    .data_length_code = 8,

#define CAN_PID_EMPTY_PID(PID)  \
    .f_data = 0x00, \
    .PID_index = PID, \
    .gen_func = NULL,  \
    .is_available = 1, 
    
#define CAN_PID_REQUEST(pid_to_request) (uint8_t[]){0x02, 0x01, pid_to_request, 0x55, 0x55, 0x55, 0x55, 0x55} 
#define PID_LIST_SIZE 200

typedef struct PID_data PID_data;
struct PID_data
{
    union
    {
        int8_t i_data;
        double d_data;
        float f_data;
    };
        uint8_t PID_index;
        esp_err_t (*gen_func)(PID_data *data);
        BOOL is_available;
};

typedef struct {
    twai_message_t sender_node;
    twai_message_t receiver_node;
    struct {
    uint8_t is_extended: 1;                /**< Extended frame format */
    uint8_t is_set: 1;                   /**< Indicates if the CAN bus is set up */
    uint8_t reserved: 6;
    };
} CAN_Data_handler;


extern twai_timing_config_t t_config;
extern const twai_filter_config_t can_pid_filters[];
extern twai_general_config_t can_pid_general_config[];

esp_err_t CAN_init(CAN_Data_handler *car_settings, twai_timing_config_t *t_config, twai_filter_config_t *filter_config, twai_general_config_t *general_config);
esp_err_t CAN_request(CAN_Data_handler *car_settings, uint8_t *data_send, uint8_t *data_expected, uint8_t mask_size, uint64_t mask, TickType_t timeout);
esp_err_t PID_data_init(PID_data *programed_pids, PID_data ***pid_list, CAN_Data_handler *car_settings);

#endif