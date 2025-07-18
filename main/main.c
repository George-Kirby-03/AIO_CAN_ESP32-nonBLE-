#include <stdio.h>
#include <canPID.h>

const twai_filter_config_t can_pid_filters[] = {
    {
        .acceptance_code = 0x7e8 << 21,
        .acceptance_mask = ~(0x7ff << 21),
        .single_filter = 1
    },
    {
        .acceptance_code = 0x18daf110 << 3,
        .acceptance_mask = (0x1FFFFFFF << 3),
        .single_filter = 1,
    }
};

twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

twai_general_config_t pid_general_config =  TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_22, TWAI_MODE_NORMAL);

CAN_Data_handler car_settings;

uint8_t A_B(uint8_t data[]){
    return 44;
}

float A(uint8_t data[]){
    return 25.0f;
}

uint8_t list_size;

void app_main(void)
{
    PID_data **pid_list = NULL;
    PID_data test[] = 
        { CAN_PID_LIST_INT(3, A_B),
            CAN_PID_LIST_FLOAT(6, A),
            {{{0}}}
        };
    CAN_init(&car_settings, &t_config, can_pid_filters, &pid_general_config);
    PID_data_init(test, &pid_list, &list_size, &car_settings);
    while(1){
    if (CAN_loop(&car_settings, &pid_list, list_size) != ESP_OK) {
        ESP_LOGE("app_main", "CAN loop failed");
        break;
    }
    CAN_print_all_pids(&pid_list, list_size);
    vTaskDelay(pdMS_TO_TICKS(100));
    }
}