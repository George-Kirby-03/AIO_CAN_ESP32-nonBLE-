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

void app_main(void)
{
    CAN_init(&car_settings, &t_config, can_pid_filters, &pid_general_config);
}