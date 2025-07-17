#include <canPID.h>
#include <common.h>

esp_err_t CAN_request(CAN_Data_handler *car_settings, uint8_t *data_send, uint8_t *data_expected, uint8_t mask_size, uint64_t mask, TickType_t timeout) {
    
    memcpy(car_settings->sender_node.data, data_send, 8); // Copy the request data into the sender node

    if (twai_transmit(&(car_settings->sender_node), pdMS_TO_TICKS(1000)) != ESP_OK) {
        ESP_LOGE("PID_data_init", "Failed to transmit initial message.");
        return ESP_FAIL;
    }

    TickType_t startTick = xTaskGetTickCount();

    while (xTaskGetTickCount() - startTick < pdMS_TO_TICKS(timeout)) {

        if (twai_receive(&(car_settings->receiver_node), pdMS_TO_TICKS(1000)) != ESP_OK) {
            ESP_LOGE("PID_data_init", "Failed to receive initial message.");
            return ESP_FAIL;
        }

        // Compare received data with expected data using the mask
        uint64_t actual = 0, expected = 0;
        for (int i = 0; i < mask_size; i++) {
            actual   |= ((uint64_t)car_settings->receiver_node.data[i]) << ((i * 8));
            expected |= ((uint64_t)data_expected[i]) << (i * 8);
        }

        if ((actual & mask) == (expected & mask)) {
            return ESP_OK;
        } else {
            ESP_LOGW("PID_data_init", "Received data doesn't match expected mask.");
        }
    }

    return ESP_ERR_TIMEOUT;  // Timeout if no valid response
}


esp_err_t CAN_init(CAN_Data_handler *car_settings, twai_timing_config_t *t_config, twai_filter_config_t *filter_config, twai_general_config_t *general_config)
{
   


    (*car_settings).sender_node = (twai_message_t){
        CAN_PID_SENSOR_SETUP_STANDARD
        .data = {0x02, 0x01, 0x00, 0x55, 0x55, 0x55, 0x55, 0x55}
    };
    car_settings->is_extended = 0; // Default to standard frame format


    uint32_t alerts;
    uint32_t alerts_to_enable = TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED;
   

    
    for (uint8_t i = 0; i < 2; i++) {

       if (twai_driver_install(general_config, t_config, &filter_config[i]) != ESP_OK) {
        ESP_LOGE("CAN_init", "Failed to install TWAI driver.");
        return ESP_FAIL;
         }
        else { 
            if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
        ESP_LOGI("CAN_init(alert reconfig)", "Alerts reconfigured");
            } else {
        ESP_LOGE("CAN_init(alert reconfig)", "Failed to reconfigure alerts");
        return ESP_FAIL;
        }
        twai_start();
        }
         
            if (twai_transmit(&(car_settings->sender_node), pdMS_TO_TICKS(1000)) == ESP_OK) {
                int res = twai_read_alerts(&alerts, pdMS_TO_TICKS(2000));
                if(res == ESP_OK) {
                   if (alerts & TWAI_ALERT_TX_SUCCESS) {
                         ESP_LOGI("CAN_init", "Message sent successfully");
                        if ((twai_receive(&(car_settings->receiver_node), pdMS_TO_TICKS(1000)) == ESP_OK) && (car_settings->receiver_node.data[2] == 0x00)) {
                               ESP_LOGI("CAN_init", "Message received successfully");
                               car_settings->is_set = 1; // Set the is_set flag to indicate CAN bus is set up
                               break;
                        }
                        else if (i == 0)  {
                            ESP_LOGE("CAN_init", "Failed to receive message back, TRYING 11bit ID");
                        }             
                        else {
                            ESP_LOGE("CAN_init", "Failed to receive on both attempts, stopping TWAI driver");
                        }    
                   }   
                    else {
                        ESP_LOGE("CAN_init", "Message transmission failed");
                        return ESP_FAIL;
                    }
                }
            
                else if ((res == ESP_ERR_TIMEOUT) && (i == 0)) {
                  ESP_LOGE("CAN_init", "Timeout while sending message (likely no other device awk), TRYING 11bit ID");
                } 
                
                else if (res == ESP_ERR_TIMEOUT) {
                    ESP_LOGE("CAN_init", "Timeout while sending message on both attempts");
                    return ESP_FAIL;  // Return failure if unable to send message
                }

                else {
                    ESP_LOGE("CAN_init", "Error on trasmissions: %s", esp_err_to_name(res));
                    return res;  // Return the error if reading alerts fails
                }
                
            }
            else {
                ESP_LOGE("CAN_init", "Failed to qeue message");
                return ESP_FAIL;
            }

            if (twai_stop() == ESP_OK) {
                                twai_driver_uninstall();
                                car_settings->sender_node = (twai_message_t){CAN_PID_SENSOR_SETUP_EXT .data = {0x02, 0x01, 0x00, 0x55, 0x55, 0x55, 0x55, 0x55}};
                            }
    }
        
if (car_settings->is_set == 0) {
    ESP_LOGE("CAN_init", "Failed to set up CAN bus after two attempts.");
    twai_driver_uninstall();  // Uninstall the TWAI driver if setup fails
    return ESP_FAIL;  // Return failure if unable to set up CAN bus
}
else {
car_settings->is_extended = car_settings->sender_node.extd; // Set the is_extended flag based on the message format
car_settings->is_extended = car_settings->sender_node.extd;
ESP_LOGI("CAN_init", "YAYYY, CAN bus initialized successfully with %s frame format", car_settings->is_extended ? "extended" : "standard");
}
return ESP_OK;
}

esp_err_t PID_data_init(PID_data *programed_pids, PID_data ***pid_list, CAN_Data_handler *car_settings)
{


    memcpy(car_settings->sender_node.data, CAN_PID_REQUEST(0x00), sizeof(CAN_PID_REQUEST(0x00))); // Create data request to find PIDs

    if(CAN_request(car_settings, CAN_PID_REQUEST(0x00), (uint8_t[]){0x00, 0x41, 0x00}, 3, 0b0, pdMS_TO_TICKS(3000)) != ESP_OK) {
        ESP_LOGE("PID_data_init", "Failed to request PID data.");
        return ESP_FAIL;  // Return error if request fails
    }

    uint32_t pid_count = 0;
    for (uint8_t i = 0; i < car_settings->sender_node.data[0] - 2; i++) {
        pid_count = pid_count << 8;  // Shift left to make space for the next PID
        pid_count |= car_settings->sender_node.data[i + 2];  // Combine the PID bytes into a single value
    }
    ESP_LOGI("PID_data_init", "Found %lu PIDs", pid_count);

    uint8_t pid_count_alt = __builtin_popcount(pid_count);  // Count the number of set bits in pid_count
    *pid_list = malloc(pid_count_alt * sizeof(PID_data*));  // Allocate memory for the list of PID_data pointers

   uint8_t index = 0;
   for (uint8_t i = 0; i < sizeof(pid_count); i++) {
        if (pid_count & (1 << i)) {
            uint8_t list_inc = 0;
            while(1){
                 if (programed_pids[list_inc].gen_func == NULL) {
                 ESP_LOGE("PID_data_init", "PID has no gen_func, stopping set PIDS");
                 (*pid_list)[index] = malloc(sizeof(PID_data));
                 *((*pid_list)[index]) = (PID_data){CAN_PID_EMPTY_PID(i)};  // Set the PID index
                 break;
                  }
                  else if (programed_pids[list_inc].PID_index == i) {
                    (*pid_list)[index] = &programed_pids[list_inc];  // Point to the existing PID_data
                  break;
                  }
                  else {
                    list_inc++;
                  }
                }
            index++;
         }
    }

    return ESP_OK;
}