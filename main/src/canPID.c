#include <canPID.h>
#include <common.h>






esp_err_t CAN_init(CAN_Data_handler *car_settings, twai_timing_config_t *t_config, twai_filter_config_t *filter_config, twai_general_config_t *general_config)
{
   


    (*car_settings).sender_node = (twai_message_t){
        CAN_PID_SENSOR_SETUP_STANDARD
        .data = {0x02, 0x01, 0x00, 0x55, 0x55, 0x55, 0x55, 0x55}
    };
    car_settings->is_extended = 0; // Default to standard frame format


    uint32_t alerts;
    uint32_t alerts_to_enable = TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED;
   

    twai_start();
    for (uint8_t i = 0; i < 2; i++) {
       if (twai_driver_install(general_config, t_config, &(filter_config[i])) != ESP_OK) {
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
              if(twai_read_alerts(&alerts, pdMS_TO_TICKS(1000)) == ESP_OK) {
                   if (alerts & TWAI_ALERT_TX_SUCCESS) {
                     ESP_LOGI("CAN_init", "Message sent successfully");
                         if (twai_receive(&(car_settings->receiver_node), pdMS_TO_TICKS(1000)) == ESP_OK) {
                               ESP_LOGI("CAN_init", "Message received successfully");
                               break;
                      } else if (i == 0)  {
                    ESP_LOGE("CAN_init", "Failed to receive message, TRYING 11bit ID");
                    if (twai_stop() == ESP_OK) {
                        twai_driver_uninstall();
                        car_settings->sender_node = (twai_message_t){CAN_PID_SENSOR_SETUP_EXT};
                    }                       
                     } else {
                        ESP_LOGE("CAN_init", "Failed to receive on both attempts, stopping TWAI driver");
                        if (twai_stop() == ESP_OK) {
                            twai_driver_uninstall();
                            return ESP_FAIL;  // Return failure if unable to receive any message
                        }
                     }
            } else if (alerts & TWAI_ALERT_TX_FAILED) {
                ESP_LOGE("CAN_init", "Message transmission failed");
                return ESP_FAIL;
            }
        } else {
            ESP_LOGE("CAN_init", "Failed to read alerts");
            return ESP_FAIL;
        }
    }
    else {
        ESP_LOGE("CAN_init", "Failed to qeue message");
        return ESP_FAIL;
    }

}
car_settings->is_extended = car_settings->sender_node.extd; // Set the is_extended flag based on the message format
return ESP_OK;
}

esp_err_t PID_data_init(PID_data *programed_pids, PID_data ***pid_list, CAN_Data_handler *car_settings)
{
    if (programed_pids == NULL || pid_list == NULL || car_settings == NULL) {
        ESP_LOGE("PID_data_init", "Invalid arguments provided.");
        return ESP_ERR_INVALID_ARG;  // Return error if any argument is NULL
    }

    memcpy(car_settings->sender_node.data, CAN_PID_REQUEST(0x00), sizeof(CAN_PID_REQUEST(0x00))); // Create data request to find PIDs

    if (twai_transmit(&(car_settings->sender_node), pdMS_TO_TICKS(1000)) != ESP_OK) {
        ESP_LOGE("PID_data_init", "Failed to transmit initial message.");
        return ESP_FAIL;  // Return error if transmission fails
    }
    if (twai_receive(&(car_settings->receiver_node), pdMS_TO_TICKS(1000)) != ESP_OK) {
        ESP_LOGE("PID_data_init", "Failed to receive initial message.");
        return ESP_FAIL;  // Return error if reception fails
    }
    
    *pid_list = malloc(PID_LIST_SIZE * sizeof(PID_data*));  // Allocate memory for the list of PID_data pointers
    for (uint8_t i = 0; i < PID_LIST_SIZE; i++) {
        if (programed_pids[i].is_available == 0) {
            (*pid_list)[i] = malloc(sizeof(PID_data)); //Create a new PID_data instance if not available
                if ((*pid_list)[i] == NULL) {
                    ESP_LOGE("PID_data_init", "Memory allocation failed for PID_data at index %d", i);
                    return ESP_ERR_NO_MEM;  // Return error if memory allocation fails
                }
        }
        else {
            (*pid_list)[i] = &programed_pids[i];  // Point to the existing PID_data
        }
    }

    return ESP_OK;
}