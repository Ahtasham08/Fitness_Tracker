#include "COMMON_CONFIG.h"
#include "parser_fitness_tracker.h"

void GenerateReport(char* report)
{
    
    static float adc_ksense = 0.0;
    // Create a new cJSON object
    cJSON *json = cJSON_CreateObject();
    //cJSON_AddNumberToObject(json, "tmp", ble_data.temperature);
    //cJSON_AddNumberToObject(json, "adc", ble_data.adc_sense);

    // Print the JSON object as a string
    char *json_string = cJSON_Print(json);
    if (json_string == NULL) {
        fprintf(stderr, "Failed to print JSON string\n");
        cJSON_Delete(json);
        return;
    }

    strcpy(report,json_string);
    // Free the allocated memory
    free(json_string);
    cJSON_Delete(json);

}

void ParseJson(char* str)
{
    //printf("Parsing %s",*str);
    cJSON *json = cJSON_Parse(str);
    if (json == NULL) {
        fprintf(stderr, "Error parsing JSON\n");
        return;
    }

    cJSON *configured = cJSON_GetObjectItem(json, "configured");
    cJSON *sync_tm = cJSON_GetObjectItem(json,"sync_tm");
    if (configured == NULL && sync_tm == NULL) {
        fprintf(stderr, "Error: 'configured' is not a number\n");
        cJSON_Delete(json);
        return;
    }
    if(configured != NULL)
    {
      //if(headerData.is_configured != configured->valueint)
      //{
      //  DeviceStatus.isDeviceStatusChanged = true;
      //}
      
    }
    else if(sync_tm != NULL)
    {
      uint32_t year, month, day, hour, minute, second;

        //Parse the provided time string
      //if (parse_time_string(sync_tm->valuestring, &year, &month, &day, &hour, &minute, &second))
      //{
      //    uint32_t timestamp = calculate_timestamp(year, month, day, hour, minute, second); // Use precise calculation
      //    set_timestamp(timestamp); // Set the timestamp
          
      //    DeviceStatus.TimeSynctime = timestamp;
      //    #if DEBUG_LOG_VERBOSITY == LOG_VERBOSE
      //    NRF_LOG_INFO("Time set to timestamp: %u\n", timestamp);
      //    #endif
      //}
      //nrf_setTime(sync_tm->valuestring);;
    
    }
    //SetLEDStatus(led_item->valueint);
    // Clean up and free the JSON object
    cJSON_Delete(json);
}

bool parse_time_string(const char *time_string, uint32_t *year, uint32_t *month, uint32_t *day,
                       uint32_t *hour, uint32_t *minute, uint32_t *second)
{
    // Validate input
    if (time_string == NULL || year == NULL || month == NULL || day == NULL ||
        hour == NULL || minute == NULL || second == NULL)
    {
        return false;
    }

    // Check string length (expected: "YYYY-MM-DDTHH:MM:SS" = 19 characters)
    if (strlen(time_string) != 19 || time_string[10] != 'T')
    {
        return false;
    }

    // Parse the string using sscanf
    if (sscanf(time_string, "%4u-%2u-%2uT%2u:%2u:%2u",
               year, month, day, hour, minute, second) != 6)
    {
        return false; // Parsing failed
    }

    // Validate ranges for each component
    if (*month < 1 || *month > 12 || *day < 1 || *day > 31 ||
        *hour > 23 || *minute > 59 || *second > 59)
    {
        return false; // Invalid values
    }

    return true;
}
