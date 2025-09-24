#ifndef FITNESS_TRACKER_PARSER_H
#define FITNESS_TRACKER_PARSER_H

#include "COMMON_CONFIG.h"

void ParseJson(char* str);
void GenerateReport(char* report);
bool parse_time_string(const char *time_string, uint32_t *year, uint32_t *month, uint32_t *day,
                       uint32_t *hour, uint32_t *minute, uint32_t *second);

#endif