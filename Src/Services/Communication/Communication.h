#ifndef COMMUNICATION_H
#define COMMUNICATION_H

/**
 * @file Communication.h
 * @brief Header file for communication services in the Fitness Tracker project.
 * @author Ahtasham Baig
 * @date 2023
 */

#include <stdint.h>
#include <stdbool.h>

// Function prototypes
void Communication_Init(void);
void Communication_SendData(const uint8_t *data, uint16_t length);
bool Communication_ReceiveData(uint8_t *buffer, uint16_t bufferSize, uint16_t *receivedLength);

#endif // COMMUNICATION_H