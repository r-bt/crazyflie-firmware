/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * hello_world.c - App layer application of a simple hello world debug print every
 *   2 seconds.
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "HELLOWORLD"
#include "debug.h"
#include "uart2.h"
#include "log.h"

#define START_BYTE 0xAA
#define END_BYTE   0x55
#define NUM_FLOATS 3
#define PAYLOAD_SIZE (NUM_FLOATS * sizeof(float))
#define PACKET_SIZE  (1 + PAYLOAD_SIZE + 1)

void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");
  vTaskDelay(M2T(2000));
  uart2Init(9600);

  uint8_t data[PACKET_SIZE];
  float array[NUM_FLOATS];
  uint8_t b;

  while (1) {
    vTaskDelay(M2T(10));

    /* Wait for start byte */
    do {
      uart2Getchar((char *)&b);
    } while (b != START_BYTE);

    data[0] = b;

    /* Read payload */
    for (int i = 0; i < PAYLOAD_SIZE; i++) {
      uart2Getchar((char *)&data[1 + i]);
    }

    /* Read end byte */
    uart2Getchar((char *)&data[PACKET_SIZE - 1]);

    if (data[PACKET_SIZE - 1] != END_BYTE) {
      DEBUG_PRINT("UART frame error\n");
      continue;
    }

    /* Deserialize floats */
    for (int i = 0; i < NUM_FLOATS; i++) {
      memcpy(&array[i],
             &data[1 + i * sizeof(float)],
             sizeof(float));
    }

    DEBUG_PRINT("RX floats: %f %f %f\n",
                (double)array[0],
                (double)array[1],
                (double)array[2]);
  }
}
