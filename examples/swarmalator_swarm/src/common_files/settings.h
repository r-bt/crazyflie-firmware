/**
 * @file settings.h
 *
 * @brief Settings for the decentralized swarm
 *
 * @author Richard Beattie
 * @date 09/03/2025
 */

#pragma once

#define MAX_ADDRESS 10 // all copter addresses must be between 0 and max(MAX_ADDRESS,9)

#define BROADCAST_FREQUENCY_HZ 10
#define BROADCAST_PERIOD_MS (1000 / BROADCAST_FREQUENCY_HZ)

#define SNIFFER_PRINT_FREQUENCY_HZ 1
#define SNIFFER_PRINT_PERIOD_MS (1000 / SNIFFER_PRINT_FREQUENCY_HZ)

#define CALC_NEXT_FREQUENCY_HZ 3
#define CALC_NEXT_PERIOD_MS (1000 / CALC_NEXT_FREQUENCY_HZ)

#define VOLTAGE_MIN 3.0f
#define VOLTAGE_MAX 4.2f