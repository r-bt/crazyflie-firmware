#include "choose_app.h"
#ifdef BUILD_SNIFFER_APP

#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "P2P"

#include "debug.h"
#include "p2p_interface.h"
#include "settings.h"

void appMain()
{
    DEBUG_PRINT("SWARMALATOR SNIFFER running\n");

    initP2P();
    initPeerStates();

    while (1) {
        vTaskDelay(M2T(SNIFFER_PRINT_PERIOD_MS));
        printPeers();
    }
}

#endif // BUILD_SNIFFER_APP