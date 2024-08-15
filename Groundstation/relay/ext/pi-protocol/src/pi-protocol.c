
#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "pi-protocol.h"
#include "pi-messages.h"

// --- sender --- //
#if (PI_MODE & PI_TX)
void piSendMsg(void * msg_raw, void (*serialWriter)(uint8_t byte)) {
    uint8_t buf[2*PI_MAX_PACKET_LEN]; // wasting some stack
    unsigned int num_bytes = piAccumulateMsg( msg_raw, buf );

    for (unsigned int i = 0; i < num_bytes; i++) {
        serialWriter(buf[i]);
    }
}
unsigned int piAccumulateMsg(void * msg_raw, uint8_t * buf) {
    // cast to uint8_t pointer
    uint8_t * msg = (uint8_t *) msg_raw;

    unsigned int i = 0;

    // start byte first. Next field (id) will be handled with the usual escape
    buf[i++] = PI_STX;

    // this is not that nice, because it assume that id and len are 1 byte
    bool id_sent = false;
    uint8_t id = *(msg++);
    uint8_t len = *(msg++) + 1 + 1; // one extra for id, one for checksum
    uint8_t checksum = id;

    while (len-- > 0) {
        uint8_t byte;
        if (!id_sent) {
            // send id
            byte = id;
            id_sent = true;
        } else if (len > 0) {
            // send message payload
            byte = *(msg++);
            checksum ^= byte;
        } else {
            // send checksum byte
            byte = checksum;
        }
        switch(byte) {
            case PI_STX:
                buf[i++] = PI_ESC;
                buf[i++] = PI_STX_ESC;
                break;
            case PI_ESC:
                buf[i++] = PI_ESC;
                buf[i++] = PI_ESC_ESC;
                break;
            default:
                buf[i++] = byte;
        }
    }
    return i;
}
#endif // #if (PI_MODE & PI_TX)

// --- parser ---
#if (PI_MODE & PI_RX)
__attribute__((unused)) uint8_t piParse(pi_parse_states_t * p, uint8_t byte) {
    uint8_t res = PI_MSG_NONE_ID;

    //p->piEscHit = false;
    //p->piState = PI_IDLE;
    //p->msgId = PI_MSG_NONE_ID;
    //p->byteCount = 0;
    p->msgParseResult = PI_PARSE_MSG_NO_ERROR;

#ifdef PI_STATS
    piStats[PI_PARSE_INVOKE]++;
#endif

    if (byte == PI_STX) {
#ifdef PI_STATS
        piStats[PI_STX_COUNT]++;
#endif
        p->piEscHit = false;
        p->piState = PI_STX_FOUND;
        return res;
    }

    if (p->piState == PI_IDLE)
        return res;

    if (p->piEscHit) {
        p->piEscHit = false;
        switch(byte) {
            case PI_STX_ESC:
                byte = PI_STX;
                break;
            case PI_ESC_ESC:
                byte = PI_ESC;
                break;
            default:
                // failure, next byte MUST have been PI_STX_ESC or PI_ESC_ESC
#ifdef PI_DEBUG
                printf("Escaping error: next byte was neither PI_STX_ESC nor PI_ESC_ESC, but rather %02hhX\n", byte);
#endif
#ifdef PI_STATS
                piStats[PI_ESC_ERROR]++;
#endif
                p->piState = PI_IDLE;
                return res;
        }
    } else {
        if (byte == PI_ESC) {
            p->piEscHit = true;
            return res;
        }
    }


    switch(p->piState) {
        case PI_IDLE: // can never happen, so fall through to satisfy GCC -Wall
            break;
        case PI_STX_FOUND:
            // parse id
            p->msgId = byte;
            p->checksum = byte;
            p->piState = PI_ID_FOUND;
            p->byteCount = 0;
            break;
        case PI_ID_FOUND:
            // payload time
            p->msgParseResult = piParseIntoMsg(p, byte);
            (p->byteCount)++;
            if (p->msgParseResult == PI_PARSE_MSG_SUCCESS) 
            {
                res = p->msgId;
#ifdef PI_STATS
                piStats[PI_SUCCESS]++;
#endif
            } 
            if (p->msgParseResult > PI_PARSE_MSG_SUCCESS) {
#ifdef PI_DEBUG
                printf("\n msgParseResult > PI_PARSE_MSG_SUCCESS at id 0x%02hhX, byte 0x%02hhX: %d\n", p->msgId, byte, p->msgParseResult);
#endif
                p->msgId = PI_MSG_NONE_ID;
                p->piState = PI_IDLE;
#ifdef PI_STATS
                piStats[p->msgParseResult]++;
#endif
            }
            break;
    }
    return res;
}

#ifdef PI_STATS
unsigned int piStats[NUM_PI_STATS_RESULT] = { 0,0,0,0, 0,0,0, 0,0,0 };

__attribute__((unused)) void piPrintStats(int (*printer)(const char * s, ...)) {
    static int i = 0;
    printer("\n+------------ piPrintStats invokation %d -------------+\n", i++);
    printer("|%-30s|%-10s|\n", "Result", "Occurence");
    printer("|------------------------------|----------|\n");
    for (int j=0; j < NUM_PI_STATS_RESULT; j++)
        printer("|%30s|%10d|\n", piStatsNames[j], piStats[j]);
    printer("|------------------------------|----------|\n");
}
#endif// #ifdef PI_STATS

#endif// #if (PI_MODE & PI_RX)


#ifdef __cplusplus
}
#endif
