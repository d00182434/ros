/**
 *
 */

#ifndef MESSAGE_TYPES_H_
#define MESSAGE_TYPES_H_

#define STOP_CMD 0xfffffffe

enum LatCommandType{
    CMD_UNKNOWN,
    CMD_DISCOVERY,
    CMD_TRANSPORT_START,
    CMD_TRANSPORT_STOP
};

#endif
