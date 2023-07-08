#ifndef CO_PROG_F28335_H
#define CO_PROG_F28335_H

#include "CO_Prog.h"

#if (CO_CONFIG_PROG & CO_CONFIG_PROG_ENABLE) || defined CO_DOXYGEN

CO_ReturnError_t CO_Prog_F28335_init(CO_CANmodule_t * CANmodule);
bool CheckAppCrc();
bool CheckAppAndJump();

#endif /* (CO_CONFIG_PROG & CO_CONFIG_PROG_ENABLE) */
#endif /* CO_PROG_F28335_H */
