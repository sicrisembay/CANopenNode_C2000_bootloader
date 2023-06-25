#include "string.h"
#include "CO_Prog_F28335.h"
#include "OD.h"

#if (CO_CONFIG_PROG & CO_CONFIG_PROG_ENABLE)

static CO_Program_t program;

static ODR_t OD_write_1F50_callback(OD_stream_t * stream,
        const void * buf,
        OD_size_t count,
        OD_size_t * countWritten)
{
    ODR_t ret = ODR_OK;

    /*
     * Note1: Buffer size is equal to Page size of 256 words
     * Note2:
     */
    return ret;
}


static ODR_t OD_write_1F51_callback(OD_stream_t * stream,
        const void * buf,
        OD_size_t count,
        OD_size_t * countWritten)
{
    ODR_t ret = ODR_OK;

    return ret;
}


CO_ReturnError_t CO_Prog_F28335_init(CO_CANmodule_t * CANmodule)
{
    CO_ReturnError_t ret = CO_ERROR_NO;

    memset(&program, 0, sizeof(program));

    /* verify arguments */
    if (CANmodule == NULL) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* COnfigure object variables */
    program.CANmodule = CANmodule;

    ret = CO_Prog_init(&program,
                       OD_ENTRY_H1F50_downloadProgramData,
                       OD_ENTRY_H1F51_programControl,
                       OD_write_1F50_callback,
                       OD_write_1F51_callback);

    return ret;
}

#endif /* (CO_CONFIG_PROG & CO_CONFIG_PROG_ENABLE) */
