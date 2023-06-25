#include <302/CO_Prog.h>

#if ((CO_CONFIG_PROG) & CO_CONFIG_PROG_ENABLE)

/******************************************************************************/
CO_ReturnError_t CO_Prog_init(CO_Program_t * ProgObj,
                              OD_entry_t * OD_1F50_ProgramDownloadData,
                              OD_entry_t * OD_1F51_ProgramControl,
                              ODR_t OD_write_1F50(OD_stream_t * stream,
                                                  const void * buf,
                                                  OD_size_t count,
                                                  OD_size_t * countWritten),
                              ODR_t OD_write_1F51(OD_stream_t * stream,
                                                  const void * buf,
                                                  OD_size_t count,
                                                  OD_size_t * countWritten))
{
    /* verify arguments */
    if (ProgObj == NULL) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* configure extensions */
    if (OD_1F50_ProgramDownloadData != NULL) {
        ProgObj->OD_1F50_extensions.object = ProgObj;
        ProgObj->OD_1F50_extensions.read = OD_readOriginal;
        if(OD_write_1F50 != NULL) {
            ProgObj->OD_1F50_extensions.write = OD_write_1F50;
        } else {
            ProgObj->OD_1F50_extensions.write = OD_writeOriginal;
        }
        OD_extension_init(OD_1F50_ProgramDownloadData, &ProgObj->OD_1F50_extensions);
    }

    if (OD_1F51_ProgramControl != NULL) {
        ProgObj->OD_1F51_extensions.object = ProgObj;
        ProgObj->OD_1F51_extensions.read = OD_readOriginal;
        if(OD_write_1F51 != NULL) {
            ProgObj->OD_1F51_extensions.write = OD_write_1F51;
        } else {
            ProgObj->OD_1F51_extensions.write = OD_writeOriginal;
        }
        OD_extension_init(OD_1F51_ProgramControl, &ProgObj->OD_1F51_extensions);
    }

    return CO_ERROR_NO;
}

#endif /* ((CO_CONFIG_PROG) & CO_CONFIG_PROG_ENABLE) */
