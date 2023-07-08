/*****************************************************************************/
/* Reference implementation of a CRC calculation function */
/* */
/* gen_crc is the interface function which should be called from the */
/* application. There is also a stand-alone test mode that can be used */
/* if _RUN_MAIN is defined. */
/*****************************************************************************/
/*---------------------------------------------------------------------------*/
/* This file does NOT implement a general-purpose CRC function. */
/* Specifically, it does not handle parameterization by initial value, bit */
/* reflection, or final XOR value. This implementation is intended only to */
/* implement the CRC functions used by the linker for C28x CRC tables. The */
/* algorithms used by the linker are selected to match the CRC algorithms in */
/* the PRIME and IEEE 802.15.4-2006 standards, which use the polynomials */
/* supported by the C28x VCU hardware. To understand CRCs in general, */
/* especially what other parameters exist, see: */
/* */
/* "A Painless Guide To CRC Error Detection Algorithms" likely at: */
/* http://www.ross.net/crc/download/crc_v3.txt */
/* Author : Ross Williams (ross@guest.adelaide.edu.au.). */
/* Date : 3 June 1993. */
/* Status : Public domain (C code). */
/*---------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
/*---------------------------------------------------------------------------*/
/* These are the CRC algorithms supported by the linker, which match the */
/* polynomials supported in C28x VCU hardware, which match the PRIME and */
/* IEEE 802.15.4-2006 standards. These must match the values in crc_tbl.h. */
/*---------------------------------------------------------------------------*/
#define CRC32_PRIME 0
#define CRC16_802_15_4 1
#define CRC16_ALT 2
#define CRC8_PRIME 3

typedef struct crc_config_t {
    int id;
    int degree;
    unsigned long poly;
} crc_config_t;

const crc_config_t crc_config[] = {
    { CRC32_PRIME, 32, 0x04c11db7 },
    { CRC16_802_15_4, 16, 0x1021 },
    { CRC16_ALT, 16, 0x8005 },
    { CRC8_PRIME, 8, 0x07 }
};

unsigned long crc_table[256] = { 0 };

const crc_config_t *find_config(int id)
{
    size_t i;
    for (i = 0; i < sizeof(crc_config) / sizeof(*crc_config); i++) {
        if (crc_config[i].id == id) {
            return &crc_config[i];
        }
    }
    fprintf(stderr, "invalid config id %d\n", id);
    exit(EXIT_FAILURE);
    return NULL;
}


/*---------------------------------------------------------------------------*/
/* Table-driven version */
/*---------------------------------------------------------------------------*/
unsigned long generate_mask(int degree)
{
    unsigned long half = (1ul << (degree / 2)) - 1;
    return half << (degree / 2) | half;
}


void generate_crc_table(const crc_config_t *config)
{
    int i, j;
    unsigned long bit, crc;
    unsigned long high_bit = (1ul << (config->degree - 1));
    unsigned long mask = generate_mask(config->degree);
    for (i = 0; i < 256; i++) {
        crc = (unsigned long)i << config->degree - 8;
        for (j = 0; j < 8; j++) {
            bit = crc & high_bit;
            crc <<= 1;
            if (bit) {
                crc^= config->poly;
            }
        }
        crc_table[i] = crc & mask;
    }
}


/*****************************************************************************/
/* gen_crc - Return the CRC value for the data using the given CRC algorithm */
/* int id : identifies the CRC algorithm */
/* char *data : the data */
/* size_t len : the size of the data */
/*****************************************************************************/
unsigned long gen_crc(int id, const unsigned char *data, size_t len)
{
    /*-----------------------------------------------------------------------*/
    /* Note: this is not a general-purpose CRC function. It does not handle */
    /* parameterization by initial value, bit reflection, or final XOR */
    /* value. This CRC function is specialized to the CRC algorithms in the */
    /* linker used for C28x CRC tables. */
    /*-----------------------------------------------------------------------*/
    /*-----------------------------------------------------------------------*/
    /* This CRC function is not intended to be optimal; it is written such */
    /* that it works and generates the same result on all 8-bit and 16-bit */
    /* targets, including C28x, other TI DSPs, and typical desktops. */
    /*-----------------------------------------------------------------------*/
    const crc_config_t *config = find_config(id);
    unsigned long crc = 0;
    unsigned long mask = generate_mask(config->degree);
    size_t i;
    generate_crc_table(config);
    for (i = 0; i < len; i++) {
        unsigned int datum = data[i];
        /*--------------------------------------------------------------------*/
        /* This loop handles 16-bit chars when we compile on 16-bit machines. */
        /*--------------------------------------------------------------------*/
        int n;
        for (n = 0; n < (CHAR_BIT / 8); n++) {
            /*----------------------------------------------------------------*/
            /* For 16-bit machines, we need to feed the octets in an */
            /* arbitrary order. For C2000, the arbitrary order we choose is */
            /* to feed the LEAST significant octet of char 0 first. The */
            /* first octet fed to the CRC is the LEAST-significant octet of */
            /* char 0; the second octet is the MOST-significant octet of char */
            /* 0. See the "Special Note regarding 16-bit char" in the */
            /* Assembly Language Tools User's Guide. */
            /*----------------------------------------------------------------*/
#if __TMS320C28XX__
            /*----------------------------------------------------------------*/
            /* Using __byte is not necessary; we use it here to illustrate */
            /* how it relates to octet order. */
            /*----------------------------------------------------------------*/
            unsigned long octet = __byte((int*)&datum, n);
#else
            unsigned long octet = ((datum >> (8 * n)) & 0xff);
#endif
            unsigned long term1 = (crc << 8);
            int idx = ((crc >> (config->degree - 8)) & 0xff) ^ octet;
            crc = term1 ^ crc_table[idx];
        }
    }
    return crc & mask;
}


#ifdef _RUN_MAIN
/*****************************************************************************/
/* main - If requested, compute the CRC of test data using each algorithm. */
/*****************************************************************************/
int main(void)
{
#if CHAR_BIT == 16
    const unsigned char data[] = { 'a', 'b', 'c', 'd' };
#elif CHAR_BIT == 8
    /*-----------------------------------------------------------------------*/
    /* This represents "abcd" as it would appear in C2000 memory if we view */
    /* C2000 memory as octets, least-significant octet first; see "a special */
    /* note regarding 16-bit char" in Assembly Language Tools User's Guide. */
    /*-----------------------------------------------------------------------*/
    const unsigned char data[] = { 'a', 0, 'b', 0, 'c', 0, 'd', 0 };
#endif
    /* CRC_8_PRIME: 0x70 */
    /* CRC_16_802: 0x1bd3 */
    /* CRC_32_PRIME: 0x4beab53b */
    const unsigned char *p = (const unsigned char *)data;
    unsigned long crc;
    crc = gen_crc(CRC32_PRIME, p, sizeof data);
    printf("CRC_32_PRIME: %08lx\n", crc);
    crc = gen_crc(CRC8_PRIME, p, sizeof data);
    printf("CRC_8_PRIME: %02lx\n", crc);
    crc = gen_crc(CRC16_802_15_4, p, sizeof data);
    printf("CRC16_802_15_4: %04lx\n", crc);
    return 0;
}
#endif
