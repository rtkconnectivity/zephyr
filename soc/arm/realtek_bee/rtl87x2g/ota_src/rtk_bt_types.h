#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>

#define APP_ERR                 0x0D00

#define APP_SUCCESS                             0x00
#define APP_ERR_PENDING                         0x01
#define APP_ERR_ACCEPT                          0x03
#define APP_ERR_REJECT                          0x04
#define APP_ERR_NOT_RELEASE                     0x05

/* Manufacturer specific error codes that are "missing" in GATT spec. >= 0xC0:   */
#define ATT_ERR_INVALID_CCC_BITS            0xC0 /**< Invalid client characteristic config bits. */
#define ATT_ERR_INVALID_SIGNED_COUNT        0xC1 /**< Invalid sign count. */
#define ATT_ERR_INVALID_SIGNED_MAC_FAILED   0xC2 /**< Invalid sign mac value. */
#define ATT_ERR_MIN_APPLIC_CODE             0xC3

#define ATT_ERR                 0x0400


/** @brief  APP Return Result List */
typedef enum
{
    APP_RESULT_SUCCESS                    = (APP_SUCCESS),
    APP_RESULT_PENDING                    = (APP_ERR | APP_ERR_PENDING),
    APP_RESULT_ACCEPT                     = (APP_ERR | APP_ERR_ACCEPT),
    APP_RESULT_REJECT                     = (APP_ERR | APP_ERR_REJECT),
    APP_RESULT_NOT_RELEASE                = (APP_ERR | APP_ERR_NOT_RELEASE),

    APP_RESULT_PREP_QUEUE_FULL            = (ATT_ERR | BT_ATT_ERR_PREPARE_QUEUE_FULL),
    APP_RESULT_INVALID_OFFSET             = (ATT_ERR | BT_ATT_ERR_INVALID_OFFSET),
    APP_RESULT_INVALID_VALUE_SIZE         = (ATT_ERR | BT_ATT_ERR_INVALID_ATTRIBUTE_LEN),
    APP_RESULT_INVALID_PDU                = (ATT_ERR | BT_ATT_ERR_INVALID_PDU),
    APP_RESULT_ATTR_NOT_FOUND             = (ATT_ERR | BT_ATT_ERR_ATTRIBUTE_NOT_FOUND),
    APP_RESULT_ATTR_NOT_LONG              = (ATT_ERR | BT_ATT_ERR_ATTRIBUTE_NOT_LONG),
    APP_RESULT_INSUFFICIENT_RESOURCES     = (ATT_ERR | BT_ATT_ERR_INSUFFICIENT_RESOURCES),
    APP_RESULT_VALUE_NOT_ALLOWED          = (ATT_ERR | BT_ATT_ERR_VALUE_NOT_ALLOWED),
    APP_RESULT_APP_ERR                    = (ATT_ERR | ATT_ERR_MIN_APPLIC_CODE),
    APP_RESULT_CCCD_IMPROPERLY_CONFIGURED = (ATT_ERR | BT_ATT_ERR_CCC_IMPROPER_CONF),
    APP_RESULT_PROC_ALREADY_IN_PROGRESS   = (ATT_ERR | BT_ATT_ERR_PROCEDURE_IN_PROGRESS),
} T_APP_RESULT;


typedef uint8_t T_SERVER_ID;    //!< Service ID

/** Calculate integer bit count of b'1 */
#define INT_BIT_COUNT(integer, count)   {               \
        count = 0;                                      \
        while (integer)                                 \
        {                                               \
            count++;                                    \
            integer &= integer - 1;                     \
        }                                               \
    }

/** Stream skip len */
#define STREAM_SKIP_LEN(s, len)     {                   \
        s += len;                                       \
    }

/** Stream to array */
#define STREAM_TO_ARRAY(a, s, len)  {                   \
        uint32_t ii;                                    \
        for (ii = 0; ii < len; ii++)                    \
        {                                               \
            *((uint8_t *)(a) + ii) = *s++;              \
        }                                               \
    }

/** Array to stream */
#define ARRAY_TO_STREAM(s, a, len)  {                   \
        uint32_t ii;                                    \
        for (ii = 0; ii < len; ii++)                    \
        {                                               \
            *s++ = *((uint8_t *)(a) + ii);              \
        }                                               \
    }

/** Little Endian stream to uint8 */
#define LE_STREAM_TO_UINT8(u8, s)   {                   \
        u8  = (uint8_t)(*s);                            \
        s  += 1;                                        \
    }

/** Little Endian stream to uint16 */
#define LE_STREAM_TO_UINT16(u16, s) {                   \
        u16  = ((uint16_t)(*(s + 0)) << 0) +            \
               ((uint16_t)(*(s + 1)) << 8);             \
        s   += 2;                                       \
    }

/** Little Endian stream to uint24 */
#define LE_STREAM_TO_UINT24(u24, s) {                   \
        u24  = ((uint32_t)(*(s + 0)) <<  0) +           \
               ((uint32_t)(*(s + 1)) <<  8) +           \
               ((uint32_t)(*(s + 2)) << 16);            \
        s   += 3;                                       \
    }

/** Little Endian stream to uint32 */
#define LE_STREAM_TO_UINT32(u32, s) {                   \
        u32  = ((uint32_t)(*(s + 0)) <<  0) +           \
               ((uint32_t)(*(s + 1)) <<  8) +           \
               ((uint32_t)(*(s + 2)) << 16) +           \
               ((uint32_t)(*(s + 3)) << 24);            \
        s   += 4;                                       \
    }

/** Little Endian stream to uint64 */
#define LE_STREAM_TO_UINT64(u64, s) {                   \
        u64  = ((uint64_t)(*(s + 0)) <<  0) +           \
               ((uint64_t)(*(s + 1)) <<  8) +           \
               ((uint64_t)(*(s + 2)) << 16) +           \
               ((uint64_t)(*(s + 3)) << 24) +           \
               ((uint64_t)(*(s + 4)) << 32) +           \
               ((uint64_t)(*(s + 5)) << 40) +           \
               ((uint64_t)(*(s + 6)) << 48) +           \
               ((uint64_t)(*(s + 7)) << 56);            \
        s   += 8;                                       \
    }

/** Little Endian uint8 to stream */
#define LE_UINT8_TO_STREAM(s, u8)   {                   \
        *s++ = (uint8_t)(u8);                           \
    }

/** Little Endian uint16 to stream */
#define LE_UINT16_TO_STREAM(s, u16) {                   \
        *s++ = (uint8_t)((u16) >> 0);                   \
        *s++ = (uint8_t)((u16) >> 8);                   \
    }

/** Little Endian uint24 to stream */
#define LE_UINT24_TO_STREAM(s, u24) {                   \
        *s++ = (uint8_t)((u24) >>  0);                  \
        *s++ = (uint8_t)((u24) >>  8);                  \
        *s++ = (uint8_t)((u24) >> 16);                  \
    }

/** Little Endian uint32 to stream */
#define LE_UINT32_TO_STREAM(s, u32) {                   \
        *s++ = (uint8_t)((u32) >>  0);                  \
        *s++ = (uint8_t)((u32) >>  8);                  \
        *s++ = (uint8_t)((u32) >> 16);                  \
        *s++ = (uint8_t)((u32) >> 24);                  \
    }

/** Little Endian uint64 to stream */
#define LE_UINT64_TO_STREAM(s, u64) {                   \
        *s++ = (uint8_t)((u64) >>  0);                  \
        *s++ = (uint8_t)((u64) >>  8);                  \
        *s++ = (uint8_t)((u64) >> 16);                  \
        *s++ = (uint8_t)((u64) >> 24);                  \
        *s++ = (uint8_t)((u64) >> 32);                  \
        *s++ = (uint8_t)((u64) >> 40);                  \
        *s++ = (uint8_t)((u64) >> 48);                  \
        *s++ = (uint8_t)((u64) >> 56);                  \
    }

/** Little Endian array to uint8 */
#define LE_ARRAY_TO_UINT8(u8, a)    {                   \
        u8  = (uint8_t)(*(a + 0));                      \
    }

/** Little Endian array to uint16 */
#define LE_ARRAY_TO_UINT16(u16, a)  {                   \
        u16 = ((uint16_t)(*(a + 0)) << 0) +             \
              ((uint16_t)(*(a + 1)) << 8);              \
    }

/** Little Endian array to uint24 */
#define LE_ARRAY_TO_UINT24(u24, a)  {                   \
        u24 = ((uint32_t)(*(a + 0)) <<  0) +            \
              ((uint32_t)(*(a + 1)) <<  8) +            \
              ((uint32_t)(*(a + 2)) << 16);             \
    }

/** Little Endian array to uint32 */
#define LE_ARRAY_TO_UINT32(u32, a) {                    \
        u32 = ((uint32_t)(*(a + 0)) <<  0) +            \
              ((uint32_t)(*(a + 1)) <<  8) +            \
              ((uint32_t)(*(a + 2)) << 16) +            \
              ((uint32_t)(*(a + 3)) << 24);             \
    }

/** Little Endian array to uint64 */
#define LE_ARRAY_TO_UINT64(u64, a) {                    \
        u64 = ((uint64_t)(*(a + 0)) <<  0) +            \
              ((uint64_t)(*(a + 1)) <<  8) +            \
              ((uint64_t)(*(a + 2)) << 16) +            \
              ((uint64_t)(*(a + 3)) << 24) +            \
              ((uint64_t)(*(a + 4)) << 32) +            \
              ((uint64_t)(*(a + 5)) << 40) +            \
              ((uint64_t)(*(a + 6)) << 48) +            \
              ((uint64_t)(*(a + 7)) << 56);             \
    }

/** Little Endian uint8 to array */
#define LE_UINT8_TO_ARRAY(a, u8)    {                   \
        *((uint8_t *)(a) + 0) = (uint8_t)(u8);          \
    }

/** Little Endian uint16 to array */
#define LE_UINT16_TO_ARRAY(a, u16)  {                   \
        *((uint8_t *)(a) + 0) = (uint8_t)((u16) >> 0);  \
        *((uint8_t *)(a) + 1) = (uint8_t)((u16) >> 8);  \
    }

/** Little Endian uint24 to array */
#define LE_UINT24_TO_ARRAY(a, u24) {                    \
        *((uint8_t *)(a) + 0) = (uint8_t)((u24) >>  0); \
        *((uint8_t *)(a) + 1) = (uint8_t)((u24) >>  8); \
        *((uint8_t *)(a) + 2) = (uint8_t)((u24) >> 16); \
    }

/** Little Endian uint32 to array */
#define LE_UINT32_TO_ARRAY(a, u32) {                    \
        *((uint8_t *)(a) + 0) = (uint8_t)((u32) >>  0); \
        *((uint8_t *)(a) + 1) = (uint8_t)((u32) >>  8); \
        *((uint8_t *)(a) + 2) = (uint8_t)((u32) >> 16); \
        *((uint8_t *)(a) + 3) = (uint8_t)((u32) >> 24); \
    }

/** Little Endian uint64 to array */
#define LE_UINT64_TO_ARRAY(a, u64) {                    \
        *((uint8_t *)(a) + 0) = (uint8_t)((u64) >>  0); \
        *((uint8_t *)(a) + 1) = (uint8_t)((u64) >>  8); \
        *((uint8_t *)(a) + 2) = (uint8_t)((u64) >> 16); \
        *((uint8_t *)(a) + 3) = (uint8_t)((u64) >> 24); \
        *((uint8_t *)(a) + 4) = (uint8_t)((u64) >> 32); \
        *((uint8_t *)(a) + 5) = (uint8_t)((u64) >> 40); \
        *((uint8_t *)(a) + 6) = (uint8_t)((u64) >> 48); \
        *((uint8_t *)(a) + 7) = (uint8_t)((u64) >> 56); \
    }

/** Big Endian stream to uint8 */
#define BE_STREAM_TO_UINT8(u8, s)   {                   \
        u8   = (uint8_t)(*(s + 0));                     \
        s   += 1;                                       \
    }

/** Big Endian stream to uint16 */
#define BE_STREAM_TO_UINT16(u16, s) {                   \
        u16  = ((uint16_t)(*(s + 0)) << 8) +            \
               ((uint16_t)(*(s + 1)) << 0);             \
        s   += 2;                                       \
    }

/** Big Endian stream to uint24 */
#define BE_STREAM_TO_UINT24(u24, s) {                   \
        u24  = ((uint32_t)(*(s + 0)) << 16) +           \
               ((uint32_t)(*(s + 1)) <<  8) +           \
               ((uint32_t)(*(s + 2)) <<  0);            \
        s   += 3;                                       \
    }

/** Big Endian stream to uint32 */
#define BE_STREAM_TO_UINT32(u32, s) {                   \
        u32  = ((uint32_t)(*(s + 0)) << 24) +           \
               ((uint32_t)(*(s + 1)) << 16) +           \
               ((uint32_t)(*(s + 2)) <<  8) +           \
               ((uint32_t)(*(s + 3)) <<  0);            \
        s   += 4;                                       \
    }

/** Big Endian stream to uint64 */
#define BE_STREAM_TO_UINT64(u64, s) {                   \
        u64  = ((uint64_t)(*(s + 0)) << 56) +           \
               ((uint64_t)(*(s + 1)) << 48) +           \
               ((uint64_t)(*(s + 2)) << 40) +           \
               ((uint64_t)(*(s + 3)) << 32) +           \
               ((uint64_t)(*(s + 4)) << 24) +           \
               ((uint64_t)(*(s + 5)) << 16) +           \
               ((uint64_t)(*(s + 6)) <<  8) +           \
               ((uint64_t)(*(s + 7)) <<  0);            \
        s   += 8;                                       \
    }

/** Big Endian uint8 to stream */
#define BE_UINT8_TO_STREAM(s, u8)   {                   \
        *s++ = (uint8_t)(u8);                           \
    }

/** Big Endian uint16 to stream */
#define BE_UINT16_TO_STREAM(s, u16) {                   \
        *s++ = (uint8_t)((u16) >> 8);                   \
        *s++ = (uint8_t)((u16) >> 0);                   \
    }

/** Big Endian uint24 to stream */
#define BE_UINT24_TO_STREAM(s, u24) {                   \
        *s++ = (uint8_t)((u24) >> 16);                  \
        *s++ = (uint8_t)((u24) >>  8);                  \
        *s++ = (uint8_t)((u24) >>  0);                  \
    }

/** Big Endian uint32 to stream */
#define BE_UINT32_TO_STREAM(s, u32) {                   \
        *s++ = (uint8_t)((u32) >> 24);                  \
        *s++ = (uint8_t)((u32) >> 16);                  \
        *s++ = (uint8_t)((u32) >>  8);                  \
        *s++ = (uint8_t)((u32) >>  0);                  \
    }

/** Big Endian uint64 to stream */
#define BE_UINT64_TO_STREAM(s, u64) {                   \
        *s++ = (uint8_t)((u64) >> 56);                  \
        *s++ = (uint8_t)((u64) >> 48);                  \
        *s++ = (uint8_t)((u64) >> 40);                  \
        *s++ = (uint8_t)((u64) >> 32);                  \
        *s++ = (uint8_t)((u64) >> 24);                  \
        *s++ = (uint8_t)((u64) >> 16);                  \
        *s++ = (uint8_t)((u64) >>  8);                  \
        *s++ = (uint8_t)((u64) >>  0);                  \
    }

/** Big Endian array to uint8 */
#define BE_ARRAY_TO_UINT8(u8, a)    {                   \
        u8  = (uint8_t)(*(a + 0));                      \
    }

/** Big Endian array to uint16 */
#define BE_ARRAY_TO_UINT16(u16, a)  {                   \
        u16 = ((uint16_t)(*(a + 0)) << 8) +             \
              ((uint16_t)(*(a + 1)) << 0);              \
    }

/** Big Endian array to uint24 */
#define BE_ARRAY_TO_UINT24(u24, a)  {                   \
        u24 = ((uint32_t)(*(a + 0)) << 16) +            \
              ((uint32_t)(*(a + 1)) <<  8) +            \
              ((uint32_t)(*(a + 2)) <<  0);             \
    }

/** Big Endian array to uint32 */
#define BE_ARRAY_TO_UINT32(u32, a)  {                   \
        u32 = ((uint32_t)(*(a + 0)) << 24) +            \
              ((uint32_t)(*(a + 1)) << 16) +            \
              ((uint32_t)(*(a + 2)) <<  8) +            \
              ((uint32_t)(*(a + 3)) <<  0);             \
    }

/** Big Endian array to uint64 */
#define BE_ARRAY_TO_UINT64(u64, a)  {                   \
        u64 = ((uint64_t)(*(a + 0)) << 56) +            \
              ((uint64_t)(*(a + 1)) << 48) +            \
              ((uint64_t)(*(a + 2)) << 40) +            \
              ((uint64_t)(*(a + 3)) << 32) +            \
              ((uint64_t)(*(a + 4)) << 24) +            \
              ((uint64_t)(*(a + 5)) << 16) +            \
              ((uint64_t)(*(a + 6)) <<  8) +            \
              ((uint64_t)(*(a + 7)) <<  0);             \
    }

/** Big Endian uint8 to array */
#define BE_UINT8_TO_ARRAY(a, u8)    {                   \
        *((uint8_t *)(a) + 0) = (uint8_t)(u8);          \
    }

/** Big Endian uint16 to array */
#define BE_UINT16_TO_ARRAY(a, u16)  {                   \
        *((uint8_t *)(a) + 0) = (uint8_t)((u16) >> 8);  \
        *((uint8_t *)(a) + 1) = (uint8_t)((u16) >> 0);  \
    }

/** Big Endian uint24 to array */
#define BE_UINT24_TO_ARRAY(a, u24)  {                   \
        *((uint8_t *)(a) + 0) = (uint8_t)((u24) >> 16); \
        *((uint8_t *)(a) + 1) = (uint8_t)((u24) >>  8); \
        *((uint8_t *)(a) + 2) = (uint8_t)((u24) >>  0); \
    }

/** Big Endian uint32 to array */
#define BE_UINT32_TO_ARRAY(a, u32)  {                   \
        *((uint8_t *)(a) + 0) = (uint8_t)((u32) >> 24); \
        *((uint8_t *)(a) + 1) = (uint8_t)((u32) >> 16); \
        *((uint8_t *)(a) + 2) = (uint8_t)((u32) >>  8); \
        *((uint8_t *)(a) + 3) = (uint8_t)((u32) >>  0); \
    }

/** Big Endian uint64 to array */
#define BE_UINT64_TO_ARRAY(a, u64)  {                   \
        *((uint8_t *)(a) + 0) = (uint8_t)((u64) >> 56); \
        *((uint8_t *)(a) + 1) = (uint8_t)((u64) >> 48); \
        *((uint8_t *)(a) + 2) = (uint8_t)((u64) >> 40); \
        *((uint8_t *)(a) + 3) = (uint8_t)((u64) >> 32); \
        *((uint8_t *)(a) + 4) = (uint8_t)((u64) >> 24); \
        *((uint8_t *)(a) + 5) = (uint8_t)((u64) >> 16); \
        *((uint8_t *)(a) + 6) = (uint8_t)((u64) >>  8); \
        *((uint8_t *)(a) + 7) = (uint8_t)((u64) >>  0); \
    }
