#include "nvm_cal.h"
#include "motor_config.h"
#include "qwr_FOC.h"
#include <stdio.h>
#include <stddef.h>
#include <string.h>

#define NVM_CAL_MAGIC    0xCA1BF0C1U
#define NVM_CAL_VERSION  1U

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    float    theta_offset_rad;
    uint32_t pole_pairs;
    int32_t  enc_dir;
    uint32_t crc32;
} nvm_cal_record_t;

_Static_assert(sizeof(nvm_cal_record_t) == 24U, "nvm_cal_record must be 24 bytes");

static const nvm_cal_record_t *nvm_flash_ptr(void)
{
    return (const nvm_cal_record_t *)(uintptr_t)NVM_CAL_FLASH_ADDR;
}

static uint32_t crc32_word(uint32_t crc, uint32_t data)
{
    crc ^= data;
    for (uint8_t i = 0U; i < 32U; i++) {
        if (crc & 1U) {
            crc = (crc >> 1) ^ 0xEDB88320U;
        } else {
            crc >>= 1U;
        }
    }
    return crc;
}

static uint32_t record_crc(const nvm_cal_record_t *rec)
{
    uint32_t crc = 0xFFFFFFFFU;
    const uint32_t *words = (const uint32_t *)rec;
    const size_t word_count = (sizeof(*rec) - sizeof(rec->crc32)) / sizeof(uint32_t);
    for (size_t i = 0U; i < word_count; i++) {
        crc = crc32_word(crc, words[i]);
    }
    return ~crc;
}

static int record_valid(const nvm_cal_record_t *rec)
{
    if (rec->magic != NVM_CAL_MAGIC || rec->version != NVM_CAL_VERSION) {
        return 0;
    }
    if (record_crc(rec) != rec->crc32) {
        return 0;
    }
    if (rec->pole_pairs != (uint32_t)MOTOR_POLE_PAIRS) {
        return 0;
    }
    if (rec->enc_dir != (int32_t)MOTOR_ENC_DIR) {
        return 0;
    }
    return 1;
}

static void fill_record(nvm_cal_record_t *rec, float theta_offset_rad)
{
    memset(rec, 0, sizeof(*rec));
    rec->magic             = NVM_CAL_MAGIC;
    rec->version           = NVM_CAL_VERSION;
    rec->theta_offset_rad  = theta_offset_rad;
    rec->pole_pairs        = (uint32_t)MOTOR_POLE_PAIRS;
    rec->enc_dir           = (int32_t)MOTOR_ENC_DIR;
    rec->crc32             = record_crc(rec);
}

void NVM_Cal_PrintLoadFail(void)
{
    const nvm_cal_record_t *rec = nvm_flash_ptr();

    if (rec->magic == 0xFFFFFFFFU || rec->magic == 0U) {
        printf("# cal: flash page empty — use 'make flash-dap-keep-cal' or 'cal align'+'cal save'\r\n");
        return;
    }
    if (rec->magic != NVM_CAL_MAGIC || rec->version != NVM_CAL_VERSION) {
        printf("# cal: bad magic/version (0x%08lX v%u)\r\n",
               (unsigned long)rec->magic, (unsigned)rec->version);
        return;
    }
    if (record_crc(rec) != rec->crc32) {
        printf("# cal: CRC mismatch\r\n");
        return;
    }
    if (rec->pole_pairs != (uint32_t)MOTOR_POLE_PAIRS) {
        printf("# cal: pole_pairs mismatch flash=%lu fw=%u (check CAN_NODE_ID / MOTOR_PROFILE)\r\n",
               (unsigned long)rec->pole_pairs, (unsigned)MOTOR_POLE_PAIRS);
        return;
    }
    if (rec->enc_dir != (int32_t)MOTOR_ENC_DIR) {
        printf("# cal: enc_dir mismatch flash=%ld fw=%d\r\n",
               (long)rec->enc_dir, (int)MOTOR_ENC_DIR);
        return;
    }
    printf("# cal: unknown load failure\r\n");
}

int NVM_Cal_TryLoad(float *theta_offset_rad)
{
    const nvm_cal_record_t *rec = nvm_flash_ptr();

    if (!record_valid(rec)) {
        return 0;
    }
    if (theta_offset_rad) {
        *theta_offset_rad = rec->theta_offset_rad;
    }
    return 1;
}

int NVM_Cal_Save(float theta_offset_rad)
{
    nvm_cal_record_t rec;
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0U;
    HAL_StatusTypeDef st;

    fill_record(&rec, theta_offset_rad);

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_BANK_1;
    erase.Page      = NVM_CAL_FLASH_PAGE;
    erase.NbPages   = 1U;

    st = HAL_FLASHEx_Erase(&erase, &page_error);
    if (st != HAL_OK) {
        HAL_FLASH_Lock();
        return -1;
    }

    const uint64_t *dw = (const uint64_t *)&rec;
    const size_t count = (sizeof(rec) + sizeof(uint64_t) - 1U) / sizeof(uint64_t);
    for (size_t i = 0U; i < count; i++) {
        const uint32_t addr = NVM_CAL_FLASH_ADDR + (uint32_t)(i * sizeof(uint64_t));
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, dw[i]);
        if (st != HAL_OK) {
            HAL_FLASH_Lock();
            return -2;
        }
    }

    HAL_FLASH_Lock();

    if (!record_valid(nvm_flash_ptr())) {
        return -3;
    }
    return 0;
}

void NVM_Cal_Show(float runtime_theta_rad)
{
    const nvm_cal_record_t *rec = nvm_flash_ptr();
    const int valid = record_valid(rec);

    if (valid) {
        printf("# cal flash: valid theta=%.6f rad (%.2f deg) pp=%lu dir=%ld\r\n",
               (double)rec->theta_offset_rad,
               (double)(rec->theta_offset_rad * 57.2957795f),
               (unsigned long)rec->pole_pairs,
               (long)rec->enc_dir);
    } else {
        printf("# cal flash: invalid or empty (magic=0x%08lX)\r\n",
               (unsigned long)rec->magic);
    }

    printf("# cal runtime: theta=%.6f rad aligned=%u\r\n",
           (double)runtime_theta_rad,
           (unsigned)g_foc.aligned);
}
