#include "board.h"
#include "periph/rtc.h"
#include "backup_reg.h"

static inline void backup_reg_disable_wp(void){
       /* Enable write access to RTC registers */
#if defined(CPU_FAM_STM32L4)
    periph_clk_en(APB1, RCC_APB1ENR1_PWREN);
#else
    periph_clk_en(APB1, RCC_APB1ENR_PWREN);
#endif
#if defined(CPU_FAM_STM32F7) || defined(CPU_FAM_STM32L4)
    PWR->CR1 |= PWR_CR1_DBP;
#else
    PWR->CR |= PWR_CR_DBP;
#endif
    /* Unlock RTC write protection */
    RTC->WPR = RTC_WRITE_PROTECTION_KEY1;
    RTC->WPR = RTC_WRITE_PROTECTION_KEY2;
}


static inline void backup_reg_enable_wp(void){
    /* Enable RTC write protection */
    RTC->WPR = RTC_WRITE_PROTECTION_WRONG_KEY;
}

int backup_reg_read(uint32_t idx, uint32_t *data){
    if (idx < BACKUP_REG_COUNT) {
        *data = *(&(RTC->BKP0R) + idx);
        return 0;
    }
    return -1;
}

int backup_reg_write(uint32_t idx, uint32_t val){
    if (idx < BACKUP_REG_COUNT) {
        backup_reg_disable_wp();
        *(&(RTC->BKP0R) + idx) = val;
        backup_reg_enable_wp();
        return 0;
    }
    return -1;
}

int backup_reg_write_u64(uint32_t idx, uint64_t val){
    if (idx < BACKUP_REG_COUNT) {
        backup_reg_disable_wp();
        *(&(RTC->BKP0R) + idx) = (val & 0xFFFFFFFF);
        *(&(RTC->BKP0R) + (idx + 1)) = (val >> 32);
        backup_reg_enable_wp();
        return 0;
    }
    return -1;
}

int backup_reg_read_u64(uint32_t idx, uint64_t *val){
    if (idx < BACKUP_REG_COUNT) {
        backup_reg_disable_wp();
        *val = *(&(RTC->BKP0R) + (idx + 1));
        *val = (*val << 32);
        *val |= *(&(RTC->BKP0R) + idx);
        backup_reg_enable_wp();
        return 0;
    }
    return -1;
}

/* this can be optimized further to require less read write accesses to the
   (slower) RTC backup registers */
int backup_reg_write_bytes(uint32_t byte_idx, uint8_t *data, size_t cnt){
    backup_reg_disable_wp();
    volatile uint32_t *backup_reg_base = &(RTC->BKP0R);
    uint32_t temp = 0;
    for (unsigned i = 0; i < cnt; i++) {
        uint32_t byte_num = (byte_idx + i) % 4;
        uint32_t reg_num = (byte_idx + i) / 4;
        temp = backup_reg_base[reg_num];
        temp = (temp & ~(0xFF << (byte_num * 8))) | ( data[i] << (byte_num * 8));
        backup_reg_base[reg_num] = temp;
    }
    backup_reg_enable_wp();
    return 0;
}

int backup_reg_read_bytes(uint32_t byte_idx, uint8_t *data, size_t cnt){
    volatile uint32_t *backup_reg_base = &(RTC->BKP0R);
    for (unsigned i = 0; i < cnt; i++) {
        uint32_t byte_num = (byte_idx + i) % 4;
        data[i] = (backup_reg_base[(byte_idx + i) / 4] >> (byte_num * 8))
                  & 0xFF;
    }
    return 0;
}
