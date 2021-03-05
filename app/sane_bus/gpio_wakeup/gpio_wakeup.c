#include "board.h"
#include "xtimer.h"
#include "gpio_wakeup.h"
#include "periph/gpio.h"

int gpio_wakeup_pull_config(gpio_t pin, gpio_mode_t mode){
    periph_clk_en(APB1, RCC_APB1ENR1_PWREN); /* enable access to PWR interface */

    int port_num = ((pin >> 10) & 0x0f);
    int pin_num = (pin & 0x0f);

    switch(mode){
        case GPIO_IN_PD: /* configure pull down */
                         (&(PWR->PDCRA))[2*port_num] |= (1<<pin_num);
                         /* apply PDCRA configuration (apply pull down config)*/
                         PWR->CR3   |= PWR_CR3_APC;
                         return 0;

        case GPIO_IN_PU: /* configure pull up */
                         (&(PWR->PUCRA))[2*port_num] |= (1<<pin_num);
                         /* apply PUCRA configuration (apply pull up config)*/
                         PWR->CR3   |= PWR_CR3_APC;
                         return 0;

        case GPIO_IN:    /* configure no pull up and no pull down */
                         (&(PWR->PDCRA))[2*port_num] &= ~(1<<pin_num);
                         (&(PWR->PUCRA))[2*port_num] &= ~(1<<pin_num);
                         return 0;
        default:         return -1;
    }
}

int gpio_wakeup_enable(gpio_t pin, gpio_flank_t flank){
    uint8_t wkup_pin_idx = _get_wkup_idx(pin);

    if(wkup_pin_idx && (wkup_pin_idx <= WKUP_PIN_NUM)){
        periph_clk_en(APB1, RCC_APB1ENR1_PWREN); /* enable access to PWR interface */
        if (flank == GPIO_RISING) {
            PWR->CR4 &= ~(1<<(wkup_pin_idx-1)); /* 0: Detection on high level (rising edge)
                                                   1: Detection on low level (falling edge) */
        } else if (flank == GPIO_FALLING) {
            PWR->CR4 |=  (1<<(wkup_pin_idx-1)); /* 0: Detection on high level (rising edge)
                                                   1: Detection on low level (falling edge) */
        }else{
            return -1;
        }
        PWR->CR3 |= (1<<(wkup_pin_idx-1));   /* Enable Wakeup pin (additional function) */
        return 0;
    }
    return -2;
}

int gpio_wakeup_disable(gpio_t pin){
    uint8_t wkup_pin_idx = _get_wkup_idx(pin);

    if(wkup_pin_idx && (wkup_pin_idx <= WKUP_PIN_NUM)){
        periph_clk_en(APB1, RCC_APB1ENR1_PWREN); /* enable access to PWR interface */
        PWR->CR3 &= ~(1<<(wkup_pin_idx-1));   /* Enable Wakeup pin WKUP1 (additional function of PA0) */
        PWR->SCR |=  (1<<(wkup_pin_idx-1));   /* clear wakeup flag */
        return 0;
    }
    return -1;
}

int gpio_wakeup_clear(gpio_t pin){
    uint8_t wkup_pin_idx = _get_wkup_idx(pin);

    if(wkup_pin_idx && (wkup_pin_idx <= WKUP_PIN_NUM)){
        periph_clk_en(APB1, RCC_APB1ENR1_PWREN); /* enable access to PWR interface */
        PWR->SCR |=  (1<<(wkup_pin_idx-1));   /* clear wakeup flag */
        return 0;
    }

    return -1;
}
