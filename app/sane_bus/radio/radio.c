/*
 * Copyright (C) 2019 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     radio
 * @{
 *
 * @file
 * @brief       Radio control interface
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * @}
 */
 #include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#ifdef UPLINK_LOWPAN_ENABLED
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#endif
#include <unistd.h>

#include "xtimer.h"
#include "net/netopt.h"
#include "net/gnrc.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/sixlowpan.h"
#include "net/gnrc/netif.h"
#include "net/gnrc/netif/hdr.h"
#include "net/gnrc/udp.h"
#include "net/gnrc/pktbuf.h"
#include "net/gnrc/pktdump.h"
#include "net/gnrc/netif/ieee802154.h"
#include "net/gnrc/netif/internal.h"
#include "net/gnrc/netif.h"
#include "thread.h"
#include "kernel_types.h"
#include "msg.h"
#include "control_messages.h"
#define LOG_LEVEL LOG_NONE
#include "log.h"
#include "radio.h"
#include "backup_reg.h"
#include "backup_reg_alloc.h"
#include "gpio_wakeup.h"
#include "node_time.h"

char radio_thread_stack[THREAD_STACKSIZE_RADIO];
static msg_t msg_queue[THREAD_MSG_QUEUE_SIZE_RADIO];

#ifdef MODULE_AT86RF2XX
#include "at86rf2xx.h"
#include "at86rf2xx_params.h"

/**
 * @brief   Define stack parameters for the MAC layer thread
 * @{
 */
#define AT86RF2XX_MAC_STACKSIZE     (THREAD_STACKSIZE_DEFAULT)
#ifndef AT86RF2XX_MAC_PRIO
#define AT86RF2XX_MAC_PRIO          (GNRC_NETIF_PRIO)
#endif

#define AT86RF2XX_NUM (sizeof(at86rf2xx_params) / sizeof(at86rf2xx_params[0]))

static at86rf2xx_t at86rf2xx_devs[AT86RF2XX_NUM];
static char _at86rf2xx_stacks[AT86RF2XX_NUM][AT86RF2XX_MAC_STACKSIZE];
#endif /* MODULE_AT86RF2XX */

#ifdef MODULE_SX127X
#include "log.h"
#include "board.h"
#ifdef MODULE_GNRC_LORAWAN
#define LORAWAN_PORT (2U)
#include "net/gnrc/netif/lorawan_base.h"
#else
#include "net/gnrc/netif/raw.h"
#endif
#include "net/gnrc.h"

#include "sx127x.h"
#include "sx127x_params.h"

gnrc_netif_t *lora_netif;

#if !defined(MODULE_GNRC_LORAWAN) || doxygen
/**
 * @brief   Calculate the number of configured SX127x devices
 */
#define SX127X_NUMOF        (sizeof(sx127x_params) / sizeof(sx127x_params_t))
#else
/* GNRC LoRaWAN only supports one interface so far */
#define SX127X_NUMOF        (1)
#endif

/**
 * @brief   Define stack parameters for the MAC layer thread
 */
#define SX127X_STACKSIZE           (THREAD_STACKSIZE_DEFAULT)
#ifndef SX127X_PRIO
#define SX127X_PRIO                (GNRC_NETIF_PRIO)
#endif /* MODULE_SX127X */

/**
 * @brief   Allocate memory for device descriptors, stacks, and GNRC adaption
 */
sx127x_t sx127x_devs[SX127X_NUMOF];
char sx127x_stacks[SX127X_NUMOF][SX127X_STACKSIZE];

#define UPSTATE_POLL_INTERVAL_MS (200)

static int _netapi_up(kernel_pid_t pid, uint32_t timeout_us)
{
    netopt_t opt = NETOPT_LINK_CONNECTED;
    netopt_enable_t opt_val = NETOPT_ENABLE;
    uint16_t ctx = 0;

    if (gnrc_netapi_set(pid, opt, ctx, &opt_val, sizeof(opt_val)) < 0) {
        LOG_ERROR("_netapi_up error: unable to set opt");
        return 1;
    }
    else {
        LOG_DEBUG("enabled raio...\n");
    }

    for (uint32_t tried_us = 0; tried_us < timeout_us; tried_us += UPSTATE_POLL_INTERVAL_MS * 1000) {
        LOG_DEBUG("waiting for interface to come up...\n");
        gnrc_netapi_get(pid, NETOPT_LINK_CONNECTED, 0,
                        &opt_val, 1);
        if (opt_val == NETOPT_ENABLE) {
            LOG_DEBUG("interface up!\n");
            return 0;
        }
        xtimer_usleep(UPSTATE_POLL_INTERVAL_MS * 1000);
    }

    return -1;
}

int gnrc_lorawan_save_cb(uint8_t pos, uint8_t *buf, size_t len) {
    LOG_DEBUG("[radio] gnrc_lorawan_save_cb pos: %d, len: %d\n", pos, len);
    for (size_t i = 0; i < len; i++) {
        LOG_DEBUG("0x%02X ", buf[i]);
    }
    LOG_DEBUG("\n");

    if(len > 0) {
        backup_reg_write_bytes(BACKUP_REG_LORA_0 * sizeof(uint32_t) + pos,
                               buf, len);
    }

    return len;
}

int gnrc_lorawan_restore_cb(uint8_t pos, uint8_t *buf, size_t len) {
    LOG_DEBUG("[radio] gnrc_lorawan_restore_cb pos: %d, len: %d\n", pos, len);

    uint32_t sys_state = 0;
    backup_reg_read(BACKUP_REG_SYS_STATE, &sys_state);

    if (sys_state & SYS_STATE_LORA_JOINED) {
        if(len > 0) {
            LOG_DEBUG("restoring %d bytes...\n", len);
            backup_reg_read_bytes(BACKUP_REG_LORA_0 * sizeof(uint32_t) + pos,
                                  buf, len);

            for (size_t i = 0; i < len; i++) {
               LOG_DEBUG("0x%02X ", buf[i]);
            }
            LOG_DEBUG("\n");
        }
        LOG_DEBUG("nothing to restore...\n");
        return len;
    }
    LOG_DEBUG("wasn't joined before... not restoring!\n");
    return -1;
}

static int _lora_tx(gnrc_netif_t *lora_netif, uint8_t port, uint8_t *data,
                    size_t len, uint32_t *toa)
{
    gnrc_pktsnip_t *pkt;

    if (port == 0 || port >= 224) {
        LOG_ERROR("error: invalid port given '%d', "
               "port can only be between 1 and 223\n", port);
        return 1;
    }

    pkt = gnrc_pktbuf_add(NULL, data, len, GNRC_NETTYPE_UNDEF);

    gnrc_netapi_set(lora_netif->pid, NETOPT_TX_PORT, 0, &port, sizeof(port));
    gnrc_netapi_send(lora_netif->pid, pkt);

    /* replace this with sync tx handling */
    gnrc_lorawan_t *mac = &lora_netif->lorawan.mac;

    while (mac->busy) {
        xtimer_usleep(UPSTATE_POLL_INTERVAL_MS * 1000);
    }

    LOG_DEBUG("[radio] LoRa time-on-air was: %lu\n", mac->toa);

    *toa = mac->toa;

    return 0;
}

void _radio_pins_set_to_power_down(void)
{
    gpio_init(SX1272_SCK_PIN, GPIO_IN_PD);
    gpio_init(SX1272_MISO_PIN, GPIO_IN_PD);
    gpio_init(SX1272_MOSI_PIN, GPIO_IN_PD);
    gpio_init(SX1272_CS_PIN, GPIO_IN_PD);
    gpio_init(SX1272_DIO0_PIN, GPIO_IN_PD);
    gpio_init(SX1272_DIO1_PIN, GPIO_IN_PD);
    gpio_init(SX1272_DIO2_PIN, GPIO_IN_PD);
    gpio_init(SX1272_DIO3_PIN, GPIO_IN_PD);

    gpio_wakeup_pull_config(SX1272_SCK_PIN, GPIO_IN_PD);
    gpio_wakeup_pull_config(SX1272_MISO_PIN, GPIO_IN_PD);
    gpio_wakeup_pull_config(SX1272_MOSI_PIN, GPIO_IN_PD);
    gpio_wakeup_pull_config(SX1272_CS_PIN, GPIO_IN_PD);
    gpio_wakeup_pull_config(SX1272_DIO0_PIN, GPIO_IN_PD);
    gpio_wakeup_pull_config(SX1272_DIO1_PIN, GPIO_IN_PD);
    gpio_wakeup_pull_config(SX1272_DIO2_PIN, GPIO_IN_PD);
    gpio_wakeup_pull_config(SX1272_DIO3_PIN, GPIO_IN_PD);
}

void radio_quick_power_down(void)
{
    sx127x_setup(&sx127x_devs[0], &sx127x_params[0]);
    sx127x_init(&sx127x_devs[0]);
    _radio_pins_set_to_power_down();
}
#endif /* MODULE_GNRC_LORAWAN */

void _radio_power_off(void){
  netopt_state_t new_state = NETOPT_STATE_RESET;
  gnrc_netif_t* netif = NULL;

  while((netif = gnrc_netif_iter(netif)) != NULL){
    LOG_DEBUG("going to shut down radio %d\n", netif->pid);

    new_state = NETOPT_STATE_OFF;
    int resu = gnrc_netapi_set(netif->pid, NETOPT_STATE, 0, &new_state, sizeof(netopt_state_t));
    if(resu >= 0 ){
      LOG_DEBUG("set radio to OFF!\n");
    }else{
      LOG_DEBUG("setting radio to OFF failed %d\n", resu);
    }

  }

#ifdef MODULE_AT86RF2XX
  /* disable interrupt to avoid entering ISR when powered off */
  gpio_irq_disable(at86rf2xx_params[0].int_pin);
#endif

/* This *dirty* hack can be used to disable and re-enable during runtime
   without reset inbetween */
//  netif = NULL;
//  while( (netif = gnrc_netif_iter(netif)) != NULL){
//      /* block others from doing anything with this netif to prevent Lockup
//         after powering down the radio */
//      //gnrc_netif_acquire(netif);
//      // dirty hack to remove thread at runtime
//      unsigned int state = irq_disable();
//      thread_t *thread = (thread_t *)thread_get(netif->pid);
//      sched_set_status(thread, STATUS_STOPPED);
//      sched_threads[netif->pid] = NULL;
//      sched_num_threads--;
//      irq_restore(state);
//  }

#ifdef MODULE_AT86RF2XX
  gpio_mode_t gpio_power_down_mode = RADIO_PWR_HIGH_SIDE_SWITCH ? GPIO_IN_PD : GPIO_IN_PU;

  gpio_init(AT86RF2XX_PARAM_CS, gpio_power_down_mode);
  gpio_init(AT86RF2XX_PARAM_INT, gpio_power_down_mode);
  gpio_init(AT86RF2XX_PARAM_SLEEP, gpio_power_down_mode);
  gpio_init(AT86RF2XX_PARAM_RESET, gpio_power_down_mode);
  gpio_init(RADIO_SPI_SCK_PIN, gpio_power_down_mode);
  gpio_init(RADIO_SPI_MISO_PIN, gpio_power_down_mode);
  gpio_init(RADIO_SPI_MOSI_PIN, gpio_power_down_mode);

  /* power down radio */
  gpio_write(AT86RF2XX_PARAM_POWER, !RADIO_PWR_SWITCH_HIGH_ACTIVE);
#endif
}

static gnrc_netif_t *_manual_init_sx127x(void)
{
    for (unsigned i = 0; i < SX127X_NUMOF; ++i) {
        sx127x_setup(&sx127x_devs[i], &sx127x_params[i]);
        gnrc_netif_t *netif = gnrc_netif_lorawan_create(sx127x_stacks[i],
                                         SX127X_STACKSIZE, SX127X_PRIO,
                                         "sx127x", (netdev_t *)&sx127x_devs[i]);
        return netif;
    }

    return NULL;
}

int _radio_init(void){
  gnrc_pktbuf_init();
#ifdef UPLINK_LOWPAN_ENABLED
  gnrc_sixlowpan_init();
  gnrc_ipv6_init();
  gnrc_udp_init();
  gcoap_init();
#endif

#ifdef MODULE_AT86RF2XX
  /*power up radio */
  gpio_init(AT86RF2XX_PARAM_POWER, GPIO_OUT);
  /* power down radio */
  gpio_write(AT86RF2XX_PARAM_POWER, RADIO_PWR_SWITCH_HIGH_ACTIVE);

  xtimer_usleep(100 * 1000); /* wait 100 ms for the supply voltage to rise */

  /* reset all netifs */
  gnrc_netif_t *netif = NULL;
  while( (netif = gnrc_netif_iter(netif)) != NULL){
      memset(netif, 0, sizeof(gnrc_netif_t));
  }

  for (unsigned i = 0; i < AT86RF2XX_NUM; i++) {
      memset(&at86rf2xx_devs[i], 0, sizeof(at86rf2xx_t));
      memset(&_at86rf2xx_stacks[i], 0, AT86RF2XX_MAC_STACKSIZE);
      spi_init(at86rf2xx_params[i].spi);
      LOG_DEBUG("Initializing at86rf2xx... (channel %d)\n", AT86RF2XX_DEFAULT_CHANNEL);
      at86rf2xx_setup(&at86rf2xx_devs[i], &at86rf2xx_params[i]);
      LOG_DEBUG("creating netif_ieee802154...\n");
      gnrc_netif_ieee802154_create(_at86rf2xx_stacks[i],
                                  AT86RF2XX_MAC_STACKSIZE,
                                  AT86RF2XX_MAC_PRIO, "at86rf2xx",
                                  (netdev_t *)&at86rf2xx_devs[i]);
  }

  //xtimer_usleep(500 *1000); /* wait 500 ms for interrupts to settle */
  LOG_DEBUG("done with radio stuff init...\n");
#endif

#ifdef MODULE_GNRC_LORAWAN
  lora_netif = _manual_init_sx127x();

  uint32_t lora_state;
  backup_reg_read(BACKUP_REG_SYS_STATE, &lora_state);

  if (lora_state & SYS_STATE_LORA_JOINED) {
      LOG_INFO("LoRa was joined before...\n");
  }
  else{
      LOG_DEBUG("lora wasn't joined before (sysstae: 0x%08lX)\n", lora_state);
      int res = _netapi_up(lora_netif->pid, 10000 * 1000);
      /* if join was successful set the flag in the sys state register */
      if (res == 0) {
          backup_reg_read(BACKUP_REG_SYS_STATE, &lora_state);
          backup_reg_write(BACKUP_REG_SYS_STATE,
                           lora_state |= SYS_STATE_LORA_JOINED);
      }
      else{
          printf("join failed! will retry next time\n");
          return -1;
      }
  }

  printf("giving the radio a bit time to set up..\n");
  xtimer_usleep(1000 * 2000);
#endif

  return 0;
}

void _ctl_radio(msg_t *m)
{
    msg_t dummy_reply;
    switch (CTLMSG_TYPE2(m->type)) {
        case CTLMSG_TYPE2_INIT:
            LOG_DEBUG("CTLMSG_TYPE2_INIT\n");
            _radio_init();
            // TODO check if the sender actually waits for reply...
            msg_reply(m, &dummy_reply);
            break;

        case CTLMSG_TYPE2_DATA_CBOR:
            LOG_DEBUG("CTLMSG_TYPE2_DATA_CBOR\n");
            cbor_buffer_t *cbor_data = (cbor_buffer_t*)m->content.ptr;
            _lora_tx(lora_netif, LORAWAN_PORT, cbor_data->buf,
                     cbor_data->len, &dummy_reply.content.value);
            msg_reply(m, &dummy_reply);
            break;

        case CTLMSG_TYPE2_POWER_ON:
            LOG_DEBUG("CTLMSG_TYPE2_POWER_ON\n");
            break;

        case CTLMSG_TYPE2_POWER_OFF:
            LOG_DEBUG("CTLMSG_TYPE2_POWER_OFF\n");
            _radio_power_off();
            _radio_pins_set_to_power_down();
            break;

        default: LOG_ERROR("_ctl_radio: unknown message type 2\n");
    }
}

void *radio_thread(void *arg)
{
    (void) arg;
    msg_t m;

    msg_init_queue(msg_queue,THREAD_MSG_QUEUE_SIZE_RADIO);

    LOG_DEBUG("radio thread started, pid: %" PRIkernel_pid "\n", thread_getpid());

    while (1) {
        msg_receive(&m);
        LOG_DEBUG("[radio_thread] got msg from %" PRIkernel_pid "\n",
                 m.sender_pid);

        switch (CTLMSG_TYPE1(m.type)) {
            case CTLMSG_TYPE1_RADIO: _ctl_radio(&m); break;
            default: LOG_ERROR("radio_thread: unknown message type\n");
        }
    }

    _radio_power_off();

    return NULL;
}

kernel_pid_t radio_init_thread(void)
{
    return thread_create(radio_thread_stack, sizeof(radio_thread_stack),
                         THREAD_PRIORITY_RADIO, THREAD_CREATE_STACKTEST,
                         radio_thread, NULL, "radio");
}
