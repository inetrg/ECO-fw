# name of your application
APPLICATION = app-sane-bus

# If no BOARD is found in the environment, use this default:
BOARD ?= nucleo-l476rg

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../../RIOT

#disable auto init so no sensor/radio/ etc. is initialized (wastes energy if not needed)
DISABLE_MODULE += auto_init

FEATURES_REQUIRED += periph_gpio
FEATURES_REQUIRED += periph_i2c
FEATURES_REQUIRED += periph_rtc
FEATURES_REQUIRED += periph_uart

USEPKG += tinycbor
USEPKG += minmea

USEMODULE += fmt
USEMODULE += minmea_helper
USEMODULE += pnmea

USEMODULE += xtimer
USEMODULE += tinycbor_float
USEMODULE += printf_float
USEMODULE += saul
USEMODULE += pm_layered

USEMODULE += eco
USEMODULE += sensors
USEMODULE += cbor_util
USEMODULE += node_time
USEMODULE += backup_reg
USEMODULE += gpio_wakeup

USEMODULE += od
USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps

USEMODULE += random

UPLINK_LORA   = ENABLED
UPLINK_LOWPAN = DISABLED

# setting this to 0  is important to enable reboot on kernel panic!
DEVELHELP ?= 0

CFLAGS += -DSTM32L4_SRAM2_RETENTION=1

ifneq (,$(filter ENABLED,$(UPLINK_LOWPAN) $(UPLINK_LORA)))
  # include helper module for sending data to "any" uplink
  USEMODULE += radio
  USEMODULE += gnrc_netdev_default
endif

ifneq (,$(filter ENABLED,$(UPLINK_LOWPAN)))
  CFLAGS += -DUPLINK_LOWPAN_ENABLED=1

#------------------------------------------------------------vvvv
# used Interrupt Lines - CHeck this if you want to add some interrupt pin
# YOU CAN NOT USE THE SAME PIN NUMBERS (EVEN ON DIFFERENT PORTS!)

#SX127X_PARAM_DIO0                   GPIO_PIN(0, 10)      /* D2 */
#SX127X_PARAM_DIO1                   GPIO_PIN(1, 3)       /* D3 */
#SX127X_PARAM_DIO2                   GPIO_PIN(1, 5)       /* D4 */
#SX127X_PARAM_DIO3                   GPIO_PIN(1, 4)       /* D5 */

#ina226 alert:
#INA22X_PARAM_ALERT_PIN GPIO_PIN(2,5)

#WKUP PINS:
#WKUP 1: PA0  -> XXX conflict with LORA RESET
#WKUP 2: PC13 -> BTN1 -> try to avoid..
#WKUP 3: PE6  -> XXX Not routed on nucleo64
#WKUP 4: PA2  -> XXX Conflicts with UART (which can not be moved)
#WKUP 5: PC5  -> XXX IRQ conflict with LORA DIO2 (PA5)

# .dev        = USART2,
# .rx_pin     = GPIO_PIN(PORT_A, 3),  --> map to PD6 -> no, N/A nucleo 64
# .tx_pin     = GPIO_PIN(PORT_A, 2),  --> map to PD5 -> no, N/A nucleo 64

# .dev        = USART3,
# .rx_pin     = GPIO_PIN(PORT_B, 11), --> XXX used for sds011 already
# .tx_pin     = GPIO_PIN(PORT_C, 4),  --> XXX used for sds011 already

# .dev        = USART1,
# .rx_pin     = GPIO_PIN(PORT_B, 7),  --> XXX used for GPS already
# .tx_pin     = GPIO_PIN(PORT_A, 9),  --> XXX used for GPS already
#------------------------------------------------------------^^^^

#------------------------------------------------------------vvvv
# low level radio config
USEMODULE += at86rf233
CFLAGS += -DAT86RF2XX_PARAM_SPI=SPI_DEV\(0\)
CFLAGS += -DAT86RF2XX_PARAM_SPI_CLK=SPI_CLK_1MHZ
CFLAGS += -DAT86RF2XX_PARAM_CS=GPIO_PIN\(1,6\)
CFLAGS += -DAT86RF2XX_PARAM_INT=GPIO_PIN\(0,8\)
CFLAGS += -DAT86RF2XX_PARAM_SLEEP=GPIO_PIN\(1,10\)
CFLAGS += -DAT86RF2XX_PARAM_RESET=GPIO_PIN\(1,4\)
CFLAGS += -DAT86RF2XX_PARAM_POWER=GPIO_PIN\(0,4\)

# specify exact SPI pins (needed for gpio config for power-down)
CFLAGS += -DRADIO_SPI_SCK_PIN=GPIO_PIN\(0,5\)
CFLAGS += -DRADIO_SPI_MISO_PIN=GPIO_PIN\(0,6\)
CFLAGS += -DRADIO_SPI_MOSI_PIN=GPIO_PIN\(0,7\)

CFLAGS += -DRADIO_PWR_HIGH_SIDE_SWITCH=false
CFLAGS += -DRADIO_PWR_SWITCH_HIGH_ACTIVE=true

CFLAGS += -DIEEE802154_DEFAULT_CHANNEL=15
#------------------------------------------------------------^^^^

USEMODULE += coap
USEMODULE += netopt
USEMODULE += gnrc_ipv6_router_default
USEMODULE += gnrc_pktdump
USEMODULE += gnrc_icmpv6_echo
USEMODULE += posix_sockets
USEMODULE += gnrc_udp
USEMODULE += gnrc_sock_udp

CFLAGS += -DUPLINK_LLADDR=\"fe80::509e:602e:33b7:1d55\"
CFLAGS += -DNTP_SERVER_ADDR=UPLINK_LLADDR
CFLAGS += -DGCOAP_BLOCKING_TIMEOUT_MS=5000
CFLAGS += -DGNRC_IPV6_NIB_CONF_SLAAC=1
endif

ifneq (,$(filter ENABLED,$(UPLINK_LORA)))
  CFLAGS += -DUPLINK_LORAWAN_ENABLED=1
  CFLAGS += -DCONFIG_GNRC_LORAWAN_ENABLE_LPM_STORAGE

  DRIVER = sx1272
  USEMODULE += $(DRIVER)
  USEMODULE += gnrc_lorawan
  USEMODULE += gnrc_netapi_callbacks
  USEMODULE += netdev_layer
  USEMODULE += gnrc_neterr
  USEMODULE += gnrc_netreg
  USEMODULE += hashes
  USEMODULE += crypto

  # Required for the cipher module */
  CFLAGS += -DCRYPTO_AES
  CFLAGS += -DLORAMAC_APP_KEY_DEFAULT=\{0x00\,0x00\,0x00\,0x00\,0x00\,0x00\,0x00\,0x00\,0x00\,0x00\,0x00\,0x00\,0x00\,0x00\,0x00\,0x00\}
  CFLAGS += -DLORAMAC_APP_EUI_DEFAULT=\{0x00\,0x00\,0x00\,0x00\,0x00\,0x00\,0x00\,0x00\}
  CFLAGS += -DLORAMAC_DEV_EUI_DEFAULT=\{0x00\,0x00\,0x00\,0x00\,0x00\,0x00\,0x00\,0x00\}

  # Additionally needed by nucleo-l476rg to improve accuracy of clock
  # and to adapt timing of LoRa join procedure
  CFLAGS += -DCLOCK_MSI_LSE_PLL=1
  CFLAGS += -DCLOCK_LSE=1
  CFLAGS += -DCONFIG_GNRC_LORAWAN_TIMER_DRIFT=0
  CFLAGS += -DCONFIG_GNRC_LORAWAN_MIN_SYMBOLS_TIMEOUT=25

  # Comment/uncomment as necessary
  CFLAGS += -DLORAMAC_DEFAULT_JOIN_PROCEDURE=LORAMAC_JOIN_OTAA
  #CFLAGS += -DLORAMAC_DEFAULT_JOIN_PROCEDURE=LORAMAC_JOIN_ABP

  # Set to the highest DR for the EU868 in order to maximize the throughput
  CFLAGS += -DLORAMAC_DEFAULT_DR=LORAMAC_DR_5

  # Set the default RX2 datarate to DR3 (used by The Things Network)
  CFLAGS += -DLORAMAC_DEFAULT_RX2_DR=LORAMAC_DR_3

  # Set default messages to unconfirmable
  CFLAGS += -DLORAMAC_DEFAULT_TX_MODE=LORAMAC_TX_UNCNF

  CFLAGS += -DSX1272_SCK_PIN=GPIO_PIN\(0,5\)
  CFLAGS += -DSX1272_MISO_PIN=GPIO_PIN\(0,6\)
  CFLAGS += -DSX1272_MOSI_PIN=GPIO_PIN\(0,7\)
  CFLAGS += -DSX1272_CS_PIN=GPIO_PIN\(1,6\)

  CFLAGS += -DSX1272_DIO0_PIN=GPIO_PIN\(0,10\)
  CFLAGS += -DSX1272_DIO1_PIN=GPIO_PIN\(1,3\)
  CFLAGS += -DSX1272_DIO2_PIN=GPIO_PIN\(1,5\)
  CFLAGS += -DSX1272_DIO3_PIN=GPIO_PIN\(1,4\)

  CFLAGS += -DSX1272_SPI_PINS_POWER_DOWN_MODE=GPIO_IN_PD
  #CFLAGS += -DSX127X_PARAM_PASELECT=SX127X_PA_BOOST
endif

# ADC line 1 maps to PA1 on nucleo-l476
CFLAGS += -DREHMON_PARAM_PV_ADC_LINE=3
CFLAGS += -DLTC3105_PARAM_SHDN_PIN=GPIO_PIN\(0,15\)

#------------------------------------------------------------vvvv
# GPS module defines
CFLAGS += -DGPS_ENABLE_PIN=GPIO_PIN\(2,12\)
CFLAGS += -DGPS_ENABLE_PIN_AH=\(false\)

# USART1 must be configured to to PA10 / PA9 for RX / TX
# see periph_conf.h for details
CFLAGS += -DGPS_UART_DEV=\(UART_DEV\(2\)\)
#------------------------------------------------------------^^^^

#------------------------------------------------------------vvvv
# general purpose power pins
CFLAGS += -DGP_PWR_GND_SW_PIN=GPIO_PIN\(2,3\)
CFLAGS += -DGP_PWR_VCC_SW_PIN=GPIO_PIN\(1,2\)
CFLAGS += -DGP_PWR_GND_SW_PIN_AH=\(true\)
#------------------------------------------------------------^^^^

#------------------------------------------------------------vvvv
# general purpose IO pins
CFLAGS += -DGPD_PIN=GPIO_PIN\(2,5\)
CFLAGS += -DGPAD_PIN=GPIO_PIN\(0,1\)
#------------------------------------------------------------^^^^

#------------------------------------------------------------vvvv
# BME280 pin mappings
CFLAGS += -DBMX280_PARAM_I2C_DEV=I2C_DEV\(0\)
CFLAGS += -DBMX280_PARAM_I2C_ADDR=0x77
CFLAGS += -DBMX280_PWR_PIN=GP_PWR_GND_SW_PIN
CFLAGS += -DBMX280_PWR_PIN_AH=GP_PWR_GND_SW_PIN_AH
#------------------------------------------------------------^^^^

#------------------------------------------------------------vvvv
# SDS011 pin mappings
CFLAGS += -DSDS011_PARAM_UART_DEV=\(UART_DEV\(1\)\)
#CFLAGS += -DSDS011_PARAM_PWR_PIN=GP_PWR_VCC_SW_PIN
CFLAGS += -DSDS011_PARAM_PWR_PIN=GPAD_PIN
CFLAGS += -DSDS011_PARAM_PWR_PIN_AH=\(true\)

# these are only needed to explicitly configure the pins for sleep mode
# so that no leakage currrents occur
CFLAGS += -DSDS011_RX_PIN=GPIO_PIN\(1,11\)
CFLAGS += -DSDS011_TX_PIN=GPIO_PIN\(2,4\)

CFLAGS += -DSDS011_WARMUP_TIME_MS=4000
#------------------------------------------------------------^^^^

#------------------------------------------------------------vvvv
# INA226 pin mappings
CFLAGS += -DINA22X_SHUNT_RESISTOR_MILLI_OHM=\(270LL\)
CFLAGS += -DINA22X_CAL_AMM_MEAS_CURRENT=\(48470\)
CFLAGS += -DINA22X_CAL_REPORTED_CURRENT=\(51137\)

CFLAGS += -DINA22X_PARAM_I2C=I2C_DEV\(0\)
CFLAGS += -DINA22X_PARAM_ADDR=0x40
CFLAGS += -DINA22X_PARAM_ALERT_PIN=\(GPIO_PIN\(2,13\)\)
CFLAGS += -DTEST_INA22X_MODEL=INA226
#------------------------------------------------------------^^^^

CFLAGS += -DTHREAD_MSG_QUEUE_SIZE_SENSORS=8
CFLAGS += -DTHREAD_MSG_QUEUE_SIZE_ECO=8
CFLAGS += -DTHREAD_MSG_QUEUE_SIZE_RADIO=8

CFLAGS += -DTHREAD_PRIORITY_ECO=\(THREAD_PRIORITY_MAIN-1\)
CFLAGS += -DTHREAD_PRIORITY_SENSORS=\(THREAD_PRIORITY_MAIN-2\)
CFLAGS += -DTHREAD_PRIORITY_RADIO=\(THREAD_PRIORITY_MAIN-3\)

CFLAGS += -DTHREAD_STACKSIZE_MAIN=\(5120\)
CFLAGS += -DTHREAD_STACKSIZE_ECO=\(THREAD_STACKSIZE_MAIN\)
CFLAGS += -DTHREAD_STACKSIZE_RADIO=\(THREAD_STACKSIZE_MAIN\)
CFLAGS += -DTHREAD_STACKSIZE_SENSORS=\(THREAD_STACKSIZE_MAIN\)

CFLAGS += -DECO_SAMPLE_BUFFER_CNT=3072

CFLAGS += -DUSER_BUTTON_WAKEUP_PIN=2

# overwrite default config that assumes no LSE is installed
# all nucleo64 boards I use are revisions newer MB1136 C-01 and come with LSE
CFLAGS += -DCLOCK_LSE=1
#enable MSI/LSE-trimming (PLL mode to enhance accuracy of MSI)
CFLAGS += -DCLOCK_MSI_LSE_PLL=1

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

include Makefile.dep
include $(RIOTBASE)/Makefile.include
