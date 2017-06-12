
#ifndef NETIO_H
#define NETIO_H

#include "stdtypes.h"
#include "stdinc.h"

void net_init_nRF24_ports();

void net_config_nRF24( // starts in stdby RX Mode
                      uint8 pipe_autoack_bits, // 0-3F
                      uint8 pipe_enable_rx_bits,  // 0-3F
                      uint8 addr_width_3_4_5,  // 3-5
                      uint8 auto_retransmit_count_0_to_15, // 0 = disabled, 0-15
                      uint8 auto_retransmit_delay_0_to_15, // only relevant is above !0. (250us*n)+250 e.g. 3 = 1009 microsec delay, 0-15
                      uint8 channel_0_to_125,  // 2.400 ghz + (n * 1Mhz)
                      uint8 * rx_addr_p0_5_bytes, // reverse byte order if Shockburst is enabled
                      uint8 * rx_addr_p1_5_bytes,
                      uint8 rx_addr_p2,
                      uint8 rx_addr_p3,
                      uint8 rx_addr_p4,
                      uint8 rx_addr_p5,
                      uint8 * tx_addr_5_bytes,        // ...
                      uint8 rx_payload_width   // for all pipes
                      );

void net_report_regs();

void net_set_rx_mode();
void net_set_tx_mode();

bool net_irq_pin_active();
bool net_irq_rx_dr_active();
bool net_irq_tx_ds_active();
bool net_irq_max_rt_active();
bool net_irq_tx_ds_or_max_rt_active();

void net_set_rx0_tx_addr(uint8 * addr_5_bytes); // updates tx/rx_p0 address


#define SPI_SEND(s)                        SPIBUF = (s); while(!SPISTDIO); SPIBUF;
#define SPI_SENDREC(s,r)                   SPIBUF = (s); while(!SPISTDIO); r = SPIBUF;

#define net_exec_cmd_w(cmdbyte,databyte)   nRF_CSN=0; SPI_SEND((cmdbyte)); SPI_SEND(databyte); nRF_CSN=1;
#define net_exec_cmd_r(cmdbyte,resbyte)    nRF_CSN=0; SPI_SEND((cmdbyte)); SPI_SENDREC(resbyte,resbyte); nRF_CSN=1;

#define net_write_reg(cmdbyte,databyte)    net_exec_cmd_w(nrf24l01_W_REGISTER | ((cmdbyte) & nrf24l01_W_REGISTER_DATA),databyte);
#define net_read_reg(cmdbyte,resbyte)      net_exec_cmd_r((cmdbyte) & nrf24l01_R_REGISTER_DATA,resbyte);

#define net_spi_data_begin(cmdbyte)        nRF_CSN=0; SPI_SEND((cmdbyte));
#define net_spi_data_w(databyte)           SPI_SEND(databyte);
#define net_spi_data_r(resbyte)            SPI_SENDREC(resbyte,resbyte);
#define net_spi_data_end()                 nRF_CSN=1;

#define net_clear_rx_fifo()                net_spi_data_begin(nrf24l01_FLUSH_RX); net_spi_data_end();
#define net_clear_tx_fifo()                net_spi_data_begin(nrf24l01_FLUSH_TX); net_spi_data_end();

#define net_clear_all_irqs()               net_write_reg(nrf24l01_STATUS, nrf24l01_STATUS_RX_DR | nrf24l01_STATUS_TX_DS | nrf24l01_STATUS_MAX_RT);

#define net_queue_transmit_begin()         net_spi_data_begin(nrf24l01_W_TX_PAYLOAD);
#define net_queue_transmit_end()           net_spi_data_end();
#define net_receive_begin()                net_spi_data_begin(nrf24l01_R_RX_PAYLOAD);
#define net_receive_end()                  net_spi_data_end();

#define net_turn_radio_on()                nRF_CE=1; DELAY_US(11);
#define net_turn_radio_off()               nRF_CE=0;

#define net_get_status_byte(resbyte)       nRF_CSN=0; SPI_SENDREC((nrf24l01_NOP),resbyte); nRF_CSN=1;

////////////////////////////////////////////////////////////////////////////////////
// SPI commands
//
// The following are defines for all of the commands and data masks on the SPI 
//   interface.
////////////////////////////////////////////////////////////////////////////////////
//SPI command defines
#define nrf24l01_R_REGISTER          0x00
#define nrf24l01_W_REGISTER          0x20
#define nrf24l01_R_RX_PAYLOAD        0x61
#define nrf24l01_W_TX_PAYLOAD        0xA0
#define nrf24l01_FLUSH_TX            0xE1
#define nrf24l01_FLUSH_RX            0xE2
#define nrf24l01_REUSE_TX_PL         0xE3
#define nrf24l01_R_RX_PL_WID         0x60  // new for 24L01P!
#define nrf24l01_W_ACK_PAYLOAD       0xA8  // new for 24L01P!
#define nrf24l01_W_TX_PAYLOAD_NOACK  0xB0  // new for 24L01P!
#define nrf24l01_NOP                 0xFF

//SPI command data mask defines
#define nrf24l01_R_REGISTER_DATA     0x1F
#define nrf24l01_W_REGISTER_DATA     0x1F
#define nrf24l01_W_ACK_PAYLOAD_PIPE  0x07   // NEW!

////////////////////////////////////////////////////////////////////////////////////
// Register definitions
//
// Below are the defines for each register's address in the 24L01.
////////////////////////////////////////////////////////////////////////////////////
#define nrf24l01_CONFIG           0x00
#define nrf24l01_EN_AA            0x01
#define nrf24l01_EN_RXADDR        0x02
#define nrf24l01_SETUP_AW         0x03
#define nrf24l01_SETUP_RETR       0x04
#define nrf24l01_RF_CH            0x05
#define nrf24l01_RF_SETUP         0x06
#define nrf24l01_STATUS           0x07
#define nrf24l01_OBSERVE_TX       0x08
#define nrf24l01_CD               0x09
#define nrf24l01_RX_ADDR_P0       0x0A
#define nrf24l01_RX_ADDR_P1       0x0B
#define nrf24l01_RX_ADDR_P2       0x0C
#define nrf24l01_RX_ADDR_P3       0x0D
#define nrf24l01_RX_ADDR_P4       0x0E
#define nrf24l01_RX_ADDR_P5       0x0F
#define nrf24l01_TX_ADDR          0x10
#define nrf24l01_RX_PW_P0         0x11
#define nrf24l01_RX_PW_P1         0x12
#define nrf24l01_RX_PW_P2         0x13
#define nrf24l01_RX_PW_P3         0x14
#define nrf24l01_RX_PW_P4         0x15
#define nrf24l01_RX_PW_P5         0x16
#define nrf24l01_FIFO_STATUS      0x17
// new registers for 24L01P
#define nrf24l01_DYNPL            0x1C  // bits 0-5 (pipe), requires EN_DPL and AA set for that pipe
#define nrf24l01_FEATREG          0x1D  // bits: 0 = EN_DPL - enable dynamic payload length (for 1 or more pipes)
//       1 = EN_ACK_PAY - enable ACK payload
//       2 = EN_DYN_ACK - enable W_TX_PAYLOAD_NOACK command
// refer to pg 51 for new commands

////////////////////////////////////////////////////////////////////////////////////
// Register bitwise definitions
//
// Below are the defines for each register's bitwise fields in the 24L01.
////////////////////////////////////////////////////////////////////////////////////
//CONFIG register bitwise definitions
#define nrf24l01_CONFIG_RESERVED    0x80
#define nrf24l01_CONFIG_MASK_RX_DR  0x40
#define nrf24l01_CONFIG_MASK_TX_DS  0x20
#define nrf24l01_CONFIG_MASK_MAX_RT 0x10
#define nrf24l01_CONFIG_EN_CRC      0x08
#define nrf24l01_CONFIG_CRCO        0x04
#define nrf24l01_CONFIG_PWR_UP      0x02
#define nrf24l01_CONFIG_PRIM_RX     0x01

//EN_AA register bitwise definitions
#define nrf24l01_EN_AA_RESERVED     0xC0
#define nrf24l01_EN_AA_ENAA_ALL     0x3F
#define nrf24l01_EN_AA_ENAA_P5      0x20
#define nrf24l01_EN_AA_ENAA_P4      0x10
#define nrf24l01_EN_AA_ENAA_P3      0x08
#define nrf24l01_EN_AA_ENAA_P2      0x04
#define nrf24l01_EN_AA_ENAA_P1      0x02
#define nrf24l01_EN_AA_ENAA_P0      0x01
#define nrf24l01_EN_AA_ENAA_NONE    0x00

//EN_RXADDR register bitwise definitions
#define nrf24l01_EN_RXADDR_RESERVED 0xC0
#define nrf24l01_EN_RXADDR_ERX_ALL  0x3F
#define nrf24l01_EN_RXADDR_ERX_P5   0x20
#define nrf24l01_EN_RXADDR_ERX_P4   0x10
#define nrf24l01_EN_RXADDR_ERX_P3   0x08
#define nrf24l01_EN_RXADDR_ERX_P2   0x04
#define nrf24l01_EN_RXADDR_ERX_P1   0x02
#define nrf24l01_EN_RXADDR_ERX_P0   0x01
#define nrf24l01_EN_RXADDR_ERX_NONE 0x00

//SETUP_AW register bitwise definitions
#define nrf24l01_SETUP_AW_RESERVED  0xFC
#define nrf24l01_SETUP_AW           0x03
#define nrf24l01_SETUP_AW_5BYTES    0x03
#define nrf24l01_SETUP_AW_4BYTES    0x02
#define nrf24l01_SETUP_AW_3BYTES    0x01
#define nrf24l01_SETUP_AW_ILLEGAL   0x00

//SETUP_RETR register bitwise definitions
#define nrf24l01_SETUP_RETR_ARD       0xF0
#define nrf24l01_SETUP_RETR_ARD_4000  0xF0
#define nrf24l01_SETUP_RETR_ARD_3750  0xE0
#define nrf24l01_SETUP_RETR_ARD_3500  0xD0
#define nrf24l01_SETUP_RETR_ARD_3250  0xC0
#define nrf24l01_SETUP_RETR_ARD_3000  0xB0
#define nrf24l01_SETUP_RETR_ARD_2750  0xA0
#define nrf24l01_SETUP_RETR_ARD_2500  0x90
#define nrf24l01_SETUP_RETR_ARD_2250  0x80
#define nrf24l01_SETUP_RETR_ARD_2000  0x70
#define nrf24l01_SETUP_RETR_ARD_1750  0x60
#define nrf24l01_SETUP_RETR_ARD_1500  0x50
#define nrf24l01_SETUP_RETR_ARD_1250  0x40
#define nrf24l01_SETUP_RETR_ARD_1000  0x30
#define nrf24l01_SETUP_RETR_ARD_750   0x20
#define nrf24l01_SETUP_RETR_ARD_500   0x10
#define nrf24l01_SETUP_RETR_ARD_250   0x00
#define nrf24l01_SETUP_RETR_ARC       0x0F
#define nrf24l01_SETUP_RETR_ARC_15    0x0F
#define nrf24l01_SETUP_RETR_ARC_14    0x0E
#define nrf24l01_SETUP_RETR_ARC_13    0x0D
#define nrf24l01_SETUP_RETR_ARC_12    0x0C
#define nrf24l01_SETUP_RETR_ARC_11    0x0B
#define nrf24l01_SETUP_RETR_ARC_10    0x0A
#define nrf24l01_SETUP_RETR_ARC_9     0x09
#define nrf24l01_SETUP_RETR_ARC_8     0x08
#define nrf24l01_SETUP_RETR_ARC_7     0x07
#define nrf24l01_SETUP_RETR_ARC_6     0x06
#define nrf24l01_SETUP_RETR_ARC_5     0x05
#define nrf24l01_SETUP_RETR_ARC_4     0x04
#define nrf24l01_SETUP_RETR_ARC_3     0x03
#define nrf24l01_SETUP_RETR_ARC_2     0x02
#define nrf24l01_SETUP_RETR_ARC_1     0x01
#define nrf24l01_SETUP_RETR_ARC_0     0x00

//RF_CH register bitwise definitions
#define nrf24l01_RF_CH_RESERVED       0x80

//RF_SETUP register bitwise definitions
#define nrf24l01_RF_SETUP_RESERVED    0xE0
#define nrf24l01_RF_SETUP_PLL_LOCK    0x10
#define nrf24l01_RF_SETUP_RF_DR       0x08
#define nrf24l01_RF_SETUP_RF_PWR      0x06
#define nrf24l01_RF_SETUP_RF_PWR_0    0x06
#define nrf24l01_RF_SETUP_RF_PWR_6    0x04
#define nrf24l01_RF_SETUP_RF_PWR_12   0x02
#define nrf24l01_RF_SETUP_RF_PWR_18   0x00
#define nrf24l01_RF_SETUP_LNA_HCURR   0x01

//STATUS register bitwise definitions
#define nrf24l01_STATUS_RESERVED                  0x80
#define nrf24l01_STATUS_RX_DR                     0x40
#define nrf24l01_STATUS_TX_DS                     0x20
#define nrf24l01_STATUS_MAX_RT                    0x10
#define nrf24l01_STATUS_RX_P_NO                   0x0E
#define nrf24l01_STATUS_RX_P_NO_RX_FIFO_NOT_EMPTY 0x0E
#define nrf24l01_STATUS_RX_P_NO_UNUSED            0x0C
#define nrf24l01_STATUS_RX_P_NO_5                 0x0A
#define nrf24l01_STATUS_RX_P_NO_4                 0x08
#define nrf24l01_STATUS_RX_P_NO_3                 0x06
#define nrf24l01_STATUS_RX_P_NO_2                 0x04
#define nrf24l01_STATUS_RX_P_NO_1                 0x02
#define nrf24l01_STATUS_RX_P_NO_0                 0x00
#define nrf24l01_STATUS_TX_FULL                   0x01

//OBSERVE_TX register bitwise definitions
#define nrf24l01_OBSERVE_TX_PLOS_CNT    0xF0
#define nrf24l01_OBSERVE_TX_ARC_CNT     0x0F

//CD register bitwise definitions
#define nrf24l01_CD_RESERVED            0xFE
#define nrf24l01_CD_CD                  0x01

//RX_PW_P0 register bitwise definitions
#define nrf24l01_RX_PW_P0_RESERVED      0xC0

//RX_PW_P0 register bitwise definitions
#define nrf24l01_RX_PW_P0_RESERVED      0xC0

//RX_PW_P1 register bitwise definitions
#define nrf24l01_RX_PW_P1_RESERVED      0xC0

//RX_PW_P2 register bitwise definitions
#define nrf24l01_RX_PW_P2_RESERVED      0xC0

//RX_PW_P3 register bitwise definitions
#define nrf24l01_RX_PW_P3_RESERVED      0xC0

//RX_PW_P4 register bitwise definitions
#define nrf24l01_RX_PW_P4_RESERVED      0xC0

//RX_PW_P5 register bitwise definitions
#define nrf24l01_RX_PW_P5_RESERVED      0xC0

//FIFO_STATUS register bitwise definitions
#define nrf24l01_FIFO_STATUS_RESERVED   0x8C
#define nrf24l01_FIFO_STATUS_TX_REUSE   0x40
#define nrf24l01_FIFO_STATUS_TX_FULL    0x20
#define nrf24l01_FIFO_STATUS_TX_EMPTY   0x10
#define nrf24l01_FIFO_STATUS_RX_FULL    0x02
#define nrf24l01_FIFO_STATUS_RX_EMPTY   0x01

// NEW! Dynamic payload register bitwise definitions (per pipe) (nrf24l01_DYNPL)
#define nrf24l01_DYNPD_P0               0x01
#define nrf24l01_DYNPD_P1               0x02
#define nrf24l01_DYNPD_P2               0x04
#define nrf24l01_DYNPD_P3               0x08
#define nrf24l01_DYNPD_P4               0x10
#define nrf24l01_DYNPD_P5               0x20

// NEW! Feature register bitwise definitions (nrf24l01_FEATREG) pg.63
#define nrf24l01_FEATBIT_EN_DPL         0x04      // enable dynamic payload
#define nrf24l01_FEATBIT_EN_ACK_PAY     0x02      // enable payload with ack
#define nrf24l01_FEATBIT_EN_DYN_ACK     0x01      // enable W_TX_PAYLOAD_NOACK command

#endif  /* NETIO_H */

