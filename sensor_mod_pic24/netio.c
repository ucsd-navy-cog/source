
// originally factored by C. Dabinett
// encapsulates nRF24L01P functionality

#include "netio.h"
#include "sysio.h"

void net_init_nRF24_ports()
{
    //set SDO and MO to output
    TRISBbits.TRISB0 = 0;
    TRISBbits.TRISB1 = 0;
    
    
    TRISAbits.TRISA3 = 0;
    nRF_CE = 0;
    
    TRISAbits.TRISA2 = 0;
    nRF_CSN = 1;
    

    //set RP pins
    RPINR22bits.SDI2R = 3; //NRF MI is RP3
    RPOR0bits.RP0R = 11; //nRF SCK is RP0
    RPOR0bits.RP1R = 10; //nRF MO is RP1

    //init SPI2
    SPI2CON1bits.DISSCK = 0;
    SPI2CON1bits.DISSDO = 0;
    SPI2CON1bits.MODE16 = 0;
    SPI2CON1bits.SMP = 0;
    SPI2CON1bits.CKE = 1; // as suggested at http://www.microchip.com/forums/m342635.aspx
    SPI2CON1bits.SSEN = 0;
    SPI2CON1bits.CKP = 0;
    SPI2CON1bits.MSTEN = 1;
    
    SPI2CON1bits.SPRE = 0b111; //1:1 for TX
    //SPI2CON1bits.SPRE = 0b110; //1:2 for TX
    
    SPI2CON1bits.PPRE = 0b11; //1:1

    SPI2CON2 = 0;

    //SPI Enabled
    SPI2STATbits.SPIEN = 1;

    DELAY_MS(2); // 100

}

void net_config_nRF24(
    uint8 pipe_autoack_bits,
    uint8 pipe_enable_rx_bits,
    uint8 addr_width_3_4_5,
    uint8 auto_retransmit_count_0_to_15,
    uint8 auto_retransmit_delay_0_to_15,
    uint8 channel_0_to_125,
    uint8 * rx_addr_p0_5_bytes, // reverse byte order if Shockburst is enabled
    uint8 * rx_addr_p1_5_bytes,
    uint8 rx_addr_p2,
    uint8 rx_addr_p3,
    uint8 rx_addr_p4,
    uint8 rx_addr_p5,
    uint8 * tx_addr_5_bytes,
    uint8 rx_payload_width
    )
{
  uint8 x, pe, aw, pw, config;
  
  pw = rx_payload_width;
  if (pw > 32)
    pw = 32;
  else if (!pw)
    pw = 1;
  
  x = pipe_autoack_bits & nrf24l01_EN_AA_ENAA_ALL;
  net_write_reg(nrf24l01_EN_AA,x);
  
  pe = (pipe_enable_rx_bits & nrf24l01_EN_RXADDR_ERX_ALL) | 1; // pipe is always enabled for rx
  net_write_reg(nrf24l01_EN_RXADDR,pe);
  
  aw = (addr_width_3_4_5 - 2) & 0x03;
  if (!aw)
    aw = 1;
  net_write_reg(nrf24l01_SETUP_AW,aw);
  
  x = (auto_retransmit_count_0_to_15 & 0x07) | ((auto_retransmit_count_0_to_15 & 0x07) << 4);
  net_write_reg(nrf24l01_SETUP_RETR,x);
  
  x = channel_0_to_125;
  if (x > 125)
    x = 125;
  net_write_reg(nrf24l01_RF_CH,x);
  
  net_write_reg(nrf24l01_RF_SETUP,0x0F); 
  
  net_spi_data_begin(nrf24l01_W_REGISTER | ((nrf24l01_RX_ADDR_P0) & nrf24l01_W_REGISTER_DATA)); // 2MB, -0 db
  net_spi_data_w(rx_addr_p0_5_bytes[0]);
  net_spi_data_w(rx_addr_p0_5_bytes[1]);
  net_spi_data_w(rx_addr_p0_5_bytes[2]);
  net_spi_data_w(rx_addr_p0_5_bytes[3]);
  net_spi_data_w(rx_addr_p0_5_bytes[4]);
  net_spi_data_end();

  net_spi_data_begin(nrf24l01_W_REGISTER | ((nrf24l01_RX_ADDR_P1) & nrf24l01_W_REGISTER_DATA));
  net_spi_data_w(rx_addr_p1_5_bytes[0]);
  net_spi_data_w(rx_addr_p1_5_bytes[1]);
  net_spi_data_w(rx_addr_p1_5_bytes[2]);
  net_spi_data_w(rx_addr_p1_5_bytes[3]);
  net_spi_data_w(rx_addr_p1_5_bytes[4]);
  net_spi_data_end();
  
  net_write_reg(nrf24l01_RX_ADDR_P2,rx_addr_p2);

  net_write_reg(nrf24l01_RX_ADDR_P3,rx_addr_p3);

  net_write_reg(nrf24l01_RX_ADDR_P4,rx_addr_p4);

  net_write_reg(nrf24l01_RX_ADDR_P5,rx_addr_p5);

  net_spi_data_begin(nrf24l01_W_REGISTER | ((nrf24l01_TX_ADDR) & nrf24l01_W_REGISTER_DATA));
  net_spi_data_w(tx_addr_5_bytes[0]);
  net_spi_data_w(tx_addr_5_bytes[1]);
  net_spi_data_w(tx_addr_5_bytes[2]);
  net_spi_data_w(tx_addr_5_bytes[3]);
  net_spi_data_w(tx_addr_5_bytes[4]);
  net_spi_data_end();
   
  net_write_reg(nrf24l01_RX_PW_P0, pw);
  net_write_reg(nrf24l01_RX_PW_P1, (pe & 0x02) ? pw : 0);
  net_write_reg(nrf24l01_RX_PW_P2, (pe & 0x04) ? pw : 0);
  net_write_reg(nrf24l01_RX_PW_P3, (pe & 0x08) ? pw : 0);
  net_write_reg(nrf24l01_RX_PW_P4, (pe & 0x10) ? pw : 0);
  net_write_reg(nrf24l01_RX_PW_P5, (pe & 0x20) ? pw : 0);
  
  config = nrf24l01_CONFIG_PWR_UP | /*nrf24l01_CONFIG_EN_CRC*/ nrf24l01_CONFIG_CRCO | nrf24l01_CONFIG_PRIM_RX; // OVERRIDE: use 2 byte (16 bit) CRC - start in RX mode -- UPDATE: CRC is now 8 bits.. great perform increase.
  config |= nrf24l01_CONFIG_MASK_MAX_RT; // comment this out for shockwave, or retransmissions
  net_write_reg(nrf24l01_CONFIG, config);
  DELAY_US(1500);
  
  nRF_CE = 0; // enter RX "stdby mode"
  
}

void net_report_regs()
{
  uint8 x[37], i,n;

  for (i=0,n=0;i<=0x17;i++) {
    if (i != 0x0a && i != 0x0B && i != 0x10) {
      net_read_reg(i,x[n]); n++;
    }
    else {
      net_spi_data_begin((i) & nrf24l01_R_REGISTER_DATA);
      net_spi_data_r(x[n]); n++;
      net_spi_data_r(x[n]); n++;
      net_spi_data_r(x[n]); n++;
      net_spi_data_r(x[n]); n++;
      net_spi_data_r(x[n]); n++;
      net_spi_data_end();
    }
  }
  net_read_reg(35,x[35]);
  net_read_reg(36,x[36]);
  
  for (i=0;i<37;i++)
    printf("%02X ",x[i]);
  
  printf("\n");

}

void net_set_rx_mode()
{
  uint8 x = 0;
  net_read_reg(nrf24l01_CONFIG,x);
  if(x & nrf24l01_CONFIG_PRIM_RX)
    return;
  x |= nrf24l01_CONFIG_PRIM_RX;
  net_write_reg(nrf24l01_CONFIG,x);  
}

void net_set_tx_mode()
{
  uint8 x = 0;
  net_read_reg(nrf24l01_CONFIG,x);
  if(!(x & nrf24l01_CONFIG_PRIM_RX))
    return;
  x &= (~nrf24l01_CONFIG_PRIM_RX);
  net_write_reg(nrf24l01_CONFIG,x);  
}

bool net_irq_pin_active()
{
  return (nRF_IRQ == 0) ? true : false;
}

bool net_irq_rx_dr_active()
{
  uint8 x;
  net_get_status_byte(x);
  return (x & nrf24l01_STATUS_RX_DR) ? true : false;
}

bool net_irq_tx_ds_active()
{
  uint8 x;
  net_get_status_byte(x);
  return (x & nrf24l01_STATUS_TX_DS) ? true : false;
}

bool net_irq_max_rt_active()
{
  uint8 x;
  net_get_status_byte(x);
  return (x & nrf24l01_STATUS_MAX_RT) ? true : false;
}

bool net_irq_tx_ds_or_max_rt_active()
{
  uint8 x;
  net_get_status_byte(x);
  return (x & (nrf24l01_STATUS_TX_DS|nrf24l01_STATUS_MAX_RT)) ? true : false;
}

void net_set_rx0_tx_addr(uint8 * addr_5_bytes)
{
  net_spi_data_begin(nrf24l01_W_REGISTER | ((nrf24l01_RX_ADDR_P0) & nrf24l01_W_REGISTER_DATA)); // 2MB, -0 db
  net_spi_data_w(addr_5_bytes[0]);
  net_spi_data_w(addr_5_bytes[1]);
  net_spi_data_w(addr_5_bytes[2]);
  net_spi_data_w(addr_5_bytes[3]);
  net_spi_data_w(addr_5_bytes[4]);
  net_spi_data_end();

  net_spi_data_begin(nrf24l01_W_REGISTER | ((nrf24l01_TX_ADDR) & nrf24l01_W_REGISTER_DATA));
  net_spi_data_w(addr_5_bytes[0]);
  net_spi_data_w(addr_5_bytes[1]);
  net_spi_data_w(addr_5_bytes[2]);
  net_spi_data_w(addr_5_bytes[3]);
  net_spi_data_w(addr_5_bytes[4]);
  net_spi_data_end();
  
}