#include <stm32g0xx_hal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define LED_PORT    GPIOA
#define LED_PIN_ACT GPIO_PIN_4

static uint8_t swv_buf[256];
static size_t swv_buf_ptr = 0;
__attribute__ ((noinline, used))
void swv_trap_line()
{
  *(volatile char *)swv_buf;
}
static inline void swv_putchar(uint8_t c)
{
  // ITM_SendChar(c);
  if (c == '\n') {
    swv_buf[swv_buf_ptr >= sizeof swv_buf ?
      (sizeof swv_buf - 1) : swv_buf_ptr] = '\0';
    swv_trap_line();
    swv_buf_ptr = 0;
  } else if (++swv_buf_ptr <= sizeof swv_buf) {
    swv_buf[swv_buf_ptr - 1] = c;
  }
}
static void swv_printf(const char *restrict fmt, ...)
{
  char s[256];
  va_list args;
  va_start(args, fmt);
  int r = vsnprintf(s, sizeof s, fmt, args);
  for (int i = 0; i < r && i < sizeof s - 1; i++) swv_putchar(s[i]);
  if (r >= sizeof s) {
    for (int i = 0; i < 3; i++) swv_putchar('.');
    swv_putchar('\n');
  }
}

#define PIN_ADC_CK  GPIO_PIN_11
#define PIN_ADC_DA  GPIO_PIN_12
void adc_configure();

SPI_HandleTypeDef spi1 = { 0 };
#define PIN_nRF_CS  GPIO_PIN_0
#define PIN_nRF_CE  GPIO_PIN_3

static inline void nRF_send_len(const uint8_t *cmd, uint32_t size)
{
  HAL_GPIO_WritePin(GPIOA, PIN_nRF_CS, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&spi1, (uint8_t *)cmd, size, 1000);
  HAL_GPIO_WritePin(GPIOA, PIN_nRF_CS, GPIO_PIN_SET);
}
#define nRF_send(...) do { \
  uint8_t buf[] = {__VA_ARGS__}; \
  nRF_send_len(buf, sizeof buf); \
} while (0)

static inline uint8_t bit_rev(uint8_t x)
{
  x = (x & 0b00001111) << 4 | (x & 0b11110000) >> 4;
  x = (x & 0b00110011) << 2 | (x & 0b11001100) >> 2;
  x = (x & 0b01010101) << 1 | (x & 0b10101010) >> 1;
  return x;
}

static inline void ble_encode_packet(uint8_t *packet, uint8_t len, uint8_t ch)
{
  // CRC
  uint8_t crc[3] = {0x55, 0x55, 0x55};
  for (uint8_t i = 0; i < len; i++) {
    uint8_t d = packet[i];
    for (uint8_t v = 0; v < 8; v++, d >>= 1) {
      uint8_t t = 0;
      if (crc[0] & 0x80) { t = 1;       } crc[0] <<= 1;
      if (crc[1] & 0x80) { crc[0] |= 1; } crc[1] <<= 1;
      if (crc[2] & 0x80) { crc[1] |= 1; } crc[2] <<= 1;
      if (t != (d & 1)) {
        crc[2] ^= 0x5B;
        crc[1] ^= 0x06;
      }
    }
  }
  packet[len + 0] = bit_rev(crc[0]);
  packet[len + 1] = bit_rev(crc[1]);
  packet[len + 2] = bit_rev(crc[2]);

  // Whiten
  uint8_t whiten_coeff = bit_rev(ch) | 2;
  for (uint8_t i = 0; i < len + 3; i++) {
    for (uint8_t m = 1; m != 0; m <<= 1) {
      if (whiten_coeff & 0x80) {
        whiten_coeff ^= 0x11;
        packet[i] ^= m;
      }
      whiten_coeff <<= 1;
    }
  }

  // Reverse all bits
  for (uint8_t i = 0; i < len + 3; i++)
    packet[i] = bit_rev(packet[i]);
}

void compress_24b_values(uint32_t *values, size_t count, uint8_t *buffer, size_t length)
{
  buffer[0] = (values[0] >>  0) & 0xff;
  buffer[1] = (values[0] >>  8) & 0xff;
  buffer[2] = (values[0] >> 16) & 0xff;
  size_t n = 1; // Values pointer
  size_t p = 3; // Buffer pointer
  while (n < count && p < length) {
    float x[n][n];
    for (int i = 0; i < n; i++)
      x[i][i] = 0;
  }
}

static volatile uint32_t adc_value = 0;

int main()
{
  HAL_Init();

  // ======== GPIO ========
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef gpio_init;

  gpio_init.Pin = LED_PIN_ACT;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_PORT, &gpio_init);
  HAL_GPIO_WritePin(LED_PORT, LED_PIN_ACT, GPIO_PIN_RESET);

  // SWD (PA13, PA14)
  gpio_init.Pin = GPIO_PIN_13 | GPIO_PIN_14;
  gpio_init.Mode = GPIO_MODE_AF_PP; // Pass over control to AF peripheral
  gpio_init.Alternate = GPIO_AF0_SWJ;
  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_init);

  // Clocks
  RCC_OscInitTypeDef osc_init = { 0 };
  osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  osc_init.HSIState = RCC_HSI_ON;
  osc_init.PLL.PLLState = RCC_PLL_OFF;
  HAL_RCC_OscConfig(&osc_init);

  RCC_ClkInitTypeDef clk_init = { 0 };
  clk_init.ClockType =
    RCC_CLOCKTYPE_SYSCLK |
    RCC_CLOCKTYPE_HCLK |
    RCC_CLOCKTYPE_PCLK1;
  clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk_init.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_2);

  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  HAL_GPIO_WritePin(GPIOA, PIN_ADC_CK, GPIO_PIN_SET);

  // ======== SPIx ADC ========
  // ADC_CK
  gpio_init.Pin = PIN_ADC_CK;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_init);
  // ADC_DA
  gpio_init.Pin = PIN_ADC_DA;
  gpio_init.Mode = GPIO_MODE_INPUT;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_init);

  HAL_GPIO_WritePin(GPIOA, PIN_ADC_CK, GPIO_PIN_RESET);
  HAL_Delay(1);
  while (HAL_GPIO_ReadPin(GPIOA, PIN_ADC_DA) == GPIO_PIN_SET) { }
  adc_configure();

  // PA12 - EXTI line 12
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  // Pull down clock / chip select signal and wait for DRDY
  HAL_GPIO_WritePin(GPIOA, PIN_ADC_CK, GPIO_PIN_RESET);

  // ======== SPI ========
  // GPIO ports
  // SPI1_SCK (PA1), SPI1_MOSI (PA2), SPI1_MISO (PA6)
  gpio_init.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_6;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Alternate = GPIO_AF0_SPI1;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_init);
  // nRF24L01+ CSN (PA0)
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pin = PIN_nRF_CS | PIN_nRF_CE;
  HAL_GPIO_Init(GPIOA, &gpio_init);
  HAL_GPIO_WritePin(GPIOA, PIN_nRF_CS, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, PIN_nRF_CE, GPIO_PIN_RESET);

  __HAL_RCC_SPI1_CLK_ENABLE();
  spi1.Instance = SPI1;
  spi1.Init.Mode = SPI_MODE_MASTER;
  spi1.Init.Direction = SPI_DIRECTION_2LINES;
  spi1.Init.CLKPolarity = SPI_POLARITY_LOW; // CPOL = 0
  spi1.Init.CLKPhase = SPI_PHASE_1EDGE;     // CPHA = 0
  spi1.Init.NSS = SPI_NSS_SOFT;
  spi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  spi1.Init.TIMode = SPI_TIMODE_DISABLE;
  spi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  spi1.Init.DataSize = SPI_DATASIZE_8BIT;
  spi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  HAL_SPI_Init(&spi1);
  __HAL_SPI_ENABLE(&spi1);

  // Set up nRF24L01+
  nRF_send(0x20, 0x72); // CONFIG: Disable IRQ and CRC, power up
  nRF_send(0x21, 0x00); // EN_AA: Disable auto ack.
  nRF_send(0x22, 0x00); // EN_RXADDR: Disable RX completely
  nRF_send(0x23, 0x02); // SETUP_AW: 4-byte address
  nRF_send(0x24, 0x00); // SETUP_RETR: Disable auto restransmission
  nRF_send(0x26, 0x06); // RF_SETUP: 1 Mbps, 0 dBm
  nRF_send(0x27, 0x70); // STATUS: Clear status flags
  nRF_send(0x31, 32);   // RX_PW_P0: Pipe 0 payload length 32 bytes
  nRF_send(0x22, 0x01); // EN_RXADDR: Enable RX on pipe 0
  nRF_send(0x2A, bit_rev(0x8E), bit_rev(0x89), bit_rev(0xBE), bit_rev(0xD6));
    // RX_ADDR_P0: Set pipe 0 receive address
  nRF_send(0x30, bit_rev(0x8E), bit_rev(0x89), bit_rev(0xBE), bit_rev(0xD6));
    // TX_ADDR: Set transmit address

  // Read register 0x06 (RF_SETUP)
  HAL_GPIO_WritePin(GPIOA, PIN_nRF_CS, GPIO_PIN_RESET);
  uint8_t spi_tx[2] = {0x06, 0x00};
  uint8_t spi_rx[2];
  HAL_SPI_TransmitReceive(&spi1, spi_tx, spi_rx, 2, 1000);
  HAL_GPIO_WritePin(GPIOA, PIN_nRF_CS, GPIO_PIN_SET);
  swv_printf("received data = %02x %02x\n", spi_rx[0], spi_rx[1]);
  // 0e 06

  uint8_t nrf_ch[3] = { 2, 26, 80};
  uint8_t ble_ch[3] = {37, 38, 39};
  uint8_t cur_ch = 0;

  uint8_t timestamp = 0;
  uint32_t collected_data[10] = { 0 };

  while (true) {
    timestamp += 1;   // May overflow and wrap around
    for (int i = 9; i > 0; i--) collected_data[i] = collected_data[i - 1];
    // collected_data[0] = (uint32_t)timestamp * timestamp * 5;
    collected_data[0] = adc_value;

    uint8_t nRF_cmd_buf[33];
    uint8_t *buf = nRF_cmd_buf + 1;
    uint8_t p = 0;
    // PDU - Protocol Data Unit
    // AD - Advertising Data
    // For assigned numbers, see https://www.bluetooth.com/specifications/an/
    // PDU header
    buf[p++] = 0x42;  // Type: ADV_NONCONN_IND; TxAdd is random
    buf[p++] = 0;     // Payload length, to be filled
    // PDU payload
    buf[p++] = 0xAA;  // Address
    buf[p++] = 0xBB;
    buf[p++] = 0xCC;
    buf[p++] = 0xEE;
    buf[p++] = 0xDF;
    buf[p++] = 0xC0;
    buf[p++] = 2;     // AD length
    buf[p++] = 0x01;  // Type: Flags
    buf[p++] = 0x05;
    buf[p++] = 3;     // AD length
    buf[p++] = 0x08;  // Type: Shortened Local Name
    buf[p++] = 'R';
    buf[p++] = 'C';
    // buf[p++] = 13;    // AD length
    buf[p++] = 5;     // AD length
    buf[p++] = 0xFF;  // Type: Manufacturer Specific Data
    buf[p++] = timestamp;
    // compress_24b_values(collected_data, 10, buf + p, 11);
    // p += 11;
    buf[p++] = (collected_data[0] >> 16) & 0xff;
    buf[p++] = (collected_data[0] >>  8) & 0xff;
    buf[p++] = (collected_data[0] >>  0) & 0xff;
    buf[1] = p - 2;   // Payload length

    // Encode packet
    cur_ch = (cur_ch + 1) % 3;
    ble_encode_packet(buf, p, ble_ch[cur_ch]);

    nRF_send(0x25, nrf_ch[cur_ch]); // RF_CH: Set channel
    nRF_send(0x27, 0x70); // STATUS: Clear status flags
    nRF_send(0xE1); // FLUSH_TX
    nRF_send(0xE2); // FLUSH_RX
    nRF_cmd_buf[0] = 0xA0;  // W_TX_PAYLOAD
    nRF_send_len(nRF_cmd_buf, p + 4);
    HAL_GPIO_WritePin(GPIOA, PIN_nRF_CE, GPIO_PIN_SET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(GPIOA, PIN_nRF_CE, GPIO_PIN_RESET);

    swv_printf("ADC value = %06x\n", collected_data[0]);
  }

  while (true) {
    HAL_GPIO_WritePin(LED_PORT, LED_PIN_ACT, GPIO_PIN_RESET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN_ACT, GPIO_PIN_SET);
    HAL_Delay(200);
  }
}

void SysTick_Handler()
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

static inline void adc_ser_delay()
{
  // >= 0.5 us
  for (volatile int i = 0; i < 16; i++) asm volatile ("nop");
}

inline void adc_configure()
{
  GPIO_InitTypeDef gpio_init;
  gpio_init.Pin = PIN_ADC_DA;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_WritePin(GPIOA, PIN_ADC_CK, GPIO_PIN_RESET);
  adc_ser_delay();
  for (int i = 0; i < 29; i++) {
    HAL_GPIO_WritePin(GPIOA, PIN_ADC_CK, GPIO_PIN_SET);
    adc_ser_delay();
    HAL_GPIO_WritePin(GPIOA, PIN_ADC_CK, GPIO_PIN_RESET);
    adc_ser_delay();
  }

  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOA, &gpio_init);
  uint8_t writes[2] = {0x65 << 1, 0b00010100};
  for (int i = 0; i < 16; i++) {
    HAL_GPIO_WritePin(GPIOA, PIN_ADC_CK, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, PIN_ADC_DA, (writes[i / 8] >> (7 - i % 8)) & 1);
    adc_ser_delay();
    HAL_GPIO_WritePin(GPIOA, PIN_ADC_CK, GPIO_PIN_RESET);
    adc_ser_delay();
  }

  gpio_init.Mode = GPIO_MODE_IT_FALLING;
  HAL_GPIO_Init(GPIOA, &gpio_init);
  HAL_GPIO_WritePin(GPIOA, PIN_ADC_CK, GPIO_PIN_SET);
  adc_ser_delay();
  HAL_GPIO_WritePin(GPIOA, PIN_ADC_CK, GPIO_PIN_RESET);
  adc_ser_delay();
}

static inline uint32_t adc_read()
{
  uint32_t value = 0;
  for (int i = 0; i < 24; i++) {
    HAL_GPIO_WritePin(GPIOA, PIN_ADC_CK, GPIO_PIN_SET);
    adc_ser_delay();
    uint32_t bit = HAL_GPIO_ReadPin(GPIOA, PIN_ADC_DA);
    value = (value << 1) | bit;
    HAL_GPIO_WritePin(GPIOA, PIN_ADC_CK, GPIO_PIN_RESET);
    adc_ser_delay();
  }
  return value;
}

void EXTI4_15_IRQHandler()
{
  /*
  static uint32_t ticks[20];
  static int ticks_count = 0;
  if (ticks_count < 20) {
    ticks[ticks_count++] = adc_read();
    ticks[ticks_count++] = HAL_GetTick();
    if (ticks_count == 20)
      for (int i = 0; i < 20; i++) {
        swv_printf("[%02d] = %06x\n", i, ticks[i]);
      }
  }
  */
  adc_value = adc_read();
  __HAL_GPIO_EXTI_CLEAR_FALLING_IT(PIN_ADC_DA);
}
