#include <stm32g0xx_hal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define LED_PORT    GPIOA
#define LED_PIN_ACT GPIO_PIN_4

// #define RELEASE
#ifndef RELEASE
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
#else
#define swv_printf(...)
#endif

#define PIN_ADC_CK  GPIO_PIN_11
#define PIN_ADC_DA  GPIO_PIN_12
void adc_configure();

SPI_HandleTypeDef spi1 = { 0 };
#define PIN_nRF_CS  GPIO_PIN_0
#define PIN_nRF_CE  GPIO_PIN_3

TIM_HandleTypeDef tim14;

// Disabled; see notes in main()
// ADC_HandleTypeDef adc1 = { 0 };

static inline void sleep_delay(uint32_t ticks) {
  uint32_t start_tick = HAL_GetTick();
  while (HAL_GetTick() - start_tick < ticks)
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}
#define HAL_Delay sleep_delay

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

static inline void write_bits(uint8_t *buffer, size_t start, size_t length, uint32_t value)
{
  for (int i = 0; i < length; i++) {
    int index = (start + i) / 8;
    int bitpos = (start + i) % 8;
    if (value & (1 << i))
      buffer[index] |= (1 << bitpos);
    else
      buffer[index] &= ~(1 << bitpos);
  }
}

static volatile uint32_t adc_value = 0;
static volatile uint8_t adc_updated = 0;

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
  HAL_Delay(1);
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
  // Wait for DRDY (not necessary — see below)
  // while (HAL_GPIO_ReadPin(GPIOA, PIN_ADC_DA) == GPIO_PIN_SET) { }
  // XXX: Are these unarticulated design aspects inside CS1237?
  //   Configuration cannot be processed when data conversion is in progress?
  //   DRDY is still present when data is invalid during the first 3/4 samples?
  // Anyway, blink the LED while we wait
  HAL_GPIO_WritePin(LED_PORT, LED_PIN_ACT, GPIO_PIN_SET);
  HAL_Delay(160);
  HAL_GPIO_WritePin(LED_PORT, LED_PIN_ACT, GPIO_PIN_RESET);
  HAL_Delay(160);
  adc_configure();  // Read data (discarded) and configure
  HAL_GPIO_WritePin(LED_PORT, LED_PIN_ACT, GPIO_PIN_SET);
  HAL_Delay(320);   // Skip 3 samples, otherwise the first few read-outs are zero
  HAL_GPIO_WritePin(LED_PORT, LED_PIN_ACT, GPIO_PIN_RESET);

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
  // nRF_send(0x31, 32);   // RX_PW_P0: Pipe 0 payload length 32 bytes
  nRF_send(0x22, 0x01); // EN_RXADDR: Enable RX on pipe 0
  // nRF_send(0x2A, bit_rev(0x8E), bit_rev(0x89), bit_rev(0xBE), bit_rev(0xD6));
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
  if (spi_rx[0] != 0x0E || spi_rx[1] != 0x06) {
    // Error? Try a reset
    for (int i = 0; i < 11; i++) {
      HAL_GPIO_WritePin(LED_PORT, LED_PIN_ACT, i % 2);
      HAL_Delay(60);
    }
    HAL_Delay(500);
    NVIC_SystemReset();
  }

  // ======== Timer ========
  // APB1 = 16 MHz
  // period = 4 kHz = 4000 cycles
  // LED Red, TIM14
  gpio_init.Pin = LED_PIN_ACT;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Alternate = GPIO_AF4_TIM14;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_PORT, &gpio_init);
  __HAL_RCC_TIM14_CLK_ENABLE();
  tim14 = (TIM_HandleTypeDef){
    .Instance = TIM14,
    .Init = {
      .Prescaler = 1 - 1,
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 4000 - 1,
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
      .RepetitionCounter = 0,
    },
  };
  HAL_TIM_PWM_Init(&tim14);
  TIM_OC_InitTypeDef tim14_ch1_oc_init = {
    .OCMode = TIM_OCMODE_PWM1,
    .Pulse = 0, // to be filled
    .OCPolarity = TIM_OCPOLARITY_HIGH,
  };
  HAL_TIM_PWM_ConfigChannel(&tim14, &tim14_ch1_oc_init, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&tim14, TIM_CHANNEL_1);
/*
from math import *
N=256
print(', '.join('%d' % round(1200*((1+sin(i/N*2*pi))/2)**2) for i in range(N)))
*/
  static const uint16_t LED_STEPS[] = {
300, 315, 330, 346, 362, 378, 394, 411, 428, 446, 463, 481, 499, 518, 536, 555, 574, 592, 611, 630, 650, 669, 688, 707, 726, 745, 764, 783, 801, 820, 838, 856, 874, 892, 909, 926, 943, 959, 975, 991, 1006, 1021, 1035, 1049, 1062, 1075, 1088, 1099, 1110, 1121, 1131, 1140, 1149, 1157, 1164, 1171, 1177, 1182, 1187, 1191, 1194, 1197, 1199, 1200, 1200, 1200, 1199, 1197, 1194, 1191, 1187, 1182, 1177, 1171, 1164, 1157, 1149, 1140, 1131, 1121, 1110, 1099, 1088, 1075, 1062, 1049, 1035, 1021, 1006, 991, 975, 959, 943, 926, 909, 892, 874, 856, 838, 820, 801, 783, 764, 745, 726, 707, 688, 669, 650, 630, 611, 592, 574, 555, 536, 518, 499, 481, 463, 446, 428, 411, 394, 378, 362, 346, 330, 315, 300, 285, 271, 257, 244, 231, 218, 206, 194, 183, 172, 161, 151, 141, 132, 123, 114, 106, 98, 91, 84, 77, 71, 65, 59, 54, 49, 44, 40, 36, 32, 29, 26, 23, 20, 18, 15, 13, 12, 10, 9, 7, 6, 5, 4, 3, 3, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 9, 10, 12, 13, 15, 18, 20, 23, 26, 29, 32, 36, 40, 44, 49, 54, 59, 65, 71, 77, 84, 91, 98, 106, 114, 123, 132, 141, 151, 161, 172, 183, 194, 206, 218, 231, 244, 257, 271, 285
  };

  // ======== On-chip ADC ========
  // Disabled; HAL_ADC_GetValue() disturbs timing of CS1237?
/*
  __HAL_RCC_ADC_CLK_ENABLE();
  adc1.Instance = ADC1;
  adc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  adc1.Init.Resolution = ADC_RESOLUTION_12B;
  adc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  adc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  adc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  adc1.Init.LowPowerAutoWait = DISABLE;
  adc1.Init.LowPowerAutoPowerOff = ENABLE;
  adc1.Init.ContinuousConvMode = DISABLE;
  adc1.Init.NbrOfConversion = 1;
  adc1.Init.DiscontinuousConvMode = DISABLE;
  adc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  adc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_LOW;
  HAL_ADC_Init(&adc1);

  ADC_ChannelConfTypeDef adc_ch;
  adc_ch.Channel = ADC_CHANNEL_VREFINT;
  adc_ch.Rank = ADC_REGULAR_RANK_1;
  adc_ch.SamplingTime = ADC_SAMPLETIME_79CYCLES_5; // Stablize
  HAL_ADC_ConfigChannel(&adc1, &adc_ch);

  HAL_ADCEx_Calibration_Start(&adc1);

  HAL_ADC_Start(&adc1);
  HAL_ADC_PollForConversion(&adc1, 1000);
  uint32_t adc_value = HAL_ADC_GetValue(&adc1);
  HAL_ADC_Stop(&adc1);
  uint32_t vdd_mV = 3000 * *VREFINT_CAL_ADDR / adc_value;
  swv_printf("ADC ref = %lu\n", *VREFINT_CAL_ADDR);
  swv_printf("ADC value = %lu, VDD = %lu mV\n", adc_value, vdd_mV);
  if (vdd_mV < 3100) {
    // Blinks: wait + 2 short + (1 long per 100 mV drop)
    HAL_Delay(200);
    for (int i = 0; i < 2; i++) {
      TIM14->CCR1 = 4000; HAL_Delay(60);
      TIM14->CCR1 = 0; HAL_Delay(60);
    }
    int blinks = (3100 - vdd_mV) / 100 + 1;
    for (int i = 0; i < blinks; i++) {
      TIM14->CCR1 = 4000; HAL_Delay(120);
      TIM14->CCR1 = 0; HAL_Delay(120);
    }
  }
*/

  uint8_t nrf_ch[3] = { 2, 26, 80};
  uint8_t ble_ch[3] = {37, 38, 39};
  uint8_t cur_ch = 2;

  uint8_t led_phase = 0;
  uint16_t timestamp = 0;
  uint32_t collected_data[15] = { 0 };

  uint8_t cur_offset_counter = 0;
  uint8_t startup_counter = 0;  // Counts up to 15, then stops

  while (!adc_updated) { }  // Wait for the first sample

  uint32_t last_tick = HAL_GetTick();
  while (true) {
    cur_ch = (cur_ch + 1) % 3;
    if (adc_updated) {
      timestamp += 1;   // May overflow and wrap around
      if (startup_counter < 15) startup_counter++;
      for (int i = 14; i > 0; i--) collected_data[i] = collected_data[i - 1];
      collected_data[0] = adc_value;
      adc_updated = 0;
      cur_offset_counter = 0;
    } else {
      cur_offset_counter = (cur_offset_counter + 1) % 5;
    }

    // Payload format:
    // 10 bits timestamp
    // 22*5 bits data
    uint8_t cur_offset =
      (cur_offset_counter < 3 ? 0 : (cur_offset_counter - 2) * 5);
    if (startup_counter >= 5 && startup_counter < 15) {
      // Prevent out-of-bound data
      if (cur_offset + 5 > startup_counter)
        cur_offset = startup_counter - 5;
    }
    uint8_t tx_payload[15];
    uint16_t tx_timestamp = (timestamp - cur_offset) & ((1 << 10) - 1);
    write_bits(tx_payload, 0, 10, tx_timestamp);
    for (int i = 0; i < 5; i++)
      write_bits(tx_payload, 10 + i * 22, 22,
        collected_data[cur_offset + i] >> 2);

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
    buf[p++] = 0x48;  // Address (byte reversed)
    buf[p++] = 0xFE;
    buf[p++] = 0x21;
    buf[p++] = 0x61;
    buf[p++] = 0x75;
    buf[p++] = 0xCB;  // Highest bit should be set
  /*
    buf[p++] = 2;     // AD length
    buf[p++] = 0x01;  // Type: Flags
    buf[p++] = 0x05;
  */
    buf[p++] = 3;     // AD length
    buf[p++] = 0x08;  // Type: Shortened Local Name
    buf[p++] = 'R';
    buf[p++] = 'T';
    buf[p++] = 16;    // AD length
    buf[p++] = 0xFF;  // Type: Manufacturer Specific Data
    for (int i = 0; i < 15; i++) buf[p++] = tx_payload[i];
    buf[1] = p - 2;   // Payload length

    // Encode packet
    ble_encode_packet(buf, p, ble_ch[cur_ch]);

    nRF_send(0x25, nrf_ch[cur_ch]); // RF_CH: Set channel
    nRF_send(0x27, 0x70); // STATUS: Clear status flags
    nRF_send(0xE1); // FLUSH_TX
    nRF_send(0xE2); // FLUSH_RX
    nRF_cmd_buf[0] = 0xA0;  // W_TX_PAYLOAD
    nRF_send_len(nRF_cmd_buf, p + 4);
    HAL_GPIO_WritePin(GPIOA, PIN_nRF_CE, GPIO_PIN_SET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(GPIOA, PIN_nRF_CE, GPIO_PIN_RESET);

    TIM14->CCR1 = LED_STEPS[++led_phase];
    swv_printf("ADC value = %06x\n", collected_data[0]);

    // Accurate delay
    uint32_t cur_tick = HAL_GetTick();
    if (cur_tick - last_tick >= 18) {
      last_tick = cur_tick;
    } else {
      while (HAL_GetTick() - last_tick < 18)
        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
      last_tick += 18;
    }
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
  // SPEED_SEL = 0b00: 10 Hz
  // PGA_SEL = 00: PGA gain = 1
  // ADC value normalized to [-0.5, +0.5)
  // Calculated R = R5 (see hardware design) * (1 / (0.5 + ADC value / (2^24)) - 1)
  uint8_t writes[2] = {0x65 << 1, 0b00000000};
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
  adc_value = adc_read();
  adc_updated = 1;
  __HAL_GPIO_EXTI_CLEAR_FALLING_IT(PIN_ADC_DA);
}
