#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/gpio.h"

#define TAG "RS485"

#define TXD      32
#define RXD      35
#define RE       27
#define DE       33
#define EN_9V    13

#define RTS   UART_PIN_NO_CHANGE

#define CTS   UART_PIN_NO_CHANGE

#define BUF_SIZE        127
#define BAUD_RATE       19200

// Read packet timeout
#define PACKET_READ_TICS        (100 / portTICK_RATE_MS)
#define STACK_SIZE    (2048)
#define PRIO          (10)
#define UART_PORT          UART_NUM_2

#define READ_TOUT          (3) // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks
#define GPIO_BIT_MASK  ((1ULL<<GPIO_NUM_13) | (1ULL<<GPIO_NUM_13))

static void uart_send(const int port, const char* str, uint8_t length)
{
    if (uart_write_bytes(port, str, length) != length) {
        ESP_LOGE(TAG, "Send data critical failure.");
        abort();
    }
}

void app_main(void)
{
  //Reconfigure pin 13 as GPIO
  gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = GPIO_BIT_MASK;
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	gpio_config(&io_conf);

  //Setting up UART
  const int uart_num = UART_PORT;
  uart_config_t uart_config = {
      .baud_rate = BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_2,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 122,
      .source_clk = UART_SCLK_APB,
  };

  // Set UART log level
  esp_log_level_set(TAG, ESP_LOG_INFO);

  ESP_LOGI(TAG, "Start RS485 application test and configure UART.");

  // Install UART driver (we don't need an event queue here)
  // In this example we don't even use a buffer for sending data.
  ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0));

  // Configure UART parameters
  ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

  ESP_LOGI(TAG, "UART set pins, mode and install driver.");

  // Set UART pins as per KConfig settings
  ESP_ERROR_CHECK(uart_set_pin(uart_num, TXD, RXD, RTS, CTS));

  // Set RS485 half duplex mode
  ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX));

  // Set read timeout of UART TOUT feature
  ESP_ERROR_CHECK(uart_set_rx_timeout(uart_num, READ_TOUT));

  // Allocate buffers for UART
  uint8_t* data = (uint8_t*) malloc(BUF_SIZE);

  ESP_LOGI(TAG, "UART Initialized.\r\n");


  ESP_LOGI(TAG, "Starting up sensor.\r\n");

  //Setting up tranceiver as master
  gpio_set_direction(RE, GPIO_MODE_OUTPUT);
  gpio_set_level(RE, 0);
  gpio_set_direction(DE, GPIO_MODE_OUTPUT);
  //gpio_set_level(DE, 0);

  gpio_set_level(EN_9V, 1);
  vTaskDelay(500 / portTICK_PERIOD_MS);
  ESP_LOGI(TAG, "4\r\n");
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  ESP_LOGI(TAG, "3\r\n");
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  ESP_LOGI(TAG, "2\r\n");
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  ESP_LOGI(TAG, "1\r\n");
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  ESP_LOGI(TAG, "Setup complete.\r\n");

  //Sensor firmware date quary
  uint8_t slaveID = 0x01;
  uint8_t functionCode = 0x03;
  uint8_t registerAddr_high = 0x03;
  uint8_t registerAddr_low = 0xFF;
  uint8_t numOfRegister_high = 0x00;
  uint8_t numOfRegister_low = 0x08;
  uint8_t crc_high = 0x74;
  uint8_t crc_low = 0x78;

  //Put quary in a array
  char reg_data[8];
  reg_data[0] = (char)slaveID;

  reg_data[1] = (char)functionCode;

  reg_data[2] = (char)registerAddr_high;
  reg_data[3] = (char)registerAddr_low;

  reg_data[4] = (char)numOfRegister_high;
  reg_data[5] = (char)numOfRegister_low;

  reg_data[6] = (char)crc_high;
  reg_data[7] = (char)crc_low;

  //Starting a modbus message
  gpio_set_level(RE, 1);
  //Send data
  /*for (int i = 0; i < 8; i++)
  {
    uart_send(uart_num, &reg_data[i], 1);
  }*/
  uart_send(uart_num, reg_data, (sizeof(reg_data)));

  //Ending a modbus message
  ets_delay_us(4600);
  gpio_set_level(RE, 0);
  //gpio_set_level(DE, 0);

  //Recive data_bits
  ESP_LOGI(TAG, "Getting data?\r\n");

  while(1)
  {
    int len = uart_read_bytes(uart_num, data, BUF_SIZE, PACKET_READ_TICS);
    for (int i = 0; i < len; i++)
    {
      printf("0x%.2x ",(uint8_t)data[i]);
    }
  }
}
