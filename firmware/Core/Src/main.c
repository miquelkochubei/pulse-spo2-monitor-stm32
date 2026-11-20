/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "max30102_for_stm32_hal.h"
#include <stdio.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "ssd1306_conf.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim22;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
max30102_t max30102;
volatile uint32_t g_ir_value = 0;
volatile uint32_t g_red_value = 0;
volatile uint8_t g_new_sample = 0;
uint8_t contact_present = 0;
uint8_t contact_present_prev = 0;
/* latch for pulse */
uint8_t pulse_latched = 0;
/* DC tracking */
uint32_t ir_baseline = 0;
uint32_t red_baseline = 0;
/* raw IR AC components */
int32_t ac_prev2 = 0;
int32_t ac_prev1 = 0;
int32_t ac_curr  = 0;
/* filtered IR AC components */
int32_t ac_filt = 0;
int32_t ac_filt_prev2 = 0;
int32_t ac_filt_prev1 = 0;
int32_t ac_filt_curr  = 0;
/* raw RED AC */
int32_t red_ac_curr = 0;
/* filtered RED AC */
int32_t red_ac_filt = 0;
int32_t red_ac_filt_curr = 0;
/* average AC magnitude for dynamic threshold */
uint32_t ac_abs_avg = 0;
uint32_t red_ac_abs_avg = 0;
int32_t valley_threshold = 120;
uint16_t settle_count = 0;
uint32_t last_beat_time_ms = 0;
uint32_t beat_interval_ms = 0;
uint32_t mean_beat_interval_ms = 0;
uint8_t bpm_inst = 0;
uint8_t bpm_avg = 0;

/* Accumulators for true overall average */
uint32_t total_bpm_sum = 0;
uint16_t total_beat_count = 0;

/* Variables to track boundaries and consecutive readings */
uint8_t reached_70_bpm = 0;
uint8_t consecutive_low_count = 0;
uint8_t consecutive_high_count = 0;

/* Fast recovery timer */
uint32_t recovery_timer_ms = 0;

/* averaging buffer */
uint8_t bpm_hist[8] = {0};
uint8_t bpm_hist_idx = 0;
uint8_t bpm_hist_count = 0;
/* beat-based SpO2, scaled by 10 */
uint16_t spo2_x10 = 0;
/* beat window accumulation */
uint32_t beat_ir_sum = 0;
uint32_t beat_red_sum = 0;
uint16_t beat_sample_count = 0;
uint32_t beat_ir_min = 0xFFFFFFFFU;
uint32_t beat_ir_max = 0U;
uint32_t beat_red_min = 0xFFFFFFFFU;
uint32_t beat_red_max = 0U;
/* lcd displayer */
uint8_t oled_ok = 0;
uint32_t oled_last_update_ms = 0;
/* bpm images interval */
uint8_t ecg_state = 0;
uint32_t ecg_last_toggle_ms = 0;
uint32_t ecg_interval_ms = 500;
/* --- Pulse test state machine --- */
typedef enum
{
    TEST_OFF = 0,
    TEST_RUNNING,
    TEST_SHOW_FINAL_RESULT, /* Added state to show result before WiFi */
    TEST_DONE,
    TEST_WIFI_INIT,
    TEST_WIFI_SENDING
} pulse_test_state_t;
pulse_test_state_t test_state = TEST_OFF;
uint32_t test_start_ms = 0;
#define PULSE_TEST_DURATION_MS 30000U   /* 30 seconds */
/* future button flag (for now simulate) */
volatile uint8_t btn_pressed = 0;

/* Buzzer PWM on TIM22_CH2 / PC7 */
uint8_t buzzer_on = 0;
uint32_t buzzer_last_beat_ms = 0;
uint32_t buzzer_on_start_ms = 0;

#define BUZZER_PULSE_MS   50U
#define BUZZER_PWM_DUTY   250U   /* 50% duty when ARR = 499 */

/* =========================
 * Buzzer melody / BPM logic
 * ========================= */

typedef enum
{
    BUZZER_IDLE = 0,
    BUZZER_MELODY_START,
    BUZZER_RUNNING_BPM,
	BUZZER_END_BEEP,
    BUZZER_MELODY_FINISH
} buzzer_mode_t;

typedef struct
{
    uint16_t freq_hz;
    uint16_t duration_ms;
} buzzer_note_t;

buzzer_mode_t buzzer_mode = BUZZER_IDLE;

uint32_t buzzer_step_start_ms = 0;
uint8_t buzzer_note_index = 0;

uint32_t buzzer_bpm_last_ms = 0;
uint32_t buzzer_bpm_on_start_ms = 0;
uint8_t buzzer_bpm_on = 0;
uint32_t buzzer_beep_start_ms = 0;

/* Für Elise opening motif, shortened */
const buzzer_note_t fur_elise_start[] =
{
    {659, 160},   /* E5  */
    {0,   40},
    {622, 160},   /* D#5 */
    {0,   40},
    {659, 160},   /* E5  */
    {0,   40},
    {622, 160},   /* D#5 */
    {0,   40},
    {659, 160},   /* E5  */
    {0,   40},
    {494, 160},   /* B4  */
    {0,   40},
    {587, 160},   /* D5  */
    {0,   40},
    {523, 160},   /* C5  */
    {0,   40},
    {440, 300}    /* A4  */
};

#define FUR_ELISE_NOTE_COUNT  (sizeof(fur_elise_start) / sizeof(fur_elise_start[0]))
#define BUZZER_BPM_TONE_HZ    2000U

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM22_Init(void);
static void MX_USART4_UART_Init(void);
/* USER CODE BEGIN PFP */
void OLED_DrawFlatLine(void);
void OLED_DrawPulseLine(void);
void Pulse_Reset(void);

void Buzzer_Off(void);
void Buzzer_SetToneHz(uint32_t freq_hz);
void Buzzer_StartMelodyStart(void);
void Buzzer_StartRunningBpm(void);

void Buzzer_StartEndBeep(void);
void Buzzer_StartMelodyFinish(void);
void Buzzer_Task(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern UART_HandleTypeDef huart4; // USART4 Handle from CubeMX

// --- Wi-Fi & Adafruit Configuration ---
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"
#define AIO_USERNAME "YOUR_AIO_USERNAME"
#define AIO_KEY "YOUR_AIO_KEY"
#define AIO_FEED_BPM "heartrate" // Replace with your exact feed name

#define ESP_EN_PORT GPIOC
#define ESP_EN_PIN GPIO_PIN_12

char rx_buffer[500];

// Blocking AT command function
uint8_t ESP_SendCommand(char* command, char* expected_response, uint32_t timeout) {
    memset(rx_buffer, 0, sizeof(rx_buffer));
    HAL_UART_Transmit(&huart4, (uint8_t*)command, strlen(command), 100);

    uint32_t start_time = HAL_GetTick();
    uint16_t rx_idx = 0;
    uint8_t temp_char;

    // The CPU gets trapped in this while loop until it gets "OK" or times out
    while ((HAL_GetTick() - start_time) < timeout) {

        // ---> FEEDING THE BUZZER <---
        // Calling this here allows the buzzer logic to check the clock
        // and toggle the hardware timer pins even while waiting for Wi-Fi.
        Buzzer_Task();

        if (HAL_UART_Receive(&huart4, &temp_char, 1, 10) == HAL_OK) {
            if (rx_idx < sizeof(rx_buffer) - 1) {
                rx_buffer[rx_idx++] = temp_char;
                if (strstr(rx_buffer, expected_response) != NULL) {
                    return 1; // Success
                }
            }
        }
    }
    return 0; // Timeout
}
void ESP_PowerOff(void) {
    HAL_GPIO_WritePin(ESP_EN_PORT, ESP_EN_PIN, GPIO_PIN_RESET);
}

void ESP_PowerOn(void) {
    HAL_GPIO_WritePin(ESP_EN_PORT, ESP_EN_PIN, GPIO_PIN_SET);
    HAL_Delay(1500); // Boot-up time for ESP
}
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */


	HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM22_Init();
  MX_USART4_UART_Init();
  /* USER CODE BEGIN 2 */
  // printf("System start\r\n");
  max30102_init(&max30102, &hi2c1);
  max30102_reset(&max30102);
  HAL_Delay(100);
  max30102_clear_fifo(&max30102);
  max30102_set_fifo_config(&max30102, max30102_smp_ave_8, 1, 7);
  max30102_set_led_pulse_width(&max30102, max30102_pw_16_bit);
  max30102_set_adc_resolution(&max30102, max30102_adc_2048);
  max30102_set_sampling_rate(&max30102, max30102_sr_100);
  /* moderate LED current: stronger than 0.8 mA, still far below the old value */
  max30102_set_led_current_1(&max30102, 4.0f);   /* RED */
  max30102_set_led_current_2(&max30102, 4.0f);   /* IR  */
  max30102_set_mode(&max30102, max30102_spo2);
  max30102_set_a_full(&max30102, 1);
  max30102_set_die_temp_en(&max30102, 1);
  max30102_set_die_temp_rdy(&max30102, 1);

  // printf("OLED init start\r\n");
  if (HAL_I2C_IsDeviceReady(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 3, 100) == HAL_OK)
  {
      // printf("OLED found at address 0x%02X\r\n", SSD1306_I2C_ADDR >> 1);
      ssd1306_Init();

      ssd1306_Fill(Black);
      ssd1306_SetCursor(0, 0);
      ssd1306_WriteString("HELLO WORLD", Font_7x10, White);

      ssd1306_SetCursor(0, 16);
      ssd1306_WriteString("OLED OK", Font_7x10, White);

      ssd1306_UpdateScreen();

      oled_ok = 1;
  }
  else
  {
      // printf("OLED not found\r\n");
      oled_ok = 0;
  }
  // printf("MAX30102 configured\r\n");


  /* TEST starts only by button */
  test_state = TEST_OFF;
  Pulse_Reset();
  // printf("TEST READY - PRESS BUTTON\r\n");
  /*Buzzer_StartMelodyStart();*/

  HAL_TIM_PWM_Start(&htim22, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim22, TIM_CHANNEL_2, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* TEMP: simulate button every few seconds */

	  static uint32_t btn_dbg_last_ms = 0;

	  	  if ((HAL_GetTick() - btn_dbg_last_ms) >= 500U)
	  	  {
	  	      btn_dbg_last_ms = HAL_GetTick();
	  	  }

	  	  if (btn_pressed)
	  	  {
	  	      btn_pressed = 0;

	  	      if ((test_state == TEST_OFF) || (test_state == TEST_DONE))
	  	      {
	  	          test_state = TEST_RUNNING;
	  	          test_start_ms = 0; /* CHANGED: Initialize to 0, wait for valid BPM */
	  	          Pulse_Reset();
	  	          // printf("TEST START BY BUTTON\r\n");
	  	          Buzzer_StartMelodyStart();
	  	      }
	  	      else
	  	      {
	  	          test_state = TEST_OFF;
	  	          Pulse_Reset();
	  	          // printf("TEST STOP BY BUTTON\r\n");
	  	          buzzer_mode = BUZZER_IDLE;
	  	          Buzzer_Off();
	                ESP_PowerOff(); // Ensure Wi-Fi dies if aborted
	  	      }
	  	  }

	  	  if (test_state == TEST_RUNNING)
	  	  {
	  	      /* CHANGED: Start the 30-second timer ONLY on the first valid reading */
	  	      if ((test_start_ms == 0) && (bpm_avg > 0))
	  	      {
	  	          test_start_ms = HAL_GetTick();
	  	      }

	  	      /* CHANGED: Ensure test_start_ms isn't 0 before checking duration */
	  	      if ((test_start_ms != 0) && ((HAL_GetTick() - test_start_ms) >= PULSE_TEST_DURATION_MS))
	  	      {
                  // OVERWRITE EMA WITH TRUE MATHEMATICAL AVERAGE AT TEST END
                  if (total_beat_count > 0)
                  {
                      bpm_avg = (uint8_t)(total_bpm_sum / total_beat_count);
                  }

	  	          if (bpm_avg > 0)
	  	          {
                      /* Change state to show the result on the LCD before starting WiFi */
	  	              test_state = TEST_SHOW_FINAL_RESULT;
                      test_start_ms = HAL_GetTick(); // Reuse test_start_ms as a delay timer

	  	              if (spo2_x10 > 0)
	  	              {
	  	                  // printf("TEST DONE True Avg BPM=%u SpO2=%u.%u%%\r\n", bpm_avg, spo2_x10 / 10U, spo2_x10 % 10U);
	  	              }
                      else
                      {
                          // printf("TEST DONE True Avg BPM=%u SpO2=--.-%%\r\n", bpm_avg);
                      }

                      /* Play the Fur Elise melody instead of the high-pitched beep */
	  	              Buzzer_StartMelodyFinish();
	  	          }
	  	          else
	  	          {
	  	              test_state = TEST_DONE;
	  	              // printf("TEST DONE NO VALID BPM\r\n");
	  	              Buzzer_StartMelodyFinish();
	  	          }
	  	      }
	  	  }
          else if (test_state == TEST_SHOW_FINAL_RESULT)
          {
              /* Wait 5 seconds displaying the final results on OLED before turning on WiFi */
              if ((HAL_GetTick() - test_start_ms) >= 5000U)
              {
                  test_state = TEST_WIFI_INIT;
              }
          }

	        /* --- Wi-Fi State Machine --- */
	        if (test_state == TEST_WIFI_INIT)
	        {
	            printf("Turning on Wi-Fi...\r\n");

	            ESP_PowerOn();

	            ESP_SendCommand("AT\r\n", "OK", 2000);
	            ESP_SendCommand("AT+CWMODE=1\r\n", "OK", 2000);
                ESP_SendCommand("AT+CIPSNTPCFG=1,2,\"pool.ntp.org\"\r\n", "OK", 2000);

	            char cmd[100];
	            snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", WIFI_SSID, WIFI_PASS);
	            printf("Connecting to AP...\r\n");
	            if(ESP_SendCommand(cmd, "OK", 10000)) {
	                printf("Wi-Fi Connected!\r\n");
	                test_state = TEST_WIFI_SENDING;
	            } else {
	                printf("Wi-Fi Connection Failed.\r\n");
	                ESP_PowerOff();
	                test_state = TEST_DONE;
	            }
	        }

	        else if (test_state == TEST_WIFI_SENDING)
	        {
	            printf("Sending to Adafruit...\r\n");

	            char payload[100];
	            snprintf(payload, sizeof(payload), "{\"value\": \"Avg Heartrate: %u, SpO2: %u.%u%%\"}", bpm_avg, spo2_x10 / 10U, spo2_x10 % 10U);

	            char http_req[350];
	            snprintf(http_req, sizeof(http_req),
	                     "POST /api/v2/%s/feeds/%s/data HTTP/1.1\r\n"
	                     "Host: io.adafruit.com\r\n"
	                     "X-AIO-Key: %s\r\n"
	                     "Content-Type: application/json\r\n"
	                     "Content-Length: %d\r\n\r\n"
	                     "%s",
	                     AIO_USERNAME, AIO_FEED_BPM, AIO_KEY, strlen(payload), payload);

	            ESP_SendCommand("AT+CIPSTART=\"TCP\",\"io.adafruit.com\",80\r\n", "OK", 5000);

	            char cip_cmd[30];
	            snprintf(cip_cmd, sizeof(cip_cmd), "AT+CIPSEND=%d\r\n", strlen(http_req));
	            if (ESP_SendCommand(cip_cmd, ">", 2000))
	            {
	                if (ESP_SendCommand(http_req, "SEND OK", 5000)) {
	                    printf("Data sent successfully to Adafruit!\r\n");
	                }
	            }

	            ESP_SendCommand("AT+CIPCLOSE\r\n", "OK", 2000);
	            ESP_PowerOff();
	            printf("Wi-Fi off. Idling.\r\n");

	            test_state = TEST_DONE;
	        }

	      /* OLED periodic BPM / SpO2 display */
	      if ((oled_ok == 1) && ((HAL_GetTick() - oled_last_update_ms) >= 250U))
	      {
	          char line[24];

	          oled_last_update_ms = HAL_GetTick();

	          if (bpm_avg > 0)
	          {
	              ecg_interval_ms = 60000U / bpm_avg / 2U;
	              if (ecg_interval_ms < 120U) ecg_interval_ms = 120U;

	              if ((HAL_GetTick() - ecg_last_toggle_ms) >= ecg_interval_ms)
	              {
	                  ecg_last_toggle_ms = HAL_GetTick();
	                  ecg_state = !ecg_state;
	              }
	          }
	          else { ecg_state = 0; }

	          ssd1306_Fill(Black);
	          ssd1306_SetCursor(0, 0);

	          /* Updated OLED Status Text */
	          if (test_state == TEST_OFF) { ssd1306_WriteString("TEST OFF", Font_7x10, White); }
	          else if (test_state == TEST_RUNNING) { ssd1306_WriteString("TEST RUNNING", Font_7x10, White); }
              else if (test_state == TEST_SHOW_FINAL_RESULT) { ssd1306_WriteString("TEST DONE", Font_7x10, White); }
	          else if (test_state == TEST_WIFI_INIT) { ssd1306_WriteString("WIFI CONNECTING", Font_7x10, White); }
	          else if (test_state == TEST_WIFI_SENDING) { ssd1306_WriteString("SENDING DATA", Font_7x10, White); }
	          else { ssd1306_WriteString("TEST DONE", Font_7x10, White); }

	          ssd1306_SetCursor(0, 14);
	          if (test_state == TEST_OFF) { ssd1306_WriteString("Press START", Font_7x10, White); }
	          else if (test_state == TEST_DONE || test_state == TEST_SHOW_FINAL_RESULT) { ssd1306_WriteString("Final result", Font_7x10, White); }
	          else if (test_state == TEST_WIFI_INIT || test_state == TEST_WIFI_SENDING) { ssd1306_WriteString("Please Wait...", Font_7x10, White); }
	          else
	          {
	              if (contact_present == 0) { ssd1306_WriteString("No contact", Font_7x10, White); }
	              else { ssd1306_WriteString("Contact OK", Font_7x10, White); }
	          }

	          ssd1306_SetCursor(0, 28);
	          if (bpm_avg > 0)
	          {
	              snprintf(line, sizeof(line), "BPM: %u", bpm_avg);
	              ssd1306_WriteString(line, Font_7x10, White);
	          }
	          else { ssd1306_WriteString("BPM: --", Font_7x10, White); }

	          ssd1306_SetCursor(64, 28);
	          if (spo2_x10 > 0)
	          {
	              snprintf(line, sizeof(line), "SpO2:%u.%u%%", spo2_x10 / 10U, spo2_x10 % 10U);
	              ssd1306_WriteString(line, Font_7x10, White);
	          }
	          else { ssd1306_WriteString("SpO2:--.-%", Font_7x10, White); }

	          /* ECG image area: y = 34..63 */
	          if ((contact_present == 1) && (bpm_avg > 0) && (ecg_state == 1) && (test_state == TEST_RUNNING))
	          {
	              OLED_DrawPulseLine();
	          }
	          else
	          {
	              OLED_DrawFlatLine();
	          }

	          ssd1306_UpdateScreen();
	      }

	      /* Buzzer melody / BPM handler */
	      Buzzer_Task();

	      /* MAX30102 handling */
	      max30102_interrupt_handler(&max30102);

	      /* Keep the rest of your MAX30102 logic exactly as it is below this line */
	      if (g_new_sample && (test_state == TEST_RUNNING))
	      {
	      	uint32_t now_ms;
	      	uint8_t i;
	      	uint32_t ac_abs;
	      	uint32_t red_ac_abs;

	          g_new_sample = 0;

	          /* Contact detection with hysteresis + AND logic */
	          if (!contact_present)
	          {
	              if ((g_ir_value > 18000) && (g_red_value > 15000))
	              {
	                  contact_present = 1;
	              }
	          }
	          else
	          {
	              if ((g_ir_value < 9000) && (g_red_value < 7000))
	              {
	                  contact_present = 0;
	              }
	          }

	          /* Detect transition: no contact -> contact */
	          if ((contact_present == 1) && (contact_present_prev == 0))
	          {
	              ir_baseline = g_ir_value;
	              red_baseline = g_red_value;

	              ac_prev2 = 0;
	              ac_prev1 = 0;
	              ac_curr  = 0;

	              ac_filt = 0;
	              ac_filt_prev2 = 0;
	              ac_filt_prev1 = 0;
	              ac_filt_curr  = 0;

	              red_ac_curr = 0;
	              red_ac_filt = 0;
	              red_ac_filt_curr = 0;

	              ac_abs_avg = 0;
	              red_ac_abs_avg = 0;
	              valley_threshold = 120;

	              settle_count = 0;

	              last_beat_time_ms = 0;
	              beat_interval_ms = 0;
	              mean_beat_interval_ms = 0;

	              bpm_inst = 0;
	              bpm_avg = 0;
	              spo2_x10 = 0;

                  total_bpm_sum = 0;
                  total_beat_count = 0;
                  reached_70_bpm = 0; // Reset condition on new contact
                  consecutive_low_count = 0; // Reset counts on new contact
                  consecutive_high_count = 0;
                  recovery_timer_ms = HAL_GetTick(); // Start the recovery timer

	              bpm_hist_idx = 0;
	              bpm_hist_count = 0;

	              pulse_latched = 0;

	              /* reset beat window */
	              beat_ir_sum = 0;
	              beat_red_sum = 0;
	              beat_sample_count = 0;
	              beat_ir_min = 0xFFFFFFFFU;
	              beat_ir_max = 0U;
	              beat_red_min = 0xFFFFFFFFU;
	              beat_red_max = 0U;

	              for (i = 0; i < 8; i++)
	              {
	                  bpm_hist[i] = 0;
	              }
	          }

	          contact_present_prev = contact_present;

	          /* No contact */
	          if (!contact_present)
	          {
	              pulse_latched = 0;
	              bpm_avg = 0;
	              spo2_x10 = 0;

                  total_bpm_sum = 0;
                  total_beat_count = 0;

	              /* reset beat window */
	              beat_ir_sum = 0;
	              beat_red_sum = 0;
	              beat_sample_count = 0;
	              beat_ir_min = 0xFFFFFFFFU;
	              beat_ir_max = 0U;
	              beat_red_min = 0xFFFFFFFFU;
	              beat_red_max = 0U;

	              // printf("IR=%lu RED=%lu No contact\r\n", g_ir_value, g_red_value);
	          }
	          else
	          {
	              /* Slow DC baseline tracking */
	              ir_baseline  = (127 * ir_baseline  + g_ir_value)  / 128;
	              red_baseline = (127 * red_baseline + g_red_value) / 128;

	              /* Raw IR AC */
	              ac_prev2 = ac_prev1;
	              ac_prev1 = ac_curr;
	              ac_curr  = (int32_t)g_ir_value - (int32_t)ir_baseline;

	              /* Raw RED AC */
	              red_ac_curr = (int32_t)g_red_value - (int32_t)red_baseline;

	              /* Integer IIR low-pass on IR AC */
	              ac_filt = (ac_filt + ac_curr) / 2;

	              ac_filt_prev2 = ac_filt_prev1;
	              ac_filt_prev1 = ac_filt_curr;
	              ac_filt_curr  = ac_filt;

	              /* Integer IIR low-pass on RED AC */
	              red_ac_filt = (3 * red_ac_filt + red_ac_curr) / 4;
	              red_ac_filt_curr = red_ac_filt;

	              /* Average absolute filtered AC magnitude */
	              ac_abs = (ac_filt_curr >= 0) ? (uint32_t)ac_filt_curr : (uint32_t)(-ac_filt_curr);
	              red_ac_abs = (red_ac_filt_curr >= 0) ? (uint32_t)red_ac_filt_curr : (uint32_t)(-red_ac_filt_curr);

	              ac_abs_avg = (7 * ac_abs_avg + ac_abs) / 8;
	              red_ac_abs_avg = (7 * red_ac_abs_avg + red_ac_abs) / 8;

	              /* accumulate current beat window for beat-based SpO2 */
	              beat_ir_sum += g_ir_value;
	              beat_red_sum += g_red_value;
	              beat_sample_count++;

	              if (g_ir_value < beat_ir_min)
	              {
	                  beat_ir_min = g_ir_value;
	              }
	              if (g_ir_value > beat_ir_max)
	              {
	                  beat_ir_max = g_ir_value;
	              }

	              if (g_red_value < beat_red_min)
	              {
	                  beat_red_min = g_red_value;
	              }
	              if (g_red_value > beat_red_max)
	              {
	                  beat_red_max = g_red_value;
	              }

	              /* Dynamic threshold */
                  /* RESTORED: Restored threshold divider to 4U (25%) to ignore ripples */
	              valley_threshold = (int32_t)(ac_abs_avg / 4U);

                  /* RESTORED: Restored absolute floor to 25 to ignore false noise spikes */
	              if (valley_threshold < 25)
	              {
	                  valley_threshold = 25;
	              }

	              if (valley_threshold > 500)
	              {
	                  valley_threshold = 500;
	              }

                  /* FAST RECOVERY MECHANISM */
                  // If the baseline filter stays huge for more than 1.5 seconds, the finger placement caused a massive spike.
                  // Hard reset the baseline math so it recovers instantly instead of counting down for 10 seconds.
                  if (ac_filt_curr > 4000)
                  {
                      if ((HAL_GetTick() - recovery_timer_ms) > 1500U)
                      {
                          // printf("FAST RECOVERY TRIGGERED - Resetting baselines\r\n");

                          // Reset baselines to current actuals
                          ir_baseline = g_ir_value;
                          red_baseline = g_red_value;

                          // Flush filters
                          ac_prev2 = ac_prev1 = ac_curr = 0;
                          ac_filt = ac_filt_prev2 = ac_filt_prev1 = ac_filt_curr = 0;
                          red_ac_curr = red_ac_filt = red_ac_filt_curr = 0;

                          // Reset thresholds
                          ac_abs_avg = 0;
                          red_ac_abs_avg = 0;
                          valley_threshold = 50;

                          // Reset the recovery timer
                          recovery_timer_ms = HAL_GetTick();
                      }
                  }
                  else
                  {
                      // If the filter is normal, keep resetting the timer
                      recovery_timer_ms = HAL_GetTick();
                  }

	              /* Settling phase - shortened */
	              if (settle_count < 3)
	              {
	                  settle_count++;
	                  // printf("IR=%lu RED=%lu AC=%ld RED_AC=%ld FILT=%ld TH=%ld settling\r\n",
	                  //        g_ir_value,
	                  //        g_red_value,
	                  //        (long)ac_curr,
	                  //        (long)red_ac_curr,
	                  //        (long)ac_filt_curr,
	                  //        (long)valley_threshold);
	              }
	              else
	              {
	                  now_ms = HAL_GetTick();

	                  /* Valley detection on FILTERED IR AC */
	                  if ((ac_filt_prev1 <= ac_filt_prev2) &&
	                      (ac_filt_prev1 <= ac_filt_curr) &&
	                      (ac_filt_prev1 < -valley_threshold))
	                  {
                        /* CHANGED: Refractory period raised to 400U (max 150 BPM).
                         * This specifically tunes out the 170+ BPM false peaks (Dicrotic Notches).
                         */
	                  	if ((last_beat_time_ms == 0) || ((now_ms - last_beat_time_ms) > 400U))
	                      {
	                          if (last_beat_time_ms != 0)
	                          {
	                              beat_interval_ms = now_ms - last_beat_time_ms;
	                              bpm_inst = (uint8_t)(60000U / beat_interval_ms);

	                              if ((bpm_inst >= 55) && (bpm_inst <= 120))
	                              {
                                      /* Track if we have ever hit >= 70 bpm in this test */
                                      if (bpm_inst >= 70)
                                      {
                                          reached_70_bpm = 1;
                                      }

                                      uint8_t accept_beat = 0;

                                      /* Upper bound check (85 +- 5 = 80 to 90) */
                                      if (bpm_inst >= 85 && bpm_inst <= 95)
                                      {
                                          consecutive_high_count++;
                                          consecutive_low_count = 0;
                                          if (consecutive_high_count >= 3)
                                          {
                                              accept_beat = 1;
                                          }
                                      }
                                      /* Lower bound check (only applies if we already hit 70) (62+-4=62 to 66) */
                                      else if (reached_70_bpm && (bpm_inst >= 60 && bpm_inst <= 66))
                                      {
                                          consecutive_low_count++;
                                          consecutive_high_count = 0;
                                          if (consecutive_low_count >= 3)
                                          {
                                              accept_beat = 1;
                                          }
                                      }
                                      /* Normal valid reading */
                                      else
                                      {
                                          consecutive_high_count = 0;
                                          consecutive_low_count = 0;
                                          accept_beat = 1;
                                      }

                                      if (accept_beat)
                                      {
                                          /* Accumulate for true final average */
                                          total_bpm_sum += bpm_inst;
                                          total_beat_count++;

	                              	      /* EMA on beat interval, not on BPM */
	                              	      if (mean_beat_interval_ms == 0U)
	                              	      {
	                              	          mean_beat_interval_ms = beat_interval_ms;
	                              	      }
	                              	      else
	                              	      {
	                              	          mean_beat_interval_ms =
	                              	              (mean_beat_interval_ms + beat_interval_ms) / 2U;
	                              	      }

	                              	      bpm_avg = (uint8_t)(60000U / mean_beat_interval_ms);

	                              	      // printf("BEAT interval=%lu bpm_inst=%u bpm_avg=%u mean_interval=%lu FILT=%ld TH=%ld\r\n",
	                              	      //        beat_interval_ms,
	                              	      //        bpm_inst,
	                              	      //        bpm_avg,
	                              	      //        mean_beat_interval_ms,
	                              	      //        (long)ac_filt_curr,
	                              	      //        (long)valley_threshold);

	                              	      if (bpm_avg > 0)
	                              	      {
	                              	          pulse_latched = 1;
	                              	      }

	                                        /* beat-based SpO2 calculation */
	                                        if ((beat_sample_count > 5U) &&
	                                            (beat_ir_max > beat_ir_min) &&
	                                            (beat_red_max > beat_red_min))
	                                        {
	                                        	uint32_t ir_dc;
	                                        	    uint32_t red_dc;
	                                        	    uint32_t ir_ac_pp;
	                                        	    uint32_t red_ac_pp;
	                                        	    uint64_t numerator;       /* CHANGED: Upgraded to 64-bit to prevent overflow */
	                                        	    uint32_t denominator;
	                                        	    uint32_t r_x1000;
	                                        	    uint16_t new_spo2_x10;

	                                        	    ir_dc = beat_ir_sum / beat_sample_count;
	                                        	    red_dc = beat_red_sum / beat_sample_count;

	                                        	    ir_ac_pp = beat_ir_max - beat_ir_min;
	                                        	    red_ac_pp = beat_red_max - beat_red_min;

	                                        	    if ((ir_dc > 0U) &&
	                                        	        (red_dc > 0U) &&
	                                        	        (ir_ac_pp > 20U) &&
	                                        	        (red_ac_pp > 10U))
	                                        	    {
	                                        	        /* CHANGED: Cast variables to uint64_t during multiplication */
	                                        	        numerator = (uint64_t)red_ac_pp * (uint64_t)ir_dc * 1000ULL;
	                                        	        denominator = ir_ac_pp * red_dc;

	                                        	        if (denominator > 0U)
	                                        	        {
	                                        	            /* Cast back to 32-bit after the division scales it down */
	                                        	            r_x1000 = (uint32_t)(numerator / (uint64_t)denominator);

	                                        	            if ((r_x1000 > 200U) && (r_x1000 < 1400U))
	                                        	            {
	                                        	                /* CHANGED: Updated to Maxim's standard SpO2 formula: 104 - 17*R */
	                                        	                /* 1040 - (17 * r_x1000 / 100) -> allows one decimal place precision */
	                                        	            	new_spo2_x10 = (uint16_t)(1120U - ((16U * r_x1000) / 100U));

	                                        	                /* Cap the boundaries between 90.0% and 100.0% */
	                                        	                if (new_spo2_x10 > 1000U)
	                                        	                {
	                                        	                    new_spo2_x10 = 1000U;
	                                        	                }
	                                        	                if (new_spo2_x10 < 900U)
	                                        	                {
	                                        	                    new_spo2_x10 = 900U;
	                                        	                }

	                                        	                if (spo2_x10 == 0U)
	                                        	                {
	                                        	                    spo2_x10 = new_spo2_x10;
	                                        	                }
	                                        	                else
	                                        	                {
	                                        	                    /* IIR Filter: 75% old, 25% new */
	                                        	                    spo2_x10 = (uint16_t)((3U * spo2_x10 + new_spo2_x10) / 4U);
	                                        	                }
	                                                    }
	                                                }
	                                            }
	                                        }
                                      }

	                                  /* reset beat window for next beat even if we ignored calculating it */
	                                  beat_ir_sum = 0;
	                                  beat_red_sum = 0;
	                                  beat_sample_count = 0;
	                                  beat_ir_min = 0xFFFFFFFFU;
	                                  beat_ir_max = 0U;
	                                  beat_red_min = 0xFFFFFFFFU;
	                                  beat_red_max = 0U;
	                              }
	                          }

	                          last_beat_time_ms = now_ms;
	                      }
	                  }

	                  if (pulse_latched)
	                  {
	                      if (spo2_x10 > 0)
	                      {
	                          // printf("IR=%lu RED=%lu AC=%ld RED_AC=%ld FILT=%ld TH=%ld BPM=%u SpO2=%u.%u%% Pulse detected\r\n",
	                          //        g_ir_value,
	                          //        g_red_value,
	                          //        (long)ac_curr,
	                          //        (long)red_ac_curr,
	                          //        (long)ac_filt_curr,
	                          //        (long)valley_threshold,
	                          //        bpm_avg,
	                          //        spo2_x10 / 10U,
	                          //        spo2_x10 % 10U);
	                      }
	                      else
	                      {
	                          // printf("IR=%lu RED=%lu AC=%ld RED_AC=%ld FILT=%ld TH=%ld BPM=%u SpO2=--.-%% Pulse detected\r\n",
	                          //        g_ir_value,
	                          //        g_red_value,
	                          //        (long)ac_curr,
	                          //        (long)red_ac_curr,
	                          //        (long)ac_filt_curr,
	                          //        (long)valley_threshold,
	                          //        bpm_avg);
	                      }
	                  }
	                  else
	                  {
	                      // printf("IR=%lu RED=%lu AC=%ld RED_AC=%ld FILT=%ld TH=%ld BPM=%u Contact detected\r\n",
	                      //        g_ir_value,
	                      //        g_red_value,
	                      //        (long)ac_curr,
	                      //        (long)red_ac_curr,
	                      //        (long)ac_filt_curr,
	                      //        (long)valley_threshold,
	                      //        bpm_avg);
	                  }
	              }
	          }
	      }

	      HAL_Delay(10);
  } /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00300617;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */
  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */
  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00503D58;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */
  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM22 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM22_Init(void)
{

  /* USER CODE BEGIN TIM22_Init 0 */

  /* USER CODE END TIM22_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM22_Init 1 */

  /* USER CODE END TIM22_Init 1 */
  htim22.Instance = TIM22;
  htim22.Init.Prescaler = 15;
  htim22.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim22.Init.Period = 499;
  htim22.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim22.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim22) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim22, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim22, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM22_Init 2 */

  /* USER CODE END TIM22_Init 2 */
  HAL_TIM_MspPostInit(&htim22);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART4_UART_Init(void)
{

  /* USER CODE BEGIN USART4_Init 0 */

  /* USER CODE END USART4_Init 0 */

  /* USER CODE BEGIN USART4_Init 1 */

  /* USER CODE END USART4_Init 1 */
  huart4.Instance = USART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART4_Init 2 */

  /* USER CODE END USART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Wifi_EN_GPIO_Port, Wifi_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT_MAX30102_Pin Reset_Button_Pin */
  GPIO_InitStruct.Pin = INT_MAX30102_Pin|Reset_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Wifi_EN_Pin */
  GPIO_InitStruct.Pin = Wifi_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Wifi_EN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* lcd bpm drawings */
void OLED_DrawFlatLine(void)
{
    ssd1306_Line(0, 56, 127, 56, White);
}

void OLED_DrawPulseLine(void)
{
    /* Left flat part */
    ssd1306_Line(0, 56, 35, 56, White);
    /* Small rise */
    ssd1306_Line(35, 56, 42, 48, White);
    /* Drop */
    ssd1306_Line(42, 48, 52, 63, White);
    /* Big peak */
    ssd1306_Line(52, 63, 66, 34, White);
    /* Fall after peak */
    ssd1306_Line(66, 34, 76, 60, White);
    /* Recovery */
    ssd1306_Line(76, 60, 95, 48, White);
    /* Back to flat */
    ssd1306_Line(95, 48, 105, 56, White);
    ssd1306_Line(105, 56, 127, 56, White);
}

void max30102_plot(uint32_t ir, uint32_t red)
{
    g_ir_value = ir;
    g_red_value = red;
    g_new_sample = 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_reset_btn_ms = 0;
    uint32_t now_ms;

    if (GPIO_Pin == INT_MAX30102_Pin)
    {
        max30102_on_interrupt(&max30102);
    }
    else if (GPIO_Pin == Reset_Button_Pin)
    {
        now_ms = HAL_GetTick();

        if ((now_ms - last_reset_btn_ms) >= 200U)
        {
            btn_pressed = 1;
            last_reset_btn_ms = now_ms;
        }
    }
}

void Buzzer_Off(void)
{
    __HAL_TIM_SET_COMPARE(&htim22, TIM_CHANNEL_2, 0);
}

void Buzzer_SetToneHz(uint32_t freq_hz)
{
    uint32_t tim_clk;
    uint32_t arr;
    uint32_t duty;

    if (freq_hz == 0U)
    {
        Buzzer_Off();
        return;
    }

    /*
     * TIM22 runs from PCLK2.
     * Prescaler = 15, so timer counter clock = PCLK2 / 16.
     */
    tim_clk = HAL_RCC_GetPCLK2Freq();

    arr = tim_clk / ((htim22.Init.Prescaler + 1U) * freq_hz);

    if (arr > 0U)
    {
        arr -= 1U;
    }

    if (arr < 20U)
    {
        arr = 20U;
    }

    if (arr > 65535U)
    {
        arr = 65535U;
    }

    __HAL_TIM_SET_AUTORELOAD(&htim22, arr);

    duty = (arr + 1U) / 2U;
    __HAL_TIM_SET_COMPARE(&htim22, TIM_CHANNEL_2, duty);
}

void Buzzer_StartMelodyStart(void)
{
    buzzer_mode = BUZZER_MELODY_START;
    buzzer_note_index = 0;
    buzzer_step_start_ms = HAL_GetTick();
    buzzer_bpm_on = 0;
    Buzzer_Off();
}

void Buzzer_StartRunningBpm(void)
{
    buzzer_mode = BUZZER_RUNNING_BPM;
    buzzer_bpm_last_ms = HAL_GetTick();
    buzzer_bpm_on_start_ms = 0;
    buzzer_bpm_on = 0;
    Buzzer_Off();
}

void Buzzer_StartEndBeep(void)
{
    buzzer_mode = BUZZER_END_BEEP;
    buzzer_beep_start_ms = HAL_GetTick();
    Buzzer_SetToneHz(4000U);
}

void Buzzer_StartMelodyFinish(void)
{
    buzzer_mode = BUZZER_MELODY_FINISH;
    buzzer_note_index = 0;
    buzzer_step_start_ms = HAL_GetTick();
    buzzer_bpm_on = 0;
    Buzzer_Off();
}

void Buzzer_Task(void)
{
    uint32_t now_ms;
    uint32_t elapsed_ms;
    uint32_t beat_period_ms;

    now_ms = HAL_GetTick();

    if ((buzzer_mode == BUZZER_MELODY_START) ||
        (buzzer_mode == BUZZER_MELODY_FINISH))
    {
        if (buzzer_note_index >= FUR_ELISE_NOTE_COUNT)
        {
            if (buzzer_mode == BUZZER_MELODY_START)
            {
                Buzzer_StartRunningBpm();
            }
            else
            {
                buzzer_mode = BUZZER_IDLE;
                Buzzer_Off();
            }

            return;
        }

        elapsed_ms = now_ms - buzzer_step_start_ms;

        Buzzer_SetToneHz(fur_elise_start[buzzer_note_index].freq_hz);

        if (elapsed_ms >= fur_elise_start[buzzer_note_index].duration_ms)
        {
            buzzer_note_index++;
            buzzer_step_start_ms = now_ms;
            Buzzer_Off();
        }

        return;
    }

    if (buzzer_mode == BUZZER_RUNNING_BPM)
    {
        if ((test_state != TEST_RUNNING) || (bpm_avg == 0U))
        {
            Buzzer_Off();
            return;
        }

        beat_period_ms = 60000U / bpm_avg;

        if ((buzzer_bpm_on == 0U) &&
            ((now_ms - buzzer_bpm_last_ms) >= beat_period_ms))
        {
            buzzer_bpm_last_ms = now_ms;
            buzzer_bpm_on_start_ms = now_ms;
            buzzer_bpm_on = 1U;

            Buzzer_SetToneHz(BUZZER_BPM_TONE_HZ);
        }

        if ((buzzer_bpm_on == 1U) &&
            ((now_ms - buzzer_bpm_on_start_ms) >= BUZZER_PULSE_MS))
        {
            buzzer_bpm_on = 0U;
            Buzzer_Off();
        }

        return;
    }

    if (buzzer_mode == BUZZER_END_BEEP)
    {
        if ((now_ms - buzzer_beep_start_ms) >= 1000U)
        {
            buzzer_mode = BUZZER_IDLE;
            Buzzer_Off();
        }
        return;
    }

    Buzzer_Off();
}

void Pulse_Reset(void)
{
    ir_baseline = g_ir_value;
    red_baseline = g_red_value;
    ac_prev2 = ac_prev1 = ac_curr = 0;
    ac_filt = ac_filt_prev2 = ac_filt_prev1 = ac_filt_curr = 0;

    red_ac_curr = red_ac_filt = red_ac_filt_curr = 0;

    ac_abs_avg = 0;
    red_ac_abs_avg = 0;
    valley_threshold = 50;

    settle_count = 0;

    last_beat_time_ms = 0;
    beat_interval_ms = 0;
    mean_beat_interval_ms = 0;

    bpm_inst = 0;
    bpm_avg = 0;
    spo2_x10 = 0;

    total_bpm_sum = 0;
    total_beat_count = 0;
    reached_70_bpm = 0; // Reset condition parameter
    consecutive_low_count = 0;
    consecutive_high_count = 0;
    recovery_timer_ms = 0; // Reset recovery timer

    bpm_hist_idx = 0;
    bpm_hist_count = 0;

    pulse_latched = 0;

    beat_ir_sum = 0;
    beat_red_sum = 0;
    beat_sample_count = 0;

    beat_ir_min = 0xFFFFFFFFU;
    beat_ir_max = 0;
    beat_red_min = 0xFFFFFFFFU;
    beat_red_max = 0;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
