/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 國立雲林科技大學 電子工程系「微計算機原理及應用」專案
  *                   智慧防盜泡麵微波爐守護系統 (STM32G474RETx @ 170MHz)
  *                   架構：CMSIS 純暫存器位元操作 + CubeMX 框架相容
  *                   更新功能：警報解除時強制檢查 PA1 是否插回接地
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    STATE_IDLE = 0,
    STATE_ARMED,
    STATE_UNLOCKED,
    STATE_ALARM
} SystemState_t;

#define UART_RX_BUFFER_SIZE 64
#define CLI_LINE_MAX_LEN    64
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static volatile SystemState_t current_state = STATE_IDLE;
static char system_password[16] = "12345";
static uint8_t failed_attempts = 0;

/* USART2 環形緩衝區 */
static volatile uint8_t rx_ring_buffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

/* CLI 命令行緩衝區 */
static char cli_line[CLI_LINE_MAX_LEN];
static uint16_t cli_index = 0;

/* EXTI13 按鍵事件旗標 */
static volatile bool exti13_flag = false;

/* 蜂鳴器發聲剩餘毫秒數 */
static volatile uint32_t buzzer_beep_remaining_ms = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void UART_SendChar(char ch);
static void UART_SendString(const char *str);
static void Buzzer_Tone_Toggle(void);
static void Buzzer_Mute(void);
static void Buzzer_TriggerTone(uint32_t duration_ms);
static void Trigger_Alarm(const char *reason);
static void Process_CLI_Command(char *cmd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  透過 CMSIS 暫存器 TDR 發送單一字元
  */
static void UART_SendChar(char ch)
{
    while (!(USART2->ISR & USART_ISR_TXE_TXFNF)); // 等待發送資料暫存器為空
    USART2->TDR = (uint8_t)ch;
}

/**
  * @brief  透過 USART2 發送字串
  */
static void UART_SendString(const char *str)
{
    while (*str) {
        if (*str == '\n') {
            UART_SendChar('\r');
        }
        UART_SendChar(*str++);
    }
}

/**
  * @brief  PB1 輸出方波震盪（約 2.5kHz，兼顧有源與無源蜂鳴器）
  */
static void Buzzer_Tone_Toggle(void)
{
    GPIOB->BSRR = (1U << 1);            // PB1 拉高
    for (volatile int d = 0; d < 700; d++); // 延遲約 200us
    GPIOB->BSRR = (1U << (1 + 16));     // PB1 拉低
    for (volatile int d = 0; d < 700; d++); // 延遲約 200us
}

/**
  * @brief  蜂鳴器靜音
  */
static void Buzzer_Mute(void)
{
    GPIOB->BSRR = (1U << (1 + 16));     // 拉低靜音
}

/**
  * @brief  排程提示音
  */
static void Buzzer_TriggerTone(uint32_t duration_ms)
{
    buzzer_beep_remaining_ms = duration_ms;
}

/**
  * @brief  觸發警報（只印出一次警報訊息，不狂跳洗版）
  */
static void Trigger_Alarm(const char *reason)
{
    current_state = STATE_ALARM;
    UART_SendString("\r\n\r\n=======================================================\r\n");
    UART_SendString(" [警報啟動] 偵測到非法操作！\r\n");
    if (reason != NULL) {
        UART_SendString(" 原因：");
        UART_SendString(reason);
        UART_SendString("\r\n");
    }
    UART_SendString(" 警示狀態：紅燈恆亮、蜂鳴器狂鳴。\r\n");
    UART_SendString(" 解除流程：【必須先插回 PA1 感測線（接地）】後，再輸入 'RESET'！\r\n");
    UART_SendString("=======================================================\r\n> ");
}

/**
  * @brief  HAL EXTI 下降緣中斷回呼（PC13 板載藍色按鈕）
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13) {
        exti13_flag = true;
    }
}

/**
  * @brief  CLI 命令解析器（中文介面）
  */
static void Process_CLI_Command(char *cmd)
{
    while (*cmd == ' ') cmd++;
    if (*cmd == '\0') return;

    /* 查詢狀態指令 */
    if (strncmp(cmd, "STATUS", 6) == 0) {
        char status_msg[160];
        const char *state_names[] = {"待機 (IDLE)", "守護上鎖 (ARMED)", "授權解鎖 (UNLOCKED)", "警報中 (ALARM)"};
        uint8_t pa1_val = (GPIOA->IDR & (1U << 1)) ? 1 : 0;
        snprintf(status_msg, sizeof(status_msg),
                 "\n[系統狀態] 模式: %s | 托盤感測線(PA1): %d (%s) | 密碼錯誤次數: %d\n> ",
                 state_names[current_state], pa1_val,
                 pa1_val == 0 ? "泡麵在位(接地)" : "托盤拔除(空)", failed_attempts);
        UART_SendString(status_msg);
        return;
    }

    switch (current_state) {
        case STATE_IDLE:
            if (strncmp(cmd, "LOCK", 4) == 0) {
                char *arg = cmd + 4;
                while (*arg == ' ') arg++;

                /* 檢查托盤感測線 PA1 是否接地（低電位 0 為在位） */
                if (GPIOA->IDR & (1U << 1)) {
                    UART_SendString("\n[錯誤] 托盤為空（PA1 未接地）！請先放置泡麵接好感測線後再上鎖。\n> ");
                } else {
                    if (strlen(arg) > 0) {
                        strncpy(system_password, arg, sizeof(system_password) - 1);
                        system_password[sizeof(system_password) - 1] = '\0';
                    }
                    failed_attempts = 0;
                    current_state = STATE_ARMED;
                    Buzzer_TriggerTone(120);
                    UART_SendString("\n[系統通知] 上鎖完成，系統進入【守護上鎖模式】。\n");
                    UART_SendString("[提示] 綠燈規律閃爍中。解鎖請直接輸入密碼。\n> ");
                }
            } else {
                UART_SendString("\n[待機指令] 輸入 'LOCK <密碼>' 進行上鎖，或輸入 'STATUS' 查看狀態。\n> ");
            }
            break;

        case STATE_ARMED:
            if (strcmp(cmd, system_password) == 0) {
                failed_attempts = 0;
                current_state = STATE_UNLOCKED;
                Buzzer_TriggerTone(200); // 授權成功嗶一聲
                UART_SendString("\n[認證成功] 密碼正確！微波爐門已解鎖（綠燈恆亮）。\n");
                UART_SendString("[取餐提示] 請開門並拔開 PA1 感測線取出泡麵。\n> ");
            } else {
                failed_attempts++;
                UART_SendString("\n[認證失敗] 密碼錯誤！\n");
                if (failed_attempts >= 3) {
                    /* 連續輸錯 3 次：短響兩聲警告後進入警報 */
                    for (int k = 0; k < 2; k++) {
                        for (int t = 0; t < 200; t++) Buzzer_Tone_Toggle();
                        HAL_Delay(80);
                    }
                    Trigger_Alarm("密碼連續輸錯達 3 次！");
                } else {
                    char err_msg[64];
                    snprintf(err_msg, sizeof(err_msg), "[警告] 密碼剩餘嘗試次數：%d\n> ", 3 - failed_attempts);
                    UART_SendString(err_msg);
                    Buzzer_TriggerTone(80);
                }
            }
            break;

        case STATE_UNLOCKED:
            UART_SendString("\n[取餐中] 請拔開 PA1 感測線取出泡麵，取餐後系統將自動返回待機。\n> ");
            break;

        case STATE_ALARM:
            if (strcmp(cmd, "RESET") == 0) {
                /* 安全防護檢查：必須先將 PA1 感測線插回 GND（低電位 0）才能解除警報 */
                if (GPIOA->IDR & (1U << 1)) {
                    UART_SendString("\n[拒絕解除] 泡麵尚未歸位（PA1 仍為高電位/拔開狀態）！\r\n"
                                    "[安全鎖定] 請先將 PA1 感測線插回 GND 接好，再輸入 'RESET'！\n> ");
                    Buzzer_TriggerTone(80); // 警示嗶聲
                } else {
                    current_state = STATE_IDLE;
                    failed_attempts = 0;
                    GPIOB->BSRR = (1U << (0 + 16)); // 紅燈滅
                    Buzzer_Mute();                  // 蜂鳴器停
                    UART_SendString("\n\n[警報解除] 感測線已確認歸位，系統復原並返回【待機模式】。\n> ");
                }
            } else {
                UART_SendString("\n[警報中] 請先插回 PA1 感測線，再輸入 'RESET' 解除警報！\n> ");
            }
            break;
    }
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 開啟 USART2 RXNE 暫存器接收中斷 */
  USART2->CR1 |= USART_CR1_RXNEIE_RXFNEIE;

  /* 設定並開啟 NVIC 中斷 */
  NVIC_SetPriority(USART2_IRQn, 1);
  NVIC_EnableIRQ(USART2_IRQn);
  NVIC_SetPriority(EXTI15_10_IRQn, 2);
  NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* 開機提示字串 */
  UART_SendString("\r\n=======================================================\r\n");
  UART_SendString("  國立雲林科技大學 電子工程系「微計算機原理及應用」\r\n");
  UART_SendString("  智慧防盜泡麵微波爐守護系統 (STM32G474RE)\r\n");
  UART_SendString("  核心時脈: 170 MHz | 架構: CMSIS 純暫存器位元控制\r\n");
  UART_SendString("=======================================================\r\n");
  UART_SendString("[待機中] 系統已就緒。預設密碼: 12345\r\n");
  UART_SendString("[操作引導] 請放妥泡麵(PA1接地)後輸入 'LOCK 密碼' 啟動守護。\r\n> ");

  /* 非阻塞計時變數 */
  uint32_t last_tick_led = HAL_GetTick();
  uint32_t last_tick_alarm = HAL_GetTick();

  /* PA5 綠燈呼吸燈參數 */
  const uint16_t pwm_period = 100;
  uint16_t duty_cycle = 0;
  int8_t duty_step = 1;
  uint32_t last_pwm_update_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t current_tick = HAL_GetTick();

    /* ========================================================================= */
    /* 1. CLI 環形緩衝區字元接收、即時回顯與退格鍵處理                             */
    /* ========================================================================= */
    while (rx_head != rx_tail) {
        uint8_t ch = rx_ring_buffer[rx_tail];
        rx_tail = (rx_tail + 1) % UART_RX_BUFFER_SIZE;

        if (ch == '\r' || ch == '\n') {
            UART_SendChar('\r');
            UART_SendChar('\n');
            cli_line[cli_index] = '\0';
            if (cli_index > 0) {
                Process_CLI_Command(cli_line);
                cli_index = 0;
            } else {
                UART_SendString("> ");
            }
        } else if (ch == '\b' || ch == 127) { // 處理退格鍵
            if (cli_index > 0) {
                cli_index--;
                UART_SendString("\b \b");
            }
        } else if (cli_index < (CLI_LINE_MAX_LEN - 1)) {
            cli_line[cli_index++] = (char)ch;
            UART_SendChar((char)ch); // 即時字元回顯
        }
    }

    /* ========================================================================= */
    /* 2. PC13 板載按鈕（門開關中斷）事件處理                                    */
    /* ========================================================================= */
    if (exti13_flag) {
        exti13_flag = false;
        if (current_state == STATE_ARMED) {
            Trigger_Alarm("未授權解鎖，強行開門（按下板載藍色按鈕 PC13）！");
        } else if (current_state == STATE_UNLOCKED) {
            UART_SendString("\r\n[門已開啟] 微波爐門開啟，請取出泡麵。\r\n> ");
            Buzzer_TriggerTone(60);
        } else if (current_state == STATE_IDLE) {
            UART_SendString("\r\n[門已開啟] 系統處於待機模式。\r\n> ");
        }
    }

    /* ========================================================================= */
    /* 3. 有限狀態機 (FSM) 核心執行區                                            */
    /* ========================================================================= */
    switch (current_state) {
        case STATE_IDLE:
            /* PB0 紅燈滅、蜂鳴器靜音 */
            GPIOB->BSRR = (1U << (0 + 16));
            if (buzzer_beep_remaining_ms == 0) Buzzer_Mute();

            /* PA5 綠燈非阻塞呼吸燈效果 */
            if (current_tick - last_pwm_update_tick >= 15) {
                last_pwm_update_tick = current_tick;
                duty_cycle += duty_step;
                if (duty_cycle >= pwm_period) {
                    duty_cycle = pwm_period;
                    duty_step = -1;
                } else if (duty_cycle <= 0) {
                    duty_cycle = 0;
                    duty_step = 1;
                }
            }

            for (uint16_t cycle = 0; cycle < pwm_period; cycle++) {
                if (cycle < duty_cycle) {
                    GPIOA->BSRR = (1U << 5);         // 點亮 PA5
                } else {
                    GPIOA->BSRR = (1U << (5 + 16));  // 熄滅 PA5
                }
            }
            break;

        case STATE_ARMED:
            /* PB0 紅燈滅、蜂鳴器靜音 */
            GPIOB->BSRR = (1U << (0 + 16));
            if (buzzer_beep_remaining_ms == 0) Buzzer_Mute();

            /* PA5 綠燈每 250ms 規律閃爍 */
            if (current_tick - last_tick_led >= 250) {
                last_tick_led = current_tick;
                if (GPIOA->ODR & (1U << 5)) {
                    GPIOA->BSRR = (1U << (5 + 16));
                } else {
                    GPIOA->BSRR = (1U << 5);
                }
            }

            /* 托盤防盜感測：若 PA1 未經解鎖遭拔開變為高電位 1 */
            if (GPIOA->IDR & (1U << 1)) {
                Trigger_Alarm("泡麵遭偷竊！未經密碼解鎖，托盤感測線 (PA1) 遭直接拔開！");
            }
            break;

        case STATE_UNLOCKED:
            /* PA5 綠燈恆亮，PB0 紅燈滅 */
            GPIOA->BSRR = (1U << 5);
            GPIOB->BSRR = (1U << (0 + 16));
            if (buzzer_beep_remaining_ms == 0) Buzzer_Mute();

            /* 授權取餐：主人拔開 PA1 感測線 */
            if (GPIOA->IDR & (1U << 1)) {
                UART_SendString("\r\n=======================================================\r\n");
                UART_SendString(" [取餐完畢] 泡麵已安全取出，請慢用！\r\n");
                UART_SendString("=======================================================\r\n");
                for (int t = 0; t < 600; t++) Buzzer_Tone_Toggle(); // 取餐長嗶一聲
                current_state = STATE_IDLE;
                UART_SendString("[待機中] 系統已自動返回待機模式。\r\n> ");
            }
            break;

        case STATE_ALARM:
            /* PA5 綠燈滅，PB0 紅燈恆亮 */
            GPIOA->BSRR = (1U << (5 + 16));
            GPIOB->BSRR = (1U << 0);

            /* 300ms 間歇週期：150ms 狂鳴、150ms 靜音 */
            if ((current_tick - last_tick_alarm) >= 300) {
                last_tick_alarm = current_tick;
            }

            if ((current_tick - last_tick_alarm) < 150) {
                Buzzer_Tone_Toggle(); // 產生警報頻率方波
            } else {
                Buzzer_Mute();        // 靜音
            }
            break;
    }

    /* ========================================================================= */
    /* 4. 非警報狀態下的短嗶聲驅動                                               */
    /* ========================================================================= */
    if (buzzer_beep_remaining_ms > 0 && current_state != STATE_ALARM) {
        Buzzer_Tone_Toggle();
        buzzer_beep_remaining_ms--;
        if (buzzer_beep_remaining_ms == 0) {
            Buzzer_Mute();
        }
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  由 stm32g4xx_it.c 中的 USART2_IRQHandler 呼叫的暫存器中斷函式
  */
void UART_RxInterrupt_Handler(void)
{
    if (USART2->ISR & USART_ISR_RXNE_RXFNE) {
        uint8_t data = (uint8_t)(USART2->RDR);
        uint16_t next_head = (rx_head + 1) % UART_RX_BUFFER_SIZE;
        if (next_head != rx_tail) {
            rx_ring_buffer[rx_head] = data;
            rx_head = next_head;
        }
    }

    if (USART2->ISR & USART_ISR_ORE) {
        USART2->ICR = USART_ICR_ORECF; // 清除 Overrun Error
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
