/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : National Yunlin University of Science and Technology (NYUST)
  *                   Department of Electronic Engineering
  *                   Course: Microcomputer Principles and Applications
  *                   Project: Smart Instant Noodle Guard System (Anti-theft FSM)
  *                   Target : STM32G474RETx (NUCLEO-G474RE) @ 170MHz
  *                   Design : CubeMX Structure with CMSIS Direct Register Operations
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

/* USART2 Ring Buffer */
static volatile uint8_t rx_ring_buffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

/* CLI Line Buffer */
static char cli_line[CLI_LINE_MAX_LEN];
static uint16_t cli_index = 0;

/* EXTI13 Event Flag */
static volatile bool exti13_flag = false;

/* Buzzer Tone Timer */
static volatile uint32_t buzzer_beep_remaining_ms = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void UART_SendChar(char ch);
static void UART_SendString(const char *str);
static void Buzzer_TriggerTone(uint32_t duration_ms);
static void Process_CLI_Command(char *cmd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Transmit a single character via USART2 using CMSIS TDR register
  */
static void UART_SendChar(char ch)
{
    while (!(USART2->ISR & USART_ISR_TXE_TXFNF)); // Wait until TX FIFO / register empty
    USART2->TDR = (uint8_t)ch;
}

/**
  * @brief  Transmit a null-terminated string via USART2
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
  * @brief  Schedule a non-blocking confirmation beep
  */
static void Buzzer_TriggerTone(uint32_t duration_ms)
{
    buzzer_beep_remaining_ms = duration_ms;
}

/**
  * @brief  HAL EXTI Falling Edge Callback for PC13 (Blue Button)
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13) {
        exti13_flag = true;
    }
}

/**
  * @brief  Parse and execute CLI commands
  */
static void Process_CLI_Command(char *cmd)
{
    while (*cmd == ' ') cmd++;
    if (*cmd == '\0') return;

    if (strncmp(cmd, "STATUS", 6) == 0) {
        char status_msg[128];
        const char *state_names[] = {"IDLE", "ARMED", "UNLOCKED", "ALARM"};
        uint8_t pa1_val = (GPIOA->IDR & (1U << 1)) ? 1 : 0;
        snprintf(status_msg, sizeof(status_msg),
                 "\n[STATUS] State: %s | Tray Sensor PA1: %d (%s) | Failed Pwd: %d\n> ",
                 state_names[current_state], pa1_val,
                 pa1_val == 0 ? "Noodle Present" : "Empty", failed_attempts);
        UART_SendString(status_msg);
        return;
    }

    switch (current_state) {
        case STATE_IDLE:
            if (strncmp(cmd, "LOCK", 4) == 0) {
                char *arg = cmd + 4;
                while (*arg == ' ') arg++;

                /* Check Tray Sensor: PA1 must be connected to GND (low = 0) */
                if (GPIOA->IDR & (1U << 1)) {
                    UART_SendString("\n[ERROR] Tray is empty (PA1 is HIGH)! Place instant noodles before locking.\n> ");
                } else {
                    if (strlen(arg) > 0) {
                        strncpy(system_password, arg, sizeof(system_password) - 1);
                        system_password[sizeof(system_password) - 1] = '\0';
                    }
                    failed_attempts = 0;
                    current_state = STATE_ARMED;
                    Buzzer_TriggerTone(100);
                    UART_SendString("\n[SYSTEM] Lock Confirmed. System entered STATE_ARMED.\n> ");
                }
            } else {
                UART_SendString("\n[IDLE CMD] Use 'LOCK <password>' to arm system or 'STATUS'.\n> ");
            }
            break;

        case STATE_ARMED:
            if (strcmp(cmd, system_password) == 0) {
                failed_attempts = 0;
                current_state = STATE_UNLOCKED;
                Buzzer_TriggerTone(200);
                UART_SendString("\n[AUTH SUCCESS] Password accepted! Microwave door unlocked.\n"
                                "[INFO] Safe to open door (PC13) and remove noodle tray (PA1).\n> ");
            } else {
                failed_attempts++;
                UART_SendString("\n[AUTH FAILED] Incorrect password!\n");
                if (failed_attempts >= 3) {
                    UART_SendString("[ALERT] 3 Failed attempts! Intrusion threshold reached!\n");
                    for (int k = 0; k < 2; k++) {
                        GPIOB->BSRR = (1U << 1);
                        HAL_Delay(80);
                        GPIOB->BSRR = (1U << (1 + 16));
                        HAL_Delay(80);
                    }
                    current_state = STATE_ALARM;
                } else {
                    char err_msg[64];
                    snprintf(err_msg, sizeof(err_msg), "[WARNING] Attempts left: %d\n> ", 3 - failed_attempts);
                    UART_SendString(err_msg);
                    Buzzer_TriggerTone(80);
                }
            }
            break;

        case STATE_UNLOCKED:
            UART_SendString("\n[UNLOCKED] Please remove your noodles (disconnect PA1 wire) to finish.\n> ");
            break;

        case STATE_ALARM:
            if (strcmp(cmd, "RESET") == 0) {
                current_state = STATE_IDLE;
                failed_attempts = 0;
                GPIOB->BSRR = (1U << (0 + 16)); // Red LED OFF
                GPIOB->BSRR = (1U << (1 + 16)); // Buzzer Mute
                UART_SendString("\n[ALARM RESET] System reset complete. Returning to STATE_IDLE.\n> ");
            } else {
                UART_SendString("\n[SECURITY LOCKOUT] System under ALARM! Type 'RESET' to deactivate!\n> ");
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
  /* Enable USART2 RXNE (Receive Data Register Not Empty) Interrupt via CMSIS */
  USART2->CR1 |= USART_CR1_RXNEIE_RXFNEIE;

  /* Explicitly enable NVIC lines for USART2 and EXTI15_10 */
  NVIC_SetPriority(USART2_IRQn, 1);
  NVIC_EnableIRQ(USART2_IRQn);
  NVIC_SetPriority(EXTI15_10_IRQn, 2);
  NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* Welcome Banner */
  UART_SendString("\r\n=======================================================\r\n");
  UART_SendString("  NYUST EE - Microcomputer Principles & Applications\r\n");
  UART_SendString("  Instant Noodle Anti-Theft Guard System (STM32G474RE)\r\n");
  UART_SendString("  System Clock: 170 MHz | Architecture: CMSIS Register\r\n");
  UART_SendString("=======================================================\r\n");
  UART_SendString("[IDLE] System Ready. Default Password: 12345\r\n");
  UART_SendString("[HELP] Type 'LOCK <password>' to protect your meal.\r\n> ");

  /* Non-blocking Timing Variables */
  uint32_t last_tick_led = HAL_GetTick();
  uint32_t last_tick_alarm = HAL_GetTick();
  uint32_t last_tick_alarm_msg = HAL_GetTick();

  /* Software PWM Breathing parameters for PA5 */
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
    /* 1. CLI Ring Buffer Parser with Echo & Backspace Handling                  */
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
        } else if (ch == '\b' || ch == 127) { // Handle Backspace / Delete
            if (cli_index > 0) {
                cli_index--;
                UART_SendString("\b \b");
            }
        } else if (cli_index < (CLI_LINE_MAX_LEN - 1)) {
            cli_line[cli_index++] = (char)ch;
            UART_SendChar((char)ch); // Interactive Terminal Echo
        }
    }

    /* ========================================================================= */
    /* 2. PC13 External Interrupt Event Handler                                  */
    /* ========================================================================= */
    if (exti13_flag) {
        exti13_flag = false;
        if (current_state == STATE_ARMED) {
            UART_SendString("\r\n[SECURITY BREACH] Door Opened while ARMED without Authorization!\r\n");
            current_state = STATE_ALARM;
        } else if (current_state == STATE_UNLOCKED) {
            UART_SendString("\r\n[INFO] Microwave Door Opened. Please retrieve your meal.\r\n> ");
            Buzzer_TriggerTone(50);
        } else if (current_state == STATE_IDLE) {
            UART_SendString("\r\n[INFO] Microwave Door Opened (Standby).\r\n> ");
        }
    }

    /* ========================================================================= */
    /* 3. Finite State Machine (FSM) Execution Loop                              */
    /* ========================================================================= */
    switch (current_state) {
        case STATE_IDLE:
            /* Turn off Red LED (PB0 = 0) */
            GPIOB->BSRR = (1U << (0 + 16));

            /* PA5 Software PWM Breathing Light */
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

            /* Fast loop software PWM slice */
            for (uint16_t cycle = 0; cycle < pwm_period; cycle++) {
                if (cycle < duty_cycle) {
                    GPIOA->BSRR = (1U << 5);         // High
                } else {
                    GPIOA->BSRR = (1U << (5 + 16));  // Low
                }
            }
            break;

        case STATE_ARMED:
            /* Turn off Red LED (PB0 = 0) */
            GPIOB->BSRR = (1U << (0 + 16));

            /* PA5 Green LED Blink @ 250ms interval */
            if (current_tick - last_tick_led >= 250) {
                last_tick_led = current_tick;
                if (GPIOA->ODR & (1U << 5)) {
                    GPIOA->BSRR = (1U << (5 + 16));
                } else {
                    GPIOA->BSRR = (1U << 5);
                }
            }

            /* Anti-theft Guard: Check PA1 Tray Sensor (拔開為 1) */
            if (GPIOA->IDR & (1U << 1)) {
                UART_SendString("\r\n[INTRUSION ALERT] Meal was pulled away unlawfully! Tray wire disconnected!\r\n");
                current_state = STATE_ALARM;
            }
            break;

        case STATE_UNLOCKED:
            /* PA5 Green LED Constant ON, PB0 Red LED OFF */
            GPIOA->BSRR = (1U << 5);
            GPIOB->BSRR = (1U << (0 + 16));

            /* Check if meal has been safely taken out (PA1 wire disconnected = 1) */
            if (GPIOA->IDR & (1U << 1)) {
                UART_SendString("\r\n=======================================================\r\n");
                UART_SendString(" [BON APPETIT] Noodles retrieved safely! Enjoy your meal!\r\n");
                UART_SendString("=======================================================\r\n");
                Buzzer_TriggerTone(300);
                HAL_Delay(350);
                current_state = STATE_IDLE;
                UART_SendString("[IDLE] System returned to standby.\r\n> ");
            }
            break;

        case STATE_ALARM:
            /* PA5 Green LED OFF, PB0 Red LED Solid ON */
            GPIOA->BSRR = (1U << (5 + 16));
            GPIOB->BSRR = (1U << 0);

            /* Intermittent 150ms buzzer oscillation cycle */
            if ((current_tick - last_tick_alarm) >= 300) {
                last_tick_alarm = current_tick;
            }

            /* During the active 150ms half-cycle: toggle PB1 for ~2.5kHz square wave */
            if ((current_tick - last_tick_alarm) < 150) {
                GPIOB->BSRR = (1U << 1);
                for (volatile int d = 0; d < 80; d++);
                GPIOB->BSRR = (1U << (1 + 16));
                for (volatile int d = 0; d < 80; d++);
            } else {
                GPIOB->BSRR = (1U << (1 + 16));
            }

            /* Periodic Warning Terminal Broadcast */
            if (current_tick - last_tick_alarm_msg >= 2000) {
                last_tick_alarm_msg = current_tick;
                UART_SendString("\r\n[!!! SIREN ACTIVATED !!!] THEFT DETECTED! TYPE 'RESET' TO DEACTIVATE!\r\n> ");
            }
            break;
    }

    /* ========================================================================= */
    /* 4. Single Short Confirmation Beep Generator                               */
    /* ========================================================================= */
    if (buzzer_beep_remaining_ms > 0 && current_state != STATE_ALARM) {
        GPIOB->BSRR = (1U << 1);
        for (volatile int d = 0; d < 80; d++);
        GPIOB->BSRR = (1U << (1 + 16));
        for (volatile int d = 0; d < 80; d++);
        buzzer_beep_remaining_ms--;
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
  * @brief  USART2 RXNE Interrupt Hook (Called directly from stm32g4xx_it.c or custom handler)
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
        USART2->ICR = USART_ICR_ORECF; // Clear Overrun Error
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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
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
