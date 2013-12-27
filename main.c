
/* Includes ------------------------------------------------------------------*/

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "main.h"
#include "stm32f4xx_conf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TESTRESULT_ADDRESS         0x080FFFFC
#define ALLTEST_PASS               0x00000000
#define ALLTEST_FAIL               0x55555555

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint16_t PrescalerValue = 0;

__IO uint32_t TimingDelay;
__IO uint8_t UserButtonPressed = 0x00;


/* Private TASK function prototypes -----------------------------------------------*/
static void LED_task(void *pvParameters);
static void button_task(void *pvParameters);
static void led_pwm_task(void *pvParameters);

/* Private Initialize Configuration */
static void RCC_Configuration(void);
static void GPIO_Configuration(void);
static void TIM_Configuration(void);


/**
  * @brief  This function handles EXTI0_IRQ Handler.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
  UserButtonPressed = 0x01;

  /* Clear the EXTI line pending bit */
  EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
}

int main(void)
{
  RCC_Configuration();
  TIM_Configuration();
  GPIO_Configuration();

  xTaskCreate(led_pwm_task,
             (signed portCHAR *) "LED Flash",
             512 /* stack size */, NULL,
             tskIDLE_PRIORITY + 5, NULL);

  /* Start running the tasks. */
  vTaskStartScheduler();

  return 0;
}

static void LED_task(void *pvParameters)
{
  RCC_ClocksTypeDef RCC_Clocks;
  uint8_t togglecounter = 0x00;

  while(1)
  {
      /* Toggle LED5 */
      STM_EVAL_LEDToggle(LED5);
      vTaskDelay(100);
      /* Toggle LED6 */
      STM_EVAL_LEDToggle(LED6);
      vTaskDelay(100);
  }
}

static void button_task(void *pvParameters)
{
    while (1)
    {
        /* Waiting User Button is pressed */
        if (UserButtonPressed == 0x01)
        {
            /* Toggle LED4 */
            STM_EVAL_LEDToggle(LED4);
            vTaskDelay(100);
            /* Toggle LED3 */
            STM_EVAL_LEDToggle(LED3);
            vTaskDelay(100);
        }
        /* Waiting User Button is Released */
        while (STM_EVAL_PBGetState(BUTTON_USER) == Bit_SET);
            UserButtonPressed = 0x00;
    }
}

static void led_pwm_task(void *pvParameters)
{
  volatile int i;
  int n = 1;
  uint16_t brightness = 0;
  uint16_t who_run = 1;

  while(1)  // Do not exit
  {

    if(brightness + n <= 0)
        who_run = (who_run + 1) % 4;

    if (((brightness + n) >= 3000) || ((brightness + n) <= 0))
      n = -n; // if  brightness maximum/maximum change direction

    brightness += n;
    // TIM4->CCR1 = brightness - 1;
    // TIM4->CCR2 = brightness - 1;
    // TIM4->CCR3 = brightness - 1;
    // TIM4->CCR4 = brightness - 1;

    //Light LEDs in turn
    switch(who_run){
        case 0:
            TIM4->CCR1 = brightness - 1; // set brightness
            break;
        case 1:
            TIM4->CCR2 = brightness - 1; // set brightness
            break;
        case 2:
            TIM4->CCR3 = brightness - 1; // set brightness
            break;
        case 3:
            TIM4->CCR4 = brightness - 1; // set brightness
            break;
    }
    for(i=0;i<1000;i++);  // delay
  }
}

/**
  * @brief  This function handles the test program fail.
  * @param  None
  * @retval None
  */
void Fail_Handler(void)
{
  /* Erase last sector */
  FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);
  /* Write FAIL code at last word in the flash memory */
  FLASH_ProgramWord(TESTRESULT_ADDRESS, ALLTEST_FAIL);

  while(1)
  {
    /* Toggle Red LED */
    STM_EVAL_LEDToggle(LED5);
    vTaskDelay(5);
  }
}


void vApplicationTickHook()
{
}

/* Set RCC Config */
static void RCC_Configuration(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
}

void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure); // Reset init structure

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);

    // Setup Blue & Green LED on STM32-Discovery Board to use PWM.
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            // Alt Function - Push Pull
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init( GPIOD, &GPIO_InitStructure );
}

/* Set Timer4 for PWM */
static void TIM_Configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_OCInitTypeDef       TIM_OCInitStruct;

    // Set PWM frequency equal 100Hz.
    // Let period equal 1000. Therefore, timer runs from zero to 1000. Gives 0.1Hz resolution.
    // Solving for prescaler gives 240.

    TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV4;
    TIM_TimeBaseInitStruct.TIM_Period = 1680 - 1;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 500 - 1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM4, &TIM_TimeBaseInitStruct );

    TIM_OCStructInit( &TIM_OCInitStruct );
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;

    // Initial duty cycle equals 0%. Value can range from zero to 65535.
    // TIM_Pulse = TIM4_CCR1 register (16 bits)
    TIM_OCInitStruct.TIM_Pulse = 65535; //(0=Always Off, 65535=Always On)

    TIM_OC1Init( TIM4, &TIM_OCInitStruct ); // Channel 1  LED
    TIM_OC2Init( TIM4, &TIM_OCInitStruct ); // Channel 2  LED
    TIM_OC3Init( TIM4, &TIM_OCInitStruct ); // Channel 3  LED
    TIM_OC4Init( TIM4, &TIM_OCInitStruct ); // Channel 4  LED

    TIM_Cmd( TIM4, ENABLE );
}
