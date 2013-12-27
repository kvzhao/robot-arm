
/* Includes ------------------------------------------------------------------*/

// FreeRTOS
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

// System setting
#include "main.h"
#include "stm32f4xx_conf.h"

// Lib
#include "servo.h"

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
static void servo_testing_task(void *pvParameters);

/* Private Initialize Configuration */
static void RCC_Configuration(void);
static void GPIO_Configuration(void);
static void TIM_Configuration(void);
static void USART_Configuration(void);

void USART_puts(USART_TypeDef* USARTx, volatile char *s);
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

    Servo_Configuration();

    xTaskCreate(servo_testing_task,
             (signed portCHAR *) "LED Flash",
             512 /* stack size */, NULL,
             tskIDLE_PRIORITY + 5, NULL);

  /* Start running the tasks. */
    vTaskStartScheduler();

  return 0;
}

static void servo_testing_task(void *pvParameters)
{
    while(1) {
    Servo_set_pos(60,0);
    vTaskDelay(500);
    Servo_set_pos(120,0);
    vTaskDelay(500);
    Servo_set_pos(60,0);
    vTaskDelay(500);
    Servo_set_pos(180,0);
    vTaskDelay(500);
/*
   Servo_set_pos(60,1);
    vTaskDelay(500);
    Servo_set_pos(120,1);
    vTaskDelay(500);
    Servo_set_pos(60,1);
    vTaskDelay(500);
    Servo_set_pos(180,1);
    vTaskDelay(500);

    Servo_set_pos(60,2);
    vTaskDelay(500);
    Servo_set_pos(120,2);
    vTaskDelay(500);
    Servo_set_pos(60,2);
    vTaskDelay(500);
    Servo_set_pos(180,2);
    vTaskDelay(500);

    Servo_set_pos(60,3);
    vTaskDelay(500);
    Servo_set_pos(120,3);
    vTaskDelay(500);
    Servo_set_pos(60,3);
    vTaskDelay(500);
    Servo_set_pos(180,3);
    vTaskDelay(500);
*/
    }
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

/* Set Timer for PWM */
static void TIM_Configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_OCInitTypeDef       TIM_OCInitStruct;

    // Set PWM frequency equal 50 Hz.which is standard frequency of servo
    // Let period equal 1000. Therefore, timer runs from zero to 1000. Gives 0.1Hz resolution.
    // Solving for prescaler gives 240.

    TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV4;
    TIM_TimeBaseInitStruct.TIM_Period = 1680 - 1;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 1000 - 1;
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

static void USART_Configuration()
{
        /* Enable APB2 peripheral clock for USART2
     * and PA2->TX, PA3->RX
     */
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

        /* Enable APB2 peripheral clock for USART2 */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

        /* GPIOA Configuration for USART2 */
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;  ///< PA2(TX) and PA3(RX) are used
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            ///< Configured pins as alternate function
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       ///< IO speed
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          ///< Output type as push-pull mode
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;           ///< Activates pullup resistor
        GPIO_Init(GPIOA, &GPIO_InitStructure);                  ///< Initial GPIOA

        /* Connect GPIO pins to AF */
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    /* Configuration for USART2 */
        USART_InitTypeDef USART_InitStructure;
        USART_InitStructure.USART_BaudRate = 9600;                      ///< Baudrate is set to 9600
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;     ///< Standard word length = 8 bits
        USART_InitStructure.USART_StopBits = USART_StopBits_1;          ///< Standard stop bit = 1 bit
        USART_InitStructure.USART_Parity = USART_Parity_No;             ///< Standard parity bit = NONE
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; ///< No flow control
        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; ///< To enable receiver and transmitter
        USART_Init(USART2, &USART_InitStructure);                       ///< Initial USART2

    /* Enable receiver interrupt for USART2 and
     * Configuration for NVIC
     * */
        USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);                  ///< Enable USART2 receiver interrupt

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;                        ///< Configure USART2 interrupt
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;       ///< Set the priority group of USART2 interrupt
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                        ///< Set the subpriority inside the group
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                                ///< Globally enable USART2 interrupt
        NVIC_Init(&NVIC_InitStructure);                                                                ///< Initial NVIC

    /* Enable USART2 */
        USART_Cmd(USART2, ENABLE);
}


/* This function is used to transmit a string of characters via
 * the USART specified in USARTx.
 *
 * It takes two arguments: USARTx --> can be any of the USARTs e.g. USART1, USART2 etc.
 *                                                    (volatile) char *s is the string you want to send
 *
 * Note: The string has to be passed to the function as a pointer because
 *                  the compiler doesn't know the 'string' data type. In standard
 *                  C a string is just an array of characters
 *
 * Note 2: At the moment it takes a volatile char because the received_string variable
 *                    declared as volatile char --> otherwise the compiler will spit out warnings
 * */
void USART_puts(USART_TypeDef* USARTx, volatile char *s) {

        while(*s){
                // wait until data register is empty
                while( !(USARTx->SR & 0x00000040) );

        USART_SendData(USARTx, *s);
                *s++;
        }
}

void USART2_IRQHandler(void){

        // check if the USART2 receive interrupt flag was set
        if( USART_GetITStatus(USART2, USART_IT_RXNE) ){

        // send back
        USART_SendData( USART2, USART2->DR );

        }
}
