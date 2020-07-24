/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include "cobs.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUF_SIZE_DRV			6
#define BUF_SIZE_DRV_SEND	7

#define M_PI 3.14159265358979323846
#define WHEEL_RADIUS 0.085
#define WHEEL_SEPARATION 0.3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
float speed_drv1 =0.0f;
float speed_drv2 =0.0f;
uint8_t buf_drv1[BUF_SIZE_DRV];							// float speed
uint8_t buf_drv2[BUF_SIZE_DRV];							// float speed
uint8_t buf_drv_send[BUF_SIZE_DRV_SEND];					// uint8_t code + float speed
int uart_drv1_ready, uart_drv2_ready;

uint8_t buf_drv_decoded1[BUF_SIZE_DRV - 2];
uint8_t buf_drv_decoded2[BUF_SIZE_DRV - 2];
uint8_t buf_drv_send_decoded[BUF_SIZE_DRV_SEND - 2];

uint8_t code;

double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

float	old_pos_drv1,old_pos_drv2,pos_drv1, pos_drv2;

double distance_per_count = (2 * 3.14159265 * WHEEL_RADIUS) / 360;

double deltaRight, deltaLeft;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void cmd_vel_callback( const geometry_msgs::Twist& msg){
		speed_drv1 = msg.linear.x + msg.angular.z/2;
		speed_drv2 = -msg.linear.x +  msg.angular.z/2 ;
		if (speed_drv1 > 1.0) speed_drv1 = 1.0;
		if (speed_drv1 < -1.0) speed_drv1 = -1.0;
		if (speed_drv2 > 1.0) speed_drv2 = 1.0;
		if (speed_drv2 < -1.0) speed_drv2 = -1.0;
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

ros::NodeHandle nh;

nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);
tf::TransformBroadcaster odom_broadcaster;

ros::Time current_time = nh.now();
ros::Time last_time = nh.now();
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_callback,1);
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  USART2->CR1 |= USART_CR1_RXNEIE;
  USART3->CR1 |= USART_CR1_RXNEIE;

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(odom_pub);
  odom_broadcaster.init(nh);

  int send_freq = 1000.0 / 100; //1hz
  int send_last = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
      if (nh.connected())
      {

          if(HAL_GetTick() - send_last > send_freq)
          {
        	  current_time = nh.now();

        	  if (uart_drv1_ready == 1)
				{
					old_pos_drv1=pos_drv1;
					cobs_decode(buf_drv1, BUF_SIZE_DRV, buf_drv_decoded1);
					memcpy(&pos_drv1, buf_drv_decoded1, sizeof(float));
					pos_drv1+=180;
					deltaRight = old_pos_drv1-pos_drv1;
					if(old_pos_drv1>300 && pos_drv1<60)
						deltaRight = -((360-old_pos_drv1)+pos_drv1);
					if(old_pos_drv1<60 && pos_drv1>300)
						deltaRight = (360-pos_drv1)+old_pos_drv1;

				}

				if (uart_drv2_ready == 1)
				{
					old_pos_drv2=pos_drv2;
					cobs_decode(buf_drv2, BUF_SIZE_DRV, buf_drv_decoded2);
					memcpy(&pos_drv2, buf_drv_decoded2, sizeof(float));
					pos_drv2+=180;
					deltaLeft = pos_drv2-old_pos_drv2;
					if(old_pos_drv2>300 && pos_drv2<60)
						deltaRight = (360-old_pos_drv2)+pos_drv2;
					if(old_pos_drv2<60 && pos_drv2>300)
						deltaRight = -((360-pos_drv2)+old_pos_drv2);
				}

        	      //compute odometry in a typical way given the velocities of the robot
    	          double dt = (current_time.toSec() - last_time.toSec());

				  double v_left = (deltaLeft * distance_per_count) / dt;
				  double v_right = (deltaRight * distance_per_count) / dt;

				  vx = ((v_right + v_left) / 2);
				  vy=0;
				  vth = (v_right - v_left)/WHEEL_SEPARATION;

        	      if (fabs(vth)>2)
        	    	 vth=0;

        	      double delta_x = (vx * cos(th)) * dt;
        	      double delta_y = (vx * sin(th)) * dt;
        	      double delta_th =vth * dt;

        	      x -= delta_x;
        	      y -= delta_y;
        	      th += delta_th;

        	      //since all odometry is 6DOF we'll need a quaternion created from yaw
        	      geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(th);

        	      //first, we'll publish the transform over tf
        	      geometry_msgs::TransformStamped odom_trans;
        	      odom_trans.header.stamp = current_time;
        	      odom_trans.header.frame_id = "odom";
        	      odom_trans.child_frame_id = "base_link";

        	      odom_trans.transform.translation.x = x;
        	      odom_trans.transform.translation.y = y;
        	      odom_trans.transform.translation.z = 0.0;
        	      odom_trans.transform.rotation = odom_quat;

        	      //send the transform
        	      odom_broadcaster.sendTransform(odom_trans);

        	      //next, we'll publish the odometry message over ROS
        	      nav_msgs::Odometry odom;
        	      odom.header.stamp = current_time;
        	      odom.header.frame_id = "odom";

        	      //set the position
        	      odom.pose.pose.position.x = x;
        	      odom.pose.pose.position.y = y;
        	      odom.pose.pose.position.z = 0.0;
        	      odom.pose.pose.orientation = odom_quat;

        	      //set the velocity
        	      odom.child_frame_id = "base_link";
        	      odom.twist.twist.linear.x = vx;
        	      odom.twist.twist.linear.y = vy;
        	      odom.twist.twist.angular.z = vth;

        	      //publish the message
        	      odom_pub.publish(&odom);

        	      last_time = current_time;
        	      send_last = HAL_GetTick();
          }
      }

      nh.spinOnce();

		buf_drv_send_decoded[0] = 'v';
		memcpy(buf_drv_send_decoded + 1, &speed_drv1, sizeof(float));
		cobs_encode(buf_drv_send_decoded, BUF_SIZE_DRV_SEND - 2, buf_drv_send);
		HAL_UART_Transmit(&huart2, buf_drv_send, BUF_SIZE_DRV_SEND, 0x0FFF);

		buf_drv_send_decoded[0] = 'v';
		memcpy(buf_drv_send_decoded + 1, &speed_drv2, sizeof(float));
		cobs_encode(buf_drv_send_decoded, BUF_SIZE_DRV_SEND - 2, buf_drv_send);
		HAL_UART_Transmit(&huart3, buf_drv_send, BUF_SIZE_DRV_SEND, 0x0FFF);



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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
