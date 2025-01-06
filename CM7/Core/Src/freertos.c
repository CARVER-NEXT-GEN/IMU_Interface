/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dma.h"
#include "iwdg.h"
#include "usart.h"
#include "gpio.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <std_msgs/msg/float64_multi_array.h>

#include <BNO086_SPI/BNO086_SPI.h>
#include <BNO055_I2C/BNO055.h>
#include <BNO055_I2C/BNO055_config.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RCSOFTCHECK(fn) if (fn != RCL_RET_OK) {};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
rcl_node_t node;

rcl_publisher_t f64array_pub;
double f64array_data[35];
std_msgs__msg__Float64MultiArray f64array_msg = {
    .data = { .data = f64array_data, .capacity = 35, .size = 35 }
};

uint32_t sync_counter = 0;

extern BNO055_t IMU_055_FRTOS;
//extern BNO086_t IMU_086_FRTOS;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 5000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void BNO086_Published();
void BNO055_Published();
void SensorsPublished();

void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
uint16_t calculate_average(uint16_t *buffer, uint16_t length);
void xlr8_publish(uint16_t xlr8);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

	// micro-ROS configuration
	rmw_uros_set_custom_transport(
	true,
	(void *) &huart3,
	cubemx_transport_open,
	cubemx_transport_close,
	cubemx_transport_write,
	cubemx_transport_read);

	rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	freeRTOS_allocator.allocate = microros_allocate;
	freeRTOS_allocator.deallocate = microros_deallocate;
	freeRTOS_allocator.reallocate = microros_reallocate;
	freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
	  printf("Error on default allocators (line %d)\n", __LINE__);
	}

	// micro-ROS app
	rcl_timer_t XLR8_timer;
	rclc_support_t support;
	rclc_executor_t executor;
	rcl_allocator_t allocator;
	rcl_init_options_t init_options;

	const unsigned int timer_period = RCL_MS_TO_NS(2);
	const int timeout_ms = 1000;
	int executor_num = 1;

	const rosidl_message_type_support_t * float64_multi_arr_type_support =
	  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray);

	allocator = rcl_get_default_allocator();

	executor = rclc_executor_get_zero_initialized_executor();

	init_options = rcl_get_zero_initialized_init_options();

	RCSOFTCHECK(rcl_init_options_init(&init_options, allocator));
	RCSOFTCHECK(rcl_init_options_set_domain_id(&init_options, 198));

	// create support init_options
	rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

	// create timer
	rclc_timer_init_default(&XLR8_timer, &support, timer_period, timer_callback);

	// create node
	rclc_node_init_default(&node, "uros_H7_Node", "", &support);

	// create publisher
	rclc_publisher_init_best_effort(&f64array_pub, &node, float64_multi_arr_type_support, "cubemx_imu_data");
	// create subscriber

	// create service server

	// create service client

	// create executor
	rclc_executor_init(&executor, &support.context, executor_num, &allocator);

	rclc_executor_add_timer(&executor, &XLR8_timer);

	rclc_executor_spin(&executor);
	rmw_uros_sync_session(timeout_ms);

	for(;;)
	{
	//	osDelay(10);
	}
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	if (timer != NULL) {
		SensorsPublished();
		if (sync_counter++ >= 254) {  // Sync session at lower frequency
			if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
				rmw_uros_sync_session(1000); // Sync if agent is reachable
			}
			else {
				// If the agent is not reachable, let the watchdog reset the MCU
				NVIC_SystemReset();
			}
//			rmw_uros_sync_session(1000);
			sync_counter = 0;
		}
		HAL_IWDG_Refresh(&hiwdg1);
	}

}

void SensorsPublished(){


//	// IMU_086 acceleration
//	f64array_msg.data.data[0] = IMU_086_FRTOS.acceleration.x;
//	f64array_msg.data.data[1] = IMU_086_FRTOS.acceleration.y;
//	f64array_msg.data.data[2] = IMU_086_FRTOS.acceleration.z;
//
//	// IMU_086 linear acceleration
//	f64array_msg.data.data[3] = IMU_086_FRTOS.linear_acceleration.x;
//	f64array_msg.data.data[4] = IMU_086_FRTOS.linear_acceleration.y;
//	f64array_msg.data.data[5] = IMU_086_FRTOS.linear_acceleration.z;
//
//	// IMU_086 angular velocity
//	f64array_msg.data.data[6] = IMU_086_FRTOS.angular_velocity.x;
//	f64array_msg.data.data[7] = IMU_086_FRTOS.angular_velocity.y;
//	f64array_msg.data.data[8] = IMU_086_FRTOS.angular_velocity.z;
//
//	// IMU_086 magnetometer
//	f64array_msg.data.data[9] = IMU_086_FRTOS.magnetometer.x;
//	f64array_msg.data.data[10] = IMU_086_FRTOS.magnetometer.y;
//	f64array_msg.data.data[11] = IMU_086_FRTOS.magnetometer.z;

	// IMU_055 acceleration
	f64array_msg.data.data[12] = IMU_055_FRTOS.accel.x;
	f64array_msg.data.data[13] = IMU_055_FRTOS.accel.y;
	f64array_msg.data.data[14] = IMU_055_FRTOS.accel.z;

	// IMU_055 linear acceleration
	f64array_msg.data.data[15] = IMU_055_FRTOS.lin_acc.x;
	f64array_msg.data.data[16] = IMU_055_FRTOS.lin_acc.y;
	f64array_msg.data.data[17] = IMU_055_FRTOS.lin_acc.z;

	// IMU_055 gyro (angular velocity)
	f64array_msg.data.data[18] = IMU_055_FRTOS.gyro.x;
	f64array_msg.data.data[19] = IMU_055_FRTOS.gyro.y;
	f64array_msg.data.data[20] = IMU_055_FRTOS.gyro.z;

	// IMU_055 magnetometer
	f64array_msg.data.data[21] = IMU_055_FRTOS.mag.x;
	f64array_msg.data.data[22] = IMU_055_FRTOS.mag.y;
	f64array_msg.data.data[23] = IMU_055_FRTOS.mag.z;

	// IMU_055 euler angles
	f64array_msg.data.data[24] = IMU_055_FRTOS.euler.roll;
	f64array_msg.data.data[25] = IMU_055_FRTOS.euler.pitch;
	f64array_msg.data.data[26] = IMU_055_FRTOS.euler.yaw;

	// IMU_086 quaternion
//	f64array_msg.data.data[27] = IMU_086_FRTOS.quaternion.i;
//	f64array_msg.data.data[28] = IMU_086_FRTOS.quaternion.j;
//	f64array_msg.data.data[29] = IMU_086_FRTOS.quaternion.k;
//	f64array_msg.data.data[30] = IMU_086_FRTOS.quaternion.w;

	// IMU_055 quaternion
	f64array_msg.data.data[31] = IMU_055_FRTOS.quat.x;
	f64array_msg.data.data[32] = IMU_055_FRTOS.quat.y;
	f64array_msg.data.data[33] = IMU_055_FRTOS.quat.z;
	f64array_msg.data.data[34] = IMU_055_FRTOS.quat.w;



    RCSOFTCHECK(rcl_publish(&f64array_pub, &f64array_msg, NULL));

}

/* USER CODE END Application */

