#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f4xx.h"
#include "event_groups.h"
#include "config.h"
#include "mpu6050_driver.h"
#include "IMU.h"
#include "spi_communication.h"
#include "keypad.h"
#include "delay.h"

uint8_t first_call = 1;
float deadband = 5.0f;

TaskHandle_t readDataTask;
TaskHandle_t UItask;
TaskHandle_t decisionTask;

void collectData(void *argument);
void UItaskFunc(void *argument);
void decisionFunc(void *argument);

int main(void)
{
    systemClockConfig();
    gpioConfig();
    delayInit();

    delay_ms(100);

    i2cConfig();
    spiConfig();
    tim3Config();

    delay_ms(50);
    /* Initialize IMU and display modules. */
    mpuInit();

    MAX7219_Init();

    delay_ms(500);
    /* Measure sensor offsets before starting the control loop. */
    callibrationStuff();
    /* UI task reads keypad; decision task runs the control algorithm. */
    Display_Angle(0);
    xTaskCreate(UItaskFunc, "UI", 512, NULL, 1, &UItask);
    xTaskCreate(decisionFunc, "Decision", 512, NULL, 2, &decisionTask);

    first_call = 0;
    vTaskStartScheduler();

    while (1) {}
}

void UItaskFunc(void *argument)
{
	while(1)
	{
		Process_Keypad();
		vTaskDelay(pdMS_TO_TICKS(20));

	}
}
/* PID memory variables. */
/* PID gains tuned for the current mechanical system. */
static uint8_t recover_mode = 0;
float Kp = 59.21f;
float Ki = 250.51f;
float Kd = 2.793f;
float N = 50.f;
float i_sum = 0.0f;
float prev_err = 0.0f;
float prev_d_out = 0.0f;
static void PID_Reset(void)
{
    i_sum = 0.0f;
    prev_err = 0.0f;
    prev_d_out = 0.0f;
}
void decisionFunc(void *argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(10);
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		readMpuData();
		rawToDataAccel();
		rawToDataAngularVel();
		complementaryFilter();
		Display_MPU_Angle(currentAngle);
		if(system_ON)
		{
			/* Stop motion if the angle exceeds the safe range. */
			float faultRate=final_target_angle-currentAngle;
			if(currentAngle > 65.0f && faultRate > 0)
			{
				motorStop();
			}
			else if(currentAngle < -65.0f && faultRate < 0)
			{
				motorStop();
			}
			else if(faultRate>deadband)
			{
				motorRight(2450);
			}
			else if(faultRate<-deadband)
			{
				motorLeft(2450);
			}
			else
			{
				motorBreak();
			}
		}
		else if(pid_Mode)
		{
			float dtPid=0.01f;
			/* Smooth target transition to avoid sudden motor commands. */
			static float ramped_target = 0.0f;
			float ramp_speed = 2.0f;
			 /* Recovery mode brings the rod back from unsafe angles. */
		    if (currentAngle > 80)
		    {
		        recover_mode = 1;
		    }
		    else if (currentAngle < -80)
		    {
		        recover_mode = 2;
		    }

		    if (recover_mode == 1)
		    {
		        PID_Reset();

		        if (currentAngle > 80)
		        {
		            motorLeft(2450);
		        }
		        else
		        {
		            motorBreak();
		            recover_mode = 0;
		            ramped_target = currentAngle;
		        }

		        vTaskDelayUntil(&xLastWakeTime, xFrequency);
		        continue;
		    }
		    else if (recover_mode == 2)
		    {
		        PID_Reset();

		        if (currentAngle < -80)
		        {
		            motorRight(2450);
		        }
		        else
		        {
		            motorBreak();
		            recover_mode = 0;
		            ramped_target = currentAngle;
		        }

		        vTaskDelayUntil(&xLastWakeTime, xFrequency);
		        continue;
		    }
		    /* Apply target ramping. */
			if (final_target_angle > ramped_target) {
			    ramped_target += ramp_speed;
			    if (ramped_target > final_target_angle) ramped_target = final_target_angle;
			} else if (final_target_angle < ramped_target) {
			    ramped_target -= ramp_speed;
			    if (ramped_target < final_target_angle) ramped_target = final_target_angle;
			}

			float error = ramped_target - currentAngle;
			/* Small error zone prevents unnecessary motor vibration. */
			if(error > -1.5f && error < 1.5f)
			{
				error = 0.0f;
				i_sum = 0.0f;
			}
			/* Proportional term. */
			float p_out=Kp*error;

			/* Integral term with anti-windup limit. */
			i_sum += (error * dtPid);
			if(i_sum > 2.0f) i_sum = 2.0f;
			if(i_sum < -2.0f) i_sum = -2.0f;

			float i_out=i_sum*Ki;

			/* Filtered derivative term to reduce noise sensitivity. */
			float d_out = (Kd * N * (error - prev_err) + prev_d_out) / (1.0f + N * dtPid);
			prev_err=error;
			prev_d_out=d_out;
			float PID=p_out+i_out+d_out;
			  /* Minimum PWM helps the DC motor overcome static friction. */
			float min_pwm = 1900.0f;

			if (error > 1.5f || error < -1.5f)
			{
				if (PID > 0.0f)
				{
					uint32_t speed = (uint32_t)(min_pwm + PID);
					motorRight(speed);
				}
				else if (PID < 0.0f)
				{
					uint32_t speed = (uint32_t)(min_pwm + (-PID));
					motorLeft(speed);
				}
			}
			else
			{
				motorStop();
			}

		}
		else
		{
			 /* Stop motor and clear PID memory when no mode is active. */
			 i_sum=0.0f;
			 prev_err=0.0f;
			 motorStop();

		}

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

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
