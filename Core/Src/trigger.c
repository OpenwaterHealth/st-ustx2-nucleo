/*
 * trigger.c
 *
 *  Created on: Mar 14, 2024
 *      Author: gvigelet
 */

#include "main.h"
#include "trigger.h"
#include "cJSON.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

static OW_TimerData _timerDataConfig;
static OW_TriggerConfig _triggerConfig;

static void updateTimerDataFromPeripheral(TIM_HandleTypeDef *htim, uint32_t channel)
{
	// Assuming you have the timer configuration and status
	uint32_t preScaler = htim->Instance->PSC;
	uint32_t timerClockFrequency = HAL_RCC_GetPCLK2Freq() / (preScaler + 1);
	uint32_t TIM_ARR = htim->Instance->ARR;
	uint32_t TIM_CCRx = HAL_TIM_ReadCapturedValue(htim, channel);
	_timerDataConfig.TriggerMode = 0;
	_timerDataConfig.TriggerPulseCount = 0;
	_timerDataConfig.TriggerFrequencyHz = timerClockFrequency / (TIM_ARR + 1);

	uint32_t pulseWidthUs = (TIM_CCRx * 1000000) / timerClockFrequency;

	_timerDataConfig.TriggerPulseWidthUsec = pulseWidthUs; // Set the pulse width as needed

	// Check the timer status to determine if it's running
	_timerDataConfig.TriggerStatus = TIM_CHANNEL_STATE_GET(htim, channel);
}


static void timerDataToJson(char *jsonString, size_t max_length)
{
	memset(jsonString, 0, max_length);
	snprintf(jsonString, max_length,
			 "{"
			 "\"TriggerFrequencyHz\": %lu,"
			 "\"TriggerMode\": \"%s\","
			 "\"TriggerPulseCount\": %lu,"
			 "\"TriggerPulseWidthUsec\": %lu,"
			 "\"TriggerStatus\": \"%s\""
			 "}",
			 _timerDataConfig.TriggerFrequencyHz, _timerDataConfig.TriggerMode > 0 ? "PULSECOUNT" : "CONTINUOUS",
			 _timerDataConfig.TriggerPulseCount, _timerDataConfig.TriggerPulseWidthUsec,
			 _timerDataConfig.TriggerStatus == HAL_TIM_CHANNEL_STATE_BUSY ? "RUNNING" : "STOPPED");
}

static void errorToJson(char *jsonString, size_t max_length)
{
	memset(jsonString, 0, max_length);
	snprintf(jsonString, max_length,
			 "{"
			 "\"TriggerFrequencyHz\": 0,"
			 "\"TriggerMode\": \"%s\","
			 "\"TriggerPulseCount\": 0,"
			 "\"TriggerPulseWidthUsec\": 0,"
			 "\"TriggerStatus\": \"%s\""
			 "}",
			 "UNKNOWN",
			 "NOT CONFIGURED");
}

static int jsonToTimerData(const char *jsonString)
{
	// Parse the JSON string and extract values
	cJSON *rootTimerConfig = cJSON_Parse(jsonString);

	// Check if parsing was successful
	if (rootTimerConfig == NULL)
	{
		const char *error_ptr = cJSON_GetErrorPtr();
		if (error_ptr != NULL)
		{
			fprintf(stderr, "Error before: %s\n", error_ptr);
		}
		return 1;
	}

	// Access values from the cJSON object
	cJSON *triggerFrequency = cJSON_GetObjectItem(rootTimerConfig, "TriggerFrequencyHz");
	cJSON *triggerMode = cJSON_GetObjectItem(rootTimerConfig, "TriggerMode");
	cJSON *triggerPulseCount = cJSON_GetObjectItem(rootTimerConfig, "TriggerPulseCount");
	cJSON *triggerPulseWidthUsec = cJSON_GetObjectItem(rootTimerConfig, "TriggerPulseWidthUsec");

	_timerDataConfig.TriggerFrequencyHz = triggerFrequency->valueint;
	_timerDataConfig.TriggerMode = triggerMode->valueint;
	_timerDataConfig.TriggerPulseCount = triggerPulseCount->valueint;
	_timerDataConfig.TriggerPulseWidthUsec = triggerPulseWidthUsec->valueint;

	// Clean up cJSON object
	cJSON_Delete(rootTimerConfig);

	if ((1000000 / _timerDataConfig.TriggerFrequencyHz) <= _timerDataConfig.TriggerPulseWidthUsec)
	{
		// invalid pulsewidth
		return 1;
	}
	return 0; // Successful parsing
}

// Function to configure htim3 based on triggerFrequency and triggerPulseWidthUsec
static void configureTimer(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t triggerFrequency, uint32_t triggerPulseWidthUsec)
{
	TIM_OC_InitTypeDef sConfigOC = {0};
	uint32_t period = ((1000000/triggerFrequency)/2) - 1;

	timer->Init.Prescaler = 167;
	timer->Init.CounterMode = TIM_COUNTERMODE_UP;
	timer->Init.Period = period;
	timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	// Initialize the timer
	if (HAL_TIM_Base_Init(timer) != HAL_OK)
	{
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = triggerPulseWidthUsec/2;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;


	if (HAL_TIM_PWM_ConfigChannel(timer, &sConfigOC, channel) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_TIM_MspPostInit(timer);
}

void init_trigger_pulse(TIM_HandleTypeDef* htim, uint32_t channel)
{
	_triggerConfig.channel = channel;
	_triggerConfig.htim = htim;
	_triggerConfig.configured = true;

	updateTimerDataFromPeripheral(htim, channel);
}


void get_trigger_data(char *jsonString, size_t max_length)
{
	if(_triggerConfig.configured)
	{
		updateTimerDataFromPeripheral(_triggerConfig.htim , _triggerConfig.channel);
		timerDataToJson(jsonString, max_length);
	}
	else
	{
		errorToJson(jsonString, max_length);
	}
}

void stop_trigger_pulse()
{
	HAL_TIM_PWM_Stop(_triggerConfig.htim , _triggerConfig.channel);
	updateTimerDataFromPeripheral(_triggerConfig.htim , _triggerConfig.channel);
}


void start_trigger_pulse()
{
	HAL_TIM_PWM_Start(_triggerConfig.htim , _triggerConfig.channel);
	updateTimerDataFromPeripheral(_triggerConfig.htim , _triggerConfig.channel);
}

bool set_trigger_data(char *jsonString)
{
	bool ret = false;
	if(_timerDataConfig.TriggerStatus == HAL_TIM_CHANNEL_STATE_BUSY)
	{
		// stop timer pwm
		HAL_TIM_PWM_Stop(_triggerConfig.htim , _triggerConfig.channel);
		updateTimerDataFromPeripheral(_triggerConfig.htim , _triggerConfig.channel);
	}

	if (jsonToTimerData((const char *)jsonString) == 0)
	{
		configureTimer(_triggerConfig.htim , _triggerConfig.channel, _timerDataConfig.TriggerFrequencyHz, _timerDataConfig.TriggerPulseWidthUsec);
		ret = true;
	}

	return ret;
}
