/*
===============================================================================
 Name        : FRTOS.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/
#include "chip.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "Acelerometro.h"

#define BUZZER PORT(2),PIN(6)

SemaphoreHandle_t Semaforo_Muestras_Acelerometro;
SemaphoreHandle_t Semaforo_Analisis_Acelerometro;

QueueHandle_t Cola_PromX;
QueueHandle_t Cola_PromY;
QueueHandle_t Cola_PromZ;


#include <cr_section_macros.h>

// TODO: insert other include files here
#define DEBUGOUT(...) printf(__VA_ARGS__)

// TODO: insert other definitions and declarations here
#define PORT(x) 	((uint8_t) x)
#define PIN(x)		((uint8_t) x)

#define OUTPUT		((uint8_t) 1)
#define INPUT		((uint8_t) 0)

void uC_StartUp (void)
{
	Chip_GPIO_Init (LPC_GPIO);

	Chip_IOCON_PinMux(LPC_IOCON, 0, 19, IOCON_MODE_INACT, IOCON_FUNC2);
	Chip_IOCON_PinMux(LPC_IOCON, 0, 20, IOCON_MODE_INACT, IOCON_FUNC2);
	Chip_IOCON_EnableOD(LPC_IOCON, 0, 19);
	Chip_IOCON_EnableOD(LPC_IOCON, 0, 20);

	Chip_GPIO_SetDir (LPC_GPIO, BUZZER, OUTPUT);
	Chip_IOCON_PinMux (LPC_IOCON, BUZZER, IOCON_MODE_INACT, IOCON_FUNC0);
	Chip_GPIO_SetPinOutLow(LPC_GPIO, BUZZER);

    /* pines del stick */
	Chip_IOCON_PinMux(LPC_IOCON, 0, 0, IOCON_MODE_INACT, IOCON_FUNC3);
	Chip_IOCON_PinMux(LPC_IOCON, 0, 1, IOCON_MODE_INACT, IOCON_FUNC3);
	Chip_IOCON_EnableOD(LPC_IOCON, 0, 0);
	Chip_IOCON_EnableOD(LPC_IOCON, 0, 1);

    Chip_I2C_SetClockRate(I2C1, 100000);
	Chip_I2C_SetMasterEventHandler(I2C1, Chip_I2C_EventHandlerPolling);
}


static void xTaskMuestras(void *pvParameters)
{

	signed short int samples[7] = {0,0,0,0,0,0,0}; //cada posicion es de 16 bits, necesario para guardar
										   //la parte low y high de las muestras de accel
	static signed int promX = 0;
	static signed int promY = 0;
	static signed int promZ = 0;

	static signed short int  valxPrevio=0,valx;

	static uint8_t k;

	uint8_t rbuf[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	static uint8_t wbuf[2] = {0,0};

	static	I2C_XFER_T xfer;

	MPU6050_wakeup(&xfer);

	//Lectura de PWR_MGMMT_1 2 (para verificar si se lo saco del sleep y de standby a los ejes)
	wbuf[0] = MPU6050_RA_PWR_MGMT_1;
	I2C_XFER_config(&xfer, rbuf, 2, MPU6050_I2C_SLAVE_ADDRESS, 0, wbuf, 1);

	wbuf[0] = MPU6050_RA_ACCEL_XOUT_H;	//Configuracion de la 1era direccion desde la que se leeran los valores de los registros de los sensores

	while(1)
	{
		xSemaphoreTake(Semaforo_Muestras_Acelerometro, portMAX_DELAY );

		for ( k = 0 ; k < 100 ; k ++ )
		{
			I2C_XFER_config(&xfer, rbuf, 14, MPU6050_I2C_SLAVE_ADDRESS, 0, wbuf, 1);

			Fill_Samples(samples, rbuf);

			valx = samples[0];

			if(valx != valxPrevio)
			{
				promX += samples[0];
				promY += samples[1];
				promZ += samples[2];
				valxPrevio = valx;
			}
			else
			{
				vTaskDelay( 5 / 100 / configTICK_RATE_HZ );// delay de 500 useg
				k--;//evito que sume al contador
			}
		}

		promX /= 100; promY /= 100; promZ /= 100;

		xQueueOverwrite(Cola_PromX,&promX);
		xQueueOverwrite(Cola_PromY,&promY);
		xQueueOverwrite(Cola_PromZ,&promZ);

		promX = promY = promZ = 0;

		xSemaphoreGive(Semaforo_Analisis_Acelerometro );
		vTaskDelay(5/portTICK_RATE_MS);//delay de 5ms
	}
}

static void xTaskAcelerometro(void *pvParameters)
{
	signed int 	deltaX,deltaY,deltaZ;
	signed int 	promX,promY,promZ;
	uint16_t	difX,difY,difZ;
	static signed int promXant = XDefecto, promYant = YDefecto, promZant = ZDefecto;

	while(1)
	{
		xSemaphoreTake(Semaforo_Analisis_Acelerometro , portMAX_DELAY );
		xQueuePeek(Cola_PromX,&promX,portMAX_DELAY);
		xQueuePeek(Cola_PromY,&promY,portMAX_DELAY);
		xQueuePeek(Cola_PromZ,&promZ,portMAX_DELAY);

		//Calculo para ver si hay choque
		if(promX > promXant)
		{
			deltaX = promX - promXant;
		}
		else
		{
			deltaX = promXant - promX;
		}

		if(promY > promYant)
		{
			deltaY = promY - promYant;
		}
		else
		{
			deltaY = promYant - promY;
		}

		if(promZ > promZant)
		{
			deltaZ = promZ - promZant;
		}
		else
		{
			deltaZ = promZant - promZ;
		}

		//Calculo para ver si hay vuelco
		if(promX > XDefecto)
		{
			difX = promX - XDefecto;
		}
		else
		{
			difX = XDefecto - promX;
		}
		if(promY > YDefecto)
		{
			difY = promY - YDefecto;
		}
		else
		{
			difY = YDefecto - promY;
		}
		if(promZ > ZDefecto)
		{
			difZ = promZ - ZDefecto;
		}
		else
		{
			difZ = ZDefecto - promZ;
		}

		if ( ( deltaX > CHOQUE ) || ( deltaY > CHOQUE ) || ( deltaZ > CHOQUE ) ) //para un choque
		{
			Chip_GPIO_SetPinOutHigh(LPC_GPIO, BUZZER);
			vTaskDelay(1000/portTICK_RATE_MS);//delay de 250ms
			Chip_GPIO_SetPinOutLow(LPC_GPIO, BUZZER);
			vTaskDelay(100/portTICK_RATE_MS);//delay de 250ms
		}
		if ( ( difX > VUELCO ) || ( difY > VUELCO ) || ( difZ > VUELCO ) ) //para un choque
		{
			Chip_GPIO_SetPinOutHigh(LPC_GPIO, BUZZER);
			vTaskDelay(250/portTICK_RATE_MS);//delay de 250ms
			Chip_GPIO_SetPinOutLow(LPC_GPIO, BUZZER);

		}

		promXant = promX; promYant = promY; promZant = promZ;

		xSemaphoreGive(Semaforo_Muestras_Acelerometro);
	}
}


int main(void)
{
	uC_StartUp ();
	SystemCoreClockUpdate();

	DEBUGOUT("Prueba acelerometro..\n");


	vSemaphoreCreateBinary(Semaforo_Muestras_Acelerometro);
	vSemaphoreCreateBinary(Semaforo_Analisis_Acelerometro);

	xSemaphoreTake(Semaforo_Analisis_Acelerometro , portMAX_DELAY );

	Cola_PromX = xQueueCreate(1, sizeof(signed int));
	Cola_PromY = xQueueCreate(1, sizeof(signed int));
	Cola_PromZ = xQueueCreate(1, sizeof(signed int));


	xTaskCreate(xTaskAcelerometro, (char *) "xTaskAcelerometro",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 2UL),
				(xTaskHandle *) NULL);

	xTaskCreate(xTaskMuestras, (char *) "xTaskMuestras",
					configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
					(xTaskHandle *) NULL);

	/* Start the scheduler */
	vTaskStartScheduler();

	/* Nunca debería arribar aquí */

    return 0;
}

