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

SemaphoreHandle_t Semaforo_1;
SemaphoreHandle_t Semaforo_2;

SemaphoreHandle_t Semaforo_Muestras_Acelerometro;
SemaphoreHandle_t Semaforo_Analisis_Acelerometro;

int32_t i, k;

uint8_t wbuf[2] = {0,0};
uint8_t rbuf[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

uint16_t samples[7] = {0,0,0,0,0,0,0}; //cada posicion es de 16 bits, necesario para guardar
									   //la parte low y high de las muestras de accel
uint32_t promX = 0;
uint32_t promY = 0;
uint32_t promZ = 0;

uint32_t promXant = 0;
uint32_t promYant = 0;
uint32_t promZant = 0;

uint32_t deltaX = 0;
uint32_t deltaY = 0;
uint32_t deltaZ = 0;

uint32_t cuadX = 0;
uint32_t cuadY = 0;
uint32_t cuadZ = 0;

uint32_t cuadXant = 0;
uint32_t cuadYant = 0;
uint32_t cuadZant = 0;

I2C_XFER_T xfer;

#include <cr_section_macros.h>

// TODO: insert other include files here
#define DEBUGOUT(...) printf(__VA_ARGS__)

#define MPU6050_DEVICE_ADDRESS   0x68
#define MPU6050_RA_ACCEL_XOUT_H  0x3B
#define MPU6050_RA_PWR_MGMT_1    0x6B
#define MPU6050_RA_PWR_MGMT_2    0x6C
#define MPU6050_PWR1_SLEEP_BIT   6
#define MPU6050_I2C_SLAVE_ADDRESS 0x68
#define STATIC_0x40_REFERENCE_REGISTER 0x6B
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C

#define PUNTO_A		0
#define PUNTO_B		16384
#define PUNTO_C		32768
#define PUNTO_D		49152
#define PUNTO_E		65536


#define CUAD_1		1
#define CUAD_2		2
#define CUAD_3		3
#define CUAD_4		4

// TODO: insert other definitions and declarations here
#define PORT(x) 	((uint8_t) x)
#define PIN(x)		((uint8_t) x)

#define OUTPUT		((uint8_t) 1)
#define INPUT		((uint8_t) 0)

/* Llena el vector de muestras samples con la data de los registros de ACCEL, GYRO y TEMP del MPU
 * Al estar la informacion en 16 bits y ser levantada por registros de 8, en rbuf esta la parte low
 * 	y high de cada componente, por lo que se debe recomponer desplazando la parte high y concatenando la low]
 * 	para obtener el valor que se midio. Proceso que se realiza en esta funcion para pasar a samples.
 *
 * 	 rbuf : Tiene la data leida por el MPU con la data dividida en high y low
 * 	 samples : Tendra la data agrupada que representa al valor medido por los sensores en cada eje
 */
void Fill_Samples(uint16_t * samples, uint8_t * rbuf)
{
	//De momento leer rbuf es lo mismo que leer xfer.rxBuff porque apuntan a la misma direccion
	//Desplazamos la parte alta a los 8 ultimos bits y le hacemos un OR para tener en los 8 primeros la parte baja


	samples[0]=(rbuf[0] << 8) | rbuf[1];
	samples[1]=(rbuf[2] << 8) | rbuf[3];
	samples[2]=(rbuf[4] << 8) | rbuf[5];
	samples[3]=(rbuf[6] << 8) | rbuf[7];
	samples[4]=(rbuf[8] << 8) | rbuf[9];
	samples[5]=(rbuf[10] << 8) | rbuf[11];
	samples[6]=(rbuf[12] << 8) | rbuf[13];
}

/*
 * Se encarga de inicializar los registros de PWR_MGMT necesarios para habilitar los
 *  sensores (acelerometro y giroscopo) en cada eje, para sus lecturas.
 *  La configuracion en la que quedan seteados es la por defecto:
 *  accelerometer (±2g) , gyroscope (±250°/sec).
 *
 *  * xfer : Puntero a la estructura del tipo I2C_XFER_T necesaria para la utilizacion de Chip_I2C_MasterTransfer.
 *  		 Chip_I2C_MasterTransfer : Funcion que resuelve la interaccion i2c en funcion de lo especificado en la estructura I2C_XFER_T
 */
void MPU6050_wakeup(I2C_XFER_T * xfer)
{
		//Setea PWR_MGMT_1 y 2 en 0, el byte de cada uno


		uint8_t wbuf[3] = {MPU6050_RA_PWR_MGMT_1, 0, 0};
		/*xfer->slaveAddr = MPU6050_DEVICE_ADDRESS;
		xfer->txBuff = wbuf;
		xfer->txSz = 3;
		xfer->rxSz = 0;*/

		I2C_XFER_config(xfer, xfer->rxBuff, 0, MPU6050_I2C_SLAVE_ADDRESS, 0, wbuf, 3);
}

/*
 * Configura la estructura XFER para realizar la comunicacion i2c.
 * * xfer	  : Puntero a la estructura del tipo I2C_XFER_T necesaria para utilizar la funcion Chip_I2C_MasterTransfer.
 * 				Chip_I2C_MasterTransfer : Funcion que resuelve la interaccion i2c en funcion de lo especificado en la estructura I2C_XFER_T
 *	 rbuf 	  : Puntero al buffer de lectura donde se volcaran los bytes leidos
 *	 rxSz 	  : Cantidad de bytes que se leeran y volcaran en rbuf
 *	 slaveAddr: Direccion estatica del slave con el que se desea comunicar
 *	 status   : Estado de la comunicacion, (estado inicial 0)
 *	 wbuf	  : Buffer de escritura donde se colocara tanto el registro que se desea escribir como el dato que desea ser escrito
 *	 			Ej de uso: wbuf[] = {reg_inicial, dato} solo escribe el byte dato en reg_inicial
 *	 					   wbuf[] = {reg_inicial, dato1, dato2} escribe el byte dato1 en reg_incial y dato2 en reg_inicial+1 (el registro siguiente)
 *	 txSz	  : La cantidad de bytes que se desean enviar, osea empezando a leer wbuf desde 0 inclusive, cuantos bytes manda de ese buffer
 *	 			Ej : wbuf[] = {reg_inicial, dato1, dato2}, entonces txSz deberia ser = 3
 *	 				 wbuf[] = {reg_inicial}, (caso tipico de solo lectura de ese registro), entonces txSz deberia ser = 1
 */

void I2C_XFER_config (I2C_XFER_T * xfer,uint8_t *rbuf, int rxSz, uint8_t slaveAddr, I2C_STATUS_T status, uint8_t * wbuf, int txSz)
{
	xfer->rxBuff = rbuf; //Buffer de lectura
	xfer->rxSz = rxSz;	//cantidad de bytes que se desean leer, arbitrariamente seteamos 10
	xfer->slaveAddr = slaveAddr; //Adress estatica del dispositivo i2c a leer (MPU6050)
	xfer->status = status;
	xfer->txBuff = wbuf; //Buffer de escritura
	xfer->txSz = txSz; //cantidad de bytes que se desean escribir, solo escribimos el registro desde
					//el que comenzamos a leer
	Chip_I2C_MasterTransfer(I2C1, xfer);
}

void uC_StartUp (void)
{
	Chip_GPIO_Init (LPC_GPIO);

	Chip_IOCON_PinMux(LPC_IOCON, 0, 19, IOCON_MODE_INACT, IOCON_FUNC2);
	Chip_IOCON_PinMux(LPC_IOCON, 0, 20, IOCON_MODE_INACT, IOCON_FUNC2);
	Chip_IOCON_EnableOD(LPC_IOCON, 0, 19);
	Chip_IOCON_EnableOD(LPC_IOCON, 0, 20);

    /* pines del stick */
	Chip_IOCON_PinMux(LPC_IOCON, 0, 0, IOCON_MODE_INACT, IOCON_FUNC3);
	Chip_IOCON_PinMux(LPC_IOCON, 0, 1, IOCON_MODE_INACT, IOCON_FUNC3);
	Chip_IOCON_EnableOD(LPC_IOCON, 0, 0);
	Chip_IOCON_EnableOD(LPC_IOCON, 0, 1);

    Chip_I2C_SetClockRate(I2C1, 100000);
	Chip_I2C_SetMasterEventHandler(I2C1, Chip_I2C_EventHandlerPolling);
}

void calcularDeltas(void)
{
	/*EJE_X*/
	if ( cuadX  == CUAD_1 && cuadXant == CUAD_4 )
	{
		deltaX = (PUNTO_E - promXant) + promX;
	}
	else if ( cuadX  == CUAD_4 && cuadXant == CUAD_1 )
	{
		deltaX = (PUNTO_E - promX) + promXant;
	}
	else
	{
		if ( promX > promXant )
			deltaX = promX - promXant;
		else
			deltaX = promXant - promX;
	}

	/*EJE_Y*/
	if ( cuadY  == CUAD_1 && cuadYant == CUAD_4 )
	{
		deltaY = (PUNTO_E - promYant) + promY;
	}
	else if ( cuadY  == CUAD_4 && cuadYant == CUAD_1 )
	{
		deltaY = (PUNTO_E - promY) + promYant;
	}
	else
	{
		if ( promY > promYant )
			deltaY = promY - promYant;
		else
			deltaY = promYant - promY;
	}

	/*EJE_Z*/
	if ( cuadZ  == CUAD_1 && cuadZant == CUAD_4 )
	{
		deltaZ = (PUNTO_E - promZant) + promZ;
	}
	else if ( cuadZ  == CUAD_4 && cuadZant == CUAD_1 )
	{
		deltaZ = (PUNTO_E - promZ) + promZant;
	}
	else
	{
		if ( promZ > promZant )
			deltaZ = promZ - promZant;
		else
			deltaZ = promZant - promZ;
	}
}

void calcularCuadrantes(void)
{
	/*EJE_X*/
	if ( (promX >= PUNTO_A) && (promX < PUNTO_B) )
		cuadX = CUAD_1;
	else if ( (promX >= PUNTO_B) && (promX < PUNTO_C) )
		cuadX = CUAD_2;
	else if ( (promX >= PUNTO_C) && (promX < PUNTO_D) )
		cuadX = CUAD_3;
	else if ( (promX >= PUNTO_D) && (promX < PUNTO_E) )
		cuadX = CUAD_4;

	/*EJE_Y*/
	if ( (promY >= PUNTO_A) && (promY < PUNTO_B) )
		cuadY = CUAD_1;
	else if ( (promY >= PUNTO_B) && (promY < PUNTO_C) )
		cuadY = CUAD_2;
	else if ( (promY >= PUNTO_C) && (promY < PUNTO_D) )
		cuadY = CUAD_3;
	else if ( (promY >= PUNTO_D) && (promY < PUNTO_E) )
		cuadY = CUAD_4;

	/*EJE_Z*/
	if ( (promZ >= PUNTO_A) && (promZ < PUNTO_B) )
		cuadZ = CUAD_1;
	else if ( (promZ >= PUNTO_B) && (promZ < PUNTO_C) )
		cuadZ = CUAD_2;
	else if ( (promZ >= PUNTO_C) && (promZ < PUNTO_D) )
		cuadZ = CUAD_3;
	else if ( (promZ >= PUNTO_D) && (promZ < PUNTO_E) )
		cuadZ = CUAD_4;
}

/* LED1 toggle thread */
static void vTask1(void *pvParameters)
{
	while (1)
	{
		xSemaphoreTake(Semaforo_2 , portMAX_DELAY );

		Chip_GPIO_SetPinOutHigh (LPC_GPIO , PORT(0) , PIN(22));

		vTaskDelay( 500 / portTICK_PERIOD_MS );

		xSemaphoreGive(Semaforo_1 );
	}
}

static void xTaskMuestras(void *pvParameters)
{
	while(1)
	{
		xSemaphoreTake(Semaforo_Muestras_Acelerometro, portMAX_DELAY );

		promXant = promX; promYant = promY; promZant = promZ;
		cuadXant = cuadX; cuadYant = cuadY; cuadZant = cuadZ;

		promX = promY = promZ = 0;

		for ( k = 0 ; k < 100 ; k ++ )
		{
			I2C_XFER_config(&xfer, rbuf, 14, MPU6050_I2C_SLAVE_ADDRESS, 0, wbuf, 1);

			Fill_Samples(&samples, &rbuf);

			promX += samples[0];
			promY += samples[1];
			promZ += samples[2];

			//vTaskDelay( 1 / portTICK_PERIOD_MS );
		}

		vTaskDelay( 100 / portTICK_PERIOD_MS );

		promX /= 100; promY /= 100; promZ /= 100;

		calcularCuadrantes();

		calcularDeltas();

		xSemaphoreGive(Semaforo_Analisis_Acelerometro );
	}
}

static void xTaskAcelerometro(void *pvParameters)
{
	while(1)
	{
		xSemaphoreTake(Semaforo_Analisis_Acelerometro , portMAX_DELAY );

		if ( ( deltaX > PUNTO_C ) || ( deltaY > PUNTO_C ) || ( deltaZ > PUNTO_C ) )
		{
			DEBUGOUT("Los deltas  son: %d          %d          %d \n", deltaX, deltaY, deltaZ);
			DEBUGOUT("\n");
		}

/*
		if ( promX < 5000 )
			DEBUGOUT("Aviso en eje X con valor : %d \n", promX );

		if ( promY < 5000 )
			DEBUGOUT("Aviso en eje Y con valor : %d \n", promY );

		if ( promZ < 5000 )
			DEBUGOUT("Aviso en eje Z con valor : %d \n", promZ );
*/
		xSemaphoreGive(Semaforo_Muestras_Acelerometro);
	}
}


int main(void)
{
	uC_StartUp ();
	SystemCoreClockUpdate();

	MPU6050_wakeup(&xfer);

	//Lectura de PWR_MGMMT_1 2 (para verificar si se lo saco del sleep y de standby a los ejes)
	wbuf[0] = MPU6050_RA_PWR_MGMT_1;
	I2C_XFER_config(&xfer, rbuf, 2, MPU6050_I2C_SLAVE_ADDRESS, 0, wbuf, 1);

	//Configuracion de la 1era direccion desde la que se leeran los valores de los registros de los sensores
	wbuf[0]=MPU6050_RA_ACCEL_XOUT_H;
	wbuf[1]=0;

	DEBUGOUT("Prueba acelerometro..\n");

	vSemaphoreCreateBinary(Semaforo_1);
	vSemaphoreCreateBinary(Semaforo_2);

	vSemaphoreCreateBinary(Semaforo_Muestras_Acelerometro);
	vSemaphoreCreateBinary(Semaforo_Analisis_Acelerometro);

	xSemaphoreTake(Semaforo_1 , portMAX_DELAY );
	xSemaphoreTake(Semaforo_Analisis_Acelerometro , portMAX_DELAY );

	xTaskCreate(xTaskAcelerometro, (char *) "xTaskAcelerometro",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
				(xTaskHandle *) NULL);

	xTaskCreate(xTaskMuestras, (char *) "xTaskMuestras",
					configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
					(xTaskHandle *) NULL);

	/* Start the scheduler */
	vTaskStartScheduler();

	/* Nunca debería arribar aquí */

    return 0;
}

