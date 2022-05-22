#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

// AV2
#include "coffee/coffee.h"


#define LED1_PIO			PIOA
#define LED1_PIO_ID			ID_PIOA
#define LED1_PIO_IDX		0
#define LED1_PIO_IDX_MASK	(1<< LED1_PIO_IDX)

#define LED2_PIO			PIOC
#define LED2_PIO_ID			ID_PIOC
#define LED2_PIO_IDX		30
#define LED2_PIO_IDX_MASK	(1<< LED2_PIO_IDX)

#define LED3_PIO			PIOB
#define LED3_PIO_ID			ID_PIOB
#define LED3_PIO_IDX		2
#define LED3_PIO_IDX_MASK	(1<< LED3_PIO_IDX)

#define BUT1_PIO			PIOD
#define BUT1_PIO_ID			ID_PIOD
#define BUT1_PIO_IDX		28
#define BUT1_PIO_IDX_MASK	(1u << BUT1_PIO_IDX) 

#define BUT2_PIO			PIOC
#define BUT2_PIO_ID			ID_PIOC
#define BUT2_PIO_IDX		31
#define BUT2_PIO_IDX_MASK	(1u << BUT2_PIO_IDX) 

#define BUT3_PIO			PIOA
#define BUT3_PIO_ID			ID_PIOA
#define BUT3_PIO_IDX		19
#define BUT3_PIO_IDX_MASK	(1u << BUT3_PIO_IDX) 

#define AFEC_POT AFEC0
#define AFEC_POT_ID ID_AFEC0
#define AFEC_POT_CHANNEL 0 // Canal do pino PD30

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

/************************************************************************/
/* recursos RTOS                                                        */
/************************************************************************/

/** Queue for msg log send data */
QueueHandle_t xQueueADC;

SemaphoreHandle_t xSemaphoreCurto;
SemaphoreHandle_t xSemaphoreLongo;
SemaphoreHandle_t xSemaphorePronto;

typedef struct {
	uint value;
} adcData;


/************************************************************************/
/* glboals                                                              */
/************************************************************************/


/************************************************************************/
/* PROTOtypes                                                           */
/************************************************************************/
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

void LED_init(Pio *pio, uint32_t ul_id, uint32_t mask, int estado);
void BUT_init(Pio *pio, uint32_t ul_id, uint32_t mask, uint32_t ul_attr, void (*p_handler) (uint32_t, uint32_t), int prioridade);
void pin_toggle(Pio *pio, uint32_t mask);

static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel, afec_callback_t callback);
void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) { configASSERT( ( volatile void * ) NULL ); }

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but1_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreCurto, &xHigherPriorityTaskWoken);
};

void but2_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreLongo, &xHigherPriorityTaskWoken);
};
	

void but3_callback(void){};
	
void TC0_Handler(void) {
	volatile uint32_t ul_dummy;

	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	pin_toggle(LED1_PIO, LED1_PIO_IDX_MASK);
}
	
void TC1_Handler(void) {
	volatile uint32_t ul_dummy;

	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/* Selecina canal e inicializa conversão */
	afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
	afec_start_software_conversion(AFEC_POT);
}

void TC2_Handler(void) {
	volatile uint32_t ul_dummy;

	ul_dummy = tc_get_status(TC0, 2);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	pin_toggle(LED2_PIO, LED2_PIO_IDX_MASK);
}

void TC3_Handler(void) {
	volatile uint32_t ul_dummy;

	ul_dummy = tc_get_status(TC1, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	pin_toggle(LED1_PIO, LED1_PIO_IDX_MASK);
	pin_toggle(LED2_PIO, LED2_PIO_IDX_MASK);
	pin_toggle(LED3_PIO, LED3_PIO_IDX_MASK);
}

void RTT_Handler(void) {
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(xSemaphorePronto, &xHigherPriorityTaskWoken);
	}
	
	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {

	}

}

static void AFEC_pot_Callback(void) {
  adcData adc;
  adc.value = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL);
  adc.value = (adc.value * 100)/4095;
  BaseType_t xHigherPriorityTaskWoken = pdTRUE;
  xQueueSendFromISR(xQueueADC, &adc, &xHigherPriorityTaskWoken);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
	config_AFEC_pot(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL, AFEC_pot_Callback);
	TC_init(TC0, ID_TC0, 0, 5);
	TC_init(TC0, ID_TC1, 1, 1);
	TC_init(TC0, ID_TC2, 2, 10);
	TC_init(TC1, ID_TC3, 0, 2);
	
	tc_start(TC0, 1);

	gfx_mono_ssd1306_init();
	
	adcData data;
	char heated = 1;
	char pumping = 0;
	char power = 1;
	uint timer;
	int divider = 100000000;

	int x = 5;
	int y = 10;
	int tamanho_barra_x = 60; 
	int tamanho_barra_y = 10;

	pin_toggle(LED1_PIO, LED1_PIO_IDX_MASK);
	pin_toggle(LED2_PIO, LED2_PIO_IDX_MASK);
	pin_toggle(LED3_PIO, LED3_PIO_IDX_MASK);
	
	for (;;)  {
		if (xQueueReceive(xQueueADC, &(data), 1000)) {
			if(xSemaphoreTake(xSemaphorePronto, 0)){
				coffe_pump_off();
				pumping = 0;
			}
			if (power){
				if(!pumping){
					if (data.value >= 80 && !heated){
						gfx_mono_generic_draw_filled_rect(0, 0, 127, 31, GFX_PIXEL_CLR);
						gfx_mono_draw_string("Pronto", 0, 0, &sysfont);
						tc_stop(TC0, 0);
						tc_stop(TC0, 2);
						tc_stop(TC1, 0);
						pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
						pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
						pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
						heated = 1;
					}
					else if(data.value < 80 && heated) {
						gfx_mono_generic_draw_filled_rect(0, 0, 127, 31, GFX_PIXEL_CLR);
						gfx_mono_draw_string("Aquecendo", 0, 0, &sysfont);
						coffee_heat_on();
						heated = 0;
						tc_stop(TC0, 0);
						tc_stop(TC0, 2);
						tc_start(TC1, 0);
						RTT_init(1, 0, 0);
					}
					timer = rtt_read_timer_value(RTT);
					if(timer >= 20){
						gfx_mono_generic_draw_filled_rect(0, 0, 127, 31, GFX_PIXEL_CLR);
						pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
						pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
						pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
						coffee_heat_off();
						power = 0;
					}
				}
				else{
					timer = rtt_read_timer_value(RTT);
					gfx_mono_generic_draw_filled_rect(x, y, ((double)tamanho_barra_x/divider*timer)+1, tamanho_barra_y+1, GFX_PIXEL_SET);
				}
				if(xSemaphoreTake(xSemaphoreCurto, 0) && heated){
					gfx_mono_generic_draw_filled_rect(0, 0, 127, 31, GFX_PIXEL_CLR);
					gfx_mono_draw_string("Curto", 0, 0, &sysfont);
					RTT_init(10, 50, RTT_MR_ALMIEN);
					coffe_pump_on();
					pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
					pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
					tc_stop(TC1, 0);
					tc_start(TC0, 0);
					pumping = 1;
					divider = 50;
					gfx_mono_generic_draw_rect(x, y, tamanho_barra_x+1, tamanho_barra_y+1, GFX_PIXEL_SET);
				}
				if(xSemaphoreTake(xSemaphoreLongo, 0) && heated){
					gfx_mono_generic_draw_filled_rect(0, 0, 127, 31, GFX_PIXEL_CLR);
					gfx_mono_draw_string("Longo", 0, 0, &sysfont);
					RTT_init(10, 100, RTT_MR_ALMIEN);
					coffe_pump_on();
					pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
					pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
					tc_stop(TC1, 0);
					tc_start(TC0, 2);
					pumping = 1;
					divider = 100;
					gfx_mono_generic_draw_rect(x, y, tamanho_barra_x+1, tamanho_barra_y+1, GFX_PIXEL_SET);
				}
			}
			else{
				if(xSemaphoreTake(xSemaphoreLongo, 0) || xSemaphoreTake(xSemaphoreCurto, 0)){
					power = 1;
				}
			}
		} 
		else {
			printf("Nao chegou um novo dado em 1 segundo");
		}
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel,
                            afec_callback_t callback) {
  /*************************************
   * Ativa e configura AFEC
   *************************************/
  /* Ativa AFEC - 0 */
  afec_enable(afec);

  /* struct de configuracao do AFEC */
  struct afec_config afec_cfg;

  /* Carrega parametros padrao */
  afec_get_config_defaults(&afec_cfg);

  /* Configura AFEC */
  afec_init(afec, &afec_cfg);

  /* Configura trigger por software */
  afec_set_trigger(afec, AFEC_TRIG_SW);

  /*** Configuracao específica do canal AFEC ***/
  struct afec_ch_config afec_ch_cfg;
  afec_ch_get_config_defaults(&afec_ch_cfg);
  afec_ch_cfg.gain = AFEC_GAINVALUE_0;
  afec_ch_set_config(afec, afec_channel, &afec_ch_cfg);

  /*
  * Calibracao:
  * Because the internal ADC offset is 0x200, it should cancel it and shift
  down to 0.
  */
  afec_channel_set_analog_offset(afec, afec_channel, 0x200);

  /***  Configura sensor de temperatura ***/
  struct afec_temp_sensor_config afec_temp_sensor_cfg;

  afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
  afec_temp_sensor_set_config(afec, &afec_temp_sensor_cfg);

  /* configura IRQ */
  afec_set_callback(afec, afec_channel, callback, 1);
  NVIC_SetPriority(afec_id, 4);
  NVIC_EnableIRQ(afec_id);
}

void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq) {
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	pmc_enable_periph_clk(ID_TC);

	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	NVIC_SetPriority((IRQn_Type)ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type)ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
	
}

void LED_init(Pio *pio, uint32_t ul_id, uint32_t mask, int estado) {
	pmc_enable_periph_clk(ul_id);
	pio_set_output(pio, mask, estado, 0, 0);
};

void BUT_init(Pio *pio, uint32_t ul_id, uint32_t mask, uint32_t ul_attr, void (*p_handler) (uint32_t, uint32_t), int prioridade) {
	pmc_enable_periph_clk(ul_id);
	pio_configure(pio, PIO_INPUT, mask, PIO_PULLUP|PIO_DEBOUNCE);
	pio_set_debounce_filter(pio, mask, 60);
	pio_handler_set(pio,
	ul_id,
	mask,
	ul_attr,
	p_handler);
	pio_enable_interrupt(pio, mask);
	pio_get_interrupt_status(pio);
	NVIC_EnableIRQ(ul_id);
	NVIC_SetPriority(ul_id, prioridade);
};

void pin_toggle(Pio *pio, uint32_t mask) {
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/
void io_init(void){
	LED_init(LED1_PIO, LED1_PIO_ID, LED1_PIO_IDX_MASK, 1);
	LED_init(LED2_PIO, LED2_PIO_ID, LED2_PIO_IDX_MASK, 1);
	LED_init(LED3_PIO, LED3_PIO_ID, LED3_PIO_IDX_MASK, 1);
	
	BUT_init(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but1_callback, 5);
	BUT_init(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but2_callback, 5);
	BUT_init(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but3_callback, 5);
}

int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	io_init();

	/* Initialize the console uart */
	configure_console();
	
	xSemaphoreCurto= xSemaphoreCreateBinary();
	if (xSemaphoreCurto == NULL)
	printf("falha em criar o semaforo \n");
	
	xSemaphoreLongo= xSemaphoreCreateBinary();
	if (xSemaphoreLongo == NULL)
	printf("falha em criar o semaforo \n");
	
	xSemaphorePronto= xSemaphoreCreateBinary();
	if (xSemaphorePronto == NULL)
	printf("falha em criar o semaforo \n");	
	
	xQueueADC = xQueueCreate(100, sizeof(int));
	if (xQueueADC == NULL)
	printf("falha em criar a queue xQueueADC \n");

	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create oled task\r\n");
	}
	
	if (xTaskCreate(task_av2, "av2", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create oled task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
