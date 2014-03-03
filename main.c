/**
 *
 * include freertos and asf license info here 
 */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

TO-DO list:

	Work on global kicker addresses
		-If load1 then load2 is pressed the global var will be set to 2.
		 Check whether this is true and if so rectify

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */


/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/* Demo includes. */
//#include "partest.h"
//#include "demo-tasks.h"
//#include "conf_example.h"

/* ASF includes. */
#include "asf.h"

//my includes
#include "conf_board.h"
#include "conf_clock.h"
#include "kui.h"
#include "string.h"

//defines
#define IRQ_PRIORITY_PIO configLIBRARY_LOWEST_INTERRUPT_PRIORITY //configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 
#define IRQ_PRIORITY_USART0 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+1 //8
#define IRQ_PRIORITY_USART2 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+2 //9
#define IRQ_PRIORITY_USART1 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+3 //10
#define UART0_BAUDRATE 115200
#define USART0_BAUDRATE 115200
#define USART1_BAUDRATE 115200
#define USART2_BAUDRATE 9600
#define UART0_BUFFER_SIZE 12 //11 bytes is size of touch reply
//#define USART1_BUFFER_SIZE 12 //11 bytes is size of touch reply
#define USART0_WRITE_BUFFER_SIZE 100
#define USART0_RECEIVE_BUFFER_SIZE 12 //11 bytes is size of touch reply
#define USART2_RECEIVE_BUFFER_SIZE 100 //100 bytes for kicker psu message handling 
#define USART2_WRITE_BUFFER_SIZE 1024//200//100 
#define USART1_RECEIVE_BUFFER_SIZE 100 //100 bytes for cps message handling
#define USART1_WRITE_BUFFER_SIZE 1024 //sending messages to cps crate
#define LCD_COMMAND_BUFFER_SIZE 25//100 
#define TRASH_SIZE 2//00

#define ISIS_CONTROLS_LOGO_DISPLAY_TIME (3000 / portTICK_RATE_MS) //in ms
#define LCD_BUTTON_INIT_TIMEOUT (100 / portTICK_RATE_MS) //time between button update commands sent to the LCD at init
#define LCD_BACKLIGHT_TIMEOUT (2000/*180000/*300000*/ / portTICK_RATE_MS) //set at 3 minutes
#define LCD_TOUCH_SCAN_PERIOD (20/*66*/ / portTICK_RATE_MS) //scan touchscreen period
//#define LCD_BUTTON_LATCH_TIMEOUT (25 / portTICK_RATE_MS) 
#define LCD_BUTTON_LOAD_TIMEOUT (666/*333*/ /*250*/ / portTICK_RATE_MS) //the time that a load button is shown depressed
#define KICKER_READ_LOOP_UPDATE (500 / portTICK_RATE_MS) //poll time for kicker psu
#define SEED_READ_Q_TIMEOUT (250 / portTICK_RATE_MS) //time to seed the read Q, all 30 status' received in 7.5s loop, could reduce to 100ms, 3s loop
//the LCD command can be sent in ~1.2ms
#define SEED_LCD_READVAL_Q_TIMEOUT (100 / portTICK_RATE_MS) //time to seed the Lcd refresh values, 
#define CPS_READ_SCAN_PERIOD (20/*100*/ / portTICK_RATE_MS)
#define LCD_CLEAR_PREV_BUTTON_TIMEOUT (500 / portTICK_RATE_MS)
#define TIMEOUT_BETWEEN_RESET_COMMANDS (20 / portTICK_RATE_MS)
#define TASKMON_LISTSIZE 1024

//LCD graphics
#define BGND_LOGO 0
#define BGND_MAIN 1
#define BGND_BLIGHT_ON 2
#define G_LOCAL_BUTTON 0x00
#define G_REMOTE_BUTTON 0x01
#define G_LOAD_UP 0x02
#define G_LOAD_DOWN 0x03
#define G_OFF_UP 0x04
#define G_ON_UP 0x05
#define G_OFF_DOWN 0x06
#define G_ON_DOWN 0x07
#define G_RESET_UP 0x08
#define G_RESET_DOWN 0x09
#define G_RESET_BAD 0x0A
#define G_RESET_BAD2 0x0B
#define MAX_LCD_BUTTONS 9

#define BACKLIGHT_ON 1
#define BACKLIGHT_OFF 0
#define COUNT_UP 2
#define COUNT_DOWN 1
#define COUNT_FINE     1
#define COUNT_COARSE 100
#define LOCAL 0
#define REMOTE 1
#define BUTTON_DOWN 1
#define BUTTON_UP 0
#define PSU_HEALTHY_ACTIVE 1
#define PSU_HEALTHY_INACTIVE 0
#define HVON_ACTIVE 0
#define HVON_INACTIVE 1
#define LCD_ON_PUSHED 2
#define LCD_OFF_PUSHED 3
#define READQ_SIZE 5

//write command types
#define SET_HVON 20
#define SET_HVOFF 21
#define SET_HVRESET 22
#define SET_HVLOAD 23

//read status command types
#define READ_OUTV 10
#define READ_DEMV 11
#define READ_AMPS 12
#define READ_POWER 13
#define READ_STATUS 14
#define READ_LOCREM_STATUS 15

//Wyclef defines
#define WYCLEF_TX_KV    1
#define WYCLEF_TX_ON    2
#define WYCLEF_TX_OFF   3
#define WYCLEF_TX_RESET 4
#define WYCLEF_TX_KVREMOTE 5
#define WYCLEF_TX_CRASHOFF 6

//used for kicker reset cycle
#define KICKER_RESET_POS_HVOFF 1
#define KICKER_RESET_NEG_HVOFF 2
#define KICKER_RESET_POS_RST   3
#define KICKER_RESET_NEG_RST   4

#define ASCII_OFFSET             0x30
#define LCD_ON 1
#define LCD_OFF 0
//#define EOS 0x39//using '9' temporarily 0x0d //carriage return defines end of string
#define LCD_TOUCHEVENT_BUFFER_SIZE 11 // size of string is 11 bytes,
#define CPS_RX_BUFFER_SIZE 11
#define COUNTDISP_BUFFER_SIZE      13

//these can be combined???
#define LOCREM_BUTTON_BUFFER_SIZE 12
#define LOAD_BUTTON_BUFFER_SIZE 12
#define ONOFF_BUTTON_BUFFER_SIZE 12
#define RESET_BUTTON_BUFFER_SIZE 12

#define LCD_DISP_NUMBER_OF_DIGITS 4
#define DISP_UNIT 0
#define DISP_TENS 1
#define DISP_HUND 2
#define DISP_THOU 3

#define CIRC_BUFFER_MAX 25

#define COUNTDISP_NEXT_BUFFER_SIZE 3 //can delete this once all pdc are converted to single buffer transactions

#define LCD_BACKLIGHT_BUFFER_SIZE   9
#define LCD_BUZZ_BUFFER_SIZE        7
#define LCD_BGND_BUFFER_SIZE        9
#define LCD_BACKGROUND_BUFFER_SIZE 12
#define LCD_TXV_BUFFER_SIZE        11
#define LCD_TX_RESET_BUFFER_SIZE    8
#define LCD_TX_ON_BUFFER_SIZE       8
#define LCD_TX_OFF_BUFFER_SIZE      9
#define LCD_TX_CRASHOFF_BUFFER_SIZE 3
#define LCD_TX_HVI_READ_OUTPUT_V_BUFFER_SIZE 5
#define LCD_TX_HVI_READ_DEMAND_V_BUFFER_SIZE 6
#define LCD_TX_HVI_READ_OUTPUT_I_BUFFER_SIZE 5
#define LCD_TX_HVI_READ_OUTPUT_P_BUFFER_SIZE 5
#define LCD_TX_HVI_READ_STATUS_BUFFER_SIZE 6
#define LCD_LAMP_BUFFER_SIZE       12
#define LCD_DPOINT_BUFFER_SIZE     13
#define LCD_kVDISP_BUFFER_SIZE     14
#define LCD_REMOTEDISP_BUFFER_SIZE 18
#define LCD_LOCALDISP_BUFFER_SIZE  17
#define LCD_FRAME_BUFFER_SIZE      12
#define LCD_FRAME_NEXT_BUFFER_SIZE  4

//Local read buffer sizes
#define LOCAL_CMD_READ_STATUS_BUFFER_SIZE 6
#define LOCAL_CMD_READ_OUTV_BUFFER_SIZE 5
#define LOCAL_CMD_READ_DEMV_BUFFER_SIZE 6
#define LOCAL_CMD_READ_AMPS_BUFFER_SIZE 5
#define LOCAL_CMD_READ_POW_BUFFER_SIZE 5

//receive buffers
#define LOCAL_RECEIVE_BUFFER_SIZE_OUTV 10
#define LOCAL_RECEIVE_BUFFER_SIZE_DEMV 11
#define LOCAL_RECEIVE_BUFFER_SIZE_AMPS 10
#define LOCAL_RECEIVE_BUFFER_SIZE_POW 8
#define LOCAL_RECEIVE_BUFFER_SIZE_LOCREM 10

//error codes
#define KUI_function_success 0
#define rc_Error1  -1
#define rcLCD_WRITE_FAILED 3
#define LOAD_UPDATE_ERROR 11
#define LOCREM_UPDATE_ERROR 10


//my function prototypes
static void prvBacklightOffTimerCallback(void) ;
static void prvLcdTouchEventTimerCallback(void) ;
static void prvLcdLocremButtonLatchTimerCallback(void) ;
static void prvLcdLoadButtonPopupTimerCallback(void) ;
static void prvSeedReadQ_Timer (void) ;
static void prvLcdReadValsSeedQ_Timer(void) ;
static void prvLcdReadRefresh_Timer (void) ;
static void prvCPSReadScanTimer (void) ;
static void prvLcdClearPrevButtonInfoTimer (void) ;
static void prvLcdResetButtonTimer (void) ;
static void slck_div_init(Pio* p_pio, uint32_t ul_div) ;
static freertos_usart_if prepare_usart0_port_standard( Usart* usart_base, uint8_t* receive_buffer, uint32_t receive_buffer_size_in_bytes) ;
//static freertos_usart_if prepare_usart0_port_asynch( Usart* usart_base, uint8_t* receive_buffer, uint32_t receive_buffer_size_in_bytes) ;
static status_code_t usart0_write_string_standard(freertos_usart_if freertos_usart, uint8_t* us_string, uint8_t us_string_length) ;
static status_code_t usart0_read_string_standard(freertos_usart_if freertos_usart, uint8_t* buffer_address, uint8_t us_string_length) ;

static uint8_t usart0_read_string(freertos_usart_if freertos_usart, uint8_t* buffer_address, uint8_t us_string_length) ;
static uint8_t usart1_read_string(freertos_usart_if freertos_usart, uint8_t* buffer_address, uint8_t us_string_length) ;

static status_code_t usart1_write_string_standard(freertos_usart_if freertos_usart, uint8_t* us_string, uint8_t us_string_length) ;
static status_code_t usart2_write_string_standard(freertos_usart_if freertos_usart, uint8_t* us_string, uint8_t us_string_length) ;
static status_code_t usart1_write_string_16bit(freertos_usart_if freertos_usart, uint8_t* us_string, uint16_t us_string_length) ;
static status_code_t usart2_write_string_16bit(freertos_usart_if freertos_usart, uint8_t* us_string, uint16_t us_string_length) ;
static status_code_t usart2_read_string_standard(freertos_usart_if freertos_usart, uint8_t* buffer_address, uint8_t us_string_length) ;
static status_code_t flush_usart_buffer( freertos_usart_if freertos_usart, uint8_t* buffer_address, uint8_t us_string_length,  portTickType max_wait) ;
static freertos_usart_if prepare_usart2_port_standard( Usart* usart_base, uint8_t* receive_buffer, uint32_t receive_buffer_size_in_bytes) ;
static freertos_usart_if prepare_usart1_port_standard( Usart* usart_base, uint8_t* receive_buffer, uint32_t receive_buffer_size_in_bytes) ;
static void configure_uart0(void) ;
static void configure_uart0_pdc(void) ;
static void pio_interrupt_handler_psuhealthy(uint32_t id, uint32_t mask) ;
static void pio_interrupt_handler_hvon(uint32_t id, uint32_t mask) ;
static void ISR_pio_encoder_action(uint32_t id, uint32_t mask) ;
static void ISR_pio_HVON(uint32_t id, uint32_t mask) ;
static void ISR_pio_PSUHEALTHY(uint32_t id, uint32_t mask) ;
static void ISR_pio_pins(uint32_t id, uint32_t mask) ;
static void configure_pio_interrupt(void) ;
static uint8_t get_kicker_address_locrem(void) ;
static void set_kicker_address_locrem(uint8_t us_kicker_address) ;
static uint8_t get_kicker_address_load(void) ;
static void set_kicker_address_load(uint8_t us_kicker_address) ;
static uint8_t get_kicker_address_onoff(void) ;
static void set_kicker_address_onoff(uint8_t us_kicker_address) ;
static uint8_t get_kicker_address_reset(void) ;
static void set_kicker_address_reset(uint8_t us_kicker_address) ;
//static uint8_t get_wyclef_status(uint8_t us_kicker_address) ;
//static void set_wyclef_status(uint8_t us_kicker_address, uint8_t status) ;
static uint8_t get_wyclef_status(void) ;
static void set_wyclef_status(uint8_t status) ;
static uint8_t get_kicker_address_encoder(void) ;
static void set_kicker_address_encoder(uint8_t us_kicker_address) ;
//static void Select_CoarseFine2(uint8_t us_kick_address) ;
static void ProcessCount2(uint8_t us_kicker_address, uint8_t us_enc_status) ;
//static void draw_cv2(uint8_t us_kick_add) ;
//static void draw_cv_unit(uint8_t us_kick_add) ;
//static void draw_cv_ten(uint8_t us_kick_add) ;
//static void draw_cv_hund(uint8_t us_kick_add) ;
//static void draw_cv_thou(uint8_t us_kick_add) ;
static void has_encoder_unit_changed(uint8_t us_kick_add) ;
static void has_encoder_tens_changed(uint8_t us_kick_add) ;
static void has_encoder_hund_changed(uint8_t us_kick_add) ;
static void has_encoder_thou_changed(uint8_t us_kick_add) ;
static int8_t get_step_size(uint8_t us_kicker_address) ;
static uint16_t* get_active_kicker_count(uint8_t kicker_address) ;
static void Select_CoarseFine2(uint8_t us_kick_address) ;
static void lcd_display_set_coords_text(uint8_t us_kicker_address, uint8_t *ptr_disp_array, uint8_t us_char_offset, uint8_t lcd_text_opt) ;
static uint8_t lcd_display_count(uint8_t us_kicker_address, uint8_t us_count_val, uint8_t char_offset, uint8_t* lcd_count_buffer, uint8_t lcd_colour, uint8_t lcd_display_line) ;
static uint8_t lcd_display_readvals(uint8_t us_kicker_address, uint8_t us_count_val, uint8_t char_offset, uint8_t* lcd_count_buffer, uint8_t lcd_colour, uint8_t lcd_display_line, void* pvParameters) ;
static void lcd_set_font_colour(uint8_t* ptr_buffer, uint8_t us_font_colour) ;
static void update_kicker_string_kV(uint8_t us_kicker_address, uint8_t us_digit_descriptor, uint8_t uc_ascii_value) ;
static uint8_t lcd_chksum(uint8_t* ptr_buff, uint8_t sizeof_buff) ;
static int8_t kicker_get_hvon_status(uint8_t us_kicker_address) ;
static int8_t kicker_get_hvon_status2(uint8_t us_kicker_address) ;
static void kicker_set_hvon_status(uint8_t us_kicker_address, uint8_t hvon_status) ;
static void kicker_set_locrem_status(uint8_t us_kicker_address) ;
static int8_t kicker_get_locrem_status(uint8_t us_kicker_address) ;
static int8_t kicker_get_psuhealthy_status(uint8_t us_kicker_address) ;
static void kicker_set_psuhealthy_status(uint8_t us_kicker_address, uint8_t psuhealthy_status) ;
//static uint8_t kicker_locrem_updated(void) ;
static void lcd_display_set_coords_locrem(uint8_t us_kicker_address, Bool b_local_mode, uint8_t *ptr_disp_array) ;
static int8_t kicker_get_load_status(uint8_t us_kicker_address) ;
//static uint8_t kicker_load_updated(void) ;
static void lcd_display_set_coords_load(uint8_t us_kicker_address, Bool b_load_status, uint8_t *ptr_disp_array) ;
static void lcd_display_set_coords_onoff(uint8_t us_kicker_address, Bool b_mode, Bool b_isittouchevent, uint8_t *ptr_disp_array) ;
static void lcd_display_set_coords_reset(uint8_t us_kicker_address, Bool b_mode, Bool b_isittouchevent, uint8_t *ptr_disp_array) ;
static int8_t get_load_status(uint8_t us_kicker_address) ;
//static int8_t get_reset_status(uint8_t us_kicker_address) ;
static void set_load(uint8_t us_kicker_address) ;
static uint8_t get_load(void) ;
static void clear_load(us_kicker_address) ;
static int get_string_address_txKVRemote(void) ;
static int get_string_address_txKV(uint8_t us_kicker_address) ;
static int get_string_address_txON(uint8_t us_kicker_address) ;
static int get_string_address_txOFF(uint8_t us_kicker_address) ;
static int get_string_address_txRESET(uint8_t us_kicker_address) ;
static int get_string_address_read_status(uint8_t us_kicker_address) ;
static int get_string_address_read_output_volts(uint8_t us_kicker_address) ;
static int get_string_address_read_demand_volts(uint8_t us_kicker_address) ;
static int get_string_address_read_power(uint8_t us_kicker_address) ;
static int get_string_address_read_amps(uint8_t us_kicker_address) ;
static xRxBuffer_info_t get_receive_buffer_info(uint8_t psu_address, uint8_t command_type) ;
static void save_nonascii_value(uint8_t command_type, char* ascci_buffer, int* nonascii_value) ;




int8_t lcd_backlight_set(Bool b_backlight_on) ;
int8_t is_load_active() ; //return 0 if any of the 6 load buttons are active (pressed)
int8_t is_reset_active() ; //return 0 if any of the 6 load buttons are active (pressed)
void clear_psuhealthy_trigger(uint8_t us_kicker_address) ;
uint8_t is_psuhealthy_triggered(void) ;
uint8_t check_psuhealthy_trigger(uint8_t us_kicker_address) ;
void clear_onoff_trigger(uint8_t us_kicker_address) ;
uint8_t is_onoff_triggered(void) ;
uint8_t check_onoff_trigger(uint8_t us_kicker_address) ;
int8_t lcd_popup_load_buttons() ;
int8_t lcd_popup_reset_buttons() ;
int8_t lcd_draw_load(uint8_t us_kicker_address, Bool load_status) ;
int8_t lcd_draw_reset(uint8_t us_kicker_address, uint8_t load_status) ;
int8_t lcd_draw_onoff(uint8_t us_kicker_address, uint8_t hv_on) ;
int8_t pdc_uart0_lcd_display_loadbutton(uint8_t us_kicker_address, Bool b_button_state) ;
uint8_t find_start_of_message_from_CPS(uint8_t* ptr_buff, int32_t ul_buff_size) ;
uint8_t find_end_of_message_from_CPS(uint8_t* ptr_buff, int32_t ul_buff_size, uint8_t us_start_loc, uint8_t uc_eos) ;

//Tasks
//static void vTask_SendCommandToKickerPsu_ReadLoop(void* pvParameters) ;
//static void vTask_SendCommandToKickerPSU_Wyclef(void* pvParameters) ;
static void vTask_SendCommandToKickerPSU_LocalReadQ(void* pvParameters) ;
static void vTask_SendCommandToKickerPSU_LocalReadQ_seed(void* pvParameters) ;
static void vTask_SendCommandToLcd_ReadValsQ_seed(void* pvParameters) ;
static void vTask_SendCommandToLcd_ReadValsQ(void* pvParameters) ;
static void vTask_ProcessHVON(void* pvParameters) ;
static void vTask_ProcessPSUHEALTHY(void* pvParameters) ;
static void vTask_ProcessPioISR(void* pvParameters) ;
static void vTask_LCDButtonInit(void* pvParameters) ;



//globals

portBASE_TYPE gl_readQ_RxStatus ;

/*---- UART0 ----------------------------------------------------------------- */
uint8_t gl_uart0_receive_buffer[UART0_BUFFER_SIZE] = {'\0'} ;
static uint32_t gl_pdc_uart0_buff_lcddata_size = UART0_BUFFER_SIZE ; //Current bytes in buffer

Bool gl_bWatchdog_expired = pdFALSE ;
portTickType gl_ticks ; 


//uint8_t gl_bgnd_select = 0 ;
Bool gl_backlight_state = true ;
Bool gl_b_data_changed[LCD_DISP_NUMBER_OF_DIGITS] = {'\0'} ;
Bool gl_b_unit_changed = false ;
Bool gl_b_tens_changed = false ;
Bool gl_b_hund_changed = false ;
Bool gl_b_thou_changed = false ;

Bool gl_b_isittouchevent_ONOFF = pdFALSE ;
Bool gl_b_isittouchevent_RESET = pdFALSE ;

Bool gl_b_LCD_rx_press = pdFALSE ;
Bool gl_b_we_are_not_processing_a_touch = pdTRUE ;

uint8_t gl_previous_button_type = 0 ;
uint8_t gl_send_kicker_reset_select = KICKER_RESET_POS_HVOFF ;

xEncoderParams_t gl_enc_param ;
xPio_ISR_params_t gl_ISR_enc_params ;
xPio_ISR_params_t gl_ISR_hvon_params ;
xPio_ISR_params_t gl_ISR_psuhealthhy_params ;

//uint8_t gl_ISR_HVON_count = 0 ;
Bool gl_b_HVON_updated_array[6] = {'\0'} ;
	
Bool gl_b_PSUHEALTHY_updated_array[3] = {'\0'} ;
	
Bool gl_b_pio_interrupt_array[MAX_LCD_BUTTONS] = {'\0'} ;
	
xUsarts_t gl_usart_ports ;
xReadQ_pkt_t readQ_items[READQ_SIZE] ;
uint8_t gl_us_loccrem = 0 ; //Local mode selected by default, bit set to 1 represents remote

uint8_t gl_uc_locrem_buffer[LOCAL_RECEIVE_BUFFER_SIZE_LOCREM] = "L=000000\r\n" ;

/*Remove later*/uint8_t gl_us_hvon = 0 ; //HVON, bit set represents HVON active
uint8_t gl_wyclef_command1 = 0 ;
uint8_t gl_wyclef_command2 = 0 ;
uint8_t gl_wyclef_command3 = 0 ;
uint8_t gl_wyclef_command4 = 0 ;
uint8_t gl_wyclef_command5 = 0 ;
uint8_t gl_wyclef_command6 = 0 ;
uint8_t gl_wyclef_status = 0 ;

uint16_t gl_uw_hvon = 0 ; //4-bits for each kicker on/off status

uint8_t gl_us_loccrem_prevstate = 0 ; //Previous state of local remote status, used to determine a change caused by touching the LCD  locrem button
uint8_t gl_us_load = 0 ; //bit set to 1 represents load pushed
uint8_t gl_us_load_prevstate = 0 ; //Previous state of load, used to determine a change caused by touching the LCD load button
//uint8_t gl_us_load_active = 0 ;

//work on these
//storage for kicker addresses
uint8_t gl_us_kicker_address = 1 ;
uint8_t gl_us_kicker_address_load = 1 ;
uint8_t gl_us_kicker_address_encoder = 1 ;
uint8_t gl_us_kicker_address_locrem = 1 ;
uint8_t gl_us_kicker_address_onoff = 0 ;
uint8_t gl_us_kicker_address_reset = 0 ;

/*	new method to signify kicker addresses
  mask bits in to allow queuing up of load commands etc
  the current system will allow the global kicker address varible to be overwritten
  if for instance load 1 then load 2 is pressed, 2 will be the kicker address seen.
  
  there would probably have to be some method of determining which load button was 
  activated first to make the popping back up of buttons independent of each other */


uint8_t gl_us_psuhealthy = 0 ; //monitor psu healthy signals

static uint8_t gl_lcd_receive_buffer[50] ; //storage for lcd touch events
static uint8_t gl_latch_touchinput = 0 ;

uint8_t gl_usart0_write_buffer[USART0_WRITE_BUFFER_SIZE] = {'\0'} ;
uint8_t gl_usart0_receive_buffer[USART0_RECEIVE_BUFFER_SIZE] = {'\0'} ;

uint8_t gl_usart1_write_buffer[USART1_WRITE_BUFFER_SIZE] = {'\0'} ;
uint8_t gl_usart1_receive_buffer[USART1_RECEIVE_BUFFER_SIZE] = {'\0'} ;

uint8_t gl_usart2_write_buffer[USART2_WRITE_BUFFER_SIZE] = {'\0'} ;
uint8_t gl_usart2_receive_buffer[USART2_RECEIVE_BUFFER_SIZE] = {'\0'} ;
	
	
static uint16_t gl_uw_k1p_count = 0 ;
static uint16_t gl_uw_k1n_count = 0 ;
static uint16_t gl_uw_k2p_count = 0 ;
static uint16_t gl_uw_k2n_count = 0 ;
static uint16_t gl_uw_k3p_count = 0 ;
static uint16_t gl_uw_k3n_count = 0 ;
static uint16_t gl_uw_up_count_limit = 5999 ;// range 59.99kV divisible by 10
static uint8_t gl_us_step_count = 0 ; //Coarse range selected as default
static uint32_t gl_uw_kicker_cvs[6] = {'\0'} ; //storage array for kicker counter values 
	

xSemaphoreHandle xSendLcdCommand_BinarySemaphore ;
xSemaphoreHandle xEncoderTriggered_BinarySemaphore ;
//xSemaphoreHandle xHVONTriggered_BinarySemaphore ;
//xSemaphoreHandle xPSUHEALTHYTriggered_BinarySemaphore ;
//xSemaphoreHandle xHVONTriggered_CountingSemaphore ;
//xSemaphoreHandle xHVONTriggered_BinarySemaphore ;
//xSemaphoreHandle xPSUHEALTHYTriggered_BinarySemaphore ;
//xSemaphoreHandle xPSUHEALTHYTriggered_CountingSemaphore ;
xSemaphoreHandle xPioISRTriggered_BinarySemaphore ;
xSemaphoreHandle xLcdCommand_CountingSemaphore ;
xSemaphoreHandle xLcdBacklightOff_BinarySemaphore ;
xSemaphoreHandle xLcdBacklightOn_BinarySemaphore ;
xSemaphoreHandle xLcdCommandLocrem_CountingSemaphore ;
xSemaphoreHandle xLcdCommandLoad_CountingSemaphore ;
//xSemaphoreHandle xLcdCommandOnOff_CountingSemaphore ;
xSemaphoreHandle xLcdCommandOnOff_BinarySemaphore ;
//xSemaphoreHandle xLcdCommandReset_CountingSemaphore ;
xSemaphoreHandle xLcdCommandReset_BinarySemaphore ;
xSemaphoreHandle xPsuCommand_CountingSemaphore ;
xSemaphoreHandle xReadLcdTouchEvent_BinarySemaphore ;
xSemaphoreHandle xProcessLcdTouchEvent_BinarySemaphore ;
xSemaphoreHandle xReadQ_Seed_BinarySemaphore ;
xSemaphoreHandle xLcdReadValsQ_Seed_BinarySemaphore ;
xSemaphoreHandle xLcdReadRefresh_BinarySemaphore ;
xSemaphoreHandle xCPSRead_BinarySemaphore ;
xSemaphoreHandle xWatchdog_BinarySemaphore ;

xSemaphoreHandle xQuickReleaseCheck_BinarySemaphore ;

xTimerHandle xBacklightOffTimer ;
xTimerHandle xLcdTouchEventTimer ;
xTimerHandle xLcdLocremButtonLatchTimer ;
xTimerHandle xLcdLoadButtonPopupTimer ;
xTimerHandle xReadQSeedTimer ;
xTimerHandle xLCDReadValsRefresh ;
xTimerHandle xLcdReadValsQSeedTimer ;
xTimerHandle xCPSReadScanTimer ;
xTimerHandle xLcdClearPrevButtonInfoTimer ;
xTimerHandle xLcdResetButtonTimer ;
//xTimerHandle xLcdDisableTouchscanTimer ;


//queue handles
//xQueueHandle xLCDCommand_queue ;
xQueueHandle gl_xReadQ ;
xQueueHandle xLcdReadValsQ ;

pdc_packet_t gl_pdc_uart0_pkt_lcddata ;
Pdc *gl_ptr_uart0_pdc ; //for UART0
pdc_packet_t gl_pdc_uart_pkt_lcd_backlight ;
pdc_packet_t gl_pdc_uart_npkt_lcd_backlight ;
pdc_packet_t gl_pdc_uart_pkt_buzz ;
pdc_packet_t gl_pdc_uart_pkt_bgnd ;
pdc_packet_t gl_pdc_uart_npkt_bgnd ;
pdc_packet_t gl_pdc_uart_pkt_dpoint ;
pdc_packet_t gl_pdc_uart_pkt_kVdisp ;
pdc_packet_t gl_pdc_uart_pkt_locrem ;
pdc_packet_t gl_pdc_uart_npkt_locrem ;
pdc_packet_t gl_pdc_uart_pkt_onoff ;
pdc_packet_t gl_pdc_uart_npkt_onoff ;
pdc_packet_t gl_pdc_uart_pkt_frame ;
pdc_packet_t gl_pdc_uart_npkt_frame ;
pdc_packet_t gl_pdc_uart_pkt_txt_remote ;
pdc_packet_t gl_pdc_uart_pkt_txt_local ;
pdc_packet_t gl_pdc_uart_pkt_pb1 ;
pdc_packet_t gl_pdc_uart_npkt_pb1 ;
pdc_packet_t gl_pdc_uart_pkt_loadbutton ;
pdc_packet_t gl_pdc_uart_npkt_loadbutton ;

//new pdc packets of data
pdc_packet_t gl_pdc_uart_pkt_kickadd ;
pdc_packet_t gl_pdc_uart_npkt_kickadd ;



uint8_t gl_uc_usart0_lcd_command_buffer[LCD_COMMAND_BUFFER_SIZE] = {'\0'} ;
uint8_t gl_lcd_command_length = 0 ;

uint8_t gl_uc_usart0_lcd_bgnd[LCD_BACKGROUND_BUFFER_SIZE] = {
	0xF1, 0x0A, 0xFB, 0xA1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9C, 0xF4} ;	

uint8_t gl_uc_usart0_lcd_backlighton[LCD_BACKLIGHT_BUFFER_SIZE] = {
	0xF1, 0x07, 0xF8, 0x29, 0x06, 0x01, 0x01, 0x29, 0xF4} ;
	
uint8_t gl_uc_usart0_lcd_backlightoff[LCD_BACKLIGHT_BUFFER_SIZE] = {
	0xF1, 0x07, 0xF8, 0x29, 0x06, 0x01, 0x00, 0x28, 0xF4} ;
	
//use seperate buffers for commands and polling of the kicker psus
uint8_t gl_uc_usart2_write_buffer_poll[USART2_WRITE_BUFFER_SIZE] = {'\0'} ; //used for poll of kicker psus
uint8_t gl_uc_usart2_write_buffer_onetime[USART2_WRITE_BUFFER_SIZE] = {'\0'} ; //used for one off commands sent to the kicker psus
	


uint8_t gl_uc_pdc_uart_buff_buzz[LCD_BUZZ_BUFFER_SIZE] = {
0xF1, 0x05, 0xF6, 0xFB, 0x0A, 0xFB, 0xF4} ; //Buzzer on for 0x0A*10ms = 0.1s
uint8_t gl_uc_pdc_uart_buff_bgnd[LCD_BGND_BUFFER_SIZE] = {
0xF1, 0x0A, 0xFB, 0xA1, 0x00, 0x00, 0x00, 0x00, 0x00} ;

uint8_t gl_uc_pdc_uart_buff_dpoint[LCD_DPOINT_BUFFER_SIZE] = {
0xF1, 0x0B, 0xFC, 0xAC, 0x00, disp_102, 0x00, kpos_yval, 0x00, 0x01, 0x2E, 0x57, 0xF4} ;
uint8_t gl_uc_pdc_uart_buff_kVdisp[LCD_kVDISP_BUFFER_SIZE] = {
0xF1, 0x0C, 0xFD, 0xAC, 0x00, k1_unitx, 0x00, kpos_yval, 0x00, 0x02, 0x6B, 0x56, 0x1C, 0xF4} ;
uint8_t gl_uc_pdc_uart_buff_kickadd[COUNTDISP_BUFFER_SIZE] = {
0xF1, 0x0B, 0xFC, 0xAC, 0x00, 0xE0, 0x00, 0x09, 0x02, 0x01} ;
//uint8_t gl_uc_pdc_uart_nbuff_kickadd[COUNTDISP_NEXT_BUFFER_SIZE] = {
//0x31, 0xC5, 0xF4} ;

//Base templates to display set values (and read values)
static uint8_t gl_usart0_unit[COUNTDISP_BUFFER_SIZE] = {
	0xF1, 0x0B, 0xFC, 0xAC, 0x00, disp_100, 0x00, kpos_yval, 0x00, 0x01, 0x30, 0x79, 0xF4} ;

static uint8_t gl_usart0_ten[COUNTDISP_BUFFER_SIZE] = {
	0xF1, 0x0B, 0xFC, 0xAC, 0x00, disp_101, 0x00, kpos_yval, 0x00, 0x01, 0x30, 0x69, 0xF4} ;
	
static uint8_t gl_usart0_hund[COUNTDISP_BUFFER_SIZE] = {
	0xF1, 0x0B, 0xFC, 0xAC, 0x00, disp_103, 0x00, kpos_yval, 0x00, 0x01, 0x30, 0x49, 0xF4} ;

static uint8_t gl_usart0_thou[COUNTDISP_BUFFER_SIZE] = {
	0xF1, 0x0B, 0xFC, 0xAC, 0x00, disp_104, 0x00, kpos_yval, 0x00, 0x01, 0x30, 0x39, 0xF4} ;
	
static uint8_t gl_usart0_locrem_template[LOCREM_BUTTON_BUFFER_SIZE] = {
	0xF1, 0x0A, 0xFB, 0xA4, 0xff, 0xff, 0xff, 0xff, 0x00, 0x08, 0xff, 0xF4} ;

//can amalgamate these button commands into one in the future
static uint8_t gl_usart0_load_template[LOAD_BUTTON_BUFFER_SIZE] = {
	0xF1, 0x0A, 0xFB, 0xA4, 0xff, 0xff, 0xff, 0xff, 0x00, 0x08, 0xff, 0xF4} ;
	
	
	

//TESTING||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
struct AMessage{
	char ucMessageID ;
	char ucData [COUNTDISP_BUFFER_SIZE] ;
} xMessage ;
	
char *pcUnit_testing = &gl_usart0_unit ;



//TESTING||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

//Kicker HVI serial commands
uint8_t gl_uc_pdc_hvon_tx[LCD_TX_ON_BUFFER_SIZE] = "xHV=ON\r\n" ; //Template for HV on
uint8_t gl_uc_pdc_hvoff_tx[LCD_TX_OFF_BUFFER_SIZE] = "xHV=OFF\r\n" ; //Template for HV off
uint8_t gl_uc_pdc_hvreset_tx[LCD_TX_RESET_BUFFER_SIZE] = "xRESET\r\n" ; //Template for HV reset
uint8_t gl_uc_pdc_hvcrashoff_tx[LCD_TX_CRASHOFF_BUFFER_SIZE] = "\033\r\n" ; //Template for HV reset

//6 arrays for storing kicker counts
//update them as you write to the lcd, i.e. in pdc_uart_lcd_unit etc
uint8_t gl_uc_pdc_1kV_tx[LCD_TXV_BUFFER_SIZE] = "1HV=00.00\r\n" ; //Template for kV tx
uint8_t gl_uc_pdc_2kV_tx[LCD_TXV_BUFFER_SIZE] = "2HV=00.00\r\n" ; //Template for kV tx
uint8_t gl_uc_pdc_3kV_tx[LCD_TXV_BUFFER_SIZE] = "3HV=00.00\r\n" ; //Template for kV tx
uint8_t gl_uc_pdc_4kV_tx[LCD_TXV_BUFFER_SIZE] = "4HV=00.00\r\n" ; //Template for kV tx
uint8_t gl_uc_pdc_5kV_tx[LCD_TXV_BUFFER_SIZE] = "5HV=00.00\r\n" ; //Template for kV tx
uint8_t gl_uc_pdc_6kV_tx[LCD_TXV_BUFFER_SIZE] = "6HV=00.00\r\n" ; //Template for kV tx

uint8_t gl_uc_Tx_kV_remote[LCD_TXV_BUFFER_SIZE] = "xHV=00.00\r\n" ; //Template for kV tx to handle remote operations

//Local read commands for the HVI power supplies
uint8_t gl_uc_read_status1[LOCAL_CMD_READ_STATUS_BUFFER_SIZE] = "1RPT\r\n" ;
uint8_t gl_uc_read_status2[LOCAL_CMD_READ_STATUS_BUFFER_SIZE] = "2RPT\r\n" ;
uint8_t gl_uc_read_status3[LOCAL_CMD_READ_STATUS_BUFFER_SIZE] = "3RPT\r\n" ;
uint8_t gl_uc_read_status4[LOCAL_CMD_READ_STATUS_BUFFER_SIZE] = "4RPT\r\n" ;
uint8_t gl_uc_read_status5[LOCAL_CMD_READ_STATUS_BUFFER_SIZE] = "5RPT\r\n" ;
uint8_t gl_uc_read_status6[LOCAL_CMD_READ_STATUS_BUFFER_SIZE] = "6RPT\r\n" ;

uint8_t gl_uc_read_output_volts1[LOCAL_CMD_READ_OUTV_BUFFER_SIZE] = "1HV\r\n" ;
uint8_t gl_uc_read_output_volts2[LOCAL_CMD_READ_OUTV_BUFFER_SIZE] = "2HV\r\n" ;
uint8_t gl_uc_read_output_volts3[LOCAL_CMD_READ_OUTV_BUFFER_SIZE] = "3HV\r\n" ;
uint8_t gl_uc_read_output_volts4[LOCAL_CMD_READ_OUTV_BUFFER_SIZE] = "4HV\r\n" ;
uint8_t gl_uc_read_output_volts5[LOCAL_CMD_READ_OUTV_BUFFER_SIZE] = "5HV\r\n" ;
uint8_t gl_uc_read_output_volts6[LOCAL_CMD_READ_OUTV_BUFFER_SIZE] = "6HV\r\n" ;

uint8_t gl_uc_read_demand_volts1[LOCAL_CMD_READ_DEMV_BUFFER_SIZE] = "1HVD\r\n" ;
uint8_t gl_uc_read_demand_volts2[LOCAL_CMD_READ_DEMV_BUFFER_SIZE] = "2HVD\r\n" ;
uint8_t gl_uc_read_demand_volts3[LOCAL_CMD_READ_DEMV_BUFFER_SIZE] = "3HVD\r\n" ;
uint8_t gl_uc_read_demand_volts4[LOCAL_CMD_READ_DEMV_BUFFER_SIZE] = "4HVD\r\n" ;
uint8_t gl_uc_read_demand_volts5[LOCAL_CMD_READ_DEMV_BUFFER_SIZE] = "5HVD\r\n" ;
uint8_t gl_uc_read_demand_volts6[LOCAL_CMD_READ_DEMV_BUFFER_SIZE] = "6HVD\r\n" ;

uint8_t gl_uc_read_amps1[LOCAL_CMD_READ_AMPS_BUFFER_SIZE] = "1MA\r\n" ;
uint8_t gl_uc_read_amps2[LOCAL_CMD_READ_AMPS_BUFFER_SIZE] = "2MA\r\n" ;
uint8_t gl_uc_read_amps3[LOCAL_CMD_READ_AMPS_BUFFER_SIZE] = "3MA\r\n" ;
uint8_t gl_uc_read_amps4[LOCAL_CMD_READ_AMPS_BUFFER_SIZE] = "4MA\r\n" ;
uint8_t gl_uc_read_amps5[LOCAL_CMD_READ_AMPS_BUFFER_SIZE] = "5MA\r\n" ;
uint8_t gl_uc_read_amps6[LOCAL_CMD_READ_AMPS_BUFFER_SIZE] = "6MA\r\n" ;

uint8_t gl_uc_read_power1[LOCAL_CMD_READ_POW_BUFFER_SIZE] = "1KW\r\n" ;
uint8_t gl_uc_read_power2[LOCAL_CMD_READ_POW_BUFFER_SIZE] = "2KW\r\n" ;
uint8_t gl_uc_read_power3[LOCAL_CMD_READ_POW_BUFFER_SIZE] = "3KW\r\n" ;
uint8_t gl_uc_read_power4[LOCAL_CMD_READ_POW_BUFFER_SIZE] = "4KW\r\n" ;
uint8_t gl_uc_read_power5[LOCAL_CMD_READ_POW_BUFFER_SIZE] = "5KW\r\n" ;
uint8_t gl_uc_read_power6[LOCAL_CMD_READ_POW_BUFFER_SIZE] = "6KW\r\n" ;

	
uint8_t gl_bufferTrash[TRASH_SIZE] = {'\0'} ; 


//Buffer storage for status replies (local polling) - change to zeros
uint8_t gl_uc_rxbuffer_outv1 [LOCAL_RECEIVE_BUFFER_SIZE_OUTV] = {'\0'} ; //"HV=11.18\r\n"} ; //Send a test readback
uint8_t gl_uc_rxbuffer_outv2 [LOCAL_RECEIVE_BUFFER_SIZE_OUTV] = {'\0'} ; //{"HV=22.34\r\n"} ; //Send a test readback
uint8_t gl_uc_rxbuffer_outv3 [LOCAL_RECEIVE_BUFFER_SIZE_OUTV] = {'\0'} ; //{"HV=33.52\r\n"} ; //Send a test readback
uint8_t gl_uc_rxbuffer_outv4 [LOCAL_RECEIVE_BUFFER_SIZE_OUTV] = {'\0'} ; //{"HV=44.25\r\n"} ; //Send a test readback
uint8_t gl_uc_rxbuffer_outv5 [LOCAL_RECEIVE_BUFFER_SIZE_OUTV] = {'\0'} ; //{"HV=55.63\r\n"} ; //Send a test readback
uint8_t gl_uc_rxbuffer_outv6 [LOCAL_RECEIVE_BUFFER_SIZE_OUTV] = {'\0'} ; //{"HV=66.78\r\n"} ; //Send a test readback
	
uint8_t gl_uc_rxbuffer_demv1 [LOCAL_RECEIVE_BUFFER_SIZE_DEMV] = {'\0'} ; //{"HVD=01.01\r\n"} ;
uint8_t gl_uc_rxbuffer_demv2 [LOCAL_RECEIVE_BUFFER_SIZE_DEMV] = {'\0'} ; //{"HVD=02.02\r\n"} ;
uint8_t gl_uc_rxbuffer_demv3 [LOCAL_RECEIVE_BUFFER_SIZE_DEMV] = {'\0'} ; //{"HVD=03.03\r\n"} ;
uint8_t gl_uc_rxbuffer_demv4 [LOCAL_RECEIVE_BUFFER_SIZE_DEMV] = {'\0'} ; //{"HVD=04.04\r\n"} ;
uint8_t gl_uc_rxbuffer_demv5 [LOCAL_RECEIVE_BUFFER_SIZE_DEMV] = {'\0'} ; //{"HVD=05.05\r\n"} ;
uint8_t gl_uc_rxbuffer_demv6 [LOCAL_RECEIVE_BUFFER_SIZE_DEMV] = {'\0'} ; //{"HVD=06.06\r\n"} ;	
						
uint8_t gl_uc_rxbuffer_amps1 [LOCAL_RECEIVE_BUFFER_SIZE_AMPS] = {'\0'} ; //{"MA=111.1\r\n"} ; //Send a test readback 
uint8_t gl_uc_rxbuffer_amps2 [LOCAL_RECEIVE_BUFFER_SIZE_AMPS] = {'\0'} ; //{"MA=222.2\r\n"} ; //Send a test readback 
uint8_t gl_uc_rxbuffer_amps3 [LOCAL_RECEIVE_BUFFER_SIZE_AMPS] = {'\0'} ; //{"MA=333.3\r\n"} ; //Send a test readback 
uint8_t gl_uc_rxbuffer_amps4 [LOCAL_RECEIVE_BUFFER_SIZE_AMPS] = {'\0'} ; //{"MA=444.4\r\n"} ; //Send a test readback 
uint8_t gl_uc_rxbuffer_amps5 [LOCAL_RECEIVE_BUFFER_SIZE_AMPS] = {'\0'} ; //{"MA=555.5\r\n"} ; //Send a test readback 
uint8_t gl_uc_rxbuffer_amps6 [LOCAL_RECEIVE_BUFFER_SIZE_AMPS] = {'\0'} ; //{"MA=666.6\r\n"} ; //Send a test readback 
	
uint8_t gl_uc_rxbuffer_pow1 [LOCAL_RECEIVE_BUFFER_SIZE_POW] = {'\0'} ; //{"KW=1.6\r\n"} ; //Send a test readback 
uint8_t gl_uc_rxbuffer_pow2 [LOCAL_RECEIVE_BUFFER_SIZE_POW] = {'\0'} ; //{"KW=2.5\r\n"} ; //Send a test readback 
uint8_t gl_uc_rxbuffer_pow3 [LOCAL_RECEIVE_BUFFER_SIZE_POW] = {'\0'} ; //{"KW=3.4\r\n"} ; //Send a test readback 
uint8_t gl_uc_rxbuffer_pow4 [LOCAL_RECEIVE_BUFFER_SIZE_POW] = {'\0'} ; //{"KW=4.3\r\n"} ; //Send a test readback 
uint8_t gl_uc_rxbuffer_pow5 [LOCAL_RECEIVE_BUFFER_SIZE_POW] = {'\0'} ; //{"KW=5.2\r\n"} ; //Send a test readback 
uint8_t gl_uc_rxbuffer_pow6 [LOCAL_RECEIVE_BUFFER_SIZE_POW] = {'\0'} ; //{"KW=6.1\r\n"} ; //Send a test readback 
	
static int gl_ul_outv1 ;
static int gl_ul_outv2 ;
static int gl_ul_outv3 ;
static int gl_ul_outv4 ;
static int gl_ul_outv5 ;
static int gl_ul_outv6 ;

static int gl_ul_demv1 ;
static int gl_ul_demv2 ;
static int gl_ul_demv3 ;
static int gl_ul_demv4 ;
static int gl_ul_demv5 ;
static int gl_ul_demv6 ;

static int gl_ul_amps1 ;
static int gl_ul_amps2 ;
static int gl_ul_amps3 ;
static int gl_ul_amps4 ;
static int gl_ul_amps5 ;
static int gl_ul_amps6 ;

static int gl_ul_pow1 ;
static int gl_ul_pow2 ;
static int gl_ul_pow3 ;
static int gl_ul_pow4 ;
static int gl_ul_pow5 ;
static int gl_ul_pow6 ;


//Display read values
uint8_t gl_uc_pdc_uart_buff_read_unit[COUNTDISP_BUFFER_SIZE] = {
0xF1, 0x0B, 0xFC, 0xAC, 0x00, disp_100, 0x00, kpos_yval, 0x00, 0x01, 0x30, 0x79, 0xF4} ;
uint8_t gl_uc_pdc_uart_buff_read_ten[COUNTDISP_BUFFER_SIZE] = {
0xF1, 0x0B, 0xFC, 0xAC, 0x00, disp_101, 0x00, kpos_yval, 0x00, 0x01, 0x30, 0x69, 0xF4} ;
uint8_t gl_uc_pdc_uart_buff_read_hund[COUNTDISP_BUFFER_SIZE] = {
0xF1, 0x0B, 0xFC, 0xAC, 0x00, disp_103, 0x00, kpos_yval, 0x00, 0x01, 0x30, 0x49, 0xF4} ;
uint8_t gl_uc_pdc_uart_buff_read_thou[COUNTDISP_BUFFER_SIZE] = {
0xF1, 0x0B, 0xFC, 0xAC, 0x00, disp_104, 0x00, kpos_yval, 0x00, 0x01, 0x30, 0x39, 0xF4} ;


uint8_t gl_uc_pdc_uart_buff_remote[LCD_BGND_BUFFER_SIZE] = {
0xF1, 0x0A, 0xFB, 0xA4, 0x00, 0x98, 0x00, 0xC9, 0x00} ;
uint8_t gl_uc_pdc_uart_nbuff_remote[COUNTDISP_NEXT_BUFFER_SIZE] = {
0x03, 0x03, 0xF4} ;

uint8_t gl_uc_pdc_uart_buff_locrem[LCD_BGND_BUFFER_SIZE] = {
0xF1, 0x0A, 0xFB, 0xA4, 0x00, 0x98, 0x00, 0x4A, 0x00} ;
uint8_t gl_uc_pdc_uart_nbuff_locrem[COUNTDISP_NEXT_BUFFER_SIZE] = {
0x08, 0x89, 0xF4} ;

uint8_t gl_uc_pdc_uart_buff_onoff[LCD_BGND_BUFFER_SIZE] = {
0xF1, 0x0A, 0xFB, 0xA4, 0x00, 0x20, 0x00, 0xC6, 0x00} ;
uint8_t gl_uc_pdc_uart_nbuff_onoff[COUNTDISP_NEXT_BUFFER_SIZE] = {
0x0C, 0x91, 0xF4} ;

uint8_t gl_uc_pdc_uart_buff_frame[LCD_FRAME_BUFFER_SIZE] = {
0xF1, 0x0E, 0xFF, 0x8F, 0x00, 0x39, 0x00, 0x39, 0x01, 0x29, 0x01, 0x29} ;
uint8_t gl_uc_pdc_uart_nbuff_frame[LCD_FRAME_NEXT_BUFFER_SIZE] = {
0x30, 0x01, 0x85, 0xF4} ;

uint8_t gl_uc_pdc_uart_buff_pb1[LCD_BGND_BUFFER_SIZE] = {
0xF1, 0x0A, 0xFB, 0xA4, 0x00, 0x68, 0x00, 0xB4, 0x00} ;
uint8_t gl_uc_pdc_uart_nbuff_pb1[COUNTDISP_NEXT_BUFFER_SIZE] = {
0x0A, 0xC5, 0xF4} ;

uint8_t gl_uc_pdc_uart_buff_load1[LCD_BGND_BUFFER_SIZE] = {
0xF1, 0x0A, 0xFB, 0xA4, 0x00, 0x98, 0x00, 0x78, 0x00} ;
uint8_t gl_uc_pdc_uart_nbuff_load1[COUNTDISP_NEXT_BUFFER_SIZE] = {
0x0C, 0xBB, 0xF4} ;

uint8_t gl_uc_pdc_uart_buff_txt_remote[LCD_REMOTEDISP_BUFFER_SIZE] = {
0xF1, 0x10, 0x01, 0xAC, 0x00, 0x10, 0x00, 0xD8, 0x00, 0x06, 0x52, 0x65, 0x6D, 0x6F, 0x74, 0x65, 0x07, 0xF4} ;
uint8_t gl_uc_pdc_uart_buff_txt_local[LCD_LOCALDISP_BUFFER_SIZE] = {
0xF1, 0x0F, 0x00, 0xAC, 0x00, 0x10, 0x00, 0xA0, 0x00, 0x05, 0x4C, 0x6F, 0x63, 0x61, 0x6C, 0x4C, 0xF4} ;

/*-----------------------------------------------------------*/

/*
 * Sets up the hardware ready to run this example.
 */
static void prvSetupHardware(void);


/*
 * FreeRTOS hook (or callback) functions that are defined in this file.
 */
void vApplicationMallocFailedHook(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle pxTask,
		signed char *pcTaskName);
void vApplicationTickHook(void);

/*-----------------------------------------------------------*/





/*--- Tasks -------------------------------------------------*/
void vTask_DrawLcdBackground(void* pvParameters)
{
	status_code_t sc_returned ;
	
	//xUsarts_t* pxParameters ;
	freertos_usart_if pxParam_Usart0 ;
	
	//cast the void pointer parameter to the correct type
	//pxParameters = (xUsarts_t *) pvParameters ;
	pxParam_Usart0 = (freertos_usart_if) pvParameters ;
	
	uint8_t bgnd_select = BGND_BLIGHT_ON ;	
	
	for( ;; )
	{
		/* Only this task will be accessing the uart port, therefore
		   the port should be ready to transmit.
		   However this is only true as nothing else should be 
		   happening while the init background is drawn.  When the 
		   program is running its main loop, usart0 will can be read
		   from at any point (asynchronously) via a touch event.
		   
		   Other tasks will need to check that access to the port is
		   present before proceeding with the write. */
		
		if(bgnd_select == BGND_BLIGHT_ON)
		{
			xSemaphoreGive(xLcdBacklightOn_BinarySemaphore) ; //unblock vTask_TurnBacklightOn
			
			bgnd_select = BGND_LOGO ; //set the background next time the task is run
		}
		else if(bgnd_select == BGND_LOGO)
		{
			//prepare the string
			gl_uc_usart0_lcd_bgnd[9] = bgnd_select ;
			gl_uc_usart0_lcd_bgnd[10] = bgnd_select + 0x9C ;
			
			sc_returned = usart0_write_string_standard(pxParam_Usart0, &gl_uc_usart0_lcd_bgnd, LCD_BACKGROUND_BUFFER_SIZE) ;
			
			if(sc_returned == STATUS_OK)
			{
				//change the background
				bgnd_select = BGND_MAIN ;
			}
		}
		else if(bgnd_select == BGND_MAIN)
		{
			//prepare the string with the next background
			gl_uc_usart0_lcd_bgnd[9] = bgnd_select ;
			gl_uc_usart0_lcd_bgnd[10] = bgnd_select + 0x9C ;
			
			sc_returned = usart0_write_string_standard(pxParam_Usart0, &gl_uc_usart0_lcd_bgnd, LCD_BACKGROUND_BUFFER_SIZE) ;
			
			if(sc_returned == STATUS_OK)
			{
				//we have the correct background drawn so have no further use for this task
				//enable interrupts now
				pio_handler_set_priority(PIOA, PIOA_IRQn, IRQ_PRIORITY_PIO) ;
				pio_handler_set_priority(PIOB, PIOB_IRQn, IRQ_PRIORITY_PIO) ;
				pio_handler_set_priority(PIOC, PIOC_IRQn, IRQ_PRIORITY_PIO) ;
			
				xTaskCreate(vTask_ProcessPioISR, "ISRpio", configMINIMAL_STACK_SIZE+100,
				(Bool *) gl_b_pio_interrupt_array, 1, NULL) ;
			
				//Task to draw initial state of buttons on LCD screen
				xTaskCreate(vTask_LCDButtonInit, "BtnInit", configMINIMAL_STACK_SIZE+100,
				(void *) NULL, 1, NULL) ;
			
				//Tasks to draw the LCD read values on the LCD screen
				xTaskCreate(vTask_SendCommandToLcd_ReadValsQ_seed, "LcRdVQd", configMINIMAL_STACK_SIZE+200,
				(freertos_usart_if) pxParam_Usart0, 0, NULL) ;

				xTaskCreate(vTask_SendCommandToLcd_ReadValsQ, "LcRdVlQ", configMINIMAL_STACK_SIZE+200,
				(freertos_usart_if) pxParam_Usart0, 1, NULL) ;
			
				//These timers are all reset in their associated task
				//therefore they don't need to be started now
			
				//xTimerReset(xReadQSeedTimer, 0) ; //prvSeedReadQ_Timer, start the Read Q loop
				//xTimerReset(xLcdReadValsQSeedTimer, 0) ; //prvLcdReadValsSeedQ_Timer, start the LCd read value seed Q loop
			
				xTimerStart(xCPSReadScanTimer, 0) ; //a block time of 0 simply means do not block
				//start timer to capture touch event
			
				//this is an auto restart timer, time to kick it off is now as the working bgnd has been displayed
				xTimerStart(xLcdTouchEventTimer, 0) ; //a block time of 0 simply means do not block
				
				xTimerStart(xBacklightOffTimer, 0) ;
						
				vTaskDelete(NULL) ; //self-delete vTask_DrawLcdBackground
			}
		}
		
		if(bgnd_select == BGND_LOGO)
		{
			//time to delay before the logo is displayed
			vTaskDelay(50 / portTICK_RATE_MS) ;
		}
		else
		{
			//block time while the ISIS Controls logo is displayed
			vTaskDelay(ISIS_CONTROLS_LOGO_DISPLAY_TIME) ; //time to display the ISIS controls logo
		}
	}
}
/*-----------------------------------------------------------*/

/**
 * \brief This task, when activated, send every ten seconds on debug USART2
 * the whole report of free heap and total tasks status
 */
static void task_monitor(void *pvParameters)
{
	//const int list_size = 1024 ;
	static portCHAR szList[TASKMON_LISTSIZE];
	static char header[] = "Name\t\tState\tPty\tStack\tNum\n\r----------------------------------------------" ;
	//char* ptr_string ;
	unsigned int taskCount ;

	for (;;) {
		vTaskList((signed portCHAR *)szList);
		
		//print header
		//usart2_write_string_standard((freertos_usart_if)pvParameters, &header, (sizeof(header)/sizeof(header[0]))) ;
		
		//print data
		usart1_write_string_16bit((freertos_usart_if)pvParameters, &szList, TASKMON_LISTSIZE) ;
		
		//delay
		vTaskDelay(250 / portTICK_RATE_MS) ;
	}
}
/*-----------------------------------------------------------*/

void vTask_TurnBacklightOn(void* pvParameters)
{
	status_code_t sc_returned ;
	//uint8_t us_checksum ;
	freertos_usart_if usart0 = (freertos_usart_if)pvParameters ; //get task parameters
	
	for( ;; )
	{
		//take binary semaphore, unblock this high priority task to turn the backlight on
		xSemaphoreTake(xLcdBacklightOn_BinarySemaphore, portMAX_DELAY) ;
		
		sc_returned = usart0_write_string_standard(usart0, &gl_uc_usart0_lcd_backlighton, LCD_BACKLIGHT_BUFFER_SIZE) ;

		//what to do if returned status code is not OK?? Unblock the task again ??
		if(sc_returned == STATUS_OK)
		{
			gl_backlight_state = LCD_ON ; //set the backlight state to ON
			xTimerReset(xBacklightOffTimer, 0) ; //reset the Backlight timer
		}
		else
		{
			usart0_write_string_standard(usart0, &gl_uc_usart0_lcd_backlighton, LCD_BACKLIGHT_BUFFER_SIZE) ; //try again
			gl_backlight_state = LCD_ON ; //set the backlight state to ON
			xTimerReset(xBacklightOffTimer, 0) ; //reset the Backlight timer
		}		
	}
}
/*-----------------------------------------------------------*/

void vTask_TurnBacklightOff(void* pvParameters)
{
	status_code_t sc_returned ;
	//uint8_t us_checksum ;
	freertos_usart_if usart0 = (freertos_usart_if)pvParameters ; //get task parameters
	
	for( ;; )
	{
		//take binary semaphore from Backlight timer task, unblock this high priority task to turn the backlight off
		xSemaphoreTake(xLcdBacklightOff_BinarySemaphore, portMAX_DELAY) ;
		
		sc_returned = usart0_write_string_standard(usart0, &gl_uc_usart0_lcd_backlightoff, LCD_BACKLIGHT_BUFFER_SIZE) ;
		
		//what to do if returned status code is not OK?? Unblock the task again ??
		if(sc_returned == STATUS_OK)
		{
			gl_backlight_state = LCD_OFF ; //set the backlight state to OFF			
		}
		else
		{
			usart0_write_string_standard(usart0, &gl_uc_usart0_lcd_backlightoff, LCD_BACKLIGHT_BUFFER_SIZE) ; //try again			
			gl_backlight_state = LCD_OFF ; //set the backlight state to OFF
		}
	}
}
/*-----------------------------------------------------------*/

void vTask_SendCommandToLcd_setKV(void* pvParameters)
{
	portBASE_TYPE result = pdFAIL ;
	uint8_t i ;
	uint8_t digit_select ;
	uint16_t* uw_tempcount ;
	uint32_t lcd_unit ; 
	uint32_t lcd_ten  ; 
	uint32_t lcd_hund ; 
	uint32_t lcd_thou ; 
	Bool tx_permit = pdFALSE ;
	uint8_t us_kick_add ;
	
	for( ;; )
	{
		//This task prints the (black) kV set value onto the LCD screen
		

		xSemaphoreTake(xLcdCommand_CountingSemaphore, portMAX_DELAY) ; /* given by various
																		  has_encoder_unit_changed
																		  has_encoder_tens_changed
																		  has_encoder_hund_changed
																		  has_encoder_thou_changed
																		  and the back-light on tasks for some reason, 
																		  back-light implementation needs looking at */		
		//if(result == pdPASS){
			
			/* if any of the LCD data set kV data has changed then proceed
			 else exit the task */
			
			if(gl_b_data_changed[DISP_UNIT])
			{
				us_kick_add = get_kicker_address_encoder() ;
				uw_tempcount = get_active_kicker_count(us_kick_add) ;				
				lcd_unit = *uw_tempcount%10 ;
				digit_select = DISP_UNIT ;
			}
			else if(gl_b_data_changed[DISP_TENS])
			{
				us_kick_add = get_kicker_address_encoder() ;
				uw_tempcount = get_active_kicker_count(us_kick_add) ;
				lcd_ten  = (*uw_tempcount/10)%10 ;
				digit_select = DISP_TENS ;
			}
			else if(gl_b_data_changed[DISP_HUND])
			{
				us_kick_add = get_kicker_address_encoder() ;
				uw_tempcount = get_active_kicker_count(us_kick_add) ;
				lcd_hund = (*uw_tempcount/100)%10 ;
				digit_select = DISP_HUND ;
			}
			else if(gl_b_data_changed[DISP_THOU])
			{
				us_kick_add = get_kicker_address_encoder() ;
				uw_tempcount = get_active_kicker_count(us_kick_add) ;
				lcd_thou = (*uw_tempcount/1000)%10 ;
				digit_select = DISP_THOU ;
			}
			else
				digit_select = 4 ; //set to invalid value
			
			/*for(i=0; i<LCD_DISP_NUMBER_OF_DIGITS; i++)
			{
				if(gl_b_data_changed[i] == true){
					break ;
				}
			}//*/
		
			switch(digit_select){
				case DISP_UNIT:
					lcd_display_count(us_kick_add, (uint8_t)lcd_unit, disp_100, &gl_usart0_unit, LCD_TEXT_BLACK, LCD_TEXT_COUNT) ;
					tx_permit = pdTRUE ;
					gl_b_data_changed[DISP_UNIT] = false ;
					break ;
					
				case DISP_TENS:
					lcd_display_count(us_kick_add, (uint8_t)lcd_ten, disp_101, &gl_usart0_ten, LCD_TEXT_BLACK, LCD_TEXT_COUNT) ;
					tx_permit = pdTRUE ;
					gl_b_data_changed[DISP_TENS] = false ;
					break ;
				
				case DISP_HUND:
					lcd_display_count(us_kick_add, (uint8_t)lcd_hund, disp_103, &gl_usart0_hund, LCD_TEXT_BLACK, LCD_TEXT_COUNT) ;
					tx_permit = pdTRUE ;
					gl_b_data_changed[DISP_HUND] = false ;
					break ;
				
				case DISP_THOU:
					lcd_display_count(us_kick_add, (uint8_t)lcd_thou, disp_104, &gl_usart0_thou, LCD_TEXT_BLACK, LCD_TEXT_COUNT) ;
					tx_permit = pdTRUE ;
					gl_b_data_changed[DISP_THOU] = false ;
					break ;
				
				default:
					tx_permit = pdFALSE ;
					break ;
			}
			
			if(tx_permit == pdTRUE){
				usart0_write_string_standard((freertos_usart_if)pvParameters, &gl_uc_usart0_lcd_command_buffer, COUNTDISP_BUFFER_SIZE) ;
				tx_permit = pdFALSE ;				
			}

		//}		
	} //end of for( ;; )	
}
/*-----------------------------------------------------------*/

void vTask_SendCommandToLcd_setLocalRemote(void* pvParameters)
{
	/* Check which kicker's local/remote status has changed
	   then update the display accordingly */
	
	//portBASE_TYPE result = pdFAIL ;
	uint8_t us_kicker_address ;
	uint8_t us_checksum ;
	Bool b_local_mode ;
		
	for( ;; )
	{
		/*  When the  program is running its main loop, usart0 will can be read
		   from at any point (asynchronously) via a touch event.
		   
		   Other tasks will need to check that access to the port is
		   present before proceeding with the write. */
		
		//result = xSemaphoreTake(xSendLcdCommand_BinarySemaphore, portMAX_DELAY) ;
//		result = xSemaphoreTake(xLcdCommand_CountingSemaphore, portMAX_DELAY) ;
		xSemaphoreTake(xLcdCommandLocrem_CountingSemaphore, portMAX_DELAY) ; //given by vTask_ProcessTouchEvent
		//if(result == pdPASS){
			
			//check which kicker has been updated
			//us_kicker_address = kicker_locrem_updated() ;
			
			//get the kicker address and check it is valid
			us_kicker_address = get_kicker_address_locrem() ;
			
			if( (us_kicker_address >= 1) && (us_kicker_address <= 6)){
				
				//store new locrem status
				//gl_us_loccrem_prevstate = gl_us_loccrem ; //this may be redundant now
			
				//now get the local/remote status
				b_local_mode = kicker_get_locrem_status(us_kicker_address) ;			
			
				//assemble the command string, co-rds, button type, checksum
				lcd_display_set_coords_locrem(us_kicker_address, b_local_mode, &gl_uc_usart0_lcd_command_buffer) ;
			
				//Now get the checksum
				us_checksum = lcd_chksum(&gl_uc_usart0_lcd_command_buffer, LOCREM_BUTTON_BUFFER_SIZE) ;

				gl_uc_usart0_lcd_command_buffer[LOCREM_BUTTON_BUFFER_SIZE-2] = us_checksum ;			
			
				usart0_write_string_standard((freertos_usart_if)pvParameters, &gl_uc_usart0_lcd_command_buffer, LOCREM_BUTTON_BUFFER_SIZE) ;
			
				xTimerReset(xLcdTouchEventTimer, portMAX_DELAY) ;
				
			}
			else{
				xTimerReset(xLcdTouchEventTimer, portMAX_DELAY) ;
			}
			
		//}
		
		//xTimerStart(xLcdLocremButtonLatchTimer, 0) ; //a block time of 0 simply means do not block, calls prvLcdLocremButtonLatchTimerCallback
		
		//result = xSemaphoreTake(xLcdCommandLocrem_CountingSemaphore, (500 / portTICK_RATE_MS) ) ;//portMAX_DELAY) ; //given by vTaskProcessTouchEvent
	} //end of for( ;; )	
}
/*-----------------------------------------------------------*/


void vTask_SendCommandToLcd_setReset(void* pvParameters)
{
	/* Draw the Reset button to the LCD screen */
	
	//portBASE_TYPE result = pdFAIL ;
	uint8_t us_kicker_address ;
	uint8_t us_checksum ;
	freertos_usart_if usart0 ;

	usart0 = (freertos_usart_if) pvParameters ;
		
	for( ;; )
	{
		xSemaphoreTake(xLcdCommandReset_BinarySemaphore, portMAX_DELAY) ; //given by vTask_ProcessPSUHEALTHY or vTask_ProcessTouchEvent
		
		//get the kicker address and check it is valid
		us_kicker_address = get_kicker_address_reset() ;		
			
		if( (us_kicker_address >= 1) && (us_kicker_address <= 6)){
				
			//assemble the command string, co-ords, button type, checksum
			lcd_display_set_coords_reset(us_kicker_address, kicker_get_psuhealthy_status(us_kicker_address), gl_b_isittouchevent_RESET, &gl_uc_usart0_lcd_command_buffer) ;
			
			//Now get the checksum
			us_checksum = lcd_chksum(&gl_uc_usart0_lcd_command_buffer, RESET_BUTTON_BUFFER_SIZE) ;

			gl_uc_usart0_lcd_command_buffer[RESET_BUTTON_BUFFER_SIZE-2] = us_checksum ;			
			
			usart0_write_string_standard(usart0, &gl_uc_usart0_lcd_command_buffer, RESET_BUTTON_BUFFER_SIZE) ;
			
			xTimerReset(xLcdTouchEventTimer, 0) ;
			//xTimerReset(xLcdDisableTouchscanTimer, 0) ;
		}
		else{
			//Kicker address was not valid so restart the touch event timer and block the task
			//xTimerReset(xLcdLocremButtonLatchTimer, 0) ; //a block time of 0 simply means do not block, calls prvLcdLocremButtonLatchTimerCallback			
			xTimerReset(xLcdTouchEventTimer, 0) ;
		}//*/
	} //end of for( ;; )	//*/
}
/*-----------------------------------------------------------*/


void vTask_SendCommandToLcd_setOnOff(void* pvParameters)
{
	/* Draw the ON or OFF button to the LCD screen */
	
	//portBASE_TYPE result = pdFAIL ;
	uint8_t us_kicker_address ;
	uint8_t us_checksum ;
	freertos_usart_if usart0 ;

	usart0 = (freertos_usart_if) pvParameters ;
		
	for( ;; )
	{
		//xSemaphoreTake(xLcdCommandOnOff_CountingSemaphore, portMAX_DELAY) ; //given by vTask_ProcessHVON or vTask_ProcessTouchEvent
		
		xSemaphoreTake(xLcdCommandOnOff_BinarySemaphore, portMAX_DELAY) ;
		
		//get the kicker address and check it is valid
		us_kicker_address = get_kicker_address_onoff() ;		
			
		if( (us_kicker_address >= 1) && (us_kicker_address <= 6)){
				
			//assemble the command string, co-ords, button type, checksum
			lcd_display_set_coords_onoff(us_kicker_address, kicker_get_hvon_status(us_kicker_address), gl_b_isittouchevent_ONOFF, &gl_uc_usart0_lcd_command_buffer) ;
			
			//Now get the checksum
			us_checksum = lcd_chksum(&gl_uc_usart0_lcd_command_buffer, ONOFF_BUTTON_BUFFER_SIZE) ;

			gl_uc_usart0_lcd_command_buffer[ONOFF_BUTTON_BUFFER_SIZE-2] = us_checksum ;			
			
			usart0_write_string_standard(usart0, &gl_uc_usart0_lcd_command_buffer, ONOFF_BUTTON_BUFFER_SIZE) ;
			
			xTimerReset(xLcdTouchEventTimer, 0) ;
			//xTimerReset(xLcdDisableTouchscanTimer, 0) ;
		}
		else{
			//Kicker address was not valid so restart the touch event timer and block the task
			//xTimerReset(xLcdLocremButtonLatchTimer, 0) ; //a block time of 0 simply means do not block, calls prvLcdLocremButtonLatchTimerCallback			
			xTimerReset(xLcdTouchEventTimer, 0) ;
		}//*/
	} //end of for( ;; )	//*/
}
/*-----------------------------------------------------------*/


void vTask_SendCommandToLcd_setLoad(void* pvParameters)
{
	/* Check which kicker's load button status has changed
	   then update the display accordingly. Unblocked by touch
	   event task (draw load button down) and also load one-shot
	   timer (draw load button up) */
	
	//portBASE_TYPE result = pdFAIL ;
	uint8_t us_kicker_address ;
	uint8_t us_checksum ;
	Bool b_load_state ;
		
	for( ;; )
	{
		/*  When the  program is running its main loop, usart0 will can be read
		   from at any point (asynchronously) via a touch event.
		   
		   Other tasks will need to check that access to the port is
		   present before proceeding with the write. */
		xSemaphoreTake(xLcdCommandLoad_CountingSemaphore, portMAX_DELAY) ; //given by vTask_ProcessTouchEvent
		//if(result == pdPASS){
			
			//check which kicker load button has been pressed
			//us_kicker_address = kicker_load_updated() ;
			
			//get the kicker address and check it is valid
			us_kicker_address = get_kicker_address_load() ;			
			
			if( (us_kicker_address >= 1) && (us_kicker_address <= 6)){
								
				//store new load status, done in the timer callback function
				gl_us_load_prevstate = gl_us_load ;
				
				//now get the status of the load button
				b_load_state = kicker_get_load_status(us_kicker_address) ;
				
				//assemble the command string, co-rds, button type, checksum
				lcd_display_set_coords_load(us_kicker_address, b_load_state, &gl_uc_usart0_lcd_command_buffer) ;
				us_checksum = lcd_chksum(&gl_uc_usart0_lcd_command_buffer, LOAD_BUTTON_BUFFER_SIZE) ;
				gl_uc_usart0_lcd_command_buffer[LOAD_BUTTON_BUFFER_SIZE-2] = us_checksum ;
				
				//transmit the command string
				usart0_write_string_standard((freertos_usart_if)pvParameters, &gl_uc_usart0_lcd_command_buffer, LOAD_BUTTON_BUFFER_SIZE) ;
				
				if(b_load_state == BUTTON_DOWN){
					//start one-shot timer to pop up the load button
					xTimerStart(xLcdLoadButtonPopupTimer, portMAX_DELAY) ; //0 ) ; //a block time of 0 simply means do not block, calls prvLcdLoadButtonPopupTimerCallback
				}
				else if(b_load_state == BUTTON_UP){
					//finished processing of the Load button, restart touch event timer
					xTimerReset(xLcdTouchEventTimer, portMAX_DELAY) ; //0 ) ;
					//xTimerReset(xLcdDisableTouchscanTimer, 0) ;
				}
				else{
					//b_load_state is invalid, clear load bit and restart touch event timer
					clear_load(get_kicker_address_load()) ;
					xTimerReset(xLcdTouchEventTimer, portMAX_DELAY) ; //0 ) ;
				}
			}
			else{
				//Kicker address was not valid so restart the touch event timer and block the task
				xTimerReset(xLcdTouchEventTimer, portMAX_DELAY) ; //0 ) ;
				//gl_latch_touchinput = 0 ;				
			}						
		//}	
		
		//result = xSemaphoreTake(xLcdCommandLoad_CountingSemaphore, portMAX_DELAY) ; //given by vTaskProcessTouchEvent
	} //end of for( ;; )	
}
/*-----------------------------------------------------------*/

void vTask_SendCommandToKickerPSU_Wyclef(void* pvParameters)
{
	/* This task will be unblocked when there is something to send to the kicker power supplies one time.
	This will normally be a command that originates from either the touch screen or kicker cps crate  */
	
	//portBASE_TYPE result = pdFAIL ;
	
	uint8_t us_kicker_address ;
	uint8_t number_of_bytes_to_tx ; //= LCD_TXV_BUFFER_SIZE ; //16 ; //size of string to transmit kV to PSU
	
	char* ptr_string ;
	
	for( ;; )
	{
		/*  When the  program is running its main loop, usart2 will be polling
		    the power supplies (main read loop).  It will exit this loop if higher priority events occur.  These are:
				pio interrupts - encoders, psuhealthy, hvon
				touch events - anything to process on the lcd
				commands from cps crate
				
			This task will check the nature of the command to send to the power supply
			and set up the bytes to transmit and the required command
		*/
		xSemaphoreTake(xPsuCommand_CountingSemaphore, portMAX_DELAY) ; //given by vTask_ProcessTouchEvent
		
		switch(get_wyclef_status())
		{
			case WYCLEF_TX_KV:
				ptr_string = get_string_address_txKV(get_kicker_address_load()) ;
				number_of_bytes_to_tx = LCD_TXV_BUFFER_SIZE ;
				if(ptr_string != NULL){
					usart2_write_string_standard((freertos_usart_if)pvParameters, ptr_string, number_of_bytes_to_tx) ;
				}
				break ;
				
			case WYCLEF_TX_KVREMOTE:
				ptr_string = get_string_address_txKVRemote() ;//get_kicker_address_load()) ;
				number_of_bytes_to_tx = LCD_TXV_BUFFER_SIZE ;
				if(ptr_string != NULL){
					usart2_write_string_standard((freertos_usart_if)pvParameters, ptr_string, number_of_bytes_to_tx) ;
				}
				break ;
				
			case WYCLEF_TX_ON:				
				ptr_string = get_string_address_txON(get_kicker_address_onoff()) ;
				number_of_bytes_to_tx = LCD_TX_ON_BUFFER_SIZE ;
				*ptr_string = get_kicker_address_onoff() + ASCII_OFFSET ; //set address
				if(ptr_string != NULL){
					usart2_write_string_standard((freertos_usart_if)pvParameters, ptr_string, number_of_bytes_to_tx) ;
				}
				break ;
		
			case WYCLEF_TX_OFF:			
				ptr_string = get_string_address_txOFF(get_kicker_address_onoff()) ;
				number_of_bytes_to_tx = LCD_TX_OFF_BUFFER_SIZE ;
				*ptr_string = get_kicker_address_onoff() + ASCII_OFFSET ;
				if(ptr_string != NULL){
					usart2_write_string_standard((freertos_usart_if)pvParameters, ptr_string, number_of_bytes_to_tx) ;
				}
				break ;
				
			case WYCLEF_TX_CRASHOFF:
				usart2_write_string_standard((freertos_usart_if)pvParameters, &gl_uc_pdc_hvcrashoff_tx, LCD_TX_CRASHOFF_BUFFER_SIZE) ;
				break ;
			
			case WYCLEF_TX_RESET:
				switch(gl_send_kicker_reset_select)
				{
					case KICKER_RESET_POS_HVOFF:
						ptr_string = get_string_address_txOFF(get_kicker_address_reset()) ;
						number_of_bytes_to_tx = LCD_TX_OFF_BUFFER_SIZE ;
						*ptr_string = get_kicker_address_reset() + ASCII_OFFSET ;
						if(ptr_string != NULL){
							usart2_write_string_standard((freertos_usart_if)pvParameters, ptr_string, number_of_bytes_to_tx) ;
						}
						xTimerReset(xLcdResetButtonTimer, portMAX_DELAY) ; //calls prvLcdResetButtonTimer
						break ;
					
					case KICKER_RESET_NEG_HVOFF:
						ptr_string = get_string_address_txOFF(get_kicker_address_reset()) ;
						number_of_bytes_to_tx = LCD_TX_OFF_BUFFER_SIZE ;
						*ptr_string = get_kicker_address_reset() + ASCII_OFFSET + 1 ;
						if(ptr_string != NULL){
							usart2_write_string_standard((freertos_usart_if)pvParameters, ptr_string, number_of_bytes_to_tx) ;
						}
						break ;
					
					case KICKER_RESET_POS_RST:
						ptr_string = get_string_address_txRESET(get_kicker_address_reset()) ;
						number_of_bytes_to_tx = LCD_TX_RESET_BUFFER_SIZE ;
						*ptr_string = get_kicker_address_reset() + ASCII_OFFSET ;
						if(ptr_string != NULL){
							usart2_write_string_standard((freertos_usart_if)pvParameters, ptr_string, number_of_bytes_to_tx) ;
						}
						break ;
					
					case KICKER_RESET_NEG_RST:
						ptr_string = get_string_address_txRESET(get_kicker_address_reset()) ;
						number_of_bytes_to_tx = LCD_TX_RESET_BUFFER_SIZE ;
						*ptr_string = get_kicker_address_reset() + ASCII_OFFSET + 1 ;
						if(ptr_string != NULL){
							usart2_write_string_standard((freertos_usart_if)pvParameters, ptr_string, number_of_bytes_to_tx) ;
						}
						break ;
					
					default:
						ptr_string = NULL ;
						break ;
				}				
				break ;
				
			default:  //command not recognised
				//do nothing except end the current task
				break ;
		}		
	} //end of for( ;; )	
}
/*-----------------------------------------------------------*/


void vTask_SendCommandToKickerPSU_LocalReadQ(void* pvParameters)
{
	xReadQ_pkt_t Q_item ;
	xRxBuffer_info_t local_rx_buff ;
	freertos_usart_if usart2 ;
	
	portBASE_TYPE xStatus ;
	//portTickType xTicksToWait = portMAX_DELAY ;// / portTICK_RATE_MS ;
	
	/* The USART is passed into this task as the task parameter.  Cast the void 
	   pointer back to a freertos_usart_if */
	usart2 = (freertos_usart_if) pvParameters ;

	for( ;; )
	{		
		//receive item from Q
		
		/* First parameter is the Queue from which data is to be received.  The Q is created before the 
		scheduler is started, and therefore before this task runs for the first time.   
		
		Second parameter is the buffer into which received data will be placed.  In this case the buffer 
		is simply the address of a variable that has the required size to hold the received data.
		
		Last parameter is the block time - the MAXIMUM time that the task should remain in the blocked 
		state to wait for the data to be available should the queue already be empty.  */
		//gpio_set_pin_low(LED2_GPIO) ; //LED off - monitor on scope to show Task in use
		
		gl_readQ_RxStatus = xQueueReceive( gl_xReadQ, &Q_item, portMAX_DELAY ) ;
		
		//gpio_set_pin_high(LED2_GPIO) ; //LED on - monitor on scope to show Task in use
		
		//if (xStatus == pdPASS)
		//{
			//data successfully removed from the Q
		
			//Process item in the Read Q		
			
			//get receive buffer info, size of buffer, address of buffer
			local_rx_buff = get_receive_buffer_info(Q_item.psu_address, Q_item.command_type) ;
			
			//Insert check for LCD quick release
			/*if(gl_b_LCD_rx_press){
				xSemaphoreGive(xQuickReleaseCheck_BinarySemaphore) ; //unblock vTask_ReadCommandFromLcd_QuickReleaseCheck
			}//*/
			
			//If the receive buffer is valid, tx the status command and store the reply
			if(local_rx_buff.ptr_to_buffer != NULL)
			{
				//empty the USART receive buffer
				flush_usart_buffer( usart2, gl_bufferTrash, 1, 20) ;
					
				//send command via usart2
				usart2_write_string_standard( usart2, 
											  Q_item.ptr_to_command, 
											  Q_item.number_of_bytes) ;		
											  
				//store the reply
				xStatus = usart2_read_string_standard( usart2,
											 local_rx_buff.ptr_to_buffer,
											 local_rx_buff.buffer_size) ;
				
				//if a reply was received, store the integer value
				if(xStatus == pdPASS)
				{
					//translate the received string into an integer value
					save_nonascii_value(Q_item.command_type, local_rx_buff.ptr_to_buffer, local_rx_buff.nonascii_value) ;
				}
				else
				{
					//no local reply received, pass the error on to the CPS crate
					*local_rx_buff.ptr_to_buffer = NULL ; //this will propagate through to the CPS crate and display an error on the control screen
				}
				
				/* Temporary method to start the lcd read refresh timer 
				   once the last command in the Q has been processed */
				//if( (Q_item.command_type == READ_POWER) && (Q_item.psu_address == 6) )
				//{
					//xTimerReset(xLCDReadValsRefresh, 0) ; //start to update the LCD with the readbacks, prvLcdReadRefresh_Timer
					//xSemaphoreGive(xLcdReadRefresh_BinarySemaphore) ; //unblocks vTask_DrawLcd_ReadVals
				//}
				/* Change this so that LCD refresh is unblocked after each local poll command.
				   This way the Lcd commands should nicely interleave the local poll commands.
				   The Lcd read vals are only update IF they have CHANGED - no point sending unnecessary commands */				
											 
				/* The CPS crate will receive status from the receive buffers.
				   These are what will be transmitted when the CPS crate polls.
				   These values also need to be displayed on the LCD screen, done in another task */
			}			
		//}
	} //end of for ( ;; )
	
}
/*-----------------------------------------------------------*/


void vTask_SendCommandToKickerPSU_LocalReadQ_seed(void* pvParameters)
{
	portBASE_TYPE xStatus ;
	portBASE_TYPE result = pdFAIL ;
	xReadQ_pkt_t* ptr_to_Q_item ;
	xReadQ_pkt_t Q_item ;
	uint next_position_in_Q = 7 ;//1 ;
	portTickType wait_for_space = (500 / portTICK_RATE_MS) ; //introduce a 500ms wait for space in the queue
	portTickType wait_for_timer = (1000 / portTICK_RATE_MS) ;
	uint32_t ul_remaining_stack = 0 ;
	
		
	for ( ;; )
	{		
		/* First parameter is the Q, this is created before this task started to execute 
		
		Second parameter is the address of the data to be sent or placed in the Q.
		
		Thirdly, the block time.  The time the task should be kept in the blocked state 
		to wait for space to become available on the Q, should it already be full.  As items
		should be removed once they are placed in the Q it should never get full.  Try specifying
		a block time of 0 */
		
		//the list of read commands are iterated here
		
		//anything that takes priority will interrupt this task but the place should be saved and 
		//processing will continue once the higher priority task completes
		
		xSemaphoreTake(xReadQ_Seed_BinarySemaphore, portMAX_DELAY) ; //given by prvSeedReadQ_Timer
		//if( result == pdPASS){
			
			ptr_to_Q_item = &Q_item ;
		
			switch(next_position_in_Q)
			{
				/*case 1:
					Q_item.command_type = READ_STATUS ;
					Q_item.number_of_bytes = LOCAL_CMD_READ_STATUS_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_status(1) ;
					Q_item.psu_address = 1 ;
					next_position_in_Q = 2 ;				
					break ;
				
				case 2:
					Q_item.command_type = READ_STATUS ;
					Q_item.number_of_bytes = LOCAL_CMD_READ_STATUS_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_status(2) ;
					Q_item.psu_address = 2 ;
					next_position_in_Q = 3 ;				
					break ;
					
				case 3:
					Q_item.command_type = READ_STATUS ;
					Q_item.number_of_bytes = LOCAL_CMD_READ_STATUS_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_status(3) ;
					Q_item.psu_address = 3 ;
					next_position_in_Q = 4 ;
					break ;
				
				case 4:
					Q_item.command_type = READ_STATUS ;
					Q_item.number_of_bytes = LOCAL_CMD_READ_STATUS_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_status(4) ;
					Q_item.psu_address = 4 ;
					next_position_in_Q = 5 ;
					break ;
				
				case 5:
					Q_item.command_type = READ_STATUS ;
					Q_item.number_of_bytes = LOCAL_CMD_READ_STATUS_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_status(5) ;
					Q_item.psu_address = 5 ;
					next_position_in_Q = 6 ;
					break ;
				
				case 6:
					Q_item.command_type = READ_STATUS ;
					Q_item.number_of_bytes = LOCAL_CMD_READ_STATUS_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_status(6) ;
					Q_item.psu_address = 6 ;
					next_position_in_Q = 7 ;
					break ;//*/
				
				case 7:
					Q_item.command_type = READ_OUTV;
					Q_item.number_of_bytes = LOCAL_CMD_READ_OUTV_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_output_volts(1) ;
					Q_item.psu_address = 1 ;
					next_position_in_Q = 8 ;
					break ;	
					
				case 8:
					Q_item.command_type = READ_OUTV;
					Q_item.number_of_bytes = LOCAL_CMD_READ_OUTV_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_output_volts(2) ;
					Q_item.psu_address = 2 ;
					next_position_in_Q = 9 ;
					break ;
					
				case 9:
					Q_item.command_type = READ_OUTV;
					Q_item.number_of_bytes = LOCAL_CMD_READ_OUTV_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_output_volts(3) ;
					Q_item.psu_address = 3 ;
					next_position_in_Q = 10 ;
					break ;
					
				case 10:
					Q_item.command_type = READ_OUTV;
					Q_item.number_of_bytes = LOCAL_CMD_READ_OUTV_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_output_volts(4) ;
					Q_item.psu_address = 4 ;
					next_position_in_Q = 11 ;
					break ;
					
				case 11:
					Q_item.command_type = READ_OUTV;
					Q_item.number_of_bytes = LOCAL_CMD_READ_OUTV_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_output_volts(5) ;
					Q_item.psu_address = 5 ;
					next_position_in_Q = 12 ;
					break ;
					
				case 12:
					Q_item.command_type = READ_OUTV;
					Q_item.number_of_bytes = LOCAL_CMD_READ_OUTV_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_output_volts(6) ;
					Q_item.psu_address = 6 ;
					next_position_in_Q = 13 ;
					break ;	
					
				case 13:
					Q_item.command_type = READ_DEMV;
					Q_item.number_of_bytes = LOCAL_CMD_READ_DEMV_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_demand_volts(1) ;
					Q_item.psu_address = 1 ;
					next_position_in_Q = 14 ;
					break ;	
					
				case 14:
					Q_item.command_type = READ_DEMV;
					Q_item.number_of_bytes = LOCAL_CMD_READ_DEMV_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_demand_volts(2) ;
					Q_item.psu_address = 2 ;
					next_position_in_Q = 15 ;
					break ;
					
				case 15:
					Q_item.command_type = READ_DEMV;
					Q_item.number_of_bytes = LOCAL_CMD_READ_DEMV_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_demand_volts(3) ;
					Q_item.psu_address = 3 ;
					next_position_in_Q = 16 ;
					break ;
					
				case 16:
					Q_item.command_type = READ_DEMV;
					Q_item.number_of_bytes = LOCAL_CMD_READ_DEMV_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_demand_volts(4) ;
					Q_item.psu_address = 4 ;
					next_position_in_Q = 17 ;
					break ;
					
				case 17:
					Q_item.command_type = READ_DEMV;
					Q_item.number_of_bytes = LOCAL_CMD_READ_DEMV_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_demand_volts(5) ;
					Q_item.psu_address = 5 ;
					next_position_in_Q = 18 ;
					break ;
					
				case 18:
					Q_item.command_type = READ_DEMV;
					Q_item.number_of_bytes = LOCAL_CMD_READ_DEMV_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_demand_volts(6) ;
					Q_item.psu_address = 6 ;
					next_position_in_Q = 19 ;
					break ;
					
				case 19:
					Q_item.command_type = READ_AMPS;
					Q_item.number_of_bytes = LOCAL_CMD_READ_AMPS_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_amps(1) ;
					Q_item.psu_address = 1 ;
					next_position_in_Q = 20 ;
					break ;	
					
				case 20:
					Q_item.command_type = READ_AMPS;
					Q_item.number_of_bytes = LOCAL_CMD_READ_AMPS_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_amps(2) ;
					Q_item.psu_address = 2 ;
					next_position_in_Q = 21 ;
					break ;
					
				case 21:
					Q_item.command_type = READ_AMPS;
					Q_item.number_of_bytes = LOCAL_CMD_READ_AMPS_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_amps(3) ;
					Q_item.psu_address = 3 ;
					next_position_in_Q = 22 ;
					break ;
					
				case 22:
					Q_item.command_type = READ_AMPS;
					Q_item.number_of_bytes = LOCAL_CMD_READ_AMPS_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_amps(4) ;
					Q_item.psu_address = 4 ;
					next_position_in_Q = 23 ;
					break ;
					
				case 23:
					Q_item.command_type = READ_AMPS;
					Q_item.number_of_bytes = LOCAL_CMD_READ_AMPS_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_amps(5) ;
					Q_item.psu_address = 5 ;
					next_position_in_Q = 24 ;
					break ;
					
				case 24:
					Q_item.command_type = READ_AMPS;
					Q_item.number_of_bytes = LOCAL_CMD_READ_AMPS_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_amps(6) ;
					Q_item.psu_address = 6 ;
					next_position_in_Q = 25 ;
					break ;
					
				case 25:
					Q_item.command_type = READ_POWER;
					Q_item.number_of_bytes = LOCAL_CMD_READ_POW_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_power(1) ;
					Q_item.psu_address = 1 ;
					next_position_in_Q = 26 ;
					break ;	
					
				case 26:
					Q_item.command_type = READ_POWER;
					Q_item.number_of_bytes = LOCAL_CMD_READ_POW_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_power(2) ;
					Q_item.psu_address = 2 ;
					next_position_in_Q = 27 ;
					break ;
					
				case 27:
					Q_item.command_type = READ_POWER;
					Q_item.number_of_bytes = LOCAL_CMD_READ_POW_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_power(3) ;
					Q_item.psu_address = 3 ;
					next_position_in_Q = 28 ;
					break ;
					
				case 28:
					Q_item.command_type = READ_POWER;
					Q_item.number_of_bytes = LOCAL_CMD_READ_POW_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_power(4) ;
					Q_item.psu_address = 4 ;
					next_position_in_Q = 29 ;
					break ;
					
				case 29:
					Q_item.command_type = READ_POWER;
					Q_item.number_of_bytes = LOCAL_CMD_READ_POW_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_power(5) ;
					Q_item.psu_address = 5 ;
					next_position_in_Q = 30 ;
					break ;
					
				case 30:
					Q_item.command_type = READ_POWER;
					Q_item.number_of_bytes = LOCAL_CMD_READ_POW_BUFFER_SIZE ;
					Q_item.ptr_to_command = get_string_address_read_power(6) ;
					Q_item.psu_address = 6 ;
					next_position_in_Q = 7 ;//1 ;
					break ;
					
				default:
					next_position_in_Q = 7 ;//1 ;
					break ;
			}
			

			/* xQueueSendToBack -> ticks to wait
			 0, return immediately if the Q was full - bin current read try again next time
			    Proceed with resetting the seed timer
				
			 portMAX_DELAY, task waits indefinitely for a space to appear in the Q
			 
			 Or set a value inbetween.. */
			
			xStatus = xQueueSendToBack (gl_xReadQ, ptr_to_Q_item, 0) ;// wait_for_space) ; //unblocks vTask_SendCommandToKickerPSU_LocalReadQ
			
			if( xStatus == errQUEUE_FULL)
			{
				gl_ticks = xTaskGetTickCount() ;
				xStatus = xQueueReset(gl_xReadQ) ;
			}
				
		//this timer must be reset so block the task until there is processor time to complete the timer reset macro
		xTimerReset(xReadQSeedTimer, portMAX_DELAY) ; //wait_for_timer) ; //reset/restart the seed timer, prvSeedReadQ_Timer
		
	} //end of for ( ;; )
}
/*-----------------------------------------------------------*/

void vTask_SendCommandToLcd_ReadValsQ(void* pvParameters)
{
	//xLCDReadValsQ_pkt_t* ptr_xReadVals_Q_item ;
	xLCDReadValsQ_pkt_t xReadVals_Q_item ;
	
	xRxBuffer_info_t local_rx_buff ;
	
	//portBASE_TYPE xStatus ;
	
	//portTickType xTicksToWait = portMAX_DELAY ;// / portTICK_RATE_MS ;
	
	for( ;; )
	{		
		//receive item from Q
		
		/* First parameter is the Queue from which data is to be received.  The Q is created before the 
		scheduler is started, and therefore before this task runs for the first time.   
		
		Second parameter is the buffer into which received data will be placed.  In this case the buffer 
		is simply the address of a variable that has the required size to hold the received data.
		
		Last parameter is the block time - the MAXIMUM time that the task should remain in the blocked 
		state to wait for the data to be available should the queue already be empty.  */
		
		xQueueReceive( xLcdReadValsQ, &xReadVals_Q_item, portMAX_DELAY) ; //Item placed in Q by vTask_SendCommandToLcd_ReadValsQ_seed
		
		//Process receive
		
		//if (xStatus == pdPASS)
		//{
			//data successfully removed from the Q
		
			//Process item in the Read Q		
			
			//put this next line in the Q processing task
			lcd_display_readvals(xReadVals_Q_item.kicker_address, (uint8_t) xReadVals_Q_item.read_value, 
				xReadVals_Q_item.lcd_display_pos, xReadVals_Q_item.ptr_to_buffer,
					LCD_TEXT_RED, xReadVals_Q_item.lcd_text_type, (freertos_usart_if)pvParameters) ;//update lcd display
		//}
	} //end of for ( ;; )	
}
/*-----------------------------------------------------------*/



void vTask_SendCommandToLcd_ReadValsQ_seed(void* pvParameters)
{
	portBASE_TYPE xStatus ;
	//portBASE_TYPE result = pdFAIL ;
	
	int* ptr_readback_val ;
	int* ptr_prev_readback_val ;
	
	xLCDReadValsQ_pkt_t* ptr_xReadVals_Q_item ;
	xLCDReadValsQ_pkt_t xReadVals_Q_item ;
	uint8_t next_position_in_Q = 1 ;
	portTickType wait_for_space = (500 / portTICK_RATE_MS) ; //introduce a 500ms wait for space in the queue
	
	//storage for previous read values
	int outv1 = 0, outv2 = 0, outv3 = 0, outv4 = 0, outv5 = 0, outv6 = 0 ;
	int demv1 = 0, demv2 = 0, demv3 = 0, demv4 = 0, demv5 = 0, demv6 = 0 ;
	int amps1 = 0, amps2 = 0, amps3 = 0, amps4 = 0, amps5 = 0, amps6 = 0 ;
	int pow1 = 0, pow2 = 0, pow3 = 0, pow4 = 0, pow5 = 0, pow6 = 0 ;
	
	Bool b_status_changed = true ;
		
	for ( ;; )
	{		
		/* Anything that takes priority will interrupt this task but the place should be saved and 
		   processing will continue once the higher priority task completes.
		   
		   Only place items in the Q if there has been a change in the value.  So, check the value
		   first. */
		
		xSemaphoreTake(xLcdReadValsQ_Seed_BinarySemaphore, portMAX_DELAY) ; //given by prvLcdReadValsSeedQ_Timer
		
		//if( result == pdPASS) {			
			
			b_status_changed = false ;
			
			ptr_xReadVals_Q_item = &xReadVals_Q_item ;
			
			switch(next_position_in_Q)
			{
				/* Move through the stored local read vals
				   That is set the pointer to the address of the different
				   readback values in turn */
				
				case 1:
					ptr_readback_val = &gl_ul_outv1 ;
					ptr_prev_readback_val = &outv1 ;
					xReadVals_Q_item.kicker_address = 1 ;
					xReadVals_Q_item.lcd_text_type = LCD_TEXT_READ_V ;					
					next_position_in_Q = 2 ;
					break ;
					
				case 2:
					ptr_readback_val = &gl_ul_outv2 ;
					ptr_prev_readback_val = &outv2 ;
					xReadVals_Q_item.kicker_address = 2 ;
					xReadVals_Q_item.lcd_text_type = LCD_TEXT_READ_V ;
					next_position_in_Q = 3 ;
					break ;
					
				case 3:
					ptr_readback_val = &gl_ul_outv3 ;
					ptr_prev_readback_val = &outv3 ;
					xReadVals_Q_item.kicker_address = 3 ;
					xReadVals_Q_item.lcd_text_type = LCD_TEXT_READ_V ;
					next_position_in_Q = 4 ;
					break ;
					
				case 4:
					ptr_readback_val = &gl_ul_outv4 ;
					ptr_prev_readback_val = &outv4 ;
					xReadVals_Q_item.kicker_address = 4 ;
					xReadVals_Q_item.lcd_text_type = LCD_TEXT_READ_V ;
					next_position_in_Q = 5 ;
					break ;
					
				case 5:
					ptr_readback_val = &gl_ul_outv5 ;
					ptr_prev_readback_val = &outv5 ;
					xReadVals_Q_item.kicker_address = 5 ;
					xReadVals_Q_item.lcd_text_type = LCD_TEXT_READ_V ;
					next_position_in_Q = 6 ;
					break ;
					
				case 6:
					ptr_readback_val = &gl_ul_outv6 ;
					ptr_prev_readback_val = &outv6 ;
					xReadVals_Q_item.kicker_address = 6 ;
					xReadVals_Q_item.lcd_text_type = LCD_TEXT_READ_V ;
					next_position_in_Q = 7 ;
					break ;
					
				case 7:
					ptr_readback_val = &gl_ul_amps1 ;
					ptr_prev_readback_val = &amps1 ;
					xReadVals_Q_item.kicker_address = 1 ;
					xReadVals_Q_item.lcd_text_type = LCD_TEXT_READ_A ;
					next_position_in_Q = 8 ;
					break ;
					
				case 8:
					ptr_readback_val = &gl_ul_amps2 ;
					ptr_prev_readback_val = &amps2 ;
					xReadVals_Q_item.kicker_address = 2 ;
					xReadVals_Q_item.lcd_text_type = LCD_TEXT_READ_A ;
					next_position_in_Q = 9 ;
					break ;
					
				case 9:
					ptr_readback_val = &gl_ul_amps3 ;
					ptr_prev_readback_val = &amps3 ;
					xReadVals_Q_item.kicker_address = 3 ;
					xReadVals_Q_item.lcd_text_type = LCD_TEXT_READ_A ;
					next_position_in_Q = 10 ;
					break ;
					
				case 10:
					ptr_readback_val = &gl_ul_amps4 ;
					ptr_prev_readback_val = &amps4 ;
					xReadVals_Q_item.kicker_address = 4 ;
					xReadVals_Q_item.lcd_text_type = LCD_TEXT_READ_A ;
					next_position_in_Q = 11 ;
					break ;
					
				case 11:
					ptr_readback_val = &gl_ul_amps5 ;
					ptr_prev_readback_val = &amps5 ;
					xReadVals_Q_item.kicker_address = 5 ;
					xReadVals_Q_item.lcd_text_type = LCD_TEXT_READ_A ;
					next_position_in_Q = 12 ;
					break ;
					
				case 12:
					ptr_readback_val = &gl_ul_amps6 ;
					ptr_prev_readback_val = &amps6 ;
					xReadVals_Q_item.kicker_address = 6 ;
					xReadVals_Q_item.lcd_text_type = LCD_TEXT_READ_A ;
					next_position_in_Q = 13 ;
					break ;
					
				case 13:
					ptr_readback_val = &gl_ul_pow1 ;
					ptr_prev_readback_val = &pow1 ;
					xReadVals_Q_item.kicker_address = 1 ;
					xReadVals_Q_item.lcd_text_type = LCD_TEXT_READ_P ;
					next_position_in_Q = 14 ;
					break ;
					
				case 14:
					ptr_readback_val = &gl_ul_pow2 ;
					ptr_prev_readback_val = &pow2 ;
					xReadVals_Q_item.kicker_address = 2 ;
					xReadVals_Q_item.lcd_text_type = LCD_TEXT_READ_P ;
					next_position_in_Q = 15 ;
					break ;
					
				case 15:
					ptr_readback_val = &gl_ul_pow3 ;
					ptr_prev_readback_val = &pow3 ;
					xReadVals_Q_item.kicker_address = 3 ;
					xReadVals_Q_item.lcd_text_type = LCD_TEXT_READ_P ;
					next_position_in_Q = 16 ;
					break ;
					
				case 16:
					ptr_readback_val = &gl_ul_pow4 ;
					ptr_prev_readback_val = &pow4 ;
					xReadVals_Q_item.kicker_address = 4 ;
					xReadVals_Q_item.lcd_text_type = LCD_TEXT_READ_P ;
					next_position_in_Q = 17 ;
					break ;
					
				case 17:
					ptr_readback_val = &gl_ul_pow5 ;
					ptr_prev_readback_val = &pow5 ;
					xReadVals_Q_item.kicker_address = 5 ;
					xReadVals_Q_item.lcd_text_type = LCD_TEXT_READ_P ;
					next_position_in_Q = 18 ;
					break ;
					
				case 18:
					ptr_readback_val = &gl_ul_pow6 ;
					ptr_prev_readback_val = &pow6 ;
					xReadVals_Q_item.kicker_address = 6 ;
					xReadVals_Q_item.lcd_text_type = LCD_TEXT_READ_P ;
					next_position_in_Q = 1 ;
					break ;
				
				default:
					ptr_readback_val = NULL ;
					ptr_prev_readback_val = NULL ;
					next_position_in_Q = 1 ;
					break ;
			}
					
			//Providing the default case wasn't hit, process the readback value
			if(ptr_readback_val != NULL)
			{
				//first check for ANY change at all
				if(*ptr_readback_val != *ptr_prev_readback_val)
				{
					//Check which part(s) have changed
					
					//units??
					if( (*ptr_readback_val%10) != (*ptr_prev_readback_val%10) )	 
					{
						b_status_changed = true ;
						xReadVals_Q_item.read_value = *ptr_readback_val%10 ; //update lcd unit value
						xReadVals_Q_item.lcd_display_pos = disp_100 ;
						xReadVals_Q_item.ptr_to_buffer = &gl_usart0_unit ;
						//Things have changed, add to Q
						xStatus = xQueueSendToBack (xLcdReadValsQ, ptr_xReadVals_Q_item, wait_for_space) ; //unblocks vTask_SendCommandToLcd_ReadValsQ
					}
					
					//tens?? *ptr_read_val/10)%10
					if( ((*ptr_readback_val/10)%10) != ((*ptr_prev_readback_val/10)%10) )	 
					{
						b_status_changed = true ;
						xReadVals_Q_item.read_value = (*ptr_readback_val/10)%10 ; //update lcd unit value
						xReadVals_Q_item.lcd_display_pos = disp_102 ;
						xReadVals_Q_item.ptr_to_buffer = &gl_usart0_unit ; //doesn't matter which buffer is used as all necessary bytes get overwritten
						//Things have changed, add to Q
						xStatus = xQueueSendToBack (xLcdReadValsQ, ptr_xReadVals_Q_item, wait_for_space) ; //unblocks the LCD Read Val Q Task
					}
					
					//add in others here and check the interleaving operation etc
					
					//hunds??
					if(xReadVals_Q_item.lcd_text_type != LCD_TEXT_READ_P) //power doesn't have hunds
					{
						if( ((*ptr_readback_val/100)%10) != ((*ptr_prev_readback_val/100)%10) )	 
						{
							b_status_changed = true ;
							xReadVals_Q_item.read_value = (*ptr_readback_val/100)%10 ; //update lcd unit value
							xReadVals_Q_item.lcd_display_pos = disp_103 ;
							xReadVals_Q_item.ptr_to_buffer = &gl_usart0_unit ; //doesn't matter which buffer is used as all necessary bytes get overwritten
							//Things have changed, add to Q
							xStatus = xQueueSendToBack (xLcdReadValsQ, ptr_xReadVals_Q_item, wait_for_space) ; //unblocks the LCD Read Val Q Task
						}
					}
					
					//thous??
					if(xReadVals_Q_item.lcd_text_type == LCD_TEXT_READ_A) //only amps displays thous
					{
						if( ((*ptr_readback_val/1000)%10) != ((*ptr_prev_readback_val/1000)%10) )
						{
							b_status_changed = true ;
							xReadVals_Q_item.read_value = (*ptr_readback_val/1000)%10 ; //update lcd unit value
							xReadVals_Q_item.lcd_display_pos = disp_104 ;
							xReadVals_Q_item.ptr_to_buffer = &gl_usart0_unit ; //doesn't matter which buffer is used as all necessary bytes get overwritten
							//Things have changed, add to Q
							xStatus = xQueueSendToBack (xLcdReadValsQ, ptr_xReadVals_Q_item, wait_for_space) ; //unblocks the LCD Read Val Q Task
						}
					}
					//finished updating units, tens, etc - set the previous readback value to the new value					
					*ptr_prev_readback_val = *ptr_readback_val ;
				}
			}
		//}
		
		xTimerReset(xLcdReadValsQSeedTimer, 0) ; //reset/restart the seed timer, prvLcdReadValsSeedQ_Timer
		//result = xSemaphoreTake(xLcdReadValsQ_Seed_BinarySemaphore, portMAX_DELAY) ; //given by prvLcdReadValsSeedQ_Timer
		
	} //end of for ( ;; )
}
/*-----------------------------------------------------------*/

void vTask_ProcessTouchEvent(void* pvParameters)
{
	uint8_t* pxParams ;
	uint16_t uw_x, uw_y = 0 ;
	//uint8_t us_touchevent = 0 ;
	uint8_t us_status ;
	//uint8_t us_psu_healthy ;
	uint8_t us_button_type = 0 ;
	int8_t psu_healthy1_active = pdFALSE ;
	int8_t psu_healthy2_active = pdFALSE ;
	int8_t psu_healthy3_active = pdFALSE ;
	//portBASE_TYPE result = pdFAIL ;
	
	uint8_t previous_button_address = 0 ;
	
	Bool b_wait_for_release = pdFALSE ;
	
	for( ;; )
	{
		xSemaphoreTake(xProcessLcdTouchEvent_BinarySemaphore, portMAX_DELAY) ; //given by vTask_ReadCommandFromLcd_Chesney
		
		if(gl_backlight_state == LCD_OFF)
		{
			xSemaphoreGive(xLcdBacklightOn_BinarySemaphore) ; //unblock vTask_TurnBacklightOn
			xTimerReset(xLcdTouchEventTimer, portMAX_DELAY) ; //reset touch event timer to capture next screen press
		}
		else
		{
			xTimerReset(xBacklightOffTimer, 0) ; //reset the backlight timer on each touch event
			pxParams = (uint8_t*) pvParameters ; //set pointer to start of gl_receive_buffer
		
			uw_x = ( *(pxParams + 5) << 8) | *(pxParams + 6) ; //get co-ords
			uw_y = ( *(pxParams + 7) << 8) | *(pxParams + 8) ;
		
			psu_healthy1_active = kicker_get_psuhealthy_status(1) ; //get state of PSU HEALTHY signals
			psu_healthy2_active = kicker_get_psuhealthy_status(3) ;
			psu_healthy3_active = kicker_get_psuhealthy_status(5) ;

	//------- Local / Remote buttons ---------------------------------------------------------------------------------------
				
			if((uw_x>tch_loc1x_min && uw_x<tch_loc1x_max) && (uw_y>tch_loc_posy_min && uw_y<tch_loc_posy_max)){ //k1 local button
				set_kicker_address_locrem(1) ;			
				us_button_type = BUTTON_LOCREM ;
			}		
			else if((uw_x>tch_loc1x_min && uw_x<tch_loc1x_max) && (uw_y>tch_loc_negy_min && uw_y<tch_loc_negy_max)){//k2 local button
				set_kicker_address_locrem(2) ;
				us_button_type = BUTTON_LOCREM ;
			}		
			else if((uw_x>tch_loc2x_min && uw_x<tch_loc2x_max) && (uw_y>tch_loc_posy_min && uw_y<tch_loc_posy_max)){//k3 local button
				set_kicker_address_locrem(3) ;
				us_button_type = BUTTON_LOCREM ;
			}
			else if((uw_x>tch_loc2x_min && uw_x<tch_loc2x_max) && (uw_y>tch_loc_negy_min && uw_y<tch_loc_negy_max)){//k4 local button
				set_kicker_address_locrem(4) ;
				us_button_type = BUTTON_LOCREM ;
			}		
			else if((uw_x>tch_loc3x_min && uw_x<tch_loc3x_max) && (uw_y>tch_loc_posy_min && uw_y<tch_loc_posy_max)){//k5 local button
				set_kicker_address_locrem(5) ;
				us_button_type = BUTTON_LOCREM ;
			}		
			else if((uw_x>tch_loc3x_min && uw_x<tch_loc3x_max) && (uw_y>tch_loc_negy_min && uw_y<tch_loc_negy_max)){//k6 local button
				set_kicker_address_locrem(6) ;
				us_button_type = BUTTON_LOCREM ;
			}
				
	//------- Load buttons -------------------------------------------------------------------------------------------------
				
			//If the Pop-up timer is running then a previous load is still being serviced, exit and wait for another press

			else if((uw_x>tch_ld1x_min && uw_x<tch_ld1x_max) && (uw_y>tch_ld_posy_min && uw_y<tch_ld_posy_max)){ //k1 load button
				if(!xTimerIsTimerActive(xLcdLoadButtonPopupTimer))
				{
					set_kicker_address_load(1) ;
					if((kicker_get_locrem_status(get_kicker_address_load())== LOCAL) && (psu_healthy1_active == PSU_HEALTHY_ACTIVE))
					{ //not in remote and psu healthy is good - process the input
						us_button_type = BUTTON_LOAD ; //set button type that unblocks the task to draw load button
					}
					else
					{
						us_button_type = NULL ; //no button to update
					}
				}
				else
				{
					us_button_type = NULL ; //no button to update
				}
			}
		
			else if((uw_x>tch_ld1x_min && uw_x<tch_ld1x_max) && (uw_y>tch_ld_negy_min && uw_y<tch_ld_negy_max)){ //k2 load button
				if(!xTimerIsTimerActive(xLcdLoadButtonPopupTimer))
				{
					set_kicker_address_load(2) ;
					if((kicker_get_locrem_status(get_kicker_address_load()) == LOCAL) && (psu_healthy1_active == PSU_HEALTHY_ACTIVE))
					{ //not in remote and psu healthy is good - process the input
						us_button_type = BUTTON_LOAD ; //set button type that unblocks the task to draw load button
					}
					else
					{
						us_button_type = NULL ; //no button to update
					}
				}
				else
				{
					us_button_type = NULL ; //no button to update
				}
			}
		
			else if((uw_x>tch_ld2x_min && uw_x<tch_ld2x_max) && (uw_y>tch_ld_posy_min && uw_y<tch_ld_posy_max)){ //k3 load button
				if(!xTimerIsTimerActive(xLcdLoadButtonPopupTimer))
				{
					set_kicker_address_load(3) ;
					if((kicker_get_locrem_status(get_kicker_address_load()) == LOCAL) && (psu_healthy2_active == PSU_HEALTHY_ACTIVE))
					{ //not in remote and psu healthy is good - process the input
						us_button_type = BUTTON_LOAD ; //set button type that unblocks the task to draw load button
					}
					else
					{
						us_button_type = NULL ; //no button to update
					}
				}
				else
				{
					us_button_type = NULL ; //no button to update
				}
			}
		
			else if((uw_x>tch_ld2x_min && uw_x<tch_ld2x_max) && (uw_y>tch_ld_negy_min && uw_y<tch_ld_negy_max)){ //k4 load button
				if(!xTimerIsTimerActive(xLcdLoadButtonPopupTimer))
				{
					set_kicker_address_load(4) ;
					if((kicker_get_locrem_status(get_kicker_address_load()) == LOCAL) && (psu_healthy2_active == PSU_HEALTHY_ACTIVE))
					{ //not in remote and psu healthy is good - process the input
						us_button_type = BUTTON_LOAD ; //set button type that unblocks the task to draw load button
					}
					else
					{
						us_button_type = NULL ; //no button to update
					}
				}
				else
				{
					us_button_type = NULL ; //no button to update
				}
			}
		
			else if((uw_x>tch_ld3x_min && uw_x<tch_ld3x_max) && (uw_y>tch_ld_posy_min && uw_y<tch_ld_posy_max)){ //k5 load button
				if(!xTimerIsTimerActive(xLcdLoadButtonPopupTimer))
				{
					set_kicker_address_load(5) ;
					if((kicker_get_locrem_status(get_kicker_address_load()) == LOCAL) && (psu_healthy3_active == PSU_HEALTHY_ACTIVE))
					{ //not in remote and psu healthy is good - process the input
						us_button_type = BUTTON_LOAD ; //set button type that unblocks the task to draw load button
					}
					else
					{
						us_button_type = NULL ; //no button to update
					}
				}
				else
				{
					us_button_type = NULL ; //no button to update
				}
			}
		
			else if((uw_x>tch_ld3x_min && uw_x<tch_ld3x_max) && (uw_y>tch_ld_negy_min && uw_y<tch_ld_negy_max)){ //k6 load button
				if(!xTimerIsTimerActive(xLcdLoadButtonPopupTimer))
				{
					set_kicker_address_load(6) ;
					if((kicker_get_locrem_status(get_kicker_address_load()) == LOCAL) && (psu_healthy3_active == PSU_HEALTHY_ACTIVE))
					{ //not in remote and psu healthy is good - process the input
						us_button_type = BUTTON_LOAD ; //set button type that unblocks the task to draw load button
					}
					else
					{
						us_button_type = NULL ; //no button to update
					}
				}
				else
				{
					us_button_type = NULL ; //no button to update
				}
			}
				
	//------- On/Off buttons -------------------------------------------------------------------------------------------------

			else if((uw_x>tch_on1x_min && uw_x<tch_on1x_max) && (uw_y>tch_on_posy_min && uw_y<tch_on_posy_max)){ //k1 on/off button
				set_kicker_address_onoff(1) ;
				if((kicker_get_locrem_status(get_kicker_address_onoff()) == LOCAL)&&(psu_healthy1_active == PSU_HEALTHY_ACTIVE))
				{ //not in remote and psu healthy is good - process the input				
					us_button_type = BUTTON_ONOFF ; //set button type that unblocks the task to draw ON/OFF button
				}
				else
				{
					us_button_type = NULL ; //no button to update
				}
			}
		
			else if((uw_x>tch_on1x_min && uw_x<tch_on1x_max) && (uw_y>tch_on_negy_min && uw_y<tch_on_negy_max)){ //k2 on/off button
				set_kicker_address_onoff(2) ;
				if((kicker_get_locrem_status(get_kicker_address_onoff()) == LOCAL)&&(psu_healthy1_active == PSU_HEALTHY_ACTIVE))
				{ //not in remote and psu healthy is good - process the input
					us_button_type = BUTTON_ONOFF ; //set button type that unblocks the task to draw ON/OFF button
				}
				else
				{
					us_button_type = NULL ; //no button to update
				}
			}
		
			else if((uw_x>tch_on2x_min && uw_x<tch_on2x_max) && (uw_y>tch_on_posy_min && uw_y<tch_on_posy_max)){ //k3 on/off button
				set_kicker_address_onoff(3) ;
				if((kicker_get_locrem_status(get_kicker_address_onoff()) == LOCAL)&&(psu_healthy2_active == PSU_HEALTHY_ACTIVE))
				{ //not in remote and psu healthy is good - process the input
					us_button_type = BUTTON_ONOFF ; //set button type that unblocks the task to draw ON/OFF button
				}
				else
				{
					us_button_type = NULL ; //no button to update
				}
			}
		
			else if((uw_x>tch_on2x_min && uw_x<tch_on2x_max) && (uw_y>tch_on_negy_min && uw_y<tch_on_negy_max)){ //k4 on/off button
				set_kicker_address_onoff(4) ;
				if((kicker_get_locrem_status(get_kicker_address_onoff()) == LOCAL)&&(psu_healthy2_active == PSU_HEALTHY_ACTIVE))
				{ //not in remote and psu healthy is good - process the input
					us_button_type = BUTTON_ONOFF ; //set button type that unblocks the task to draw ON/OFF button
				}
				else
				{
					us_button_type = NULL ; //no button to update
				}
			}
		
			else if((uw_x>tch_on3x_min && uw_x<tch_on3x_max) && (uw_y>tch_on_posy_min && uw_y<tch_on_posy_max)){ //k5 on/off button
				set_kicker_address_onoff(5) ;
				if((kicker_get_locrem_status(get_kicker_address_onoff()) == LOCAL)&&(psu_healthy3_active == PSU_HEALTHY_ACTIVE))
				{ //not in remote and psu healthy is good - process the input
					us_button_type = BUTTON_ONOFF ; //set button type that unblocks the task to draw ON/OFF button
				}
				else
				{
					us_button_type = NULL ; //no button to update
				}
			}
		
			else if((uw_x>tch_on3x_min && uw_x<tch_on3x_max) && (uw_y>tch_on_negy_min && uw_y<tch_on_negy_max)){ //k6 on/off button
				set_kicker_address_onoff(6) ;
				if((kicker_get_locrem_status(get_kicker_address_onoff()) == LOCAL)&&(psu_healthy3_active == PSU_HEALTHY_ACTIVE))
				{ //not in remote and psu healthy is good - process the input
					us_button_type = BUTTON_ONOFF ; //set button type that unblocks the task to draw ON/OFF button
				}
				else
				{
					us_button_type = NULL ; //no button to update
				}
			}		
		
	//------- Reset buttons -------------------------------------------------------------------------------------------------

			else if((uw_x>tch_rst1x_min&& uw_x<tch_rst1x_max) && (uw_y>tch_rst_y_min && uw_y<tch_rst_y_max)){ //k1 pair reset button
				if(!xTimerIsTimerActive(xLcdResetButtonTimer)) //make sure the reset command isn't active
				{
					//Cannot do RESET if either K1+ OR K1- is in remote
					us_status = kicker_get_locrem_status(1) ; //get Local/Remote status for K1 positive
					us_status += kicker_get_locrem_status(2) ; //get Local/Remote status for K1 negative
					set_kicker_address_reset(1) ;
					if((us_status == LOCAL)&&(psu_healthy1_active))
					{ //not in remote and psu healthy is good - process the input				
						us_button_type = BUTTON_RESET ; //set button type that unblocks the task to draw ON/OFF button
					}
					else
					{
						us_button_type = NULL ; //no button to update
					}
				}
				else
				{
					us_button_type = NULL ; //no button to update
				}
			}
		
			else if((uw_x>tch_rst2x_min&& uw_x<tch_rst2x_max) && (uw_y>tch_rst_y_min && uw_y<tch_rst_y_max)){ //k2 pair reset button
				if(!xTimerIsTimerActive(xLcdResetButtonTimer)) //make sure the reset command isn't active
				{
					//Cannot do RESET if either K2+ OR K2- is in remote
					us_status = kicker_get_locrem_status(3) ; //get Local/Remote status for K1 positive
					us_status += kicker_get_locrem_status(4) ; //get Local/Remote status for K1 negative
					set_kicker_address_reset(3) ;
					if((us_status == LOCAL)&&(psu_healthy2_active))
					{ //not in remote and psu healthy is good - process the input
						us_button_type = BUTTON_RESET ; //set button type that unblocks the task to draw ON/OFF button
					}
					else
					{
						us_button_type = NULL ; //no button to update
					}
				}
				else
				{
					us_button_type = NULL ; //no button to update
				}
			}
		
			else if((uw_x>tch_rst3x_min&& uw_x<tch_rst3x_max) && (uw_y>tch_rst_y_min && uw_y<tch_rst_y_max)){ //k3 pair reset button
				if(!xTimerIsTimerActive(xLcdResetButtonTimer)) //make sure the reset command isn't active
				{
					//Cannot do RESET if either K3+ OR K3- is in remote
					us_status = kicker_get_locrem_status(5) ; //get Local/Remote status for K1 positive
					us_status += kicker_get_locrem_status(6) ; //get Local/Remote status for K1 negative
					set_kicker_address_reset(5) ;
					if((us_status == LOCAL)&&(psu_healthy3_active))
					{ //not in remote and psu healthy is good - process the input
						us_button_type = BUTTON_RESET ; //set button type that unblocks the task to draw ON/OFF button
					}
					else
					{
						us_button_type = NULL ; //no button to update
					}
				}
				else
				{
					us_button_type = NULL ; //no button to update
				}
			}
		
			else{
				us_button_type = NULL ; //the touch didn't hit one of the LCD buttons - no button to update!
			}
			
								
	//------- Print buttons on the LCD --------------------------------------------------------------------------------------					
		
			switch(us_button_type)
			{
				case BUTTON_LOCREM:
				if( (BUTTON_LOCREM != gl_previous_button_type) || (get_kicker_address_locrem() != previous_button_address) )
				{
					kicker_set_locrem_status(get_kicker_address_locrem()) ; //toggle local/remote status - will need to do similar for other buttons
					xSemaphoreGive(xLcdCommandLocrem_CountingSemaphore) ; //unblock vTask_SendCommandToLcd_setLocalRemote
					previous_button_address = get_kicker_address_locrem() ;
				}			
				break ;
			
				case BUTTON_LOAD:
				if( (BUTTON_LOAD != gl_previous_button_type) || (get_kicker_address_load() != previous_button_address) )
				{
					//button actions need to be moved to within this loop
					//this will stop repeated actions being sent
					set_load(get_kicker_address_load()) ; //sets the global variable, gl_us_load
					set_wyclef_status(WYCLEF_TX_KV) ; //flag transmit kV command
					xSemaphoreGive(xPsuCommand_CountingSemaphore) ; //unblock vTask_SendCommandToKickerPSU_Wyclef
					xSemaphoreGive(xLcdCommandLoad_CountingSemaphore) ; //unblock vTask_SendCommandToLcd_setLoad
					previous_button_address = get_kicker_address_load() ;				
				}			
				break ;
			
				case BUTTON_ONOFF:
				if( (BUTTON_ONOFF != gl_previous_button_type) || (get_kicker_address_onoff() != previous_button_address) )
				{
					if(kicker_get_hvon_status(get_kicker_address_onoff()) == HVON_ACTIVE){
						set_wyclef_status(WYCLEF_TX_OFF) ; //if HVON_ACTIVE send OFF command
					}
					else if(kicker_get_hvon_status(get_kicker_address_onoff()) == HVON_INACTIVE){
						set_wyclef_status(WYCLEF_TX_ON) ; //if HVON_INACTIVE send ON command
					}
					xSemaphoreGive(xPsuCommand_CountingSemaphore) ; //unblock vTask_SendCommandToKickerPSU_Wyclef
					gl_b_isittouchevent_ONOFF = pdTRUE ; //flag that the semaphore comes from a touch event
					xSemaphoreGive(xLcdCommandOnOff_BinarySemaphore) ; //unblock vTask_SendCommandToLcd_setOnOff
					previous_button_address = get_kicker_address_onoff() ;
				}			
				break ;
			
				case BUTTON_RESET:
				if( (BUTTON_RESET != gl_previous_button_type) || (get_kicker_address_reset() != previous_button_address) )
				{
					set_wyclef_status(WYCLEF_TX_RESET) ;				
					xSemaphoreGive(xPsuCommand_CountingSemaphore) ; //unblock vTask_SendCommandToKickerPSU_Wyclef
					gl_b_isittouchevent_RESET = pdTRUE ; //flag that the semaphore comes from a touch event
					xSemaphoreGive(xLcdCommandReset_BinarySemaphore) ; //unblock vTask_SendCommandToLcd_setReset
					previous_button_address = get_kicker_address_reset() ; //restore value to reset address
				}			
				break ;
			
				default: //don't unblock any task
					xTimerReset(xLcdTouchEventTimer, 0) ;
				break ;
			}
		
			//Reset timer here, remove from the Task_SendCommandTo_Wherever tasks
			xTimerReset(xLcdTouchEventTimer, portMAX_DELAY) ;
		
			gl_previous_button_type = us_button_type ; //save previous button attributes
				
			//Previous button info has been captured, this prevents the same button being actioned again
			//Now start a timer to clear the previous button info after a defined period
			//Occasionally the user will want to press the same button; to re-send a command or whatever
			//This timer will release the block and allow this to happen
		
			//Choose something sensible like 1s to begin with for this timer
			//start a timer to allow access to the screen if a release event is missed
			xTimerStart(xLcdClearPrevButtonInfoTimer, portMAX_DELAY) ;
		}
		
	}//end of for ( ;; )
}
/*-----------------------------------------------------------*/

void vTask_ReadCommandFromLcd_Chesney(void* pvParameters)
{
	//declare any variables needed
	uint8_t lcd_receive_buffer[LCD_TOUCHEVENT_BUFFER_SIZE] = {'\0'} ;
	uint32_t bytes_received ;
	portTickType max_wait_2ms = 2/portTICK_RATE_MS ;
	freertos_usart_if usart0 ;
	status_code_t xStatus ;	
	
	//define circular_buffer
	//uint8_t cb_lcd_rx[CIRC_BUFFER_MAX][LCD_TOUCHEVENT_BUFFER_SIZE] = {'\0'} ;
	uint8_t us_bytes_received = 0 ;
	
	//int cb_start_address = &cb_lcd_rx ;
	//int cb_cell_size = LCD_TOUCHEVENT_BUFFER_SIZE ;
	//int cb_end_address = cb_start_address + ((CIRC_BUFFER_MAX - 1) * cb_cell_size) ;
	//uint8_t* ptr_lcd_rx_cb = cb_start_address ; //pointer to a place in the circular receive buffer
	
	//uint8_t us_index = 0 ;
	
	//create another task with access to the receive buffer
	xTaskCreate(vTask_ProcessTouchEvent, "ProcTch", configMINIMAL_STACK_SIZE+200,
		(void *) &lcd_receive_buffer, 2, NULL) ;
		
	usart0 = (freertos_usart_if) pvParameters ; //get task params
	
	for ( ;; )
	{
		xSemaphoreTake(xReadLcdTouchEvent_BinarySemaphore, portMAX_DELAY) ; //given by prvLcdTouchEventTimerCallback
		
		if(usart0_read_string(usart0, &lcd_receive_buffer, LCD_TOUCHEVENT_BUFFER_SIZE) > 0)
		{
			if((lcd_receive_buffer[0] == 0xF2) &&
				(lcd_receive_buffer[1] == 0x09) &&
				(lcd_receive_buffer[2] == 0xFB) &&
				(lcd_receive_buffer[3] == 0x71) &&
				(lcd_receive_buffer[10] == 0xF8))
			{
				//so far so good, but is it touch or release
				if(lcd_receive_buffer[4] == 0x01) // '01' for Touch PRESS, '00' for touch RELEASE
				{
					xTimerStop(xLcdTouchEventTimer, portMAX_DELAY) ; //disable touch screen while processing a touch event
					xSemaphoreGive(xProcessLcdTouchEvent_BinarySemaphore) ;	//unblocks vTask_ProcessTouchEvent
				}
				else{//if backlight was just turned on then flag the touch release
					xTimerReset(xLcdTouchEventTimer, portMAX_DELAY) ;
				}				
			}
			else{
				xTimerReset(xLcdTouchEventTimer, portMAX_DELAY) ;
			}
		}
		else{
			xTimerReset(xLcdTouchEventTimer, portMAX_DELAY) ;
		}	
	}//end of for ( ;; )
}
/*-----------------------------------------------------------*/

void vTask_CPSReadScan(void* pvParameters)
{
	//declare any variables needed
	char cps_receive_buffer[CPS_RX_BUFFER_SIZE] = {'\0'} ;
	
	char cps_rx_buff[CPS_RX_BUFFER_SIZE] = {'\0'} ;
		
	uint8_t cps_rxbuff_select = 0 ;
	
	uint8_t psu_address ;
	uint8_t command_type ;
	xRxBuffer_info_t receive_buffer_info ;
		
	uint32_t bytes_received ;
	portTickType max_wait_1ms = 1 / portTICK_RATE_MS ;
	portBASE_TYPE result = pdFAIL ;
	
	Bool bWrite_command = pdFALSE ;
	int8_t psuhealthy_status = PSU_HEALTHY_INACTIVE ;
	int8_t reset_locrem_status = LOCAL ;
	
	//char uc_Tx_kV_remote[LCD_TXV_BUFFER_SIZE] = "xHV=00.00\r\n" ; //Storage for kV tx
		
	freertos_usart_if usart1 = (freertos_usart_if)pvParameters ;
	
	/* The receive buffer has to manage strings of different sizes.  There are several read/set commands sent by the CPS system.
	   
	   1. Receive the data (message from CPS crate)
	   
	   2. Check that number of received bytes hasn't overrun the receive buffer
	   
	   3. Process the CPS message :-
	   
			- Retrieve the required status/read val and transmit via USART1
			
				OR
			
			- Send the appropriate command via USART2
			
	*/
	
	
	//create another task with access to the receive buffer
	//xTaskCreate(vTask_ProcessTouchEvent, "ProcTch", configMINIMAL_STACK_SIZE+200,
		//(void *) &lcd_receive_buffer, 2, NULL) ;
		
	char* ptr_cps_rx_buff = cps_rx_buff ;//setup pointer to rx buffer
	
	for ( ;; )
	{
		xSemaphoreTake(xCPSRead_BinarySemaphore, portMAX_DELAY) ; //given by prvCPSReadScanTimer
			bytes_received = freertos_usart_serial_read_packet((freertos_usart_if)pvParameters, &cps_receive_buffer, CPS_RX_BUFFER_SIZE, max_wait_1ms) ;
			
			bWrite_command = pdFALSE ;
			
		if( (bytes_received > 0) && (bytes_received <= CPS_RX_BUFFER_SIZE) ) //check that received bytes hasn't overrun the buffer
		{
			//read the CPS command
			//if the command is address-less then don't bother with the address range check
			if(cps_receive_buffer[0] == 'L')
			{
				command_type = READ_LOCREM_STATUS ;
				receive_buffer_info = get_receive_buffer_info(psu_address, command_type) ; //returns address of string to Tx to CPS crate
				//send the response to the CPS crate
				usart1_write_string_standard(usart1, receive_buffer_info.ptr_to_buffer, receive_buffer_info.buffer_size) ;				
			}
			else if(cps_receive_buffer[0] == 0x1B)
			{
				//CRASH OFF
				if( (gl_us_loccrem == 0x3F) && (gl_us_psuhealthy == 0x15) )
				{
					set_wyclef_status(WYCLEF_TX_CRASHOFF) ;
					xSemaphoreGive(xPsuCommand_CountingSemaphore) ; //unblock vTask_SendCommandToKickerPSU_Wyclef
				}
			}
			else if( (cps_receive_buffer[0] > '0') && (cps_receive_buffer[0] < '7') ) //address must be in range 1-6
			{
				psu_address = cps_receive_buffer[0] ;
			
				psu_address -= ASCII_OFFSET ;
						
				//what is the command type??
				switch(cps_receive_buffer[1])
				{
					case 'H':
						if(cps_receive_buffer[3] == 'D')
						{
							command_type = READ_DEMV ;
						}
						else if(cps_receive_buffer[3] == '=') //on or off
						{
							if(cps_receive_buffer[5] == 'N') //Set ON
							{
								bWrite_command = pdTRUE ;
								command_type = SET_HVON ;
							}
							else if(cps_receive_buffer[5] == 'F')//Set OFF
							{
								bWrite_command = pdTRUE ;
								command_type = SET_HVOFF ;
							}
							else if(cps_receive_buffer[6] == '.')//Set kV
							{
								//copy received command into local storage buffer
								for(uint8_t i=0; i<LCD_TXV_BUFFER_SIZE; i++)
								{
									gl_uc_Tx_kV_remote[i] = cps_receive_buffer[i] ;
								}
								bWrite_command = pdTRUE ;
								command_type = SET_HVLOAD ;
							}
							else
								command_type = NULL ;									
						}
						else
							command_type = READ_OUTV ;
						break ;
						
					case 'M':
						command_type = READ_AMPS ;
						break ;
								
					case 'K':
						command_type = READ_POWER ;
						break ;
						
					case 'R':
						bWrite_command = pdTRUE ;
						command_type = SET_HVRESET ;
						break ;
								
					default:
						break ;
				}
				//receive_buffer_info = get_receive_buffer_info(psu_address, command_type) ;
						
				//transmit the reply to CPS
						
					//might need to do this, maybe not though
					//depends on how quick the message processing can be completed
					//empty the USART receive buffer
					//flush_usart_buffer( (freertos_usart_if)pvParameters, gl_bufferTrash, 1) ;
				if(!bWrite_command){
					receive_buffer_info = get_receive_buffer_info(psu_address, command_type) ; //returns address of string to Tx to CPS crate
					//send the response to the CPS crate
					usart1_write_string_standard(usart1, receive_buffer_info.ptr_to_buffer, receive_buffer_info.buffer_size) ;
				}
				else{
					psuhealthy_status = kicker_get_psuhealthy_status(psu_address) ;
					switch(command_type)
					{
						case SET_HVON:
							set_kicker_address_onoff(psu_address) ;
							if((kicker_get_locrem_status(get_kicker_address_onoff()) == REMOTE)&&(psuhealthy_status == PSU_HEALTHY_ACTIVE))
							{
								set_wyclef_status(WYCLEF_TX_ON) ; 
								xSemaphoreGive(xPsuCommand_CountingSemaphore) ; //unblock vTask_SendCommandToKickerPSU_Wyclef
							}
							else
								break ;
							break ;
								
						case SET_HVOFF:
							set_kicker_address_onoff(psu_address) ;
							if((kicker_get_locrem_status(get_kicker_address_onoff()) == REMOTE)&&(psuhealthy_status == PSU_HEALTHY_ACTIVE))
							{
								set_wyclef_status(WYCLEF_TX_OFF) ;
								xSemaphoreGive(xPsuCommand_CountingSemaphore) ; //unblock vTask_SendCommandToKickerPSU_Wyclef
							}
							else
								break ;
							break ;
						
						case SET_HVRESET:
							set_kicker_address_reset(psu_address) ;
							reset_locrem_status = kicker_get_locrem_status(get_kicker_address_reset()) ;
							reset_locrem_status += kicker_get_locrem_status(get_kicker_address_reset()+1) ;												
							if((reset_locrem_status == (REMOTE + REMOTE))&&(psuhealthy_status == PSU_HEALTHY_ACTIVE))
							{
								set_wyclef_status(WYCLEF_TX_RESET) ;
								xSemaphoreGive(xPsuCommand_CountingSemaphore) ; //unblock vTask_SendCommandToKickerPSU_Wyclef
							}
							else
								break ;
							break ;
						
						case SET_HVLOAD:
							set_kicker_address_load(psu_address) ;
							if((kicker_get_locrem_status(get_kicker_address_load()) == REMOTE) && (psuhealthy_status == PSU_HEALTHY_ACTIVE))
							{ 
								set_wyclef_status(WYCLEF_TX_KVREMOTE) ;
								xSemaphoreGive(xPsuCommand_CountingSemaphore) ; //unblock vTask_SendCommandToKickerPSU_Wyclef
							}
							else
								break ;
							break ;
					}
				}				
			}					
		}
	}//end of for ( ;; )
}
/*-----------------------------------------------------------*/

void vTask_LCDButtonInit(void* pvParameters)
{
	//Draw initial state of buttons on LCD screen
	
	uint8_t button_count = 0 ; 
	
	for( ;; ) //task processing in infinite loop
	{	
		switch(button_count)
		{
			case 0:
				if (gpio_pin_is_high(PSU_HEALTHY1_IDX))
				{
					gl_b_isittouchevent_RESET = pdFALSE ;
					set_kicker_address_reset(1) ;
					kicker_set_psuhealthy_status(1, PSU_HEALTHY_INACTIVE) ;
					//gpio_set_pin_high(LED1_GPIO) ;
					xSemaphoreGive(xLcdCommandReset_BinarySemaphore) ; //unblock vTask_SendCommandToLcd_setReset
				}
				else{
					kicker_set_psuhealthy_status(1, PSU_HEALTHY_ACTIVE) ;
				}		
				button_count += 1 ; //flag that a button has been initialised
				break ;
			
			case 1:
				if (gpio_pin_is_high(PSU_HEALTHY2_IDX))
				{
					gl_b_isittouchevent_RESET = pdFALSE ;
					set_kicker_address_reset(3) ;
					kicker_set_psuhealthy_status(3, PSU_HEALTHY_INACTIVE) ;
					//gpio_set_pin_high(LED1_GPIO) ;
					xSemaphoreGive(xLcdCommandReset_BinarySemaphore) ; //unblock vTask_SendCommandToLcd_setReset
				}
				else{
					kicker_set_psuhealthy_status(3, PSU_HEALTHY_ACTIVE) ;
				}
				button_count += 1 ; //flag that a button has been initialised
				break ;
				
			case 2:
				if (gpio_pin_is_high(PSU_HEALTHY3_IDX))
				{
					gl_b_isittouchevent_RESET = pdFALSE ;
					set_kicker_address_reset(5) ;
					kicker_set_psuhealthy_status(5, PSU_HEALTHY_INACTIVE) ;
					//gpio_set_pin_high(LED1_GPIO) ;
					xSemaphoreGive(xLcdCommandReset_BinarySemaphore) ; //unblock vTask_SendCommandToLcd_setReset
				}
				else{
					kicker_set_psuhealthy_status(5, PSU_HEALTHY_ACTIVE) ;
				}
				button_count += 1 ; //flag that a button has been initialised
				break ;
				
			case 3:
				if (gpio_pin_is_low(HVON1_IDX)){ //pin is low when HVON signal is high
					gl_b_isittouchevent_ONOFF = pdFALSE ; //flag as a HVON check
					set_kicker_address_onoff(1) ;
					kicker_set_hvon_status(1, HVON_ACTIVE) ;		
					xSemaphoreGive(xLcdCommandOnOff_BinarySemaphore) ; //unblock vTask_SendCommandToLcd_setOnOff
				}
				else{
					kicker_set_hvon_status(1, HVON_INACTIVE) ;
				}
				button_count += 1 ; //flag that a button has been initialised
				break ;
				
			case 4:
				if (gpio_pin_is_low(HVON1_IDX)){ //pin is low when HVON signal is high
					gl_b_isittouchevent_ONOFF = pdFALSE ; //flag as a HVON check
					set_kicker_address_onoff(2) ;
					kicker_set_hvon_status(2, HVON_ACTIVE) ;
					xSemaphoreGive(xLcdCommandOnOff_BinarySemaphore) ; //unblock vTask_SendCommandToLcd_setOnOff
				}
				else{
					kicker_set_hvon_status(2, HVON_INACTIVE) ;
				}
				button_count += 1 ; //flag that a button has been initialised
				break ;
				
			case 5:
				if (gpio_pin_is_low(HVON3_IDX)){ //pin is low when HVON signal is high
					gl_b_isittouchevent_ONOFF = pdFALSE ; //flag as a HVON check
					set_kicker_address_onoff(3) ;
					kicker_set_hvon_status(3, HVON_ACTIVE) ;
					xSemaphoreGive(xLcdCommandOnOff_BinarySemaphore) ; //unblock vTask_SendCommandToLcd_setOnOff
				}
				else{
					kicker_set_hvon_status(3, HVON_INACTIVE) ;
				}
				button_count += 1 ; //flag that a button has been initialised
				break ;
				
			case 6:	
				if (gpio_pin_is_low(HVON4_IDX)){ //pin is low when HVON signal is high
					gl_b_isittouchevent_ONOFF = pdFALSE ; //flag as a HVON check
					set_kicker_address_onoff(4) ;
					kicker_set_hvon_status(4, HVON_ACTIVE) ;
					xSemaphoreGive(xLcdCommandOnOff_BinarySemaphore) ; //unblock vTask_SendCommandToLcd_setOnOff
				}
				else{
					kicker_set_hvon_status(4, HVON_INACTIVE) ;
				}
				button_count += 1 ; //flag that a button has been initialised
				break ;
			
			case 7:	
				if (gpio_pin_is_low(HVON5_IDX)){ //pin is low when HVON signal is high
					gl_b_isittouchevent_ONOFF = pdFALSE ; //flag as a HVON check
					set_kicker_address_onoff(5) ;
					kicker_set_hvon_status(5, HVON_ACTIVE) ;
					xSemaphoreGive(xLcdCommandOnOff_BinarySemaphore) ; //unblock vTask_SendCommandToLcd_setOnOff
				}
				else{
					kicker_set_hvon_status(5, HVON_INACTIVE) ;
				}
				button_count += 1 ; //flag that a button has been initialised
				break ;
				
			case 8:	
				if (gpio_pin_is_low(HVON6_IDX)){ //pin is low when HVON signal is high
					gl_b_isittouchevent_ONOFF = pdFALSE ; //flag as a HVON check
					set_kicker_address_onoff(6) ;
					kicker_set_hvon_status(6, HVON_ACTIVE) ;
					xSemaphoreGive(xLcdCommandOnOff_BinarySemaphore) ; //unblock vTask_SendCommandToLcd_setOnOff
				}
				else{
					kicker_set_hvon_status(6, HVON_INACTIVE) ;
				}
				button_count += 1 ; //flag that a button has been initialised
				break ;
				
			default:
				button_count = MAX_LCD_BUTTONS ;
				break ;			
		}
		
		if(button_count < MAX_LCD_BUTTONS){
			vTaskDelay(LCD_BUTTON_INIT_TIMEOUT) ;
		}
		else{
			vTaskDelete(NULL) ; //self-delete vTask_LCDButtonInit
		}
	}
}
/*-----------------------------------------------------------*/

void vTask_ProcessPioISR(void* pvParameters)
{	
	Bool* ISR_updated = (Bool *)pvParameters ; //pointer to array storage for ISR status info, gl_b_pio_interrupt_array
	uint32_t ul_ISR_count = 0 ;
	Bool b_ISR_was_HVON = pdFALSE ;
	
	for( ;;) //task processing in infinite loop
	{
		if(ul_ISR_count == 0){ //all updated
			xSemaphoreTake(xPioISRTriggered_BinarySemaphore, portMAX_DELAY ) ; //given by ISR_pio_pins
			
			//task starts here after all interrupts have been serviced
			if(gl_backlight_state == LCD_OFF)
			{
				xSemaphoreGive(xLcdBacklightOn_BinarySemaphore) ; //unblock vTask_TurnBacklightOn
				vTaskDelay(LCD_BUTTON_INIT_TIMEOUT) ; //delay before start processing the pio lcd buttons
			}
			else
			{
				xTimerReset(xBacklightOffTimer, 0) ; //don't forget to reset the backlight timer
			}
			
		}
		else{			
			ul_ISR_count = 0 ; //reset count
			vTaskDelay(LCD_BUTTON_INIT_TIMEOUT) ;//delay task between iterations
		}
		
		//Process the ISR
		if(*ISR_updated){ //HVON1
			if (gpio_pin_is_high(HVON1_IDX))
			{
				set_kicker_address_onoff(1) ;
				kicker_set_hvon_status(1, HVON_INACTIVE) ;
				//gpio_set_pin_high(LED1_GPIO) ;
			}
			else
			{
				set_kicker_address_onoff(1) ;
				kicker_set_hvon_status(1, HVON_ACTIVE) ;
				//gpio_set_pin_low(LED1_GPIO) ;
			}
			*ISR_updated = pdFALSE ; //clear the 'updated' flag
			b_ISR_was_HVON = pdTRUE ; //flag the ISR origin
		}
		
		else if(*(ISR_updated + 1)){//HVON2
			if (gpio_pin_is_high(HVON2_IDX))
			{
				set_kicker_address_onoff(2) ;
				kicker_set_hvon_status(2, HVON_INACTIVE) ;
				//gpio_set_pin_high(LED1_GPIO) ;
			}
			else
			{
				set_kicker_address_onoff(2) ;
				kicker_set_hvon_status(2, HVON_ACTIVE) ;
				//gpio_set_pin_low(LED1_GPIO) ;
			}
			*(ISR_updated + 1) = pdFALSE ; //clear the 'updated' flag
			b_ISR_was_HVON = pdTRUE ; //flag the ISR origin
		}

		else if(*(ISR_updated + 2)){//HVON3
			if (gpio_pin_is_high(HVON3_IDX))
			{
				set_kicker_address_onoff(3) ;
				kicker_set_hvon_status(3, HVON_INACTIVE) ;
				//gpio_set_pin_high(LED1_GPIO) ;
			}
			else
			{
				set_kicker_address_onoff(3) ;
				kicker_set_hvon_status(3, HVON_ACTIVE) ;
				//gpio_set_pin_low(LED1_GPIO) ;
			}
			*(ISR_updated + 2) = pdFALSE ; //clear the 'updated' flag
			b_ISR_was_HVON = pdTRUE ; //flag the ISR origin
		}
		
		else if(*(ISR_updated + 3)){//HVON4
			if (gpio_pin_is_high(HVON4_IDX))
			{
				set_kicker_address_onoff(4) ;
				kicker_set_hvon_status(4, HVON_INACTIVE) ;
				//gpio_set_pin_high(LED1_GPIO) ;
			}
			else
			{
				set_kicker_address_onoff(4) ;
				kicker_set_hvon_status(4, HVON_ACTIVE) ;
				//gpio_set_pin_low(LED1_GPIO) ;
			}
			*(ISR_updated + 3) = pdFALSE ; //clear the 'updated' flag
			b_ISR_was_HVON = pdTRUE ; //flag the ISR origin
		}
		
		else if(*(ISR_updated + 4)){//HVON5
			if (gpio_pin_is_high(HVON5_IDX))
			{
				set_kicker_address_onoff(5) ;
				kicker_set_hvon_status(5, HVON_INACTIVE) ;
				//gpio_set_pin_high(LED1_GPIO) ;
			}
			else
			{
				set_kicker_address_onoff(5) ;
				kicker_set_hvon_status(5, HVON_ACTIVE) ;
				//gpio_set_pin_low(LED1_GPIO) ;
			}
			*(ISR_updated + 4) = pdFALSE ; //clear the 'updated' flag
			b_ISR_was_HVON = pdTRUE ; //flag the ISR origin
		}
		
		else if(*(ISR_updated + 5)){//HVON6
			if (gpio_pin_is_high(HVON6_IDX))
			{
				set_kicker_address_onoff(6) ;
				kicker_set_hvon_status(6, HVON_INACTIVE) ;
				//gpio_set_pin_high(LED1_GPIO) ;
			}
			else
			{
				set_kicker_address_onoff(6) ;
				kicker_set_hvon_status(6, HVON_ACTIVE) ;
				//gpio_set_pin_low(LED1_GPIO) ;
			}
			*(ISR_updated + 5) = pdFALSE ; //clear the 'updated' flag
			b_ISR_was_HVON = pdTRUE ; //flag the ISR origin
		}
		
		else if(*(ISR_updated + 6)){//PSUHEALTHY1
			if (gpio_pin_is_high(PSU_HEALTHY1_IDX))
			{
				set_kicker_address_reset(1) ;
				kicker_set_psuhealthy_status(1, PSU_HEALTHY_INACTIVE) ;
				//gpio_set_pin_high(LED1_GPIO) ;
			}
			else
			{
				set_kicker_address_reset(1) ;
				kicker_set_psuhealthy_status(1, PSU_HEALTHY_ACTIVE) ;
				//gpio_set_pin_low(LED1_GPIO) ;
			}
			*(ISR_updated + 6) = pdFALSE ; //clear the 'updated' flag
			b_ISR_was_HVON = pdFALSE ; //flag the ISR origin
		}
		
		else if(*(ISR_updated + 7)){//PSUHEALTHY2
			if (gpio_pin_is_high(PSU_HEALTHY2_IDX))
			{
				set_kicker_address_reset(3) ;
				kicker_set_psuhealthy_status(3, PSU_HEALTHY_INACTIVE) ;
				//gpio_set_pin_high(LED1_GPIO) ;
			}
			else
			{
				set_kicker_address_reset(3) ;
				kicker_set_psuhealthy_status(3, PSU_HEALTHY_ACTIVE) ;
				//gpio_set_pin_low(LED1_GPIO) ;
			}
			*(ISR_updated + 7) = pdFALSE ; //clear the 'updated' flag
			b_ISR_was_HVON = pdFALSE ; //flag the ISR origin
		}

		else if(*(ISR_updated + 8)){//PSUHEALTHY3
			if (gpio_pin_is_high(PSU_HEALTHY3_IDX))
			{
				set_kicker_address_reset(5) ;
				kicker_set_psuhealthy_status(5, PSU_HEALTHY_INACTIVE) ;
				//gpio_set_pin_high(LED1_GPIO) ;
			}
			else
			{
				set_kicker_address_reset(5) ;
				kicker_set_psuhealthy_status(5, PSU_HEALTHY_ACTIVE) ;
				//gpio_set_pin_low(LED1_GPIO) ;
			}
			*(ISR_updated + 8) = pdFALSE ; //clear the 'updated' flag
			b_ISR_was_HVON = pdFALSE ; //flag the ISR origin
		}
		
		//count how many ISR there are left to process
		for(uint8_t i=0; i<MAX_LCD_BUTTONS; i++){
			if(ISR_updated[i]){
				ul_ISR_count += 1 ;
			}
		}
		
		if(b_ISR_was_HVON){
			gl_b_isittouchevent_ONOFF = pdFALSE ;
			xSemaphoreGive(xLcdCommandOnOff_BinarySemaphore) ; //unblock vTask_SendCommandToLcd_setOnOff
		}
		else{
			gl_b_isittouchevent_RESET = pdFALSE ;
			xSemaphoreGive(xLcdCommandReset_BinarySemaphore) ; //unblock vTask_SendCommandToLcd_setReset
		}
	}
}
/*-----------------------------------------------------------*/

void vTask_ProcessEncoder2(void* pvParameters)
{
	xEncoderParams_t* pxParameters ;
	pxParameters = (xEncoderParams_t *) pvParameters ; //cast the void pointer parameter to the correct type	
	uint32_t id ;
	uint32_t mask ;	
	
	for( ;;) //task processing in infinite loop
	{
		xSemaphoreTake(xEncoderTriggered_BinarySemaphore, portMAX_DELAY ) ; //given by ISR_pio_encoder_action
		
		if(gl_backlight_state == LCD_OFF)
		{
			xSemaphoreGive(xLcdBacklightOn_BinarySemaphore) ; //unblock vTask_TurnBacklightOn			
		}
		else
		{
			xTimerReset(xBacklightOffTimer, 0) ; //don't forget to reset the backlight timer
			
			//store the interrupt id and mask locally 
			id = gl_ISR_enc_params.id ; 
			mask = gl_ISR_enc_params.mask ;
		
			//Set up the encoder params
			if(id==ID_ENC1_UP && mask == ENC1_UP){
				gl_enc_param.address = 1 ;
				gl_enc_param.direction = COUNT_UP ;
				gl_enc_param.switch_event = NULL ;
			}
		
			if(id==ID_ENC1_DN && mask == ENC1_DOWN){
				gl_enc_param.address = 1 ;
				gl_enc_param.direction = COUNT_DOWN ;
				gl_enc_param.switch_event = NULL ;
			}
		
			if(id==ID_ENC2_UP && mask == ENC2_UP){
				gl_enc_param.address = 2 ;
				gl_enc_param.direction = COUNT_UP ;
				gl_enc_param.switch_event = NULL ;
			}
		
			if(id==ID_ENC2_DN && mask == ENC2_DOWN){
				gl_enc_param.address = 2 ;
				gl_enc_param.direction = COUNT_DOWN ;
				gl_enc_param.switch_event = NULL ;
			}
		
			if(id==ID_ENC3_UP && mask == ENC3_UP){
				gl_enc_param.address = 3 ;
				gl_enc_param.direction = COUNT_UP ;
				gl_enc_param.switch_event = NULL ;
			}
		
			if(id==ID_ENC3_DN && mask == ENC3_DOWN){
				gl_enc_param.address = 3 ;
				gl_enc_param.direction = COUNT_DOWN ;
				gl_enc_param.switch_event = NULL ;
			}
		
			if(id==ID_ENC4_UP && mask == ENC4_UP){
				gl_enc_param.address = 4 ;
				gl_enc_param.direction = COUNT_UP ;
				gl_enc_param.switch_event = NULL ;
			}
		
			if(id==ID_ENC4_DN && mask == ENC4_DOWN){
				gl_enc_param.address = 4 ;
				gl_enc_param.direction = COUNT_DOWN ;
				gl_enc_param.switch_event = NULL ;
			}
		
			if(id==ID_ENC5_UP && mask == ENC5_UP){
				gl_enc_param.address = 5 ;
				gl_enc_param.direction = COUNT_UP ;
				gl_enc_param.switch_event = NULL ;
			}
		
			if(id==ID_ENC5_DN && mask == ENC5_DOWN){
				gl_enc_param.address = 5 ;
				gl_enc_param.direction = COUNT_DOWN ;
				gl_enc_param.switch_event = NULL ;
			}
		
			if(id==ID_ENC6_UP && mask == ENC6_UP){
				gl_enc_param.address = 6 ;
				gl_enc_param.direction = COUNT_UP ;
				gl_enc_param.switch_event = NULL ;
			}
		
			if(id==ID_ENC6_DN && mask == ENC6_DOWN){
				gl_enc_param.address = 6 ;
				gl_enc_param.direction = COUNT_DOWN ;
				gl_enc_param.switch_event = NULL ;
			}
		
			if(id==ID_ENC1_SW && mask == ENC1_SWITCH){
				gl_enc_param.address = 1 ;
				gl_enc_param.direction = NULL ;
				gl_enc_param.switch_event = pdTRUE ;
			}
		
			if(id==ID_ENC2_SW && mask == ENC2_SWITCH){
				gl_enc_param.address = 2 ;
				gl_enc_param.direction = NULL ;
				gl_enc_param.switch_event = pdTRUE ;
			}
		
			if(id==ID_ENC3_SW && mask == ENC3_SWITCH){
				gl_enc_param.address = 3 ;
				gl_enc_param.direction = NULL ;
				gl_enc_param.switch_event = pdTRUE ;
			}
		
			if(id==ID_ENC4_SW && mask == ENC4_SWITCH){
				gl_enc_param.address = 4 ;
				gl_enc_param.direction = NULL ;
				gl_enc_param.switch_event = pdTRUE ;
			}
		
			if(id==ID_ENC5_SW && mask == ENC5_SWITCH){
				gl_enc_param.address = 5 ;
				gl_enc_param.direction = NULL ;
				gl_enc_param.switch_event = pdTRUE ;
			}
		
			if(id==ID_ENC6_SW && mask == ENC6_SWITCH){
				gl_enc_param.address = 6 ;
				gl_enc_param.direction = NULL ;
				gl_enc_param.switch_event = pdTRUE ;
			}	

			if(((pxParameters->address > 0) && (pxParameters->address < 7)) && (pxParameters->direction != NULL)){
				//process count if address is in range and the direction is valid (non-null)
				set_kicker_address_encoder(pxParameters->address) ;
				ProcessCount2(pxParameters->address, pxParameters->direction) ;
				has_encoder_unit_changed(pxParameters->address) ;
				has_encoder_tens_changed(pxParameters->address) ;
				has_encoder_hund_changed(pxParameters->address) ;
				has_encoder_thou_changed(pxParameters->address) ;
			}
			else if(((pxParameters->address > 0) && (pxParameters->address < 7)) && (pxParameters->switch_event == pdTRUE)){
				//process switch
				Select_CoarseFine2(pxParameters->address) ;
			}
		}
	}
}
/*-----------------------------------------------------------*/

int main(void)
{
	freertos_usart_if rtos_port_usart0 ;
	freertos_usart_if rtos_port_usart1 ;
	freertos_usart_if rtos_port_usart2 ;
	xTaskHandle xLcdInitialiseDisplay ; //used to suspend or delete the task once lcd init is complete from another task
	
	int* ptr_to_read_val = NULL ;
		
	/* Prepare the hardware to run this demo. */
	prvSetupHardware() ;
	
	//gpio_set_pin_low(LED2_GPIO) ; //LED off - monitor on scope to show Task in use
	
	gl_us_loccrem_prevstate = gl_us_loccrem ; //store current value of local/remote status
	
	gl_xReadQ = xQueueCreate(1, sizeof(xReadQ_pkt_t)) ;
	
	//configASSERT(gl_xReadQ) ; //sanity check, freeze execution if queue handle is null
	
	xLcdReadValsQ = xQueueCreate(3, sizeof(xLCDReadValsQ_pkt_t)) ;
	
	//set up com ports	
	rtos_port_usart0 = prepare_usart0_port_standard(USART0, gl_usart0_receive_buffer, USART0_RECEIVE_BUFFER_SIZE) ;
	rtos_port_usart1 = prepare_usart1_port_standard(USART1, gl_usart1_receive_buffer, USART1_RECEIVE_BUFFER_SIZE) ;
	rtos_port_usart2 = prepare_usart2_port_standard(USART2, gl_usart2_receive_buffer, USART2_RECEIVE_BUFFER_SIZE) ;
	
	//assign to struct
	gl_usart_ports.port0 = rtos_port_usart0 ;
	gl_usart_ports.port1 = rtos_port_usart1 ;
	gl_usart_ports.port2 = rtos_port_usart2 ;
	
	
	//create a binary semaphore to trigger the backlight task
	vSemaphoreCreateBinary(xEncoderTriggered_BinarySemaphore) ;     //semaphore for Encoder ISR
	//vSemaphoreCreateBinary(xHVONTriggered_BinarySemaphore) ;        //semaphore for HVON ISR
	//vSemaphoreCreateBinary(xPSUHEALTHYTriggered_BinarySemaphore) ;  //semaphore for PSU HEALTHY ISR
	vSemaphoreCreateBinary(xSendLcdCommand_BinarySemaphore) ;       //trigger the backlight task
	vSemaphoreCreateBinary(xReadLcdTouchEvent_BinarySemaphore) ;    //trigger the usart0 receive task, a timer does this
	vSemaphoreCreateBinary(xProcessLcdTouchEvent_BinarySemaphore) ; 
	vSemaphoreCreateBinary(xReadQ_Seed_BinarySemaphore) ;
	vSemaphoreCreateBinary(xLcdReadRefresh_BinarySemaphore) ;
	vSemaphoreCreateBinary(xLcdReadValsQ_Seed_BinarySemaphore) ;
	vSemaphoreCreateBinary(xCPSRead_BinarySemaphore) ;
	vSemaphoreCreateBinary(xLcdCommandOnOff_BinarySemaphore) ;
	vSemaphoreCreateBinary(xLcdCommandReset_BinarySemaphore) ;
	//vSemaphoreCreateBinary(xHVONTriggered_BinarySemaphore) ;
	//vSemaphoreCreateBinary(xPSUHEALTHYTriggered_BinarySemaphore) ;
	vSemaphoreCreateBinary(xPioISRTriggered_BinarySemaphore) ;
	vSemaphoreCreateBinary(xQuickReleaseCheck_BinarySemaphore) ;
	vSemaphoreCreateBinary(xLcdBacklightOn_BinarySemaphore) ;
	vSemaphoreCreateBinary(xLcdBacklightOff_BinarySemaphore) ;
		
	xLcdCommand_CountingSemaphore = xSemaphoreCreateCounting(3, 0) ; // counting semaphore with 4 spaces in the queue
	xLcdCommandLocrem_CountingSemaphore = xSemaphoreCreateCounting(5, 0) ; // counting semaphore with 4 spaces in the queue
	xLcdCommandLoad_CountingSemaphore = xSemaphoreCreateCounting(5, 0) ; 
	xPsuCommand_CountingSemaphore = xSemaphoreCreateCounting(5, 0) ;
	//xLcdCommandOnOff_CountingSemaphore = xSemaphoreCreateCounting(5, 0) ;
	//xLcdCommandReset_CountingSemaphore = xSemaphoreCreateCounting(5, 0) ;
	//xHVONTriggered_CountingSemaphore = xSemaphoreCreateCounting(11, 0) ; //There are 6 HVON signals to monitor
	//xPSUHEALTHYTriggered_CountingSemaphore = xSemaphoreCreateCounting(2, 0) ; //There are 3 PSUHEALTHY signals to monitor
	

	// Create a timer to turn the backlight off after a period of inactivity
	xBacklightOffTimer = xTimerCreate((const signed char * const) "Backlight timeout timer", // A text name, purely to help debugging.
										LCD_BACKLIGHT_TIMEOUT, // The timer period.
										pdFALSE,			   // This is a one-shot timer, it will be reset (and restarted) by encoder or touch event.
										NULL,				   // The timer does not use its ID, so the ID is just set to NULL.
										prvBacklightOffTimerCallback	// The function that is called each time the timer expires.
										);//

	
	//configASSERT(xBacklightOffTimer) ; //sanity check, remove after debugging
	
	xLcdTouchEventTimer = xTimerCreate((const signed char * const) "LCD touch timer", 
										LCD_TOUCH_SCAN_PERIOD,
										pdTRUE, // This timer will reset and restart when expired
										NULL,
										prvLcdTouchEventTimerCallback // This function is called when the timer expires
										) ;
										
	//The timer will not actually start until the FreeRTOS kernel is started.
//	xTimerStart(xLcdTouchEventTimer, 0) ; //a block time of 0 simply means do not block	
	//configASSERT(xLcdTouchEventTimer) ; //sanity check, remove after debugging
	
	
	xLcdResetButtonTimer = xTimerCreate((const signed char * const) "LCD button latch timer",
										TIMEOUT_BETWEEN_RESET_COMMANDS,
										pdFALSE, // One-shot timer
										NULL,
										prvLcdResetButtonTimer // This function is called when the timer expires
										) ;
	
	//configASSERT(xLcdLocremButtonLatchTimer) ; //sanity check, remove after debugging
	
	xLcdLoadButtonPopupTimer = xTimerCreate((const signed char * const) "LCD load button popup timer",
										LCD_BUTTON_LOAD_TIMEOUT,
										pdFALSE, // One-shot timer
										NULL,
										prvLcdLoadButtonPopupTimerCallback // This function is called when the timer expires
										) ;
	
	//The timer will not actually start until the FreeRTOS kernel is started.
	//xTimerStart(xLcdButtonLatchTimer, 0) ; //a block time of 0 simply means do not block
	//start the timer once a button is pressed and latch is set
	//configASSERT(xLcdLoadButtonPopupTimer) ; //sanity check, remove after debugging
	
	xReadQSeedTimer = xTimerCreate((const signed char * const) "Seed Read Q",
										SEED_READ_Q_TIMEOUT,
										pdFALSE, // One-shot timer
										NULL,
										prvSeedReadQ_Timer // This function is called when the timer expires
										) ;
										
	xLcdReadValsQSeedTimer = xTimerCreate((const signed char * const) "Seed Lcd rvals",
										SEED_LCD_READVAL_Q_TIMEOUT,
										pdFALSE, // One-shot timer
										NULL,
										prvLcdReadValsSeedQ_Timer // This function is called when the timer expires
										) ;
										
	
	xCPSReadScanTimer = xTimerCreate((const signed char * const) "CPS Read scan timer",
										CPS_READ_SCAN_PERIOD,
										pdTRUE, //removed pdFALSE Dec2013// This timer will reset and restart when expired
										NULL,
										prvCPSReadScanTimer // This function is called when the timer expires
										) ;//*/
										
	//configASSERT(xCPSReadScanTimer) ; //sanity check, remove after debugging
	//configASSERT(xLCDReadValsRefresh) ; //sanity check, remove after debugging
	
	xLcdClearPrevButtonInfoTimer = xTimerCreate((const signed char * const) "Disable Touchscreen timer",
										LCD_CLEAR_PREV_BUTTON_TIMEOUT,
										pdFALSE,
										NULL,
										prvLcdClearPrevButtonInfoTimer
										) ;//*/
	
	//configASSERT(xLcdClearPrevButtonInfoTimer) ; //sanity check, remove after debugging			
		
/* ----------- Task definition in main() -------------------------------------- */

	//Priority from 0 (lowest) to configMAX_PRIORITIES-1 (4, highest)
	
	//Tab indented tasks are the most recently added in
	
	xTaskCreate(vTask_DrawLcdBackground, "LCD Bgnd", configMINIMAL_STACK_SIZE, //+200
		(void *) /*&gl_usart_ports*/rtos_port_usart0, 2/*4*/, NULL) ; //draw background on lcd
		
	xTaskCreate(vTask_SendCommandToKickerPSU_LocalReadQ, "ReadQ",
		configMINIMAL_STACK_SIZE+200, (void *) rtos_port_usart2, 1 /*5*/ /*1*/, NULL) ;
				
	xTaskCreate(vTask_SendCommandToKickerPSU_LocalReadQ_seed, "RdQseed",
		configMINIMAL_STACK_SIZE+200, (void *) NULL, 0, NULL) ;

				//this task was causing the dummy handler call
				xTaskCreate(vTask_SendCommandToLcd_setKV, "setKV", configMINIMAL_STACK_SIZE+200,
					(void *) rtos_port_usart0, 2/*3*/, NULL) ; //draw counter value on lcd
		
				//Process encoder ISR
				xTaskCreate(vTask_ProcessEncoder2, "Enc2", configMINIMAL_STACK_SIZE+100,
					(void *) &gl_enc_param, 1, NULL) ;
					
				//Process HVON ISR
//				xTaskCreate(vTask_ProcessHVON, "HVON", configMINIMAL_STACK_SIZE+100,
//					(void *) NULL, 1, NULL) ;
					
				//Process PSU_HEALTHY ISR
//				xTaskCreate(vTask_ProcessPSU_HEALTHY, "HEALTHY", configMINIMAL_STACK_SIZE+100,
//					(void *) NULL, 1, NULL) ;
				
				//create the one and only read LCD command task (USART0)
				xTaskCreate(vTask_ReadCommandFromLcd_Chesney, "Chesney", configMINIMAL_STACK_SIZE+200,
					(void *) rtos_port_usart0, 0, NULL) ;
					
				//xTaskCreate(vTask_ReadCommandFromLcd_QuickReleaseCheck, "QRCheck", configMINIMAL_STACK_SIZE+200,
				//	(void *) rtos_port_usart0, 3, NULL) ;
		
				//create a task to read from the CPS receive buffer (USART1), unblocked by a timer 
				xTaskCreate(vTask_CPSReadScan, "CPSRead", configMINIMAL_STACK_SIZE,
					(void *) rtos_port_usart1, 0, NULL) ;
		
				//Task to draw locrem buttons on the LCD
				xTaskCreate(vTask_SendCommandToLcd_setLocalRemote, "setLoRm", configMINIMAL_STACK_SIZE+200,
					(void *) rtos_port_usart0, 3, NULL) ;
		
				//Task to draw load buttons on the LCD
				xTaskCreate(vTask_SendCommandToLcd_setLoad, "setLoad", configMINIMAL_STACK_SIZE+200,
					(void *) rtos_port_usart0, 3, NULL) ;
					
				//Task to draw on/off buttons on the LCD
				xTaskCreate(vTask_SendCommandToLcd_setOnOff, "ONOFF", configMINIMAL_STACK_SIZE+100,
					(void *) rtos_port_usart0, 3, NULL) ;
					
				//Task to draw on/off pushed buttons on the LCD
				xTaskCreate(vTask_SendCommandToLcd_setReset, "setReset", configMINIMAL_STACK_SIZE+100,
					(void *) rtos_port_usart0, 3, NULL) ;
		
				//Task to send a one time command to the kicker psu
				xTaskCreate(vTask_SendCommandToKickerPSU_Wyclef, "Wyclef", configMINIMAL_STACK_SIZE+200,
					(void *) rtos_port_usart2, 3, NULL) ; //higher priority than the read loop task (currently set at 1)
		
				xTaskCreate(vTask_TurnBacklightOn, "BliteON", configMINIMAL_STACK_SIZE,
					(void *) rtos_port_usart0, configMAX_PRIORITIES-1, NULL) ; //turn backlight on, highest priority then suspend
					
				xTaskCreate(vTask_TurnBacklightOff, "BliteOFF", configMINIMAL_STACK_SIZE,
					(void *) rtos_port_usart0, configMAX_PRIORITIES-1, NULL) ; //turn backlight off, highest priority then suspend
		
				

/*********************** TASK MONITOR **********************************************/
	
//  report task status info, stack size etc
//	xTaskCreate(task_monitor, "taskmon", configMINIMAL_STACK_SIZE+100, //+1024,
//		(void *) rtos_port_usart1, 3, NULL) ; //higher priority than the read loop task (currently set at 1)
		
	/* There are certain points in the code where it is possible to get stuck in an infinite while loop
	The watchdog timer/task can be used to break out of these loop and allow the code to continue to execute. */
	
//	xTaskCreate(vTask_watchdog, "Watchdog", configMINIMAL_STACK_SIZE+100, 
//		(void *) NULL, configMAX_PRIORITIES-1, NULL) ; //highest priority user task
	
		
	//Task to send out the local read commands, this task will be controlled by vTask_SendCommandToKickerPSU_LocalReadMaster created in vTask_DrawLcdBackground
//	xTaskCreate(vTask_SendCommandToKickerPSU_SubsidiaryLocalReadLoop, "Local poll of PSUs", configMINIMAL_STACK_SIZE+200,
//		(void *) rtos_port_usart2, 0, NULL) ; //higher priority than the read loop task (currently set at 1)
	
	
	/* Start the RTOS scheduler. */
	vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following line
	will never be reached.  If the following line does execute, then there was
	insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	to be created.  See the memory management section on the FreeRTOS web site
	for more details. */
	for (;;) {
	}
	return 0;
}

/*-----------------------------------------------------------*/

static void prvSetupHardware(void)
{
	/* ASF function to setup clocking. */
	sysclk_init();
	slck_div_init(PIOB, 0x200) ;

	/* Ensure all priority bits are assigned as preemption priority bits. */
	NVIC_SetPriorityGrouping(0);

	/* Atmel library function to setup for the evaluation kit being used. */
	board_init();
	
	pmc_enable_periph_clk(ID_PIOA) ; //Enable PIO Controller clock
	pmc_enable_periph_clk(ID_PIOB) ; //Enable PIO Controller clock
	pmc_enable_periph_clk(ID_PIOC) ; //Enable PIO Controller clock
	
	configure_pio_interrupt() ; //configure the pio interrupt pins	
}
/*-----------------------------------------------------------*/

static void prvBacklightOffTimerCallback(void)
{
	xSemaphoreGive(xLcdBacklightOff_BinarySemaphore) ; //unblock vTask_TurnBacklightOff
}
/*-----------------------------------------------------------*/

static void prvLcdLoadButtonPopupTimerCallback(void)
{
	/* This function is used to pop up the LCD load buttons */
	
	int8_t us_load_status ;
	
	//check for set bits in gl_us_load
	
	us_load_status = kicker_get_load_status(get_kicker_address_load()) ;
	
	if(us_load_status == BUTTON_DOWN){
		//clear the bit
		clear_load(get_kicker_address_load()) ;
	}
	
	//ensure that global kicker address doesnt change while this is going on
	//only load touch event task writes to the kicker address at present
	//this can be done by not enabling the touch event timer until processing of the current load button has finished
	
	//clear the button latch to enable the touch screen capture again
	//gl_latch_locrem = 0 ; //if latch is set, clear it 	
	
	//unblock lcd load button task
	xSemaphoreGive(xLcdCommandLoad_CountingSemaphore) ; //unblock vTask_SendCommandToLcd_setLoad
	
	//start timer to clear the button latch, enabling touch screen capture again
	//xTimerStart(xLcdLocremButtonLatchTimer, 0) ; 	
	//xTimerReset(xLcdTouchEventTimer, 0) ;
}
/*-----------------------------------------------------------*/

static void prvLcdTouchEventTimerCallback(void) //Called by xLcdTouchEventTimer
{
	//declare any variables needed
	//uint8_t receive_buffer[50] ;
	//uint32_t bytes_received ;
	//portTickType max_wait_20ms = 20/portTICK_RATE_MS ;
	xSemaphoreGive(xReadLcdTouchEvent_BinarySemaphore) ; //unblocks vTask_ReadCommandFromLcd_Chesney
	
}
/*-----------------------------------------------------------*/

static void prvSeedReadQ_Timer (void)
{
	xSemaphoreGive(xReadQ_Seed_BinarySemaphore) ; //unblocks vTask_SendCommandToKickerPSU_LocalReadQ_seed	
}
/*-----------------------------------------------------------*/

static void prvLcdReadValsSeedQ_Timer(void)
{
	xSemaphoreGive(xLcdReadValsQ_Seed_BinarySemaphore) ; //unblocks vTask_SendCommandToLcd_ReadValsQ_seed
}
/*-----------------------------------------------------------*/

static void prvCPSReadScanTimer (void)
{
	xSemaphoreGive(xCPSRead_BinarySemaphore) ; //unblocks vTask_CPSReadScan
}
/*-----------------------------------------------------------*/

static void prvLcdClearPrevButtonInfoTimer (void)
{
	gl_previous_button_type = NULL ;
}
/*-----------------------------------------------------------*/


static void prvLcdResetButtonTimer (void)
{
	//cycle through the reset commands
	//Note, the supplies are turned OFF first
	if(gl_send_kicker_reset_select != KICKER_RESET_NEG_RST)
	{
		gl_send_kicker_reset_select++ ;
		set_wyclef_status(WYCLEF_TX_RESET) ;
		xSemaphoreGive(xPsuCommand_CountingSemaphore) ; //unblock vTask_SendCommandToKickerPSU_Wyclef		
		xTimerReset(xLcdResetButtonTimer, portMAX_DELAY) ;
	}
	else
	{		
		gl_send_kicker_reset_select = KICKER_RESET_POS_HVOFF ;
	}
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void)
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	
	//added
	size_t freeheap ;
	for (;;) {
		freeheap = xPortGetFreeHeapSize() ;
	}
}

/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(xTaskHandle pxTask,
		signed char *pcTaskName)
{
	(void) pcTaskName;
	(void) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for (;;) {
	}
}

/*-----------------------------------------------------------*/

void vApplicationTickHook(void)
{
	/* This function will be called by each tick interrupt if
	configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
	added here, but the tick hook is called from an interrupt context, so
	code must not attempt to block, and only the interrupt safe FreeRTOS API
	functions can be used (those that end in FromISR()). */
}

/*-----------------------------------------------------------*/

void slck_div_init(Pio* p_pio, uint32_t ul_div)
{
	/*
	The slow clock SLCK is used for the de-bouncing filter on selected PIO.
	The SLCK is currently set to the internal RC oscillator @ 32kHz.
	Setting the DIV value in the PIO_SCDR register divides the SLCK down.
	
	Pulses with duration < 1/2 of divided SLCK are filtered out.
	
	divided SLCK period = ((ul_div + 1) * 2) * (1/32kHz)
	
	A value of 0x200 (512) filters out pulses <16ms in duration	
	*/
	
	p_pio->PIO_SCDR = ul_div ; //set DIV value
	return ;
}
/*-----------------------------------------------------------*/

freertos_usart_if prepare_usart0_port_standard( Usart* usart_base, uint8_t* receive_buffer, uint32_t receive_buffer_size_in_bytes)
{
	/* Setup the port to be thread aware.
	   The transmit operation will wait until it has exclusive access
	   to the peripheral before commencing the PDC transfer.  A timeout
	   is returned if access isn't granted. */
	
	// Handle used to access the initialized port by other FreeRTOS ASF functions. 
	freertos_usart_if freertos_usart;
	
	// Configuration structure. 
	freertos_peripheral_options_t driver_options = {
		
													// This peripheral has full duplex asynchronous operation, so the
													// receive_buffer value is set to a valid buffer location. 
													receive_buffer,
													
													// receive_buffer_size is set to the size, in bytes, of the buffer pointed
													// to by the receive_buffer structure member (receive_buffer above). 
													receive_buffer_size_in_bytes,
													
													// The interrupt_priority value. 
													IRQ_PRIORITY_USART0,
													
													// The operation_mode value. 
													USART_RS232,
													
													( USE_TX_ACCESS_MUTEX | WAIT_TX_COMPLETE | WAIT_RX_COMPLETE ) //
													// More than one task will write to the LCD display but only one task will
													// read from the LCD display - processTouchEvent task. 
													// tx/rx_access needed when writing to usart from more than one task
													// ( USE_TX_ACCESS_MUTEX | USE_RX_ACCESS_MUTEX | WAIT_TX_COMPLETE | WAIT_RX_COMPLETE )
												};
												
	// The RS232 configuration. This structure, and the values used in its setting,
	// are from the standard ASF USART driver. 
	const sam_usart_opt_t usart_settings =
	{
		USART0_BAUDRATE,
		US_MR_CHRL_8_BIT,
		US_MR_PAR_NO,
		US_MR_NBSTOP_1_BIT,
		US_MR_CHMODE_NORMAL,
		0 // Only used in IrDA mode, so all values are ignored. 
	};
	
	// Call the USART specific FreeRTOS ASF driver initialization function. 
	freertos_usart = freertos_usart_serial_init( usart_base, &usart_settings, &driver_options );
	
	return freertos_usart;
}
/*-----------------------------------------------------------*/

freertos_usart_if prepare_usart1_port_standard( Usart* usart_base, uint8_t* receive_buffer, uint32_t receive_buffer_size_in_bytes)
{
	/* Setup the port to be thread aware.
	   The transmit operation will wait until it has exclusive access
	   to the peripheral before commencing the PDC transfer.  A timeout
	   is returned if access isn't granted. */
	
	// Handle used to access the initialized port by other FreeRTOS ASF functions. 
	freertos_usart_if freertos_usart;
	
	// Configuration structure. 
	freertos_peripheral_options_t driver_options = {
		
													// This peripheral has full duplex asynchronous operation, so the
													// receive_buffer value is set to a valid buffer location. 
													receive_buffer,
													
													// receive_buffer_size is set to the size, in bytes, of the buffer pointed
													// to by the receive_buffer structure member (receive_buffer above). 
													receive_buffer_size_in_bytes,
													
													// The interrupt_priority value. 
													IRQ_PRIORITY_USART1,
													
													// The operation_mode value. 
													USART_RS232,
													
													( USE_TX_ACCESS_MUTEX | WAIT_TX_COMPLETE | WAIT_RX_COMPLETE ) //
													// More than one task will write to the LCD display but only one task will
													// read from the LCD display - processTouchEvent task. 
													// tx/rx_access needed when writing to usart from more than one task
													// ( USE_TX_ACCESS_MUTEX | USE_RX_ACCESS_MUTEX | WAIT_TX_COMPLETE | WAIT_RX_COMPLETE )
												};
												
	// The RS232 configuration. This structure, and the values used in its setting,
	// are from the standard ASF USART driver. 
	const sam_usart_opt_t usart_settings =
	{
		USART1_BAUDRATE,
		US_MR_CHRL_8_BIT,
		US_MR_PAR_NO,
		US_MR_NBSTOP_1_BIT,
		US_MR_CHMODE_NORMAL,
		0 // Only used in IrDA mode, so all values are ignored. 
	};
	
	// Call the USART specific FreeRTOS ASF driver initialization function. 
	freertos_usart = freertos_usart_serial_init( usart_base, &usart_settings, &driver_options );
	
	return freertos_usart;
}
/*-----------------------------------------------------------*/

freertos_usart_if prepare_usart2_port_standard( Usart* usart_base, uint8_t* receive_buffer, uint32_t receive_buffer_size_in_bytes)
{
	/* Setup the port to be thread aware.
	   The transmit operation will wait until it has exclusive access
	   to the peripheral before commencing the PDC transfer.  A timeout
	   is returned if access isn't granted. */
	
	// Handle used to access the initialized port by other FreeRTOS ASF functions. 
	freertos_usart_if freertos_usart;
	
	// Configuration structure. 
	freertos_peripheral_options_t driver_options = {
		
													// This peripheral has full duplex asynchronous operation, so the
													// receive_buffer value is set to a valid buffer location. 
													receive_buffer,
													
													// receive_buffer_size is set to the size, in bytes, of the buffer pointed
													// to by the receive_buffer structure member (receive_buffer above). 
													receive_buffer_size_in_bytes,
													
													// The interrupt_priority value. 
													IRQ_PRIORITY_USART2,
													
													// The operation_mode value. 
													USART_RS232,
													
													( USE_TX_ACCESS_MUTEX | WAIT_TX_COMPLETE | WAIT_RX_COMPLETE ) //
													// More than one task will write to the LCD display but only one task will
													// read from the LCD display - processTouchEvent task. 
													// tx/rx_access needed when writing to usart from more than one task
													// ( USE_TX_ACCESS_MUTEX | USE_RX_ACCESS_MUTEX | WAIT_TX_COMPLETE | WAIT_RX_COMPLETE )
												};
												
	// The RS232 configuration. This structure, and the values used in its setting,
	// are from the standard ASF USART driver. 
	const sam_usart_opt_t usart_settings =
	{
		USART2_BAUDRATE,
		US_MR_CHRL_8_BIT,
		US_MR_PAR_NO,
		US_MR_NBSTOP_1_BIT,
		US_MR_CHMODE_NORMAL,
		0 // Only used in IrDA mode, so all values are ignored. 
	};
	
	// Call the USART specific FreeRTOS ASF driver initialization function. 
	freertos_usart = freertos_usart_serial_init( usart_base, &usart_settings, &driver_options );
	
	return freertos_usart;
}
/*-----------------------------------------------------------*/

status_code_t usart0_write_string_standard(freertos_usart_if freertos_usart, uint8_t* us_string, uint8_t us_string_length)
{
	//gl_usart0_write_buffer[USART0_WRITE_BUFFER_SIZE] is declared as a global	
	
	status_code_t result;
	
	uint8_t i ;
	
	// Send a string to the USART. The string must be in RAM, so copy it into an array.
	for(i=0; i<us_string_length; i++){
		gl_usart0_write_buffer[i] = us_string[i] ;
	}
	
	//strcpy(write_buffer, "one" ) ;
	
	// Using a block time of 10 / portTICK_RATE_MS means “don’t block any longer than 10ms"
	result = freertos_usart_write_packet( freertos_usart, gl_usart0_write_buffer, us_string_length,
	                                      10 / portTICK_RATE_MS );
										  
	if( result == STATUS_OK ) {
		/* freertos_usart_write_packet() does not return until transmission of the string has
		completed, meaning the write_buffer array can be re?used immediately without any risk
		of corrupting the original transmission. */
		
		//e.g. could send another string now
	}
	/* freertos_usart_write_packet() does not return until transmission of the string has
	completed, meaning the function can exit even though the buffer being transmitted is
	declared on the function’s stack. */
	return result;	
}
/*-----------------------------------------------------------*/

status_code_t usart0_read_string_standard(freertos_usart_if freertos_usart, uint8_t* buffer_address, uint8_t us_string_length)
{
	//store the replies in the selected receive buffer
	
	status_code_t result ;
	
	uint8_t us_bytes_received ;
	
	portTickType max_wait = 10 / portTICK_RATE_MS ; //specify maximum wait time
	
	/* Attempt to read bytes.  If fewer than expected arrive, wait for the specified 
	max_wait time for the rest to arrive 
	
	If the number of bytes available is less than the number requested then
	freertos_usart_serial_read_packet() will wait for more bytes to become
	available.  block_time_ticks specifies the maximum amount of time the
	driver will wait before returning fewer bytes than were requested.	*/
	
	us_bytes_received = freertos_usart_serial_read_packet( freertos_usart, 
	                                                       buffer_address, 
														   us_string_length, 
														   max_wait ) ;
													   
	if(us_bytes_received == us_string_length)	
	{
		//all bytes were received and safely stored in the receive buffer
		return STATUS_OK ;
	}
	else
	{
		//fewer/more than the requested bytes were received.  Restore buffer to previous value?
		return ERR_IO_ERROR ;
	}
}
/*-----------------------------------------------------------*/

uint8_t usart0_read_string(freertos_usart_if freertos_usart, uint8_t* buffer_address, uint8_t us_string_length)
{
	//store the replies in the selected receive buffer
	
	status_code_t result ;
	
	uint8_t us_bytes_received ;
	
	portTickType max_wait = 1 / portTICK_RATE_MS ; //specify maximum wait time
	
	/* Attempt to read bytes.  If fewer than expected arrive, wait for the specified 
	max_wait time for the rest to arrive 
	
	If the number of bytes available is less than the number requested then
	freertos_usart_serial_read_packet() will wait for more bytes to become
	available.  block_time_ticks specifies the maximum amount of time the
	driver will wait before returning fewer bytes than were requested.	*/
	
	us_bytes_received = freertos_usart_serial_read_packet( freertos_usart, 
	                                                       buffer_address, 
														   us_string_length, 
														   max_wait ) ;
	
	//Edited to comply with new Chesney task
	//now returns the number of bytes read
	return us_bytes_received ;												   
	/*if(us_bytes_received == us_string_length)	
	{
		//all bytes were received and safely stored in the receive buffer
		return STATUS_OK ;
	}
	else
	{
		//fewer/more than the requested bytes were received.  Restore buffer to previous value?
		return ERR_IO_ERROR ;
	}//*/
}
/*-----------------------------------------------------------*/


status_code_t usart1_write_string_standard(freertos_usart_if freertos_usart, uint8_t* us_string, uint8_t us_string_length)
{
	status_code_t result;
	
	uint8_t i ;
	
	// Send a string to the USART. The string must be in RAM, so copy it into an array.
	for(i=0; i<us_string_length; i++){
		gl_usart1_write_buffer[i] = us_string[i] ;
	}
	
	// Using a block time of 10 / portTICK_RATE_MS means “don’t block any longer than 10ms"
	result = freertos_usart_write_packet( freertos_usart, gl_usart1_write_buffer, us_string_length,
	                                      10 / portTICK_RATE_MS );
										  
	if( result == STATUS_OK ) {
		/* freertos_usart_write_packet() does not return until transmission of the string has
		completed, meaning the write_buffer array can be re?used immediately without any risk
		of corrupting the original transmission. */
		
		//e.g. could send another string now
	}
	/* freertos_usart_write_packet() does not return until transmission of the string has
	completed, meaning the function can exit even though the buffer being transmitted is
	declared on the function’s stack. */
	return result;	
}
/*-----------------------------------------------------------*/

status_code_t usart1_write_string_16bit(freertos_usart_if freertos_usart, uint8_t* us_string, uint16_t us_string_length)
{
	//gl_usart2_write_buffer[USART2_WRITE_BUFFER_SIZE] is declared as a global 
	
	status_code_t result;
	
	uint16_t i ;
	
	// Send a string to the USART. The string must be in RAM, so copy it into an array.
	for(i=0; i<us_string_length; i++){
		gl_usart1_write_buffer[i] = us_string[i] ;
	}
	
	// Using a block time of 10 / portTICK_RATE_MS means “don’t block any longer than 10ms"
	result = freertos_usart_write_packet( freertos_usart, gl_usart1_write_buffer, us_string_length,
	                                      10 / portTICK_RATE_MS );
										  
	if( result == STATUS_OK ) {
		/* freertos_usart_write_packet() does not return until transmission of the string has
		completed, meaning the write_buffer array can be re?used immediately without any risk
		of corrupting the original transmission. */
		
		//e.g. could send another string now
	}
	/* freertos_usart_write_packet() does not return until transmission of the string has
	completed, meaning the function can exit even though the buffer being transmitted is
	declared on the function’s stack. */
	return result;	
}
/*-----------------------------------------------------------*/

uint8_t usart1_read_string(freertos_usart_if freertos_usart, uint8_t* buffer_address, uint8_t us_string_length)
{
	//store the replies in the selected receive buffer
	
	status_code_t result ;
	
	uint8_t us_bytes_received ;
	
	portTickType max_wait = 1 / portTICK_RATE_MS ; //specify maximum wait time
	
	/* Attempt to read bytes.  If fewer than expected arrive, wait for the specified 
	max_wait time for the rest to arrive 
	
	If the number of bytes available is less than the number requested then
	freertos_usart_serial_read_packet() will wait for more bytes to become
	available.  block_time_ticks specifies the maximum amount of time the
	driver will wait before returning fewer bytes than were requested.	*/
	
	us_bytes_received = freertos_usart_serial_read_packet( freertos_usart, 
	                                                       buffer_address, 
														   us_string_length, 
														   max_wait ) ;
	
	//Edited to comply with new Chesney task
	//now returns the number of bytes read
	return us_bytes_received ;												   
	/*if(us_bytes_received == us_string_length)	
	{
		//all bytes were received and safely stored in the receive buffer
		return STATUS_OK ;
	}
	else
	{
		//fewer/more than the requested bytes were received.  Restore buffer to previous value?
		return ERR_IO_ERROR ;
	}//*/
}
/*-----------------------------------------------------------*/


status_code_t usart2_write_string_standard(freertos_usart_if freertos_usart, uint8_t* us_string, uint8_t us_string_length)
{
	//gl_usart2_write_buffer[USART2_WRITE_BUFFER_SIZE] is declared as a global 
	
	status_code_t result;
	
	uint8_t i ;
	
	if(us_string_length < USART2_WRITE_BUFFER_SIZE) //check sending string isn't too big
	{
		// Send a string to the USART. The string must be in RAM, so copy it into an array.
		for(i=0; i<us_string_length; i++){
			gl_usart2_write_buffer[i] = us_string[i] ;
		}
		
		// Using a block time of 10 / portTICK_RATE_MS means “don’t block any longer than 10ms"
		result = freertos_usart_write_packet( freertos_usart, gl_usart2_write_buffer, us_string_length,
	                                      10 / portTICK_RATE_MS );
										  
		if( result == STATUS_OK ) {
			/* freertos_usart_write_packet() does not return until transmission of the string has
			completed, meaning the write_buffer array can be re?used immediately without any risk
			of corrupting the original transmission. */
		
			//e.g. could send another string now
		}
		/* freertos_usart_write_packet() does not return until transmission of the string has
		completed, meaning the function can exit even though the buffer being transmitted is
		declared on the function’s stack. */
		return result;	
	}
	else
	{
		return ERR_BAD_DATA ;
	}
}
/*-----------------------------------------------------------*/

status_code_t usart2_write_string_16bit(freertos_usart_if freertos_usart, uint8_t* us_string, uint16_t us_string_length)
{
	//gl_usart2_write_buffer[USART2_WRITE_BUFFER_SIZE] is declared as a global 
	
	status_code_t result;
	
	uint16_t i ;
	
	// Send a string to the USART. The string must be in RAM, so copy it into an array.
	for(i=0; i<us_string_length; i++){
		gl_usart2_write_buffer[i] = us_string[i] ;
	}
	
	// Using a block time of 10 / portTICK_RATE_MS means “don’t block any longer than 10ms"
	result = freertos_usart_write_packet( freertos_usart, gl_usart2_write_buffer, us_string_length,
	                                      10 / portTICK_RATE_MS );
										  
	if( result == STATUS_OK ) {
		/* freertos_usart_write_packet() does not return until transmission of the string has
		completed, meaning the write_buffer array can be re?used immediately without any risk
		of corrupting the original transmission. */
		
		//e.g. could send another string now
	}
	/* freertos_usart_write_packet() does not return until transmission of the string has
	completed, meaning the function can exit even though the buffer being transmitted is
	declared on the function’s stack. */
	return result;	
}
/*-----------------------------------------------------------*/

status_code_t usart2_read_string_standard(freertos_usart_if freertos_usart, uint8_t* buffer_address, uint8_t us_string_length)
{
	//store the replies in the selected receive buffer
	
	status_code_t result ;
	
	uint8_t us_bytes_received ;
	
	/* In testing using the KUI emulator replies come after ~60ms max,
	   Therefore allow 100ms for the replies to be seen.
	   This timing may vary on the live system */
	portTickType max_wait = 100 / portTICK_RATE_MS ; //specify maximum wait time
	
	/* Attempt to read bytes.  If fewer than expected arrive, wait for the specified 
	max_wait time for the rest to arrive 
	
	If the number of bytes available is less than the number requested then
	freertos_usart_serial_read_packet() will wait for more bytes to become
	available.  block_time_ticks specifies the maximum amount of time the
	driver will wait before returning fewer bytes than were requested.	*/
	
	us_bytes_received = freertos_usart_serial_read_packet( freertos_usart, 
	                                                       buffer_address, 
														   us_string_length, 
														   max_wait ) ;
														   
	if(us_bytes_received == us_string_length)	
	{
		//all bytes were received and safely stored in the receive buffer
		return pdPASS ;
	}
	else
	{
		//fewer/more than the requested bytes were received.  Restore buffer to previous value?
		return pdFAIL ;
	}
}
/*-----------------------------------------------------------*/

status_code_t flush_usart_buffer( freertos_usart_if freertos_usart, uint8_t* buffer_address, uint8_t us_string_length, portTickType max_wait)
{
	//remove bytes from the usart buffer until none remain
		
	status_code_t result ;
	
	uint8_t us_bytes_received = 1 ; //seed with '1'
	
	portTickType max_wait_ms = max_wait / portTICK_RATE_MS ; //specify maximum wait time, changed from 50ms
	
	//Read any remaining bytes out of the USART receive buffer.  If all replies are stored there should be no bytes to read
	
	//this while loop could get stuck
	while(us_bytes_received != 0)
	{
		us_bytes_received = freertos_usart_serial_read_packet( freertos_usart, 
	                                                       buffer_address, 
														   us_string_length, 
														   max_wait_ms ) ;
	}
														   
	if(us_bytes_received == 0)	
	{
		//all bytes were received and safely stored in the receive buffer
		return pdPASS ;
	}
	else
	{
		//not sure the code could ever reach here
		//fewer than the requested bytes were received.  Restore buffer to previous value?
		return pdFAIL ;
	}
}
/*-----------------------------------------------------------*/


void configure_pio_interrupt(void)
{
	//Configure Encoder 1
	pio_handler_set(PIO_ENC1_UP, ID_ENC1_UP, ENC1_UP, (PIO_IT_RISE_EDGE | PIO_DEGLITCH), ISR_pio_encoder_action);
	pio_enable_interrupt(PIO_ENC1_UP, ENC1_UP);
	pio_handler_set(PIO_ENC1_DN, ID_ENC1_DN, ENC1_DOWN, (PIO_IT_RISE_EDGE | PIO_DEGLITCH), ISR_pio_encoder_action);
	pio_enable_interrupt(PIO_ENC1_DN, ENC1_DOWN);	
	pio_handler_set(PIO_ENC1_SW, ID_ENC1_SW, ENC1_SWITCH, PIO_IT_FALL_EDGE, ISR_pio_encoder_action);
	pio_enable_interrupt(PIO_ENC1_SW, ENC1_SWITCH);
	
	//Configure Encoder 2
	pio_handler_set(PIO_ENC2_UP, ID_ENC2_UP, ENC2_UP, (PIO_IT_RISE_EDGE | PIO_DEGLITCH), ISR_pio_encoder_action);
	pio_enable_interrupt(PIO_ENC2_UP, ENC2_UP);	
	pio_handler_set(PIO_ENC2_DN, ID_ENC2_DN, ENC2_DOWN, (PIO_IT_RISE_EDGE | PIO_DEGLITCH), ISR_pio_encoder_action);
	pio_enable_interrupt(PIO_ENC2_DN, ENC2_DOWN);	
	pio_handler_set(PIO_ENC2_SW, ID_ENC2_SW, ENC2_SWITCH, PIO_IT_FALL_EDGE, ISR_pio_encoder_action);
	pio_enable_interrupt(PIO_ENC2_SW, ENC2_SWITCH);
	
	//Configure Encoder 3
	pio_handler_set(PIO_ENC3_UP, ID_ENC3_UP, ENC3_UP, (PIO_IT_RISE_EDGE | PIO_DEGLITCH), ISR_pio_encoder_action);
	pio_enable_interrupt(PIO_ENC3_UP, ENC3_UP);	
	pio_handler_set(PIO_ENC3_DN, ID_ENC3_DN, ENC3_DOWN, (PIO_IT_RISE_EDGE | PIO_DEGLITCH), ISR_pio_encoder_action);
	pio_enable_interrupt(PIO_ENC3_DN, ENC3_DOWN);	
	pio_handler_set(PIO_ENC3_SW, ID_ENC3_SW, ENC3_SWITCH, PIO_IT_FALL_EDGE, ISR_pio_encoder_action);
	pio_enable_interrupt(PIO_ENC3_SW, ENC3_SWITCH);
	
	//Configure Encoder 4
	pio_handler_set(PIO_ENC4_UP, ID_ENC4_UP, ENC4_UP, (PIO_IT_RISE_EDGE | PIO_DEGLITCH), ISR_pio_encoder_action);
	pio_enable_interrupt(PIO_ENC4_UP, ENC4_UP);	
	pio_handler_set(PIO_ENC4_DN, ID_ENC4_DN, ENC4_DOWN, (PIO_IT_RISE_EDGE | PIO_DEGLITCH), ISR_pio_encoder_action);
	pio_enable_interrupt(PIO_ENC4_DN, ENC4_DOWN);	
	pio_handler_set(PIO_ENC4_SW, ID_ENC4_SW, ENC4_SWITCH, PIO_IT_FALL_EDGE, ISR_pio_encoder_action);
	pio_enable_interrupt(PIO_ENC4_SW, ENC4_SWITCH);
	
	//Configure Encoder 5
	pio_handler_set(PIO_ENC5_UP, ID_ENC5_UP, ENC5_UP, (PIO_IT_RISE_EDGE | PIO_DEGLITCH), ISR_pio_encoder_action);
	pio_enable_interrupt(PIO_ENC5_UP, ENC5_UP);	
	pio_handler_set(PIO_ENC5_DN, ID_ENC5_DN, ENC5_DOWN, (PIO_IT_RISE_EDGE | PIO_DEGLITCH), ISR_pio_encoder_action);
	pio_enable_interrupt(PIO_ENC5_DN, ENC5_DOWN);	
	pio_handler_set(PIO_ENC5_SW, ID_ENC5_SW, ENC5_SWITCH, PIO_IT_FALL_EDGE, ISR_pio_encoder_action);
	pio_enable_interrupt(PIO_ENC5_SW, ENC5_SWITCH);

	//Configure Encoder 6
	pio_handler_set(PIO_ENC6_UP, ID_ENC6_UP, ENC6_UP, (PIO_IT_RISE_EDGE | PIO_DEGLITCH), ISR_pio_encoder_action);
	pio_enable_interrupt(PIO_ENC6_UP, ENC6_UP);	
	pio_handler_set(PIO_ENC6_DN, ID_ENC6_DN, ENC6_DOWN, (PIO_IT_RISE_EDGE | PIO_DEGLITCH), ISR_pio_encoder_action);
	pio_enable_interrupt(PIO_ENC6_DN, ENC6_DOWN);	
	pio_handler_set(PIO_ENC6_SW, ID_ENC6_SW, ENC6_SWITCH, PIO_IT_FALL_EDGE, ISR_pio_encoder_action);
	pio_enable_interrupt(PIO_ENC6_SW, ENC6_SWITCH);	
	
//^*^ IMPORTANT - RELATED TO HVON SIGNAL HANDLING
	//Configure PB2, used for HVON interrupt
	//2 options, either interrupt on HVON change or check HVON status during a timer interrupt
	//just draw the amber button when on/off changes
	//then check level of PDSR reg during the chosen interrupt
	//This decision may depend on how the hardware responds (i.e. the real HVON signal)
	//Decided to use timer interrupt as this prevents the system getting out of synch
	//on KUI power up, if HVON signal is already present and good, i.e. no edge will be detected
	//A timer interrupt allows me to monitor BOTH high and low levels of HVON

//HVON interrupts
	pio_handler_set(PIOC, ID_PIOC, HVON1, PIO_DEGLITCH, ISR_pio_pins) ;//ISR_pio_HVON) ;
	pio_enable_interrupt(PIOC, HVON1) ;
	
	pio_handler_set(PIOC, ID_PIOC, HVON2, PIO_DEGLITCH, ISR_pio_pins) ;//ISR_pio_HVON) ;
	pio_enable_interrupt(PIOC, HVON2) ;
	
	pio_handler_set(PIOC, ID_PIOC, HVON3, PIO_DEGLITCH, ISR_pio_pins) ;//ISR_pio_HVON) ;
	pio_enable_interrupt(PIOC, HVON3) ;
	
	pio_handler_set(PIOC, ID_PIOC, HVON4, PIO_DEGLITCH, ISR_pio_pins) ;//ISR_pio_HVON) ;
	pio_enable_interrupt(PIOC, HVON4) ;
	
	pio_handler_set(PIOC, ID_PIOC, HVON5, PIO_DEGLITCH, ISR_pio_pins) ;//ISR_pio_HVON) ;
	pio_enable_interrupt(PIOC, HVON5) ;
	
	pio_handler_set(PIOC, ID_PIOC, HVON6, PIO_DEGLITCH, ISR_pio_pins) ;//ISR_pio_HVON) ;
	pio_enable_interrupt(PIOC, HVON6) ;
	
//PSU HEALTHY interrupts
	pio_handler_set(PIOC, ID_PIOC, PSU_HEALTHY1, PIO_DEGLITCH, ISR_pio_pins) ;//ISR_pio_PSUHEALTHY) ;
	pio_enable_interrupt(PIOC, PSU_HEALTHY1) ;
	
	pio_handler_set(PIOC, ID_PIOC, PSU_HEALTHY2, PIO_DEGLITCH, ISR_pio_pins) ;//ISR_pio_PSUHEALTHY) ;
	pio_enable_interrupt(PIOC, PSU_HEALTHY2) ;
	
	pio_handler_set(PIOC, ID_PIOC, PSU_HEALTHY3, PIO_DEGLITCH, ISR_pio_pins) ;//ISR_pio_PSUHEALTHY) ;
	pio_enable_interrupt(PIOC, PSU_HEALTHY3) ;
	
//CRASH OFF interrupt
//	pio_handler_set(PIOC, ID_PIOC, CRASH_OFF, PIO_DEGLITCH, ISR_pio_pins) ;
//	pio_enable_interrupt(PIOC, CRASH_OFF) ;


	//test pb2 driving psu healthy interrupt
//	pio_handler_set(PIOB, ID_PIOB, PIO_PB2, PIO_DEGLITCH, pio_interrupt_handler_psuhealthy) ;
//	pio_enable_interrupt(PIOB, PIO_PB2) ;
	

}
/*-----------------------------------------------------------*/

void ISR_pio_encoder_action(uint32_t id, uint32_t mask)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE ;
	
	//Which interrupt was it? The processing Task will need to know...
	gl_ISR_enc_params.id = id ;
	gl_ISR_enc_params.mask = mask ;
	
	gpio_toggle_pin(PIO_PA20_IDX); //flash green led when encoder changes
	
	//give semaphore to wake up process encoder event task
	xSemaphoreGiveFromISR(xEncoderTriggered_BinarySemaphore, &xHigherPriorityTaskWoken) ; //trigger vTask_ProcessEncoder2
	
	/*
	   if xHigherPriorityTaskWoken has now been set to pdTRUE then a context switch should be performed.
	   
	   The correct context switch for the Cortex-M3 FreeRTOS port is portEND_SWITCHING_ISR()
	   
	   For info:
	   
	   If there was a task that was blocked on the semaphore, and giving the
	   semaphore caused the task to unblock, and the unblocked task has a priority
	   higher than the current Running state task (the task that this interrupt
	   interrupted), then lHigherPriorityTaskWoken will have been set to pdTRUE
	   internally within xSemaphoreGiveFromISR().  Passing pdTRUE into the
	   portEND_SWITCHING_ISR() macro will result in a context switch being pended to
	   ensure this interrupt returns directly to the unblocked, higher priority,
	   task.  Passing pdFALSE into portEND_SWITCHING_ISR() has no effect. 
	*/
	
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken ) ;
	
	return ;
}
/*-----------------------------------------------------------*/


static void ISR_pio_pins(uint32_t id, uint32_t mask)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE ;
	
	/* Need to create a record of the interrupts that occur,
	 so the processing task knows which mask/id to act upon. 
	 
	 Now processing all 9 pio interrupts in the one ISR */
	Bool* ptr_pio_interrupt_array = gl_b_pio_interrupt_array ;
	
	if( id == ID_PIOC && mask == HVON1){
		*ptr_pio_interrupt_array = pdTRUE ;
	}
	else if( id == ID_PIOC && mask == HVON2){
		*(ptr_pio_interrupt_array + 1) = pdTRUE ;
	}
	else if( id == ID_PIOC && mask == HVON3){
		*(ptr_pio_interrupt_array + 2) = pdTRUE ;
	}
	else if( id == ID_PIOC && mask == HVON4){
		*(ptr_pio_interrupt_array + 3) = pdTRUE ;
	}
	else if( id == ID_PIOC && mask == HVON5){
		*(ptr_pio_interrupt_array + 4) = pdTRUE ;
	}
	else if( id == ID_PIOC && mask == HVON6){
		*(ptr_pio_interrupt_array + 5) = pdTRUE ;
	}
	else if( id == ID_PIOC && mask == PSU_HEALTHY1){
		*(ptr_pio_interrupt_array + 6) = pdTRUE ;
	}
	else if( id == ID_PIOC && mask == PSU_HEALTHY2){
		*(ptr_pio_interrupt_array + 7) = pdTRUE ;
	}
	else if( id == ID_PIOC && mask == PSU_HEALTHY3){
		*(ptr_pio_interrupt_array + 8) = pdTRUE ;
	}
	else //interrupt not recognised
	{		
		set_kicker_address_reset(0) ; //set to invalid address
	}
		
	xSemaphoreGiveFromISR(xPioISRTriggered_BinarySemaphore, &xHigherPriorityTaskWoken) ; //trigger vTask_ProcessPioISR
	
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken ) ;
	
	return ;
}
/*-----------------------------------------------------------*/

static void ProcessCount2(uint8_t us_kicker_address, uint8_t us_enc_status)
{
	//edit this to allow kicker address to be passed as an argument
	//changed to allow for the 3 Kicker pairs
	//us_enc_status determines an up or down count
	uint16_t* uw_ptr_temp_count ;
	int8_t us_step_count = 0 ;
	
	//gpio_toggle_pin(PIO_PA20_IDX); //flash green led when encoder changes (Moved to ISR)
	
	us_step_count = get_step_size(us_kicker_address) ;  //return count step size
	if(us_step_count == rc_Error1){
		//error encountered, log and exit
		return ;
	}
	
	uw_ptr_temp_count = get_active_kicker_count(us_kicker_address) ;
	
	uint16_t uw_temp_countval ;
	
	uw_temp_countval = *uw_ptr_temp_count ; //get value at pointer address
	
	if(us_enc_status == COUNT_UP)
	{
		//Count up
		if(uw_temp_countval <= (gl_uw_up_count_limit-us_step_count))
		uw_temp_countval += us_step_count ;
		else
		uw_temp_countval = (uw_temp_countval-(gl_uw_up_count_limit-(us_step_count-1))) ;
	}
	else if(us_enc_status == COUNT_DOWN)
	{
		//Count down
		if(uw_temp_countval < us_step_count)
		uw_temp_countval = (uw_temp_countval-us_step_count-(65535-gl_uw_up_count_limit)) ;
		else
		uw_temp_countval -= us_step_count ;
	}
	
	*uw_ptr_temp_count = uw_temp_countval ; //put new value at pointer address
	//draw_cv2(us_kicker_address) ;
}//*/
/*-----------------------------------------------------------*/

static void Select_CoarseFine2(uint8_t us_kick_address)
{
	//toggle coarse/fine according to kicker address
	switch(us_kick_address)
	{
		case 1:
		if((gl_us_step_count & 0x01) >> 0){ //Kicker1 select is fine, b0=1
			gl_us_step_count &= 0x3E ; //set b0=0 (coarse)
		}
		else
		gl_us_step_count |= 0x01 ;//set b0=1
		break ;
		
		case 2:
		if((gl_us_step_count & 0x02) >> 1){ //Kicker2 select is fine, b1=1
			gl_us_step_count &= 0x3D ; //set b1=0 (coarse)
		}
		else
		gl_us_step_count |= 0x02 ;//set b1=1
		break ;
		
		case 3:
		if((gl_us_step_count & 0x04) >> 2){ //Kicker1 select is fine, b2=1
			gl_us_step_count &= 0x3B ; //set b2=0 (coarse)
		}
		else
		gl_us_step_count |= 0x04 ;//set b2=1
		break ;
		
		case 4:
		if((gl_us_step_count & 0x08) >> 3){ //Kicker1 select is fine, b3=1
			gl_us_step_count &= 0x37 ; //set b3=0 (coarse)
		}
		else
		gl_us_step_count |= 0x08 ;//set b3=1
		break ;
		
		case 5:
		if((gl_us_step_count & 0x10) >> 4){ //Kicker1 select is fine, b4=1
			gl_us_step_count &= 0x2F ; //set b4=0 (coarse)
		}
		else
		gl_us_step_count |= 0x10 ;//set b4=1
		break ;

		case 6:
		if((gl_us_step_count & 0x20) >> 5){ //Kicker1 select is fine, b5=1
			gl_us_step_count &= 0x1F ; //set b5=0 (coarse)
		}
		else
		gl_us_step_count |= 0x20 ;//set b5=1
		break ;
		
		default:
		//error handling
		break ;
	}
}//*/
/*-----------------------------------------------------------*/

static int8_t get_step_size(uint8_t us_kicker_address)
{
	
	switch(us_kicker_address)
	{
		case 1:
		if((gl_us_step_count & 0x01) >> 0)
		return COUNT_FINE ; //fine
		else
		return COUNT_COARSE ; //coarse
		break ;
		
		case 2:
		if((gl_us_step_count & 0x02) >> 1)
		return COUNT_FINE ; //fine
		else
		return COUNT_COARSE ; //coarse
		break ;

		case 3:
		if((gl_us_step_count & 0x04) >> 2)
		return COUNT_FINE ; //fine
		else
		return COUNT_COARSE ; //coarse
		break ;
		
		case 4:
		if((gl_us_step_count & 0x08) >> 3)
		return COUNT_FINE ; //fine
		else
		return COUNT_COARSE ; //coarse
		break ;
		
		case 5:
		if((gl_us_step_count & 0x10) >> 4)
		return COUNT_FINE ; //fine
		else
		return COUNT_COARSE ; //coarse
		break ;
		
		case 6:
		if((gl_us_step_count & 0x20) >> 5)
		return COUNT_FINE ; //fine
		else
		return COUNT_COARSE ; //coarse
		break ;
		
		default:
		return rc_Error1 ;
	}//*/
}
/*-----------------------------------------------------------*/

static uint16_t* get_active_kicker_count(uint8_t kicker_address)
{
	switch(kicker_address)
	{
		case 1:
		return &gl_uw_k1p_count ;
		
		case 2:
		return &gl_uw_k1n_count ;
		
		case 3:
		return &gl_uw_k2p_count ;
		
		case 4:
		return &gl_uw_k2n_count ;
		
		case 5:
		return &gl_uw_k3p_count ;
		
		case 6:
		return &gl_uw_k3n_count ;
		
		default:
		return rc_Error1 ;
		//need to handle these errors at some point
		//print a useful error message and then continue with prog execution
	}
}
/*-----------------------------------------------------------*/

static uint8_t get_kicker_address_locrem(void)
{
	return gl_us_kicker_address_locrem ;
}
/*-----------------------------------------------------------*/

static void set_kicker_address_locrem(uint8_t us_kicker_address)
{
	gl_us_kicker_address_locrem = us_kicker_address ;
	return ;
}
/*-----------------------------------------------------------*/

static uint8_t get_kicker_address_encoder(void)
{
	return gl_us_kicker_address_encoder ;
}
/*-----------------------------------------------------------*/

static void set_kicker_address_encoder(uint8_t us_kicker_address)
{
	gl_us_kicker_address_encoder = us_kicker_address ;
	return ;
}
/*-----------------------------------------------------------*/

static uint8_t get_kicker_address_load(void)
{
	return gl_us_kicker_address_load ;
}
/*-----------------------------------------------------------*/

static void set_kicker_address_load(uint8_t us_kicker_address)
{
	gl_us_kicker_address_load = us_kicker_address ;
	return ;
}
/*-----------------------------------------------------------*/

static uint8_t get_kicker_address_onoff(void)
{
	return gl_us_kicker_address_onoff ;
}
/*-----------------------------------------------------------*/

static void set_kicker_address_onoff(uint8_t us_kicker_address)
{
	gl_us_kicker_address_onoff = us_kicker_address ;
	return ;
}
/*-----------------------------------------------------------*/

static uint8_t get_kicker_address_reset(void)
{
	return gl_us_kicker_address_reset ;
}
/*-----------------------------------------------------------*/

static void set_kicker_address_reset(uint8_t us_kicker_address)
{
	gl_us_kicker_address_reset = us_kicker_address ;
	return ;
}
/*-----------------------------------------------------------*/

static void set_wyclef_status(uint8_t status)
{
	gl_wyclef_status = status ;
}
/*-----------------------------------------------------------*/

static uint8_t get_wyclef_status(void)
{
	return gl_wyclef_status ;
}
/*-----------------------------------------------------------*/

static void has_encoder_unit_changed(uint8_t us_kick_add)
{
	uint16_t* uw_tempcount = get_active_kicker_count(us_kick_add) ;
	
	uint32_t lcd_unit = *uw_tempcount%10 ; // *uw_tempcount%10 ;
	
	if((((gl_uw_kicker_cvs[us_kick_add-1]) & 0x000000FF) >> 0) != lcd_unit) //Has units changed?
	{
		gl_uw_kicker_cvs[us_kick_add-1] &= 0xFFFFFF00 ; //clear previous units value
		gl_uw_kicker_cvs[us_kick_add-1] |= (lcd_unit << 0) ; //save new value
		gl_b_data_changed[DISP_UNIT] = true ;
		xSemaphoreGive(xLcdCommand_CountingSemaphore) ;	//unblock vTask_SendCommandToLcd_setKV
		
		/*vAssemble_LCDWriteCommand_digits() ; //assemble the command
		
		//put command in queue, hence unblocking the gatekeeper task
		//a block time is not specified as there should always be space in the queue
		xQueueSendToBack(xLCDCommand_queue, pcUnit_testing, 0) ;
		//xQueueSendToBack(xLCDCommand_queue, &gl_uc_usart0_lcd_command_buffer, 0) ;
		//*/
	}	
}
/*-----------------------------------------------------------*/

static void has_encoder_tens_changed(uint8_t us_kick_add)
{
	uint16_t* uw_tempcount = get_active_kicker_count(us_kick_add) ;
	
	uint32_t lcd_ten  = (*uw_tempcount/10)%10 ;
	
	if((((gl_uw_kicker_cvs[us_kick_add-1]) & 0x0000FF00) >> 8) != lcd_ten) //Has tens value changed?
	{
		gl_uw_kicker_cvs[us_kick_add-1] &= 0xFFFF00FF ; //clear previous tens value
		gl_uw_kicker_cvs[us_kick_add-1] |= (lcd_ten << 8) ; //save new value
		gl_b_data_changed[DISP_TENS] = true ;
		xSemaphoreGive(xLcdCommand_CountingSemaphore) ;	//unblock vTask_SendCommandToLcd_setKV
		/*vAssemble_LCDWriteCommand_digits() ; //assemble the command
		
		//put command in queue, hence unblocking the gatekeeper task
		//a block time is not specified as there should always be space in the queue
		xQueueSendToBack(xLCDCommand_queue, &gl_uc_usart0_lcd_command_buffer, 0) ;
		//*/
	}
}
/*-----------------------------------------------------------*/

static void has_encoder_hund_changed(uint8_t us_kick_add)
{
	uint16_t* uw_tempcount = get_active_kicker_count(us_kick_add) ;

	uint32_t lcd_hund = (*uw_tempcount/100)%10 ;
	
	if((((gl_uw_kicker_cvs[us_kick_add-1]) & 0x00FF0000) >> 16) != lcd_hund) //Has hundreds value changed?
	{			
		gl_uw_kicker_cvs[us_kick_add-1] &= 0xFF00FFFF ; //clear previous hundreds value
		gl_uw_kicker_cvs[us_kick_add-1] |= (lcd_hund << 16) ; //save new value
		gl_b_data_changed[DISP_HUND] = true ;
		xSemaphoreGive(xLcdCommand_CountingSemaphore) ; //unblock vTask_SendCommandToLcd_setKV
		/*vAssemble_LCDWriteCommand_digits() ; //assemble the command
		
		//put command in queue, hence unblocking the gatekeeper task
		//a block time is not specified as there should always be space in the queue
		xQueueSendToBack(xLCDCommand_queue, &gl_uc_usart0_lcd_command_buffer, 0) ;
		//*/
	}
}
/*-----------------------------------------------------------*/

static void has_encoder_thou_changed(uint8_t us_kick_add)
{
	uint16_t* uw_tempcount = get_active_kicker_count(us_kick_add) ;
	
	uint32_t lcd_thou = (*uw_tempcount/1000)%10 ;;	
	
	if((((gl_uw_kicker_cvs[us_kick_add-1]) & 0xFF000000) >> 24) != lcd_thou) //Has thousand value changed?
	{	
		gl_uw_kicker_cvs[us_kick_add-1] &= 0x00FFFFFF ; //clear previous value
		gl_uw_kicker_cvs[us_kick_add-1] |= (lcd_thou << 24) ; //save new value
		gl_b_data_changed[DISP_THOU] = true ;
		xSemaphoreGive(xLcdCommand_CountingSemaphore) ; //unblock vTask_SendCommandToLcd_setKV
		/*vAssemble_LCDWriteCommand_digits() ; //assemble the command 
		
		//put command in queue, hence unblocking the gatekeeper task
		//a block time is not specified as there should always be space in the queue
		xQueueSendToBack(xLCDCommand_queue, &gl_uc_usart0_lcd_command_buffer, 0) ; 
		//*/
	}
}
/*-----------------------------------------------------------*/

static uint8_t lcd_display_count(uint8_t us_kicker_address, uint8_t us_count_val, uint8_t char_offset, 
                                 uint8_t* lcd_count_buffer, uint8_t lcd_colour, uint8_t lcd_display_line)
{
	/* Updates the buffer that is sent to the lcd screen
	 The buffer used is gl_uc_usart0_lcd_command_buffer
	 and the size of the transmission is set by gl_lcd_command_length */
	
	uint8_t us_checksum ;
	uint8_t us_digit_descriptor ;
	uint8_t i ;
	
	switch(char_offset){
		case disp_100:
		us_digit_descriptor = 8 ;
		break ;
		case disp_101:
		us_digit_descriptor = 7 ;
		break ;
		case disp_102:
		us_digit_descriptor = 6 ;
		break ;
		case disp_103:
		us_digit_descriptor = 5 ;
		break ;
		case disp_104:
		us_digit_descriptor = 4 ;
		break ;
	}
	
	//set up pointer to the desired buffer address
	uint8_t* ptr_temp_buff = lcd_count_buffer ;
		
	//assign co-ords, char offset selects the x-position of the digit being displayed
	lcd_display_set_coords_text(us_kicker_address, ptr_temp_buff, char_offset, lcd_display_line) ; //lcd_disp_line	LCD_TEXT_COUNT
	
	//put ascii value in buffer
	ptr_temp_buff[10] = (us_count_val + ASCII_OFFSET) ;
	
	lcd_set_font_colour(ptr_temp_buff, lcd_colour) ;
	
	//update addressed kicker command string, ready for serial tx
	update_kicker_string_kV(us_kicker_address, us_digit_descriptor, (us_count_val + ASCII_OFFSET)) ;
	
	//Now get the checksum
	us_checksum = lcd_chksum(ptr_temp_buff, COUNTDISP_BUFFER_SIZE) ;
	
	ptr_temp_buff[COUNTDISP_BUFFER_SIZE-2] = us_checksum ;
	
	//assemble the command ready to be sent
	for(i=0; i<COUNTDISP_BUFFER_SIZE; i++)
	{
		gl_uc_usart0_lcd_command_buffer[i] = ptr_temp_buff[i] ;
	}
	//gl_lcd_command_length = COUNTDISP_BUFFER_SIZE ;
	
	return KUI_function_success ;
}
/*-----------------------------------------------------------*/

static uint8_t lcd_display_readvals(uint8_t us_kicker_address, uint8_t us_count_val, uint8_t char_offset, 
                                    uint8_t* lcd_count_buffer, uint8_t lcd_colour, uint8_t lcd_display_line, void* pvParameters)
{
	/* Updates the buffer that is sent to the lcd screen
	 The buffer used is gl_uc_usart0_lcd_command_buffer
	 and the size of the transmission is set by gl_lcd_command_length */
	
	uint8_t us_checksum ;
	uint8_t us_digit_descriptor ;
	uint8_t i ;
	
	switch(char_offset){
		case disp_100:
		us_digit_descriptor = 8 ;
		break ;
		case disp_101:
		us_digit_descriptor = 7 ;
		break ;
		case disp_102:
		us_digit_descriptor = 6 ;
		break ;
		case disp_103:
		us_digit_descriptor = 5 ;
		break ;
		case disp_104:
		us_digit_descriptor = 4 ;
		break ;
	}
	
	//set up pointer to the desired buffer address
	uint8_t* ptr_temp_buff = lcd_count_buffer ;
		
	//assign co-ords, char offset selects the x-position of the digit being displayed
	lcd_display_set_coords_text(us_kicker_address, ptr_temp_buff, char_offset, lcd_display_line) ; //lcd_disp_line	LCD_TEXT_COUNT
	
	//put ascii value in buffer
	ptr_temp_buff[10] = (us_count_val + ASCII_OFFSET) ;
	
	lcd_set_font_colour(ptr_temp_buff, lcd_colour) ;
	
	//update addressed kicker command string, ready for serial tx
	//update_kicker_string_kV(us_kicker_address, us_digit_descriptor, (us_count_val + ASCII_OFFSET)) ;
	
	//Now get the checksum
	us_checksum = lcd_chksum(ptr_temp_buff, COUNTDISP_BUFFER_SIZE) ;
	
	ptr_temp_buff[COUNTDISP_BUFFER_SIZE-2] = us_checksum ;
	
	//assemble the command ready to be sent
	for(i=0; i<COUNTDISP_BUFFER_SIZE; i++)
	{
		gl_uc_usart0_lcd_command_buffer[i] = ptr_temp_buff[i] ;
	}
	//gl_lcd_command_length = COUNTDISP_BUFFER_SIZE ;
	
	usart0_write_string_standard((freertos_usart_if)pvParameters, &gl_uc_usart0_lcd_command_buffer, COUNTDISP_BUFFER_SIZE) ;
	
	return KUI_function_success ;
}
/*-----------------------------------------------------------*/

static void lcd_display_set_coords_text(uint8_t us_kicker_address, uint8_t *ptr_disp_array, uint8_t us_char_offset, uint8_t lcd_text_opt)
{
	/* char_offset setting :-
	   0x00 for SI unit display 
	   0x10 for 10^0
	   0x20 for 10^1
	   0x30 for decimal point
	   0x40 for 10^2
	   0x50 for 10^3 */
	
	uint8_t us_hx, us_lx, us_hy, us_ly ;
	
	uint16_t y_coord ;
	
	Bool b_neg_kick = false ;
	
	if(!(us_kicker_address % 2)){ //kicker address is even number, i.e. negative kicker selected
		b_neg_kick = true ;
	}
	
	switch(lcd_text_opt)
	{
		case LCD_TEXT_COUNT:
			if(b_neg_kick)
				y_coord = kneg_yval ;
			else
				y_coord = kpos_yval ;
			break ;
			
		case LCD_TEXT_READ_V:
			if(b_neg_kick)
				y_coord = kneg_read_v_y ;
			else
				y_coord = kpos_read_v_y ;
			break ;
			
		case LCD_TEXT_READ_A:
			if(b_neg_kick)
				y_coord = kneg_read_a_y ;
			else
				y_coord = kpos_read_a_y ;
			break ;
			
		case LCD_TEXT_READ_P:
			if(b_neg_kick)
				y_coord = kneg_read_p_y ;
			else
				y_coord = kpos_read_p_y ;
			break ;
	}
	
	//switch kicker address and set x,y coords 
	switch(us_kicker_address)
	{
		case 1 :
			us_hx = (0xFF00 & (k1_unitx - us_char_offset)) >> 8 ;
			us_lx = (0x00FF & (k1_unitx - us_char_offset)) ;
			us_hy = (0xFF00 & y_coord) >> 8 ;
			us_ly = (0x00FF & y_coord) ; //y co-ord always the same for selected kicker
			break ;
		
		case 2 :
			us_hx = (0xFF00 & (k1_unitx - us_char_offset)) >> 8 ;
			us_lx = (0x00FF & (k1_unitx - us_char_offset)) ;
			us_hy = (0xFF00 & y_coord) >> 8 ;
			us_ly = (0x00FF & y_coord) ; //y co-ord always the same for selected kicker
			break ;
		
		case 3 :
			us_hx = (0xFF00 & (k2_unitx - us_char_offset)) >> 8 ;
			us_lx = (0x00FF & (k2_unitx - us_char_offset)) ;
			us_hy = (0xFF00 & y_coord) >> 8 ;
			us_ly = (0x00FF & y_coord) ; //y co-ord always the same for selected kicker
			break ;
		
		case 4 :
			us_hx = (0xFF00 & (k2_unitx - us_char_offset)) >> 8 ;
			us_lx = (0x00FF & (k2_unitx - us_char_offset)) ;
			us_hy = (0xFF00 & y_coord) >> 8 ;
			us_ly = (0x00FF & y_coord) ; //y co-ord always the same for selected kicker
			break ;
		
		case 5 :
			us_hx = (0xFF00 & (k3_unitx - us_char_offset)) >> 8 ;
			us_lx = (0x00FF & (k3_unitx - us_char_offset)) ;
			us_hy = (0xFF00 & y_coord) >> 8 ;
			us_ly = (0x00FF & y_coord) ; //y co-ord always the same for selected kicker
			break ;
		
		case 6 :		
			us_hx = (0xFF00 & (k3_unitx - us_char_offset)) >> 8 ;
			us_lx = (0x00FF & (k3_unitx - us_char_offset)) ;
			us_hy = (0xFF00 & y_coord) >> 8 ;
			us_ly = (0x00FF & y_coord) ; //y co-ord always the same for selected kicker
			break ;
		
		default:
			return ;
			break ;
	}
	
	ptr_disp_array[4] = us_hx ;
	ptr_disp_array[5] = us_lx ;
	ptr_disp_array[6] = us_hy ;
	ptr_disp_array[7] = us_ly ;
	return ;
}
/*-----------------------------------------------------------*/

static void lcd_set_font_colour(uint8_t* ptr_buffer, uint8_t us_font_colour)
{
	//change font colour, write the relevant bit
	//check checksum function to see how handling of pointer to buffer is done
	ptr_buffer[8] = us_font_colour ;
	
	return ;
}
/*-----------------------------------------------------------*/

static void update_kicker_string_kV(uint8_t us_kicker_address, uint8_t us_digit_descriptor, uint8_t uc_ascii_value)
{
	switch(us_kicker_address)
	{
		case 1:
		gl_uc_pdc_1kV_tx[us_digit_descriptor] = uc_ascii_value ;
		break ;
		
		case 2:
		gl_uc_pdc_2kV_tx[us_digit_descriptor] = uc_ascii_value ;
		break ;

		case 3:
		gl_uc_pdc_3kV_tx[us_digit_descriptor] = uc_ascii_value ;
		break ;

		case 4:
		gl_uc_pdc_4kV_tx[us_digit_descriptor] = uc_ascii_value ;
		break ;

		case 5:
		gl_uc_pdc_5kV_tx[us_digit_descriptor] = uc_ascii_value ;
		break ;

		case 6:
		gl_uc_pdc_6kV_tx[us_digit_descriptor] = uc_ascii_value ;
		break ;
	}
	return ;
}
/*-----------------------------------------------------------*/

static int get_string_address_txKV(uint8_t us_kicker_address)
{
	switch(us_kicker_address)
	{
		case 1:
		return &gl_uc_pdc_1kV_tx ;
		break ;
		
		case 2:
		return &gl_uc_pdc_2kV_tx ;
		break ;

		case 3:
		return &gl_uc_pdc_3kV_tx ;
		break ;
		
		case 4:
		return &gl_uc_pdc_4kV_tx ;
		break ;
		
		case 5:
		return &gl_uc_pdc_5kV_tx ;
		break ;

		case 6:
		return &gl_uc_pdc_6kV_tx ;
		break ;
		
		default:
		return NULL ;
		break ;
	}
}
/*-----------------------------------------------------------*/

static int get_string_address_txKVRemote(void)
{
	return &gl_uc_Tx_kV_remote ;
}
/*-----------------------------------------------------------*/

static int get_string_address_txON(uint8_t us_kicker_address)
{
	return &gl_uc_pdc_hvon_tx ;
}
/*-----------------------------------------------------------*/

static int get_string_address_txOFF(uint8_t us_kicker_address)
{
	return &gl_uc_pdc_hvoff_tx ;
}
/*-----------------------------------------------------------*/

static int get_string_address_txRESET(uint8_t us_kicker_address)
{
	return &gl_uc_pdc_hvreset_tx ;
}
/*-----------------------------------------------------------*/

static int get_string_address_read_status(uint8_t us_kicker_address)
{
	switch(us_kicker_address)
	{
		case 1:
		return &gl_uc_read_status1 ;
		break ;
		
		case 2:
		return &gl_uc_read_status2 ;
		break ;

		case 3:
		return &gl_uc_read_status3 ;
		break ;
		
		case 4:
		return &gl_uc_read_status4 ;
		break ;
		
		case 5:
		return &gl_uc_read_status5 ;
		break ;

		case 6:
		return &gl_uc_read_status6 ;
		break ;
		
		default:
		return NULL ;
		break ;
	}
}
/*-----------------------------------------------------------*/

static int get_string_address_read_output_volts(uint8_t us_kicker_address)
{
	switch(us_kicker_address)
	{
		case 1:
		return &gl_uc_read_output_volts1 ;
		break ;
		
		case 2:
		return &gl_uc_read_output_volts2 ;
		break ;

		case 3:
		return &gl_uc_read_output_volts3 ;
		break ;
		
		case 4:
		return &gl_uc_read_output_volts4 ;
		break ;
		
		case 5:
		return &gl_uc_read_output_volts5 ;
		break ;

		case 6:
		return &gl_uc_read_output_volts6 ;
		break ;
		
		default:
		return NULL ;
		break ;
	}
}
/*-----------------------------------------------------------*/

static int get_string_address_read_demand_volts(uint8_t us_kicker_address)
{
	switch(us_kicker_address)
	{
		case 1:
		return &gl_uc_read_demand_volts1 ;
		break ;
		
		case 2:
		return &gl_uc_read_demand_volts2 ;
		break ;

		case 3:
		return &gl_uc_read_demand_volts3 ;
		break ;
		
		case 4:
		return &gl_uc_read_demand_volts4 ;
		break ;
		
		case 5:
		return &gl_uc_read_demand_volts5 ;
		break ;

		case 6:
		return &gl_uc_read_demand_volts6 ;
		break ;
		
		default:
		return NULL ;
		break ;
	}
}
/*-----------------------------------------------------------*/

static int get_string_address_read_amps(uint8_t us_kicker_address)
{
	switch(us_kicker_address)
	{
		case 1:
		return &gl_uc_read_amps1 ;
		break ;
		
		case 2:
		return &gl_uc_read_amps2 ;
		break ;

		case 3:
		return &gl_uc_read_amps3 ;
		break ;
		
		case 4:
		return &gl_uc_read_amps4 ;
		break ;
		
		case 5:
		return &gl_uc_read_amps5 ;
		break ;

		case 6:
		return &gl_uc_read_amps6 ;
		break ;
		
		default:
		return NULL ;
		break ;
	}
}
/*-----------------------------------------------------------*/

static int get_string_address_read_power(uint8_t us_kicker_address)
{
	switch(us_kicker_address)
	{
		case 1:
		return &gl_uc_read_power1 ;
		break ;
		
		case 2:
		return &gl_uc_read_power2 ;
		break ;

		case 3:
		return &gl_uc_read_power3 ;
		break ;
		
		case 4:
		return &gl_uc_read_power4 ;
		break ;
		
		case 5:
		return &gl_uc_read_power5 ;
		break ;

		case 6:
		return &gl_uc_read_power6 ;
		break ;
		
		default:
		return NULL ;
		break ;
	}
}
/*-----------------------------------------------------------*/

static uint8_t lcd_chksum(uint8_t* ptr_buff, uint8_t sizeof_buff)
{
	uint8_t us_checksum = 0 ;
	uint16_t uw_sumtotal = 0 ;
		
	//add array components together
	for(int i=2; i<(sizeof_buff-2); i++)
	{
		if(i<sizeof_buff)
			uw_sumtotal += ptr_buff[i] ;
	}
	
	//mask off lower byte and return
	us_checksum = uw_sumtotal & 0x00FF ;
	
	return us_checksum ;// chksum ;
}
/*-----------------------------------------------------------*/

static void kicker_set_locrem_status(uint8_t us_kicker_address)
{
	//added update to locrem buffer for reporting status to the CPS crate
	//toggle local/remote status for selected kicker address
	switch(us_kicker_address)
	{
		case 1:
		if((gl_us_loccrem & 0x01) >> 0){ //Kicker1 is in remote, b0=1
			gl_us_loccrem &= 0x3E ; //set b0=0 (local)
			gl_uc_locrem_buffer[us_kicker_address+1] = '0' ;
		}
		else{
			gl_us_loccrem |= 0x01 ;//set b0=1 (remote)
			gl_uc_locrem_buffer[us_kicker_address+1] = '1' ;
		}
		break ;
		
		case 2:
		if((gl_us_loccrem & 0x02) >> 1){ //Kicker2 is in remote, b1=1
			gl_us_loccrem &= 0x3D ; //set b1=0 (local)
			gl_uc_locrem_buffer[us_kicker_address+1] = '0' ;
		}
		else{
			gl_us_loccrem |= 0x02 ;//set b1=1 (remote)
			gl_uc_locrem_buffer[us_kicker_address+1] = '1' ;
		}
		break ;

		case 3:
		if((gl_us_loccrem & 0x04) >> 2){ //Kicker1 is in remote, b2=1
			gl_us_loccrem &= 0x3B ; //set b2=0 (local)
			gl_uc_locrem_buffer[us_kicker_address+1] = '0' ;
		}
		else{
			gl_us_loccrem |= 0x04 ;//set b2=1 (remote)
			gl_uc_locrem_buffer[us_kicker_address+1] = '1' ;
		}
		break ;
		
		case 4:
		if((gl_us_loccrem & 0x08) >> 3){ //Kicker1 is in remote, b3=1
			gl_us_loccrem &= 0x37 ; //set b3=0 (local)
			gl_uc_locrem_buffer[us_kicker_address+1] = '0' ;
		}
		else{
			gl_us_loccrem |= 0x08 ;//set b3=1 (remote)
			gl_uc_locrem_buffer[us_kicker_address+1] = '1' ;
		}
		break ;
		
		case 5:
		if((gl_us_loccrem & 0x10) >> 4){ //Kicker1 is in remote, b4=1
			gl_us_loccrem &= 0x2F ; //set b4=0 (local)
			gl_uc_locrem_buffer[us_kicker_address+1] = '0' ;
		}
		else{
			gl_us_loccrem |= 0x10 ;//set b4=1 (remote)
			gl_uc_locrem_buffer[us_kicker_address+1] = '1' ;
		}
		break ;

		case 6:
		if((gl_us_loccrem & 0x20) >> 5){ //Kicker1 is in remote, b5=1
			gl_us_loccrem &= 0x1F ; //set b5=0 (local)
			gl_uc_locrem_buffer[us_kicker_address+1] = '0' ;
		}
		else{
			gl_us_loccrem |= 0x20 ;//set b5=1 (remote)
			gl_uc_locrem_buffer[us_kicker_address+1] = '1' ;
		}
		break ;
		
		default:
		//error handling
		break ;
	}
	
	//display the local/remote status on the LCD, unblock lcd write task?
	
	//pdc_uart0_lcd_display_localbutton(us_kicker_address, get_locrem_status(us_kicker_address)) ;//display button on lcd
	//pdc_uart0_lcd_display_onoffbutton(us_kicker_address, get_locrem_status(us_kicker_address)) ;//display on off as test
}
/*-----------------------------------------------------------*/


static int8_t kicker_get_locrem_status(uint8_t us_kicker_address)
{
	
	switch(us_kicker_address)
	{
		case 1:
		if((gl_us_loccrem & 0x01) >> 0)
		return REMOTE ;
		else
		return LOCAL ;
		break ;
		
		case 2:
		if((gl_us_loccrem & 0x02) >> 1)
		return REMOTE ;
		else
		return LOCAL ;
		break ;
		
		case 3:
		if((gl_us_loccrem & 0x04) >> 2)
		return REMOTE ;
		else
		return LOCAL ;
		break ;
		
		case 4:
		if((gl_us_loccrem & 0x08) >> 3)
		return REMOTE ;
		else
		return LOCAL ;
		break ;
		
		case 5:
		if((gl_us_loccrem & 0x10) >> 4)
		return REMOTE ;
		else
		return LOCAL ;
		break ;
		
		case 6:
		if((gl_us_loccrem & 0x20) >> 5)
		return REMOTE ;
		else
		return LOCAL ;
		break ;
		
		default:
		return rc_Error1 ;
	}
}
/*-----------------------------------------------------------*/


static void kicker_set_psuhealthy_status(uint8_t us_kicker_address, uint8_t psuhealthy_status)
{
	//set hvon status for selected kicker address
	switch(us_kicker_address)
	{
		case 1:
			if(psuhealthy_status == PSU_HEALTHY_ACTIVE){
				gl_us_psuhealthy |= 0x01 ;
			}
			else{
				gl_us_psuhealthy &= 0x3E ;
			}
			break ;
			
		/*case 2:
			if(psuhealthy_status == PSU_HEALTHY_ACTIVE){
				gl_us_psuhealthy |= 0x02 ;
			}
			else{
				gl_us_psuhealthy &= 0x3D ;
			}
			break ;*/
			
		case 3:
			if(psuhealthy_status == PSU_HEALTHY_ACTIVE){
				gl_us_psuhealthy |= 0x04 ;
			}
			else{
				gl_us_psuhealthy &= 0x3B ;
			}
			break ;			
		
		/*case 4:
			if(psuhealthy_status == PSU_HEALTHY_ACTIVE){
				gl_us_psuhealthy |= 0x08 ;
			}
			else{
				gl_us_psuhealthy &= 0x37 ;
			}
			break ;*/
		
		case 5:
			if(psuhealthy_status == PSU_HEALTHY_ACTIVE){
				gl_us_psuhealthy |= 0x10 ;
			}
			else{
				gl_us_psuhealthy &= 0x2F ;
			}
			break ;
			
		/*case 6:
			if(psuhealthy_status == PSU_HEALTHY_ACTIVE){
				gl_us_psuhealthy |= 0x20 ;
			}
			else{
				gl_us_psuhealthy &= 0x1F ;
			}
			break ;*/
		
		default:
			//error handling
			break ;
	}
}
/*-----------------------------------------------------------*/

static int8_t kicker_get_psuhealthy_status(uint8_t us_kicker_address)
{
	
	switch(us_kicker_address)
	{
		case 1:
		case 2:
		if((gl_us_psuhealthy & 0x01) >> 0)
		return PSU_HEALTHY_ACTIVE ;
		else
		return PSU_HEALTHY_INACTIVE ;
		break ;
		
		/*case 2:
		if((gl_us_psuhealthy & 0x02) >> 1)
		return PSU_HEALTHY_ACTIVE ;
		else
		return PSU_HEALTHY_INACTIVE ;
		break ;*/
		
		case 3:
		case 4:
		if((gl_us_psuhealthy & 0x04) >> 2)
		return PSU_HEALTHY_ACTIVE ;
		else
		return PSU_HEALTHY_INACTIVE ;
		break ;
		
		/*case 4:
		if((gl_us_psuhealthy & 0x08) >> 3)
		return PSU_HEALTHY_ACTIVE ;
		else
		return PSU_HEALTHY_INACTIVE ;
		break ;*/
		
		case 5:
		case 6:
		if((gl_us_psuhealthy & 0x10) >> 4)
		return PSU_HEALTHY_ACTIVE ;
		else
		return PSU_HEALTHY_INACTIVE ;
		break ;
		
		/*case 6:
		if((gl_us_psuhealthy & 0x20) >> 5)
		return PSU_HEALTHY_ACTIVE ;
		else
		return PSU_HEALTHY_INACTIVE ;
		break ;*/
		
		default:
		return rc_Error1 ;
	}
}
/*-----------------------------------------------------------*/

static void kicker_set_hvon_status(uint8_t us_kicker_address, uint8_t hvon_status)
{
	//set hvon status for selected kicker address
	switch(us_kicker_address)
	{
		case 1:
		if(hvon_status == HVON_ACTIVE){
			gl_us_hvon |= 0x01 ;
		}
		else{
			gl_us_hvon &= 0x3E ;
		}
		break ;
		
		case 2:
		if(hvon_status == HVON_ACTIVE){
			gl_us_hvon |= 0x02 ;
		}
		else{
			gl_us_hvon &= 0x3D ;
		}
		break ;
		
		case 3:
		if(hvon_status == HVON_ACTIVE){
			gl_us_hvon |= 0x04 ;
		}
		else{
			gl_us_hvon &= 0x3B ;
		}
		break ;
		
		case 4:
		if(hvon_status == HVON_ACTIVE){
			gl_us_hvon |= 0x08 ;
		}
		else{
			gl_us_hvon &= 0x37 ;
		}
		break ;
		
		case 5:
		if(hvon_status == HVON_ACTIVE){
			gl_us_hvon |= 0x10 ;
		}
		else{
			gl_us_hvon &= 0x2F ;
		}
		break ;
		
		case 6:
		if(hvon_status == HVON_ACTIVE){
			gl_us_hvon |= 0x20 ;
		}
		else{
			gl_us_hvon &= 0x1F ;
		}
		break ;
		
		default:
		//error handling
		break ;
	}
}
/*-----------------------------------------------------------*/

static int8_t kicker_get_hvon_status(uint8_t us_kicker_address)
{
	
	switch(us_kicker_address)
	{
		case 1:
		if((gl_us_hvon & 0x01) >> 0)
		return HVON_ACTIVE ;
		else
		return HVON_INACTIVE ;
		break ;
		
		case 2:
		if((gl_us_hvon & 0x02) >> 1)
		return HVON_ACTIVE ;
		else
		return HVON_INACTIVE ;
		break ;
		
		case 3:
		if((gl_us_hvon & 0x04) >> 2)
		return HVON_ACTIVE ;
		else
		return HVON_INACTIVE ;
		break ;
		
		case 4:
		if((gl_us_hvon & 0x08) >> 3)
		return HVON_ACTIVE ;
		else
		return HVON_INACTIVE ;
		break ;
		
		case 5:
		if((gl_us_hvon & 0x10) >> 4)
		return HVON_ACTIVE ;
		else
		return HVON_INACTIVE ;
		break ;
		
		case 6:
		if((gl_us_hvon & 0x20) >> 5)
		return HVON_ACTIVE ;
		else
		return HVON_INACTIVE ;
		break ;
		
		default:
		return rc_Error1 ;
	}
}
/*-----------------------------------------------------------*/

static int8_t kicker_get_load_status(uint8_t us_kicker_address)
{
	/* This function returns the status of the Load button.
	   The status returned is what should happen next to the Load button.
	   
	   e.g. The Load button is pressed - a BUTTON_DOWN status will be returned. */
	
	
	switch(us_kicker_address)
	{
		case 1:
		if((gl_us_load & 0x01) >> 0)
		return BUTTON_DOWN ;
		else
		return BUTTON_UP ;
		break ;
		
		case 2:
		if((gl_us_load & 0x02) >> 1)
		return BUTTON_DOWN ;
		else
		return BUTTON_UP ;
		break ;
		
		case 3:
		if((gl_us_load & 0x04) >> 2)
		return BUTTON_DOWN ;
		else
		return BUTTON_UP ;
		break ;
		
		case 4:
		if((gl_us_load & 0x08) >> 3)
		return BUTTON_DOWN ;
		else
		return BUTTON_UP ;
		break ;
		
		case 5:
		if((gl_us_load & 0x10) >> 4)
		return BUTTON_DOWN ;
		else
		return BUTTON_UP ;
		break ;
		
		case 6:
		if((gl_us_load & 0x20) >> 5)
		return BUTTON_DOWN ;
		else
		return BUTTON_UP ;
		break ;
		
		default:
		return rc_Error1 ;
		}//*/
	}
/*-----------------------------------------------------------*/

static void lcd_display_set_coords_locrem(uint8_t us_kicker_address, Bool b_local_mode, uint8_t *ptr_disp_array)
{
	int i ; 
	uint8_t us_hx, us_lx, us_hy, us_ly ;
	
	//copy the template for local/remote button 
	for( i=0; i<LOCREM_BUTTON_BUFFER_SIZE; i++)
	{
		ptr_disp_array[i] = gl_usart0_locrem_template[i] ;
	}	
	
	//switch kicker address and set x,y coords
	switch(us_kicker_address)
	{
		case 1 :
		us_hx = (0xFF00 & local1_x) >> 8 ;
		us_lx = (0x00FF & local1_x) ;
		us_hy = (0xFF00 & local_pos_y) >> 8 ;
		us_ly = (0x00FF & local_pos_y) ; //y co-ord always the same for selected kicker
		break ;

		case 2 :
		us_hx = (0xFF00 & local1_x) >> 8 ;
		us_lx = (0x00FF & local1_x) ;
		us_hy = (0xFF00 & local_neg_y) >> 8 ;
		us_ly = (0x00FF & local_neg_y) ; //y co-ord always the same for selected kicker
		break ;
		
		case 3 :
		us_hx = (0xFF00 & local2_x) >> 8 ;
		us_lx = (0x00FF & local2_x) ;
		us_hy = (0xFF00 & local_pos_y) >> 8 ;
		us_ly = (0x00FF & local_pos_y) ; //y co-ord always the same for selected kicker
		break ;
		
		case 4 :
		us_hx = (0xFF00 & local2_x) >> 8 ;
		us_lx = (0x00FF & local2_x) ;
		us_hy = (0xFF00 & local_neg_y) >> 8 ;
		us_ly = (0x00FF & local_neg_y) ; //y co-ord always the same for selected kicker
		break ;
		
		case 5 :
		us_hx = (0xFF00 & local3_x) >> 8 ;
		us_lx = (0x00FF & local3_x) ;
		us_hy = (0xFF00 & local_pos_y) >> 8 ;
		us_ly = (0x00FF & local_pos_y) ; //y co-ord always the same for selected kicker
		break ;
		
		case 6 :
		us_hx = (0xFF00 & local3_x) >> 8 ;
		us_lx = (0x00FF & local3_x) ;
		us_hy = (0xFF00 & local_neg_y) >> 8 ;
		us_ly = (0x00FF & local_neg_y) ; //y co-ord always the same for selected kicker
		break ;
		
		default:
		return ;
		break ;
	}
	
	ptr_disp_array[4] = us_hx ;
	ptr_disp_array[5] = us_lx ;
	ptr_disp_array[6] = us_hy ;
	ptr_disp_array[7] = us_ly ;
	
	if(!b_local_mode){
		ptr_disp_array[9] = G_LOCAL_BUTTON ; //local
	}
	else{ //button down
		ptr_disp_array[9] = G_REMOTE_BUTTON ; //remote
	}
	
	return ;
}
/*-----------------------------------------------------------*/


static void lcd_display_set_coords_load(uint8_t us_kicker_address, Bool b_load_status, uint8_t *ptr_disp_array)
{
	int i ;
	uint8_t us_hx, us_lx, us_hy, us_ly ;
	
	//copy the template for local/remote button
	for( i=0; i<LOAD_BUTTON_BUFFER_SIZE; i++)
	{
		ptr_disp_array[i] = gl_usart0_load_template[i] ;
	}
	
	//switch kicker address and set x,y coords
	switch(us_kicker_address)
	{
		case 1 :
		us_hx = (0xFF00 & load1_x) >> 8 ;
		us_lx = (0x00FF & load1_x) ;
		us_hy = (0xFF00 & load_pos_y) >> 8 ;
		us_ly = (0x00FF & load_pos_y) ; //y co-ord always the same for selected kicker
		break ;

		case 2 :
		us_hx = (0xFF00 & load1_x) >> 8 ;
		us_lx = (0x00FF & load1_x) ;
		us_hy = (0xFF00 & load_neg_y) >> 8 ;
		us_ly = (0x00FF & load_neg_y) ; //y co-ord always the same for selected kicker
		break ;
		
		case 3 :
		us_hx = (0xFF00 & load2_x) >> 8 ;
		us_lx = (0x00FF & load2_x) ;
		us_hy = (0xFF00 & load_pos_y) >> 8 ;
		us_ly = (0x00FF & load_pos_y) ; //y co-ord always the same for selected kicker
		break ;
		
		case 4 :
		us_hx = (0xFF00 & load2_x) >> 8 ;
		us_lx = (0x00FF & load2_x) ;
		us_hy = (0xFF00 & load_neg_y) >> 8 ;
		us_ly = (0x00FF & load_neg_y) ; //y co-ord always the same for selected kicker
		break ;
		
		case 5 :
		us_hx = (0xFF00 & load3_x) >> 8 ;
		us_lx = (0x00FF & load3_x) ;
		us_hy = (0xFF00 & load_pos_y) >> 8 ;
		us_ly = (0x00FF & load_pos_y) ; //y co-ord always the same for selected kicker
		break ;
		
		case 6 :
		us_hx = (0xFF00 & load3_x) >> 8 ;
		us_lx = (0x00FF & load3_x) ;
		us_hy = (0xFF00 & load_neg_y) >> 8 ;
		us_ly = (0x00FF & load_neg_y) ; //y co-ord always the same for selected kicker
		break ;
		
		default:
		return ;
		break ;
	}
	
	ptr_disp_array[4] = us_hx ;
	ptr_disp_array[5] = us_lx ;
	ptr_disp_array[6] = us_hy ;
	ptr_disp_array[7] = us_ly ;
	
	if(b_load_status == BUTTON_DOWN){
		ptr_disp_array[9] = G_LOAD_DOWN ;
	}
	else if (b_load_status == BUTTON_UP){ 
		ptr_disp_array[9] = G_LOAD_UP ;
	}
	
	return ;
}
/*-----------------------------------------------------------*/


static void lcd_display_set_coords_onoff(uint8_t us_kicker_address, Bool b_mode, Bool b_isittouchevent, uint8_t *ptr_disp_array)
{
	int i ;
	uint8_t us_hx, us_lx, us_hy, us_ly ;
	
	//copy the template for local/remote button
	for( i=0; i<ONOFF_BUTTON_BUFFER_SIZE; i++)
	{
		ptr_disp_array[i] = gl_usart0_locrem_template[i] ;
	}
	
	//switch kicker address and set x,y coords
	switch(us_kicker_address)
	{
		case 1 :
		us_hx = (0xFF00 & onoff1_x) >> 8 ;
		us_lx = (0x00FF & onoff1_x) ;
		us_hy = (0xFF00 & onoff_pos_y) >> 8 ;
		us_ly = (0x00FF & onoff_pos_y) ; //y co-ord always the same for selected kicker
		break ;

		case 2 :
		us_hx = (0xFF00 & onoff1_x) >> 8 ;
		us_lx = (0x00FF & onoff1_x) ;
		us_hy = (0xFF00 & onoff_neg_y) >> 8 ;
		us_ly = (0x00FF & onoff_neg_y) ; //y co-ord always the same for selected kicker
		break ;
		
		case 3 :
		us_hx = (0xFF00 & onoff2_x) >> 8 ;
		us_lx = (0x00FF & onoff2_x) ;
		us_hy = (0xFF00 & onoff_pos_y) >> 8 ;
		us_ly = (0x00FF & onoff_pos_y) ; //y co-ord always the same for selected kicker
		break ;
		
		case 4 :
		us_hx = (0xFF00 & onoff2_x) >> 8 ;
		us_lx = (0x00FF & onoff2_x) ;
		us_hy = (0xFF00 & onoff_neg_y) >> 8 ;
		us_ly = (0x00FF & onoff_neg_y) ; //y co-ord always the same for selected kicker
		break ;
		
		case 5 :
		us_hx = (0xFF00 & onoff3_x) >> 8 ;
		us_lx = (0x00FF & onoff3_x) ;
		us_hy = (0xFF00 & onoff_pos_y) >> 8 ;
		us_ly = (0x00FF & onoff_pos_y) ; //y co-ord always the same for selected kicker
		break ;
		
		case 6 :
		us_hx = (0xFF00 & onoff3_x) >> 8 ;
		us_lx = (0x00FF & onoff3_x) ;
		us_hy = (0xFF00 & onoff_neg_y) >> 8 ;
		us_ly = (0x00FF & onoff_neg_y) ; //y co-ord always the same for selected kicker
		break ;
		
		default:
		return ;
		break ;
	}
	
	ptr_disp_array[4] = us_hx ;
	ptr_disp_array[5] = us_lx ;
	ptr_disp_array[6] = us_hy ;
	ptr_disp_array[7] = us_ly ;
	
	if(b_isittouchevent){
		//function has been called from a touch event		
		if(b_mode == HVON_INACTIVE){
			ptr_disp_array[9] = G_ON_DOWN ; //on / down
		}
		else if(b_mode == HVON_ACTIVE){
			ptr_disp_array[9] = G_OFF_DOWN ; //off / down
		}
	}
	else{
		//function has been called from the HVON ISR
		if(b_mode == HVON_INACTIVE){
			ptr_disp_array[9] = G_OFF_UP ; //off button
		}
		else if(b_mode == HVON_ACTIVE){
			ptr_disp_array[9] = G_ON_UP ; //on button
		}
	}

	return ;
}
/*-----------------------------------------------------------*/


static void lcd_display_set_coords_reset(uint8_t us_kicker_address, Bool b_mode, Bool b_isittouchevent, uint8_t *ptr_disp_array)
{
	int i ;
	uint8_t us_hx, us_lx, us_hy, us_ly ;
	
	//copy the template for reset button
	for( i=0; i<RESET_BUTTON_BUFFER_SIZE; i++)
	{
		ptr_disp_array[i] = gl_usart0_locrem_template[i] ;
	}
	
	//switch kicker address and set x,y coords
	switch(us_kicker_address)
	{
		case 1 :
		case 2 :
		us_hx = (0xFF00 & reset1_x) >> 8 ;
		us_lx = (0x00FF & reset1_x) ;
		us_hy = (0xFF00 & reset_y) >> 8 ;
		us_ly = (0x00FF & reset_y) ; //y co-ord always the same for selected kicker
		break ;
		
		case 3 :
		case 4 :
		us_hx = (0xFF00 & reset2_x) >> 8 ;
		us_lx = (0x00FF & reset2_x) ;
		us_hy = (0xFF00 & reset_y) >> 8 ;
		us_ly = (0x00FF & reset_y) ; //y co-ord always the same for selected kicker
		break ;

		case 5 :
		case 6 :
		us_hx = (0xFF00 & reset3_x) >> 8 ;
		us_lx = (0x00FF & reset3_x) ;
		us_hy = (0xFF00 & reset_y) >> 8 ;
		us_ly = (0x00FF & reset_y) ; //y co-ord always the same for selected kicker
		break ;
		
		default:
			return ;
			break ;
	}
	
	ptr_disp_array[4] = us_hx ;
	ptr_disp_array[5] = us_lx ;
	ptr_disp_array[6] = us_hy ;
	ptr_disp_array[7] = us_ly ;
	
	if(b_isittouchevent){
		//function has been called from a touch event
		if(b_mode == PSU_HEALTHY_INACTIVE){
			ptr_disp_array[9] = G_RESET_BAD2 ; //on / down
		}
		else if(b_mode == PSU_HEALTHY_ACTIVE){
			ptr_disp_array[9] = G_RESET_DOWN ; //off / down
		}
	}
	else{
		//function has been called from the PSU HEALTHY ISR
		if(b_mode == PSU_HEALTHY_INACTIVE){
			ptr_disp_array[9] = G_RESET_BAD2 ; //off button
		}
		else if(b_mode == PSU_HEALTHY_ACTIVE){
			ptr_disp_array[9] = G_RESET_UP ; //on button
		}
	}

	return ;
}
/*-----------------------------------------------------------*/

static void set_load(uint8_t us_kicker_address)
{
	switch(us_kicker_address)
	{
		case 1:
			gl_us_load |= 0x01 ;
			break ;
		
		case 2:
			gl_us_load |= 0x02 ;
			break ;
		
		case 3:
			gl_us_load |= 0x04 ;
			break ;
		
		case 4:
			gl_us_load |= 0x08 ;
			break ;
		
		case 5:
			gl_us_load |= 0x10 ;
			break ;
		
		case 6:
			gl_us_load |= 0x20 ;
			break ;
			
		default:
			break ;
	}	
}
/*-----------------------------------------------------------*/

static void clear_load(us_kicker_address)
{
	if(gl_us_load & 0x01){
		gl_us_load &= 0xFE ;
	}
	else if(gl_us_load & 0x02){
		gl_us_load &= 0xFD ;
	}
	else if(gl_us_load & 0x04){
		gl_us_load &= 0xFB ;
	}
	else if(gl_us_load & 0x08){
		gl_us_load &= 0xF7 ;
	}
	else if(gl_us_load & 0x10){
		gl_us_load &= 0xEF ;
	}
	else if(gl_us_load & 0x20){
		gl_us_load &= 0xDF ;
	}	
	return ;
}
/*-----------------------------------------------------------*/

static xRxBuffer_info_t get_receive_buffer_info(uint8_t psu_address, uint8_t command_type)
{
	xRxBuffer_info_t receive_buffer_info ;
	
	uint8_t status = pdPASS ;
	
	receive_buffer_info.nonascii_value = NULL ; //remove after testing
	
	switch(command_type)
	{
		case READ_OUTV:			
			receive_buffer_info.buffer_size = LOCAL_RECEIVE_BUFFER_SIZE_OUTV ;			
			switch(psu_address)
			{
				case 1:
					receive_buffer_info.ptr_to_buffer = &gl_uc_rxbuffer_outv1 ;
					receive_buffer_info.nonascii_value = &gl_ul_outv1 ;
					break ;
					
				case 2:
					receive_buffer_info.ptr_to_buffer = &gl_uc_rxbuffer_outv2 ;
					receive_buffer_info.nonascii_value = &gl_ul_outv2 ;
					break ;
					
				case 3:
					receive_buffer_info.ptr_to_buffer = &gl_uc_rxbuffer_outv3 ;
					receive_buffer_info.nonascii_value = &gl_ul_outv3 ;
					break ;
				
				case 4:
					receive_buffer_info.ptr_to_buffer = &gl_uc_rxbuffer_outv4 ;
					receive_buffer_info.nonascii_value = &gl_ul_outv4 ;
					break ;
					
				case 5:
					receive_buffer_info.ptr_to_buffer = &gl_uc_rxbuffer_outv5 ;
					receive_buffer_info.nonascii_value = &gl_ul_outv5 ;
					break ;
					
				case 6:
					receive_buffer_info.ptr_to_buffer = &gl_uc_rxbuffer_outv6 ;
					receive_buffer_info.nonascii_value = &gl_ul_outv6 ;
					break ;				
					
				default:
					status = pdFAIL ;
					break ;						
			}			
			break ;			
			
		case READ_DEMV:			
			receive_buffer_info.buffer_size = LOCAL_RECEIVE_BUFFER_SIZE_DEMV ;			
			switch(psu_address)
			{
				case 1:
					receive_buffer_info.ptr_to_buffer = &gl_uc_rxbuffer_demv1 ;
					receive_buffer_info.nonascii_value = &gl_ul_demv1 ;
					break ;
					
				case 2:
					receive_buffer_info.ptr_to_buffer = &gl_uc_rxbuffer_demv2 ;
					receive_buffer_info.nonascii_value = &gl_ul_demv2 ;
					break ;
					
				case 3:
					receive_buffer_info.ptr_to_buffer = &gl_uc_rxbuffer_demv3 ;
					receive_buffer_info.nonascii_value = &gl_ul_demv3 ;
					break ;
				
				case 4:
					receive_buffer_info.ptr_to_buffer = &gl_uc_rxbuffer_demv4 ;
					receive_buffer_info.nonascii_value = &gl_ul_demv4 ;
					break ;
					
				case 5:
					receive_buffer_info.ptr_to_buffer = &gl_uc_rxbuffer_demv5 ;
					receive_buffer_info.nonascii_value = &gl_ul_demv5 ;
					break ;
					
				case 6:
					receive_buffer_info.ptr_to_buffer = &gl_uc_rxbuffer_demv6 ;
					receive_buffer_info.nonascii_value = &gl_ul_demv6 ;
					break ;				
					
				default:
					status = pdFAIL ;
					break ;						
			}	
			break ;
			
		case READ_AMPS:			
			receive_buffer_info.buffer_size = LOCAL_RECEIVE_BUFFER_SIZE_AMPS ;			
			switch(psu_address)
			{
				case 1:
					receive_buffer_info.ptr_to_buffer = &gl_uc_rxbuffer_amps1 ;
					receive_buffer_info.nonascii_value = &gl_ul_amps1 ;
					break ;
					
				case 2:
					receive_buffer_info.ptr_to_buffer = &gl_uc_rxbuffer_amps2 ;
					receive_buffer_info.nonascii_value = &gl_ul_amps2 ;
					break ;
					
				case 3:
					receive_buffer_info.ptr_to_buffer = &gl_uc_rxbuffer_amps3 ;
					receive_buffer_info.nonascii_value = &gl_ul_amps3 ;
					break ;
				
				case 4:
					receive_buffer_info.ptr_to_buffer = &gl_uc_rxbuffer_amps4 ;
					receive_buffer_info.nonascii_value = &gl_ul_amps4 ;
					break ;
					
				case 5:
					receive_buffer_info.ptr_to_buffer = &gl_uc_rxbuffer_amps5 ;
					receive_buffer_info.nonascii_value = &gl_ul_amps5 ;
					break ;
					
				case 6:
					receive_buffer_info.ptr_to_buffer = &gl_uc_rxbuffer_amps6 ;
					receive_buffer_info.nonascii_value = &gl_ul_amps6 ;
					break ;				
					
				default:
					status = pdFAIL ;
					break ;						
			}	
			break ;
			
		case READ_POWER:			
			receive_buffer_info.buffer_size = LOCAL_RECEIVE_BUFFER_SIZE_POW ;			
			switch(psu_address)
			{
				case 1:
					receive_buffer_info.ptr_to_buffer = &gl_uc_rxbuffer_pow1 ;
					receive_buffer_info.nonascii_value = &gl_ul_pow1 ;
					break ;
					
				case 2:
					receive_buffer_info.ptr_to_buffer = &gl_uc_rxbuffer_pow2 ;
					receive_buffer_info.nonascii_value = &gl_ul_pow2 ;
					break ;
					
				case 3:
					receive_buffer_info.ptr_to_buffer = &gl_uc_rxbuffer_pow3 ;
					receive_buffer_info.nonascii_value = &gl_ul_pow3 ;
					break ;
				
				case 4:
					receive_buffer_info.ptr_to_buffer = &gl_uc_rxbuffer_pow4 ;
					receive_buffer_info.nonascii_value = &gl_ul_pow4 ;
					break ;
					
				case 5:
					receive_buffer_info.ptr_to_buffer = &gl_uc_rxbuffer_pow5 ;
					receive_buffer_info.nonascii_value = &gl_ul_pow5 ;
					break ;
					
				case 6:
					receive_buffer_info.ptr_to_buffer = &gl_uc_rxbuffer_pow6 ;
					receive_buffer_info.nonascii_value = &gl_ul_pow6 ;
					break ;				
					
				default:
					status = pdFAIL ;
					break ;						
			}	
			break ;
			
		case READ_LOCREM_STATUS:			
			receive_buffer_info.buffer_size = LOCAL_RECEIVE_BUFFER_SIZE_LOCREM ;
			receive_buffer_info.nonascii_value =  NULL ;
			receive_buffer_info.ptr_to_buffer = &gl_uc_locrem_buffer ; //buffer with ALL local/remote status'			
			break ;
			
		default:
			status = pdFAIL ;
			break ;
	}
	
	if (status != pdFAIL){
		//return the valid receive buffer information
		return receive_buffer_info ;	
	}
	else{
		//encountered unknown command, return null values
		receive_buffer_info.buffer_size = NULL ;
		receive_buffer_info.ptr_to_buffer = NULL ;
		receive_buffer_info.nonascii_value = NULL ;
		return receive_buffer_info ;
	}
}
/*-----------------------------------------------------------*/

static void save_nonascii_value(uint8_t command_type, char* ascii_buffer, int* nonascii_value)
{
	/* Convert the received (from HVI) ASCII strings to integer values.
	   Forget decimal points all values are regarded without them, the 
	   decimal points are reinstated when value is displayed on LCD */
	
	Bool b_buffer_data_valid = true ;
	int old_value = *nonascii_value ; //save the original reading
	
	*nonascii_value = 0 ; //reset value first
	
	//check the ascii_buffer chars are in range '0' - '9'
	//restore 'old value' if buffer contains non-numerical values
	switch(command_type)
	{
		case READ_OUTV: //check 3, 4, 6
			if( (ascii_buffer[3] < 0x30) || (ascii_buffer[3] > 0x39) ) //outside of numerical range
			{
				b_buffer_data_valid = false ;
				break ;
			}
			else if( (ascii_buffer[4] < 0x30) || (ascii_buffer[4] > 0x39) ) //outside of numerical range
			{
				b_buffer_data_valid = false ;
				break ;
			}
			else if( (ascii_buffer[6] < 0x30) || (ascii_buffer[6] > 0x39) ) //outside of numerical range
			{
				b_buffer_data_valid = false ;
				break ;
			}
			break ;
			
		case READ_DEMV: //check 4, 5, 7
			if( (ascii_buffer[4] < 0x30) || (ascii_buffer[4] > 0x39) ) //outside of numerical range
			{
				b_buffer_data_valid = false ;
				break ;
			}
			else if( (ascii_buffer[5] < 0x30) || (ascii_buffer[5] > 0x39) ) //outside of numerical range
			{
				b_buffer_data_valid = false ;
				break ;
			}
			else if( (ascii_buffer[7] < 0x30) || (ascii_buffer[7] > 0x39) ) //outside of numerical range
			{
				b_buffer_data_valid = false ;
				break ;
			}
			break ;
			
		case READ_AMPS: //check 3, 4, 5, 7
			if( (ascii_buffer[3] < 0x30) || (ascii_buffer[3] > 0x39) ) //outside of numerical range
			{
				b_buffer_data_valid = false ;
				break ;
			}
			else if( (ascii_buffer[4] < 0x30) || (ascii_buffer[4] > 0x39) ) //outside of numerical range
			{
				b_buffer_data_valid = false ;
				break ;
			}
			else if( (ascii_buffer[5] < 0x30) || (ascii_buffer[5] > 0x39) ) //outside of numerical range
			{
				b_buffer_data_valid = false ;
				break ;
			}
			else if( (ascii_buffer[7] < 0x30) || (ascii_buffer[7] > 0x39) ) //outside of numerical range
			{
				b_buffer_data_valid = false ;
				break ;
			}
			break ;
			
		case READ_POWER: //check 3, 5
			if( (ascii_buffer[3] < 0x30) || (ascii_buffer[3] > 0x39) ) //outside of numerical range
			{
				b_buffer_data_valid = false ;
				break ;
			}
			else if( (ascii_buffer[5] < 0x30) || (ascii_buffer[5] > 0x39) ) //outside of numerical range
			{
				b_buffer_data_valid = false ;
				break ;
			}		
			break ;
			
		default:
			b_buffer_data_valid = false ;
			break ;
	}
	
	
	if(b_buffer_data_valid)
	{
		switch(command_type)
		{
			case READ_OUTV:
				//only first 3 digits are displayed on the LCD
				*nonascii_value += (ascii_buffer[3]-ASCII_OFFSET)*100 ;			
				*nonascii_value += (ascii_buffer[4]-ASCII_OFFSET)*10 ;
				*nonascii_value += (ascii_buffer[6]-ASCII_OFFSET)*1 ;
				break ;
			
			case READ_DEMV:
				//only first 3 digits are displayed on the LCD
				*nonascii_value += (ascii_buffer[4]-ASCII_OFFSET)*100 ;
				*nonascii_value += (ascii_buffer[5]-ASCII_OFFSET)*10 ;
				*nonascii_value += (ascii_buffer[7]-ASCII_OFFSET)*1 ;
				break ;
			
			case READ_AMPS:
				//all 4 digits are displayed on the LCD
				*nonascii_value += (ascii_buffer[3]-ASCII_OFFSET)*1000 ;
				*nonascii_value += (ascii_buffer[4]-ASCII_OFFSET)*100 ;
				*nonascii_value += (ascii_buffer[5]-ASCII_OFFSET)*10 ;
				*nonascii_value += (ascii_buffer[7]-ASCII_OFFSET)*1 ;
				break ;
			
			case READ_POWER:
				//Only 2 digits are displayed on the LCD
				*nonascii_value += (ascii_buffer[3]-ASCII_OFFSET)*10 ;
				*nonascii_value += (ascii_buffer[5]-ASCII_OFFSET)*1 ;
				break ;
		}
	}
	else
	{
		*nonascii_value = old_value ; //ascii_buffer was found to contain non-numerical chars
	}
	
	return ;
}
/*-----------------------------------------------------------*/

void assert_triggered(const char *file, uint32_t line)
{
	volatile uint32_t block_var = 0, line_in;
	const char *file_in;

	/* These assignments are made to prevent the compiler optimizing the
	values away. */
	file_in = file;
	line_in = line;
	(void) file_in;
	(void) line_in;

	taskENTER_CRITICAL();
	{
		while (block_var == 0) {
			/* Set block_var to a non-zero value in the debugger to
			step out of this function. */
		}
	}
	taskEXIT_CRITICAL();
}
