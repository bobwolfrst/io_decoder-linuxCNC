/********************************************************************
* Description:  io_decoder.c
* This file, 'io_decoder.c', is a HAL component that
* connect an ARDUINO based board with USB communication to drive an input/output system
* based on NPIC6C596 ICs, encoder, DAC and ADC to menage a CNC operator panel.
*
* Author: bobwolf
* License: GPL Version 2
*
* Copyright (c) 2024 All rights reserved.
* 
* 29/12/2025 testato su MICRO
********************************************************************/
/*
 * Questo componente HAL per LINUXCNC è progettato per interfacciarsi con una scheda basata su Arduino 
 * tramite comunicazione USB seriale. L'obiettivo principale è la gestione di un pannello operatore CNC, 
 * delegando ad Arduino tutte le sequenze di lettura e scrittura dei registri, degli encoder e dei convertitori, 
 * garantendo così che il componente HAL rimanga "leggero" e focalizzato sulla comunicazione.

 * Funzionalità Principali:

 * Gestione I/O Delegata: Il componente si limita a inviare comandi e ricevere dati dall'Arduino. 
 * 		La logica di scansione, la gestione degli encoder, e le sequenze di lettura/scrittura dei DAC/ADC 
 * 		sono interamente gestite dal firmware sull'Arduino, liberando LinuxCNC da queste operazioni a bassa latenza.

 * Protocollo di Comunicazione: Il sistema si basa su un protocollo USB seriale che gestisce in modo efficiente 
 * 		lo scambio di pacchetti di dati tra LinuxCNC e l'Arduino.

 * output: attraverso i pin di output del componente trasferisce il segnale verso le uscite fisiche sulla scheda USB. con 
 * 		la possibilità di far lampeggiare indipendentemente ogni uscita con frequenze differenti

 * input: gli ingressi fisici della scheda USB vengono inviati al componente HAL e questo con un semi-debounce di durata
 * 		di un periodo di comunicazione usb (20ms tipicamente) trasferisce il segnale sul pin del componente HAL

 * Lettura Encoder: Riceve i dati dagli encoder in quadratura, i cui conteggi e direzioni sono gestiti 
 * 		direttamente dal firmware di Arduino e comunica il delta dello spostamento dall'ultima comunicazione (20ms tipicamente).
 * 		se lo spostamento è 0 invia 0 sulla seriale. oltre all'inversione del conteggio il componente genera delle
 * 		sequenze di impulsi up o down per comandare componenti tipo MULTISWITCH o UPDOWN.

 * Gestione ADC: riceve dati da ADC. ha un pin di HAL con il dato grezzo ricevuto, un pin con il valore float filtrato e 
 * 		trasformato da un pin per invertire il valore ed uno per scalarlo.

 * Gestione DAC: Invia valori a DAC. trasmette il dato trasformato dal pin di inversione e scale.

 * Handshake e Stato: Il componente gestisce un protocollo di handshake iniziale per verificare la compatibilità 
 * 		del firmware e lo scambio dei dati delle funzionalità della scheda USB. Dopodiche avvia la comunicazione ordinaria
 * 		Mantiene inoltre uno stato di comunicazione (Handshake, Communicating, Error) esposto come pin HAL 
 * 		per la diagnostica, utile per identificare problemi legati a cavi o disconnessioni.
 * 		in caso di timeout di comunicazione con la scheda USB viene segnalato attaverso un messaggio di errore  ed il sistema 
 * 		torna in handshake. questo errore puo essere causato da un malfunzionamento della scheda USB
 * 		in casi di disconnessione fisica del cavo USB, o non viene impostata la porta giusta, viene generato un messaggio di errore
 * 		ed il sistema torna in handshske. se viene riconnesso il cavo il sistema segnala la situazione attraverso 
 * 		un messaggio di errore usato in modo anomalo, ma funzionale alla ricerca del guasto.
 * 		se il cavo USB capta troppe interferenze, oppure è di pessima qualità, il sistema invia dei messaggi di errore che 
 * 		indicano la percentuale di messaggi errati, se questo valore e fra 1% e 5% con un warning a stampa. se supera il 5% viene
 * 		stampato un errore ed il sistema passa allo stato di errore e quindi riavvia la comunicazione seriale da 0.
 * 		tutti i messaggi di errore descritti sopra sono visibili sulla GUI di LINUXCNC se abilitati dalla configurazione
 * 		iniziale di avvio del componente nel file .hal della macchina.

 * Panoramica del Funzionamento:

 * Handshake Iniziale: Alla partenza, il componente si connette all'Arduino e scambia dati per confermare che 
 * 		la versione del firmware (specificata come parametro del modulo) corrisponda a quella attesa.

 * Ciclo di Comunicazione: Una volta stabilita la connessione, il componente invia un pacchetto di dati all'Arduino 
 * 		contenente i valori di output e riceve un pacchetto di risposta con i dati di input 
 * 		(encoder, interruttori, ADC).

 * Thread di Comunicazione: Un thread non-realtime, creato internamente dal componente e separato, gestisce 
 * 		la comunicazione USB per evitare di bloccare il ciclo in tempo reale di LinuxCNC, 
 * 		migliorando l'affidabilità del sistema.

 * Parametri del Modulo:
 * input: numero di input installati sulla scheda USB. questi sono liberamente espandibili fino 
 * 		ad un massimo di 128 input (16 schede di epansione). default 8.
 * 
 * output: numero di output installati sulla scheda USB. questi sono liberamente espandibili fino 
 * 		ad un massimo di 128 output (16 schede di epansione). default 8. 
 * 
 * firmware: Versione del firmware hardware sull'Arduino (deve corrispondere). default 101.
 * 
 * usb_port_name: Nome della porta seriale USB default "/dev/io_decoder".
 * 
 * verbose: per abilitare il livello dei messaggi di errore sulla GUI. il numero attiva il tipo di messaggio 
 * 		indicato e quelli di valore inferiore. default 1.
		0=nessuno.
		1=solo USB. Invia messaggio in caso di disconnessione USB ed in caso di riavvio della comunicazione.
		2=minimi. Messaggi di percentuale di errore parsing.
		3=tutti.
 *
 *
 * 
*/



/* Standard Linux Includes */
#include "rtapi.h"		// RTAPI realtime OS API
#include "rtapi_app.h"		// RTAPI realtime module decls
#include "hal.h"		// HAL public API decls
//#include "rtapi_print.h"
//#include "rtapi_thread.h" // ADDED: For rtapi_id_t, rtapi_thread_new, etc.
#include <float.h>
#include <rtapi_math.h>
#include <rtapi_string.h>
//#include "rtapi_delay.h"
//#include <rtapi/rtapi_print.h> // Per rtapi_print_error()
#include <stdbool.h>
#include <termios.h>    // Per configurazione della porta seriale
#include <fcntl.h>      // Per open(), close()
#include <unistd.h>     // Per read(), write()
#include <errno.h>      // Per gestione errori
#include <sys/ioctl.h>  // Per ioctl (opzionale, per controllo flusso)
#include <pthread.h>    // Per gestione thread (se non usi rtapi_start_thread)
#include <stdio.h>   // Per snprintf
#include <string.h>  // For memset
#include <ctype.h>   // Per isprint
#include <time.h>	//per stampare l'ora

#include <linux/uinput.h>
#include <stdlib.h> // Per atoi, malloc, free, ecc.

//void rtapi_delay_ns(long ns);

// module information
MODULE_AUTHOR("bobwolf");
MODULE_DESCRIPTION("NPIC6C596 input output USB driver Component for EMC HAL");
MODULE_LICENSE("GPL");


static int input = 8;   // Valore di default
static int output = 8;  // Valore di default
static char *usb_port_name = "/dev/io_decoder"; // Valore di default per 'usb_port_name'
static int firmware = 101;  // Valore di default
static int verbose = 1;   // Valore di default
static char *keymap_file = "io_decoder-keymap.cfg"; // Valore di default per il file di configurazione della tastiera
static char *uinput_chmod_cmd = "chmod 0666 /dev/uinput"; //Valore di default per i permessi di UINPUT tutti aperti

RTAPI_MP_INT(input, "number input channels");
RTAPI_MP_INT(output, "number output channels");
RTAPI_MP_STRING(usb_port_name, "USB serial port name (e.g., /dev/io_decoder)");
RTAPI_MP_INT(firmware, "Firmware of the hardware; must match to go on"); //per essere comparato con handshake
RTAPI_MP_INT(verbose, "type of error messeges");	//0=nessuno. 1=tutti. 3=minimi. 4=solo USB
RTAPI_MP_STRING(keymap_file, "description file to set the inputs versus keyboard commands");
RTAPI_MP_STRING(uinput_chmod_cmd, "Command used to set permissions on /dev/uinput (e.g., 'chmod 0660 /dev/uinput')");

#define DEBUG 0						//0=niente 1=chiama usb 2=usb e comunicazione 3=
static int printed_debug[100] = {0};	//serve a creare uno stop alle scritte di debug

#define MAX_CHAN 256				//numero massimo degli npic totali previsto
#define MIN_CHAN 16					//numero minimo degli npic totali previsto
#define MAX_INPUT 128				//numero massimo degli npic input previsto
#define MAX_OUTPUT 128				//numero massimo degli npic output previsto
#define MAX_N_ENCODER 6				//numero massimo previsto
#define MAX_BIT_ENCODER 16			//numero bit massimo per ogni encoder
#define MAX_ADC 6					//numero massimo previsto
#define MAX_BIT_ADC 16				//numero bit massimo per ogni ADC
#define MAX_DAC 6					//numero massimo previsto
#define MAX_BIT_DAC 16				//numero bit massimo per ogni DAC
#define MAX_IN_BIT_EXPANSION 128	//numero bit massimo per ogni IN EXPANSION
#define MAX_OUT_BIT_EXPANSION 128	//numero bit massimo per ogni OUT EXPANSION

// Le dimensioni dei payload useranno queste massime per allocare i buffer
// Questi sono calcoli per le dimensioni massime dei buffer di comunicazione
#define MAX_TX_PAYLOAD_SIZE ( (MAX_OUTPUT / 8) + (MAX_DAC * ((MAX_BIT_DAC + 7) / 8)) + ((MAX_OUT_BIT_EXPANSION + 7) / 8) ) + 3
#define MAX_RX_PAYLOAD_SIZE ( (MAX_INPUT / 8) + (MAX_N_ENCODER * ((MAX_BIT_ENCODER + 7) / 8)) + (MAX_ADC * ((MAX_BIT_ADC + 7) / 8)) + ((MAX_OUT_BIT_EXPANSION + 7) / 8) ) + 3

// Byte di Controllo USB
#define COMM_INIT 0x02
#define COMM_END  0x03
#define HAND_INIT 0x04
#define HAND_END  0x05
#define ERROR_MSG  0x06

// Stati del Protocollo HAL USB
typedef enum {
    HAL_STATE_HANDSHAKE = 0,
    HAL_STATE_HANDSHAKE_WAIT_RESPONSE = 1,
    HAL_STATE_COMMUNICATING = 2,
    HAL_STATE_COMMUNICATING_WAIT_RESPONSE = 3,
    HAL_STATE_ERROR = 4,
    
    VERBOSE_NULL = 0,
    VERBOSE_USB = 1,
    VERBOSE_MIN = 2,
    VERBOSE_ALL = 3,
    
} hal_comm_state_t;

// AGGIUNTO: Definizioni per la priorità e lo stack del thread USB
//#define USB_THREAD_PRIORITY rtapi_prio_lowest() // Puoi scegliere una priorità appropriata
#define USB_THREAD_PRIORITY 50 // Puoi scegliere una priorità appropriata
#define USB_THREAD_STACKSIZE 32768 // Dimensione dello stack (potrebbe necessitare aggiustamenti)

// per filtrare il valore dell'ADC e rendere il valore più stabile. più è piccolo più il valore in uscita è stabile
#define ALPHA 0.1

// Helper functions for serial communication
static int serial_write(int fd, const unsigned char *buf, int len);
static int serial_read(int fd, unsigned char *buf, int len);

#define MAX_INPUT_2 (MAX_INPUT * 2)				//per la funzionalità keyboard a 2 tasti
static int in_bit_d[MAX_INPUT];
static int in_bit_t_state[MAX_INPUT];
static uint32_t in_bit_keycode_value[MAX_INPUT_2];
static int keycode_number = 0;
static int hal_shutdown_flag = 0;
static int state_flag = 0;
volatile int error_message;
static long next_key_attempt_time_ns = 0;

static int keyboard_initialized_ok = 0;
static int shift_count = 0;
static int ctrl_count = 0;
static int l_alt_count = 0;
static int r_alt_count = 0;


/***********************************************************************
* STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

//* This structure contains the runtime data for a single io_decoder.

// Definizioni per funzionalità keyboard *******************************************
// Definizioni dei tipi per key_type (sono molto utili!)
#define KEY_TYPE_UNUSED		0
#define KEY_TYPE_MODIFIER	1
#define KEY_TYPE_BOTH		2
#define KEY_TYPE_NORMAL		3


//#define KEY_DOWN 1
//#define KEY_UP 0

#define KEY_SCAN_DELAY_NS 100000000 // 100 ms in microsecondi
// ...

//static char keymap_filepath[512] = {0}; 

typedef struct {
    // 1. Stato Corrente (Lettura dal pin HAL)
    uint8_t current_state;  
    
    // 2. Stato Precedente (Per rilevare la transizione)
    uint8_t previous_state; 
    
    // 3. Tipo di Tasto (Modificatore o Normale o Both)
    uint8_t key_type;       
    
    // 4. Keysym Value (Il codice numerico keyboard, richiede 32 bit)
    uint32_t keycode_value;   
    
    uint32_t modifier_value;
    
    // 6. per ritardare il comando sincronizzato modificato
    uint8_t both_loop;
    
} PinData_t;

// Array monodimensionale delle strutture
PinData_t key_input_map[MAX_INPUT];

//****************************************************************************


typedef struct {

    hal_bit_t *usb_connected;
    hal_float_t *debug_f;
    hal_float_t *debug_f2;
    
    hal_s32_t *servo_thread_time; 			// input per debug
    hal_s32_t *base_thread_time; 			// input per debug
    
    hal_s32_t *servo_jitter_pin;
    hal_s32_t *base_jitter_pin;
    
    hal_s32_t *jitter_usb_comm; 		// input per debug
    hal_s32_t *jitter_usb_loop; 		// input per debug

    
    hal_bit_t *in_bit[MAX_INPUT];
    hal_bit_t *in_bit_t[MAX_INPUT];
        
    hal_bit_t *out_bit[MAX_OUTPUT];	
    hal_bit_t *out_bit_blink_en[MAX_OUTPUT];
    hal_float_t *out_bit_blink_freq[MAX_OUTPUT];
    hal_float_t *out_bit_blink_width[MAX_OUTPUT];
    long out_blink_time_ns[MAX_OUTPUT];

	hal_s32_t *enc[MAX_N_ENCODER]; 			// Per i valori degli encoder
    hal_bit_t *enc_invert[MAX_N_ENCODER]; 	// Per l'inversione
    hal_bit_t *enc_up[MAX_N_ENCODER];     	// Per i pin up (se usi anche pulsanti)
    hal_bit_t *enc_down[MAX_N_ENCODER];   	// FIXED: Typo MAX_N_ENCODERS -> MAX_N_ENCODER
    uint8_t enc_delta[MAX_N_ENCODER];   	// per gestire gli impulsi di enc_up enc_down
    uint8_t enc_delta_sign[MAX_N_ENCODER];   // per gestire gli impulsi di enc_up enc_down
    
	hal_float_t *adc[MAX_ADC]; 				// Per i valori degli ADC
    hal_float_t *adc_scale[MAX_ADC]; 		// Per scalare i valori di ogni ADC
    hal_bit_t *adc_invert[MAX_ADC]; 		// Per l'inversione
    hal_u32_t *adc_raw[MAX_ADC]; 			// Per il dato raw
    uint32_t old_adc_raw[MAX_ADC]; 			// Per il dato raw
    hal_float_t *adc_joy_center[MAX_ADC]; // Per l'offset del centro del potenziometro
    hal_float_t *adc_joy_deadb[MAX_ADC]; 	// Per la deadband del centro del potenziometro
    hal_float_t *adc_joy_factor[MAX_ADC]; 	// Per la proporzionalità del potenziometro
    hal_bit_t *adc_joy_pulse_up[MAX_ADC]; 		// Per l'impulso up comandato dal potenziometro
    hal_bit_t *adc_joy_pulse_down[MAX_ADC]; 	// Per l'impulso down comandato dal potenziometro
    hal_s32_t *adc_joy_count[MAX_ADC];	// Per il conto degli impulsi
    uint32_t adc_raw_joy[MAX_ADC]; 		// Per fornire il dato raw degli impulsi anche invertito
    long next_adc_joy_ns[MAX_ADC];
    uint8_t adc_joy_valido[MAX_ADC];		// 

	hal_float_t *dac[MAX_DAC]; 			// Per i valori degli DAC
    hal_float_t *dac_scale[MAX_DAC]; 	// Per scalare i valori di ogni DAC
    hal_bit_t *dac_invert[MAX_DAC]; 	// Per l'inversione

    hal_bit_t *out_expansion[MAX_OUT_BIT_EXPANSION]; 	// Per i bit out dell'espansione
    hal_bit_t *in_expansion[MAX_IN_BIT_EXPANSION]; 	// Per i bit in dell'espansione

	// Nuove variabili per la comunicazione USB
    char usb_port_name[HAL_NAME_LEN]; // Nome della porta seriale USB (es. /dev/io_decoder)
    int serial_fd;                     // File descriptor della porta seriale
    int baud_rate;                     // Baud rate della porta seriale
    int usb_thread_id;          // MODIFICATO: rtapi_id_t non esiste, usa int

    // Buffer per lo scambio dati tra servo-thread e usb-thread
    // Questi conterranno solo il *payload* senza header/footer/checksum
    uint8_t usb_tx_payload[MAX_TX_PAYLOAD_SIZE];
    uint8_t usb_rx_payload[MAX_RX_PAYLOAD_SIZE];
    
    uint32_t usb_tx_payload_len; // Lunghezza effettiva del payload nel buffer tx
    uint32_t usb_rx_payload_len; // Lunghezza effettiva del payload nel buffer rx

	unsigned char serial_rx_buffer[MAX_RX_PAYLOAD_SIZE];
    int serial_rx_buffer_len;
    unsigned char tx_packet_buffer[MAX_TX_PAYLOAD_SIZE];

    //definizioni per impostare firmware 101
    /*
    #define FIRMWARE    101
    #define N_ENCODER   2     //poi 4 
    #define N_DAC       2
    #define N_ADC       3
    #define IN_BIT_EXPANSION    8
    #define OUT_BIT_EXPANSION   8*/
    uint8_t firmware_n_encoder;
    uint8_t firmware_n_dac;
    uint8_t firmware_n_adc;
    uint8_t firmware_in_bit_expansion;
    uint8_t firmware_out_bit_expansion;
    uint8_t firmware_firmware;


    // Dati ricevuti dall'handshake di Arduino (per dimensioni payload)
    uint8_t arduino_byte_npic_input;	//variabile di appoggio
    uint8_t arduino_byte_npic_output;	//variabile di appoggio
    uint8_t arduino_n_encoder;
    uint8_t arduino_bit_encoder;
    uint8_t arduino_byte_encoder;	//variabile di appoggio
    uint8_t arduino_bytes_encoder;	//variabile di appoggio
    uint8_t arduino_n_dac;
    uint8_t arduino_bit_dac;
    uint8_t arduino_byte_dac;	//variabile di appoggio
    uint8_t arduino_bytes_dac;	//variabile di appoggio
    uint8_t arduino_n_adc;
    uint8_t arduino_bit_adc;
    uint8_t arduino_byte_adc;	//variabile di appoggio
    uint8_t arduino_bytes_adc;	//variabile di appoggio
    uint8_t arduino_in_bit_expansion;
    uint8_t arduino_in_byte_expansion;	//variabile di appoggio
    uint8_t arduino_out_bit_expansion;
    uint8_t arduino_out_byte_expansion;	//variabile di appoggio
    uint8_t arduino_loop_time;
    uint8_t arduino_firmware;


    // PARAMETRI DEL MODULO (ora membri della struct per accesso tramite 'io->')
    int input;      // Numero di input NPIC richiesti (copiato dal parametro del modulo)
    int output;     // Numero di output NPIC richiesti (copiato dal parametro del modulo)
    int firmware;   // Versione del firmware atteso (copiato dal parametro del modulo)
    int verbose;   	// Livello di visualizzazione dei messaggi di errore
    //char keymap_file[512];
    
    // Variabili di stato per il protocollo
    hal_comm_state_t comm_state; // Stato attuale della comunicazione (handshake/communicating)
    int comm_error_count;       // Contatore errori consecutivi
    hal_s32_t *hal_comm_state;   // Pin HAL per lo stato di comunicazione (per GUI)
    hal_float_t *hal_error_count;  // Pin HAL per il contatore errori (per GUI)
    hal_float_t *parse_error_percentile;  // Pin HAL per il contatore errori assoluto (per GUI)
    hal_bit_t *led;		// pin: output led per verifiche

    // Variabili per il timing del thread USB non-realtime
    long usb_last_read_time_ns; // Timestamp ultima lettura/scrittura USB
    long usb_mini_timeout_ns; // Timestamp ultima lettura/scrittura USB
    long usb_timeout_ns;        // Timeout per la comunicazione USB
    long loop_time_ns;
    volatile int usb_data_busy; // semaforo per bloccare la lettura contemporanea dei dati fra thread
    
    //variabili per bloccare il refresh indesiderati dei valori nelle funzioni
    volatile int encoder_refresh;
    volatile int in_npic_refresh;
	volatile int out_npic_refresh;
	volatile int adc_refresh;
	volatile int dac_refresh;
	volatile int in_exp_refresh;
	volatile int out_exp_refresh;
	
	//per implementare la funzionalità keyboard simulata
	int uinput_fd;
	
} hal_io_decoder_t;

// pointer to array of io_decoder_t structs in shared memory, 1 per gen
static hal_io_decoder_t *io_decoder_array;

// other globals
static int comp_id;		// component ID

/*******************************************************************************************************************
* LOCAL FUNCTION DECLARATIONS                         *
********************************************************************************************************************/

static int export_io_decoder(int num, hal_io_decoder_t * addr,char* prefix);
static void calc_io_decoder(void *arg, long period);


//***********************************************************************
// Funzione helper per configurare la porta seriale
// Funzione helper per configurare la porta seriale (Versione per 32u4/Leonardo)
int configure_serial_port(int fd, int baud_rate) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Errore tcgetattr\n");
        return -1;
    }

    // Imposta la velocità (anche se su Leonardo/USB è virtuale, meglio metterla)
    cfsetospeed(&tty, baud_rate);
    cfsetispeed(&tty, baud_rate);

    // --- MODALITÀ RAW TOTALE (Indispensabile per Leonardo) ---
    // Disabilita parità, stop bits e forza 8 bit di dati
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    // CLOCAL: ignora linee controllo modem, CREAD: abilita ricezione
    tty.c_cflag |= (CLOCAL | CREAD);

    // Disabilita ogni elaborazione in ingresso (niente traduzione di \r o \n)
    // Questo impedisce che i byte 0x00, 0x0A, 0x0D vengano scartati o modificati
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);

    // Disabilita ogni elaborazione in uscita (fondamentale per trasmettere i payload binari)
    tty.c_oflag &= ~OPOST;

    // Disabilita modalità canonica, eco e segnali
    tty.c_lflag &= ~(ICANON | ECHO | ECHONL | ISIG | IEXTEN);

    // Impostazioni di timeout: read non bloccante se usiamo O_NONBLOCK dopo
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 10; // Timeout di 1 secondo per tcsetattr

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Errore tcsetattr\n");
        return -1;
    }

    // --- ATTIVAZIONE DTR/RTS (Apre il canale dati della Leonardo) ---
    int status;
    if (ioctl(fd, TIOCMGET, &status) == -1) {
        rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Errore TIOCMGET\n");
    } else {
        status |= TIOCM_DTR;
        status |= TIOCM_RTS;
        if (ioctl(fd, TIOCMSET, &status) == -1) {
            rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Errore TIOCMSET\n");
        }
    }

    // Breve attesa per permettere al firmware (senza bootloader) di stabilizzare la USB
    usleep(100000); 
    
    // Pulisce i buffer da eventuali residui di accensione
    tcflush(fd, TCIOFLUSH);

    rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: Serial port configured in RAW mode (DTR/RTS ON)\n");
    return 0;
}

//***********************************************************************
// Funzione per il Calcolo del Checksum XOR
// Restituisce il byte del checksum.
uint8_t calculate_checksum(const uint8_t *payload, int payload_len) {
    uint8_t checksum = 0;
    for (int i = 0; i < payload_len; i++) {
        checksum ^= payload[i]; // XOR cumulativo di tutti i byte del payload
    }
    return checksum;
}


// Funzione per la Costruzione del Pacchetto per la Trasmissione
// Costruisce il pacchetto completo nel 'buffer' fornito.
// Restituisce la lunghezza totale del pacchetto costruito, o un valore negativo in caso di errore.
int build_packet(uint8_t *buffer, int max_buffer_len, uint8_t start_byte, uint8_t end_byte, const uint8_t *payload, int payload_len) {
    // Calcola la lunghezza minima necessaria per il pacchetto completo:
    // 1 (start_byte) + payload_len + 1 (checksum) + 1 (end_byte)
    int required_len = 1 + payload_len + 1 + 1;

    // Controlla se il buffer fornito è abbastanza grande
    if (max_buffer_len < required_len) {
        rtapi_print_msg(RTAPI_MSG_WARN, "io_decoder: ERROR: build_packet: Buffer too small (%d bytes needed, %d available).\n", required_len, max_buffer_len);
        return -1; // Errore: buffer troppo piccolo
    }

    // Calcola il checksum del payload
    uint8_t checksum = calculate_checksum(payload, payload_len);

    int current_idx = 0;

    // Aggiungi il byte di inizio
    buffer[current_idx++] = start_byte;

    // Copia il payload nel buffer
    memcpy(&buffer[current_idx], payload, payload_len);
    current_idx += payload_len;

    // Aggiungi il byte del checksum
    buffer[current_idx++] = checksum;

    // Aggiungi il byte di fine
    buffer[current_idx++] = end_byte;

    return current_idx; // Restituisce la lunghezza totale del pacchetto costruito
}


int parse_packet(const uint8_t *buffer, int buffer_len, uint8_t expected_start_byte, uint8_t expected_end_byte, uint8_t *parsed_payload_buffer, int max_parsed_payload_len) {
    
    int start_idx = -1;
    int end_idx = -1;
    uint8_t received_checksum;
    uint8_t calculated_checksum;
    error_message = 0;


    // 1. Trova il byte di fine dopo il byte di inizio
    for (int i = 0; i < buffer_len; i++) {
        if (buffer[i] == expected_end_byte) {
            end_idx = i;
            //break;
        }
    }

    if (end_idx == -1) {
        rtapi_print_msg(RTAPI_MSG_DBG, "io_decoder: DBG: parse_packet: End byte 0x%02X not found after start byte 0x%02X.\n", expected_end_byte, expected_start_byte);
        return 0; // Byte di fine non trovato, pacchetto incompleto
    }
    // 2. Trova il byte di inizio
    for (int i = 0; i < buffer_len; i++) {
        if (buffer[i] == expected_start_byte) {
            start_idx = i;
            break;
        } else if (buffer[i] == ERROR_MSG) {
			//i++;
			start_idx = i;
			error_message = 1;
			break;
        }
    }

    if (start_idx == -1) {
        rtapi_print_msg(RTAPI_MSG_DBG, "io_decoder: DBG: parse_packet: Start byte 0x%02X not found.\n", expected_start_byte);
        return 0; // Byte di inizio non trovato, pacchetto non completo o non presente
    }
    
    // 3. Verifica la lunghezza minima del pacchetto: start + payload + checksum + end (minimo 3 byte, se payload_len è 0)
    // Cioè: start_idx < (checksum_idx) < end_idx
    // end_idx - start_idx deve essere almeno 3 (start_byte + checksum + end_byte)
    if (((end_idx - start_idx) < 3) && !error_message) {
        rtapi_print_msg(RTAPI_MSG_WARN, "io_decoder: ERROR: parse_packet: Packet too short. Start: %d, End: %d.\n", start_idx, end_idx);
        return -2; // Errore: Pacchetto troppo corto
    }

    // 4. Estrai il checksum ricevuto (è il byte prima dell'end_byte)
    received_checksum = buffer[end_idx - 1];

    // 5. Calcola la lunghezza del payload
    // Il payload si trova tra start_byte (escluso) e il checksum (escluso)
    //int payload_len = (end_idx - 1) - (start_idx + 1);
    int payload_len = (end_idx - start_idx) - 2;
    
    // 6. Controlla se il buffer di output per il payload è abbastanza grande
    if ((max_parsed_payload_len < payload_len) && !error_message) {
        rtapi_print_msg(RTAPI_MSG_WARN, "io_decoder: ERROR: parse_packet: Parsed payload buffer too small (%d needed, %d available) end_idx=%d. start_idx=%d.\n", payload_len, max_parsed_payload_len, end_idx, start_idx);
        return -3; // Errore: buffer di output per il payload troppo piccolo
    }

    // 7. Calcola il checksum del payload estratto
    // Il payload inizia subito dopo lo start_byte
    calculated_checksum = calculate_checksum(&buffer[start_idx + 1], payload_len);

    // 8. Confronta i checksum
	if ((received_checksum != calculated_checksum) && !error_message) {
		rtapi_print_msg(RTAPI_MSG_WARN, "io_decoder: ERROR: parse_packet: Checksum mismatch! Received 0x%02X, Calculated 0x%02X.\n", received_checksum, calculated_checksum);
		return -4; // Errore: Checksum non corrispondente
	}


    // 9. Se tutto è valido, copia il payload nel buffer di output
    memcpy(parsed_payload_buffer, &buffer[start_idx + 1], payload_len);
    

    return payload_len; // Restituisce la lunghezza del payload valido estratto
}

// Implementazione delle funzioni di utilità per la comunicazione seriale
static int serial_write(int fd, const unsigned char *buf, int len) {
    int bytes_written = write(fd, buf, len);
    tcdrain(fd);          // <--- QUESTO forza Linux a mandare i dati sul cavo USB ora!
    if (bytes_written < 0) {
        rtapi_print_msg(RTAPI_MSG_WARN, "io_decoder: ERROR serial_write %s\n", strerror(errno));
    }
    return bytes_written;
}

static int serial_read(int fd, unsigned char *buf, int len) {
    int bytes_read = read(fd, buf, len);
    if (bytes_read < 0) {
        // EAGAIN/EWOULDBLOCK sono normali se non ci sono dati disponibili e la porta è non-bloccante
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            rtapi_print_msg(RTAPI_MSG_WARN, "io_decoder: ERROR serial_read: %s\n", strerror(errno));
       }
    }
    return bytes_read;
}

// Funzione helper per formattare un buffer di byte in una stringa esadecimale
// Ritorna un puntatore a una stringa statica. Fai attenzione a usarla subito,
// perché la prossima chiamata sovrascriverà il contenuto.
static char* format_buffer_for_debug(const unsigned char *buffer, int len) {
    // Usiamo un buffer statico. Questo limita la lunghezza della stringa di output
    // ma evita allocazioni dinamiche e problemi di memoria in ambiente RT.
    // Il buffer è 3 volte la lunghezza massima dei byte + 1 per il terminatore null
    // (es. "0xAA " occupa 5 caratteri per byte, quindi 5*len + 1. Per essere sicuri, 4*len + 1)
    // Se la lunghezza massima del tuo rx_buffer è 64, allora 64 * 5 = 320 caratteri + 1
    #define MAX_DEBUG_STR_LEN (64 * 5 + 1) // Assumendo rx_buffer max 64 byte
    static char debug_str[MAX_DEBUG_STR_LEN];
    int current_pos = 0;

    debug_str[0] = '\0'; // Inizializza la stringa vuota

    for (int i = 0; i < len; i++) {
        // Calcola lo spazio rimanente nel buffer
        int remaining_space = MAX_DEBUG_STR_LEN - current_pos;
        if (remaining_space <= 5) { // 5 è lo spazio minimo per "0xXX "
            // Non c'è più spazio sufficiente per il prossimo byte + spazio
            break;
        }
        // snprintf è più sicuro di sprintf perché non causa buffer overflow
        current_pos += snprintf(&debug_str[current_pos], remaining_space, "0x%02X ", buffer[i]);
    }
    return debug_str;
}

// Funzione helper per formattare un buffer di byte in una stringa di caratteri ASCII.
// I caratteri non stampabili verranno sostituiti con un punto (.).
// Ritorna un puntatore a una stringa statica. Fai attenzione a usarla subito,
// perché la prossima chiamata sovrascriverà il contenuto.
static char* format_as_ascii_for_debug(const unsigned char *buffer, int len) {
    // Usiamo un buffer statico. Deve essere grande quanto la lunghezza massima dei byte + 1 per il terminatore null
    #define MAX_ASCII_STR_LEN (64 + 1) // Assumendo rx_buffer max 64 byte
    static char ascii_str[MAX_ASCII_STR_LEN];
    int current_pos = 0;

    // Assicurati di non superare la dimensione del buffer
    if (len >= MAX_ASCII_STR_LEN) {
        len = MAX_ASCII_STR_LEN - 1; // Tronca se il buffer è più piccolo
    }

    for (int i = 0; i < len; i++) {
        // isprint() verifica se il carattere è stampabile (ASCII 32-126)
        if (isprint(buffer[i])) {
            ascii_str[current_pos++] = (char)buffer[i];
        } else {
            ascii_str[current_pos++] = '.'; // Sostituisci i non stampabili con un punto
        }
    }
    ascii_str[current_pos] = '\0'; // Termina la stringa con null
    
    return ascii_str;
}



// usb_thread_function************************************************************************************************
void usb_thread_function(void *arg) {
    hal_io_decoder_t *io = (hal_io_decoder_t *)arg;

    // Inizializzazione iniziale dello stato di comunicazione
    *(io->hal_comm_state) = HAL_STATE_HANDSHAKE; // Inizia sempre in stato di handshake
    state_flag = HAL_STATE_HANDSHAKE;
    *(io->hal_error_count) = 0.0f; // Azzera il contatore errori all'avvio
    *(io->parse_error_percentile) = 0.0f; // Azzera il contatore errori all'avvio
    io->loop_time_ns = 20 * 1000 * 1000; // Default a 20ms

    io->serial_fd = -1; // Inizializza il file descriptor a uno stato non valido
    io->encoder_refresh = 0;
    io->in_npic_refresh = 0;
    io->out_npic_refresh = 0;
    io->adc_refresh = 0;
    io->dac_refresh = 0;
    io->in_exp_refresh = 0;
    io->out_exp_refresh = 0;
    

    long current_time_ns;
    long next_handshake_attempt_time_ns = 0; // Tempo per il prossimo tentativo di handshake
    long next_comm_attempt_time_ns = 0;      // Tempo per il prossimo tentativo di comunicazione
    long response_timeout_ns = 1000 * 1000 * 1000; // 1000ms di timeout per la risposta
    long handshake_period_ns = 20 * 1000 * 1000; // Periodo di 20ms per i tentativi di handshake
    static long last_read_time_ns; // Inizializza al tempo corrente
    long long parse_error_time_ns = (long long)30 * 1000 * 1000 * 1000; // 60s per la statistica degli errori di parse
    long next_parse_error_time_ns;
    long loop_current_time_ns;
    long usb_current_time_ns;
    long usb2_current_time_ns;
    long usb_jitter_time_ns;
    
    static int timer_overflow_0 = 0;
    static int timer_overflow_1 = 0;
    static int timer_overflow_2 = 0;
    static int timer_overflow_3 = 0;
    static int timer_overflow_4 = 0;
    static int timer_overflow_5 = 0;
    // Dichiarazione di tutte le variabili statiche all'inizio della funzione
    static int handshake_init_dbg_printed = 0; // Per HAL_STATE_HANDSHAKE_INIT
    static int handshake_response_dbg_printed = 0; // Per HAL_STATE_HANDSHAKE_WAIT_RESPONSE
    static int comm_state_dbg_printed = 0; // Per HAL_STATE_COMMUNICATING
    static int comm_state_reenter_printed = 1; // Per HAL_STATE_COMMUNICATING
    static int error_state_dbg_printed = 0; // Per HAL_STATE_ERROR (primo ingresso)
    static int error_state_serial_failed = 0; // Per seriale non presente
    static int error_state_serial_configure = 0; // Per seriale non configurata
    static int error_state_hand_packet = 0; // Per pacchetto di handshake di lunghezza non corrispondente
    static int error_state_firmware = 0; // Per firmware errato
    static int error_state_firmware_2 = 0; // Per valori funzioni dentro il firmware errate
    static int error_state_payload = 0; // Per payload in handshake errato
    static int error_state_hand_serial_read = 0;; // Per seriale in handshake errata
    static int error_state_comm_timeout = 0;

    // Nuove flag per il controllo del pacchetto nella fase COMMUNICATING
    static int comm_packet_sent = 0;
    static int comm_packet_received_and_validated = 0; // Per controllare il loop all'interno di COMMUNICATING
	static int comm_parse_error = 0;
	static int old_comm_parse_error = 0;
	static int max_parse_diff = 0;
	static int print_parse_error = 0;
	static int comm_valid_send = 0;
	static int old_comm_valid_send = 0;
	static int comm_valid_received = 0;
	static int old_comm_valid_received = 0;
	static int comm_invalid_send = 0;
	static int old_comm_invalid_send = 0;
	static int comm_invalid_received = 0;
	static int old_comm_invalid_received = 0;
	static int hand_valid_send = 0;
	static int old_hand_valid_send = 0;
	static int hand_valid_received = 0;
	static int old_hand_valid_received = 0;
	static int hand_invalid_send = 0;
	static int old_hand_invalid_send = 0;
	static int hand_invalid_received = 0;
	static int old_hand_invalid_received = 0;
	static int old_pin_debug2 = 0;
	
	static long max_servo_thread_time; 		// input per debug
    static long max_base_thread_time; 		// input per debug
    static long min_servo_thread_time; 		// input per debug
    static long min_base_thread_time;
    
	static long old_usb_loop;
    static long old_usb_time;
    static long min_usb_loop;
    static long min_usb_time;
    static long max_usb_loop;
    static long max_usb_time;
    static uint8_t usb_time_ok;
    
	
	max_servo_thread_time = 0;
	min_servo_thread_time = 2147483647;
	max_base_thread_time = 0;
	min_base_thread_time = 2147483647;
	min_usb_loop = 2147483647;
	min_usb_time = 2147483647;
	max_usb_loop = 0;
	max_usb_time = 0;

    // Buffer per la lettura dei dati grezzi dalla seriale
    // Dovrebbe essere abbastanza grande per il pacchetto più grande, incluso start/end/checksum
    uint8_t serial_rx_buffer[MAX_RX_PAYLOAD_SIZE + 32]; // Aggiungo un po' di margine
    int serial_rx_buffer_len = 0; // Quantità di dati attualmente nel buffer
	int serial_verify;
	static uint8_t tx_packet_buffer[MAX_TX_PAYLOAD_SIZE + 4]; // Buffer per il pacchetto completo
	static int packet_len;
	
	// variabili per poter stampare l'ora
	time_t rawtime;
    struct tm *info_tempo;
    char time_buffer[80];

	
    // Loop principale del thread USB
    while (!hal_shutdown_flag) { // Utilizza !hal_exit per una chiusura pulita del componente
		
		current_time_ns = rtapi_get_time(); // Aggiorna il tempo corrente all'inizio di ogni ciclo

        if (current_time_ns < 0) {
			// Se c'è un overflow, reimposta il tempo a un valore positivo
			// per non far fallire i calcoli successivi
			current_time_ns = current_time_ns & (~0x80000000); // Maschera per il 32° bit
			if (current_time_ns < next_comm_attempt_time_ns - handshake_period_ns) {	//la prima volta che torna a 0 
				next_comm_attempt_time_ns = current_time_ns + handshake_period_ns; // Pianifica il prossimo tentativo
			}
        }
  
        // --- Sezione 1: Apertura e Configurazione della Porta Seriale ---
        // Questa logica viene eseguita solo se la porta non è ancora aperta o se si è chiusa a causa di un errore
        if (io->serial_fd < 0) {
            io->serial_fd = open(io->usb_port_name, O_RDWR | O_NOCTTY | O_NONBLOCK); // O_NONBLOCK è cruciale per letture non bloccanti
            if (io->serial_fd < 0) {
                // Errore nell'apertura della porta
                if (!error_state_serial_failed) {     
					if (verbose >= VERBOSE_USB) {           
						rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: ERROR: \nFailed to open serial port %s: %s \nOR \ncable disconnected\n", io->usb_port_name, strerror(errno));
					}
					*(io->usb_connected) = 0;
					error_state_serial_failed = 1;
					*(io->hal_comm_state) = HAL_STATE_ERROR;
                }
                *(io->hal_comm_state) = HAL_STATE_ERROR; // Segnala lo stato di errore in HAL
                usleep(500000); // Attendi 500ms prima di riprovare ad aprire la porta
                continue; // Salta al prossimo ciclo del loop per riprovare l'apertura
            }

            // Se la porta è stata aperta con successo, procedi alla configurazione
            // Utilizza il baud_rate predefinito o quello passato come parametro del modulo, se ne hai uno
            if (configure_serial_port(io->serial_fd, B115200) != 0) { // Usiamo B115200 come esempio
                *(io->hal_comm_state) = HAL_STATE_ERROR; // Segnala lo stato di errore
                usleep(500000); // Attendi 500ms
                comm_state_reenter_printed = 0;
                continue;
            }

            // Porta aperta e configurata con successo. Resetta lo stato di comunicazione a HANDSHAKE.
            *(io->hal_comm_state) = HAL_STATE_HANDSHAKE;
            serial_rx_buffer_len = 0; // Pulisci il buffer di ricezione
            next_comm_attempt_time_ns = current_time_ns; // Prepara per il primo tentativo di handshake immediato
        } else {
            if (usb_time_ok == 1) {
                loop_current_time_ns = current_time_ns - old_usb_time;
                old_usb_time= current_time_ns;
                if (loop_current_time_ns > 1000000) {
                    if (loop_current_time_ns > max_usb_loop) {
                        max_usb_loop = loop_current_time_ns;
                    }
                    if (loop_current_time_ns < min_usb_loop) {
                        min_usb_loop = loop_current_time_ns;
                    }
                }
            }
        }

		if (*(io->base_thread_time) > 0) {
			if (*(io->base_thread_time) > max_base_thread_time) {
				max_base_thread_time = *(io->base_thread_time);
			}
			if (*(io->base_thread_time) < min_base_thread_time) {
				min_base_thread_time = *(io->base_thread_time);
			}
		}
		if (*(io->servo_thread_time) > 0) {
			if (*(io->servo_thread_time) > max_servo_thread_time) {
				max_servo_thread_time = *(io->servo_thread_time);
			}
			if (*(io->servo_thread_time) < min_servo_thread_time) {
				min_servo_thread_time = *(io->servo_thread_time);
			}
		}


        // --- Sezione 2: Logica della Macchina a Stati ---
        switch (*(io->hal_comm_state)) {
			case HAL_STATE_HANDSHAKE: {
				state_flag = HAL_STATE_HANDSHAKE;
                // Gestione timeout per il tentativo di handshake
                if (current_time_ns >= next_comm_attempt_time_ns) {
                    usb_current_time_ns = rtapi_get_time();
                    // Costruisci il payload per l'handshake (HAL invia input/output NPIC richiesti)
                    uint8_t handshake_payload_tx[2];
                    handshake_payload_tx[0] = (uint8_t)io->output; // Numero di NPIC output richiesti
                    handshake_payload_tx[1] = (uint8_t)io->input;  // Numero di NPIC input richiesti

                    packet_len = build_packet(tx_packet_buffer, sizeof(tx_packet_buffer),
                                                HAND_INIT, HAND_END, handshake_payload_tx, sizeof(handshake_payload_tx));
					
//					rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: MESSAGE IN HANDSHAKE (ASCII):\n '%s'\n", format_buffer_for_debug(tx_packet_buffer, packet_len));
						
                    if (packet_len > 0) {
						int bytes_written = serial_write(io->serial_fd, tx_packet_buffer, packet_len);
						//tcdrain(io->serial_fd);          // <--- QUESTO forza Linux a mandare i dati sul cavo USB ora!
						
						if (bytes_written != packet_len) {
                            *(io->hal_comm_state) = HAL_STATE_ERROR; // O HAL_STATE_HANDSHAKE, a seconda di come vuoi ricominciare														
							hand_invalid_send++;
                        } else {
							*(io->hal_comm_state) = HAL_STATE_HANDSHAKE_WAIT_RESPONSE;
                            io->usb_last_read_time_ns = current_time_ns; // Inizia il timeout per la risposta
                            io->usb_mini_timeout_ns = current_time_ns; // Inizia il mini_timeout per la risposta
                            hand_valid_send++;
                        }
                    } else {
                        
                    }
                    next_comm_attempt_time_ns = current_time_ns + handshake_period_ns; // Pianifica il prossimo tentativo
                }
				break;
			}	// end case HAL_STATE_HANDSHAKE
			
            case HAL_STATE_HANDSHAKE_WAIT_RESPONSE: {
				state_flag = HAL_STATE_HANDSHAKE_WAIT_RESPONSE;
				// Leggi la risposta dall'Arduino
				// Tenta di leggere dati dalla seriale (non bloccante)
				int bytes_read = read(io->serial_fd, serial_rx_buffer + serial_rx_buffer_len, sizeof(serial_rx_buffer) - serial_rx_buffer_len);
				if (bytes_read > 0) {
					serial_rx_buffer_len += bytes_read;

					// Tenta di parsare un pacchetto di handshake response
					uint8_t handshake_payload_rx[10]; // Arduino risponde con 9 byte di payload + 1 byte di firmware (n_encoder, bit_encoder, n_dac, bit_dac, n_adc, bit_adc, in_exp, out_exp, loop_time, firmware)
					int parsed_len = parse_packet(serial_rx_buffer, serial_rx_buffer_len,
												HAND_INIT, HAND_END, handshake_payload_rx, sizeof(handshake_payload_rx));
					if (error_message == 1) {
						*(io->hal_comm_state) = HAL_STATE_HANDSHAKE;
						if (verbose >= VERBOSE_ALL) {
							rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: ERROR MESSAGE IN HANDSHAKE (ASCII): '%s'\n", format_as_ascii_for_debug(serial_rx_buffer, serial_rx_buffer_len));
						}
					} else {
						if (parsed_len > 0) {
							// Pacchetto handshake ricevuto e valido!
							// Analizza il payload ricevuto da Arduino
							if (parsed_len == 10) { // Arduino dovrebbe inviare 10 byte di payload (9 per config + 1 per firmware)
								switch (firmware) {
									case 255:
										// Imposta i valori specifici per il firmware 255
										io->arduino_byte_npic_output = 1;	//variabile calcolata
										io->arduino_byte_npic_input = 1;	//variabile calcolata
										break;

									default:
										io->arduino_byte_npic_output = output / 8;	//variabile calcolata
										io->arduino_byte_npic_input = input / 8;	//variabile calcolata
								}	
								io->arduino_n_encoder = handshake_payload_rx[0];	//payload
								io->arduino_bit_encoder = handshake_payload_rx[1];	//payload
								io->arduino_byte_encoder = (io->arduino_bit_encoder + 7) / 8;	//variabile calcolata
								io->arduino_bytes_encoder = io->arduino_n_encoder * io->arduino_byte_encoder;	//variabile calcolata
								io->arduino_n_dac = handshake_payload_rx[2];	//payload
								io->arduino_bit_dac = handshake_payload_rx[3];	//payload
								io->arduino_byte_dac = (io->arduino_bit_dac + 7) / 8;	//variabile calcolata
								io->arduino_bytes_dac = io->arduino_n_dac * io->arduino_byte_dac;	//variabile calcolata
								io->arduino_n_adc = handshake_payload_rx[4];	//payload
								io->arduino_bit_adc = handshake_payload_rx[5];	//payload
								io->arduino_byte_adc = (io->arduino_bit_adc + 7) / 8;	//variabile calcolata
								io->arduino_bytes_adc = io->arduino_n_adc * io->arduino_byte_adc;	//variabile calcolata
								io->arduino_in_bit_expansion = handshake_payload_rx[6];	//payload
								io->arduino_in_byte_expansion = (io->arduino_in_bit_expansion + 7) / 8;	//variabile calcolata
								io->arduino_out_bit_expansion = handshake_payload_rx[7];	//payload
								io->arduino_out_byte_expansion = (io->arduino_out_bit_expansion + 7) / 8;	//variabile calcolata
								io->arduino_loop_time = handshake_payload_rx[8];	//payload
								io->arduino_firmware = handshake_payload_rx[9];		//payload
								
								io->usb_tx_payload_len  = 	(io->arduino_byte_npic_output) +
															(io->arduino_n_dac * io->arduino_byte_dac) +     // ADC bytes (già corretto)
															(io->arduino_out_byte_expansion);  
															
								io->usb_rx_payload_len = 	(io->arduino_byte_npic_input) + 
															(io->arduino_n_encoder * io->arduino_byte_encoder) + // Encoder bytes (corretto)
															(io->arduino_n_adc * io->arduino_byte_adc) +     // ADC bytes (già corretto)
															(io->arduino_in_byte_expansion); 


								io->loop_time_ns = io->arduino_loop_time * 1000 * 1000;
								if (io->loop_time_ns == 0) { // Evita divisione per zero o loop infinito se Arduino invia 0
									io->loop_time_ns = handshake_period_ns; // Default a 20ms
								}
								
								// Controlla la corrispondenza del firmware
								if (io->arduino_firmware != io->firmware) {
									*(io->hal_comm_state) = HAL_STATE_HANDSHAKE; // Passa allo stato di errore
									if (!comm_state_dbg_printed) {
										comm_state_dbg_printed = 1;
										rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: ERROR: firmware mismatch\nsoftware setup = %d\nhardware setup = %d\nchange the software value\ncomponent stopped\n", io->firmware, io->arduino_firmware);
									}
								} else {
									if ((io->arduino_n_encoder != io->firmware_n_encoder) ||
										(io->arduino_n_dac != io->firmware_n_dac) || 
										(io->arduino_n_adc != io->firmware_n_adc) ||
										(io->arduino_in_bit_expansion != io->firmware_in_bit_expansion) ||
										(io->arduino_out_bit_expansion != io->firmware_out_bit_expansion)) 
										{
											*(io->hal_comm_state) = HAL_STATE_HANDSHAKE; // Passa allo stato di errore
									} else {
										*(io->hal_comm_state) = HAL_STATE_COMMUNICATING; // Passa allo stato di comunicazione
										//comm_parse_error = 0;
										comm_state_dbg_printed = 0;
										io->usb_last_read_time_ns = current_time_ns; // Inizia il timeout per la risposta
										hand_valid_received++;
                                        usb2_current_time_ns = rtapi_get_time();
                                        usb_time_ok = 1;
										rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: INFO: hand_valid_send=%d. hand_valid_received=%d. hand_invalid_send=%d, hand_invalid_received=%d.\n", hand_valid_send, hand_valid_received, hand_invalid_send, hand_invalid_received);

									}
								}
							} else {
								*(io->hal_comm_state) = HAL_STATE_HANDSHAKE; // Potrebbe essere un errore grave, passa a ERROR
							}
							// Pulisci il buffer dopo aver parsato un pacchetto valido
							serial_rx_buffer_len = 0; 
						} else if (parsed_len < 0) {
							// Errore nel parsing (checksum mismatch, buffer troppo piccolo, ecc.)
							// Gestisci il buffer: potresti voler scartare i dati problematici
							serial_rx_buffer_len = 0; // Scarta tutto il buffer su errore di parsing
							hand_invalid_received++;
							// Potrebbe voler ritornare a HAL_STATE_ERROR o riprovare handshake più aggressivamente
							*(io->hal_comm_state) = HAL_STATE_HANDSHAKE;
						}
						// Se parsed_len == 0, significa che il pacchetto non è ancora completo o non trovato.
						// Continua ad accumulare dati nel buffer seriale.
					}
				} else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
					// Errore di lettura non temporaneo
					*(io->hal_comm_state) = HAL_STATE_ERROR;
					continue; // Salta il resto del ciclo e riprova ad aprire la porta
				}
				
				
                // Controlla il timeout per la risposta di handshake
                if (*(io->hal_comm_state) == HAL_STATE_HANDSHAKE_WAIT_RESPONSE && (current_time_ns - io->usb_mini_timeout_ns) > handshake_period_ns) {
 					*(io->hal_comm_state) = HAL_STATE_HANDSHAKE;
                    serial_rx_buffer_len = 0; // Pulisci il buffer su timeout

               }
                break;
            }	// end case HAL_STATE_HANDSHAKE_WAIT_RESPONSE
            		
            case HAL_STATE_COMMUNICATING: {

                state_flag = HAL_STATE_COMMUNICATING;
                error_state_serial_failed = 0;
                error_state_serial_configure = 0;
                error_state_hand_packet = 0;
                error_state_firmware = 0;
                error_state_firmware_2 = 0;
                error_state_payload = 0;
                error_state_hand_serial_read = 0;
                error_state_comm_timeout = 0;
                
                *(io->usb_connected) = 1;
                              
                if (!comm_state_dbg_printed) {
                    rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: Entering COMMUNICATING state. Normal operation.\n");
                    comm_state_dbg_printed = 1;
                    comm_packet_received_and_validated = 0; // Reset for a new cycle
                    last_read_time_ns = rtapi_get_time();
                    if (!comm_state_reenter_printed) {
						comm_state_reenter_printed = 1;
						if (verbose >= VERBOSE_USB) {
							rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Communication restored. \nNormal operation resumed.\n");
						}
					}
                }

                
                if (current_time_ns >= next_comm_attempt_time_ns) {
					
					next_comm_attempt_time_ns = current_time_ns + io->loop_time_ns; // Pianifica il prossimo tentativo di comunicazione					
					//*(io->led) = 1;
					io->usb_data_busy = 1;
//					io->encoder_refresh = 0;
//					io->in_npic_refresh = 0;
					io->out_npic_refresh = 0;
//					io->adc_refresh = 0;
					io->dac_refresh = 0;
//					io->in_exp_refresh = 0;
					io->out_exp_refresh = 0;  
                    if (usb_time_ok == 1) {
					    usb_current_time_ns = rtapi_get_time();
                        usb_time_ok = 0;
                    }

					// --- Sezione di PREPARAZIONE e INVIO del Pacchetto di OUTPUT (HAL -> Arduino) ---
					// Il payload è già stato preparato nel buffer usb_tx_payload
					// dalla funzione del thread realtime e la sua lunghezza è in io->usb_tx_payload_len.

					int hal_output_payload_len = io->usb_tx_payload_len;

					// Calcola la dimensione totale del pacchetto TX (payload + header/footer/checksum)
					int tx_buffer_size = hal_output_payload_len + 3; // Payload + COMM_INIT (1) + Checksum (1) + COMM_END (1)

					// Controlli di sicurezza (prevenire overflow e valori negativi)
					// Assicurati che MAX_TX_PAYLOAD_SIZE sia definito correttamente.
					if (hal_output_payload_len < 0 || hal_output_payload_len > MAX_TX_PAYLOAD_SIZE) {
						*(io->hal_comm_state) = HAL_STATE_HANDSHAKE;

						break;
					}
					if (tx_buffer_size > sizeof(io->tx_packet_buffer)) {
						*(io->hal_comm_state) = HAL_STATE_HANDSHAKE;

						break;
					}

					// Prepara il pacchetto TX completo (incapsulando usb_tx_payload)
					io->tx_packet_buffer[0] = COMM_INIT;
					unsigned char calculated_checksum = 0;
					int current_tx_buffer_idx = 1; // Indice corrente nel tx_packet_buffer per il payload

					// Copia il payload pre-generato da usb_tx_payload
					memcpy(io->tx_packet_buffer + current_tx_buffer_idx, io->usb_tx_payload, hal_output_payload_len);
					current_tx_buffer_idx += hal_output_payload_len;

					// Calcola il checksum sul payload copiato
					for (int i = 1; i < current_tx_buffer_idx; i++) {
						calculated_checksum ^= io->tx_packet_buffer[i];
					}
					io->tx_packet_buffer[current_tx_buffer_idx] = calculated_checksum; // Posiziona il checksum
					io->tx_packet_buffer[current_tx_buffer_idx + 1] = COMM_END;        // Posiziona il byte di fine

					// Invia il pacchetto di OUTPUT di HAL
					int bytes_written = serial_write(io->serial_fd, io->tx_packet_buffer, tx_buffer_size);
					//tcdrain(fd);          // <--- QUESTO forza Linux a mandare i dati sul cavo USB ora!
						 
					if (bytes_written != tx_buffer_size) {
						*(io->hal_comm_state) = HAL_STATE_COMMUNICATING; 
						comm_invalid_send++;
						
					} else {
						//*(io->hal_comm_state) = HAL_STATE_COMMUNICATING;
						*(io->hal_comm_state) = HAL_STATE_COMMUNICATING_WAIT_RESPONSE;
						comm_valid_send++;
//						io->encoder_refresh = 1;
//						io->in_npic_refresh = 1;
						io->out_npic_refresh = 1;
//						io->adc_refresh = 1;
						io->dac_refresh = 1;
//						io->in_exp_refresh = 1;
						io->out_exp_refresh = 1;						
					}

				}
				io->usb_data_busy = 0;
				
				break;
			} // Fine del case HAL_STATE_COMMUNICATING
			
			case HAL_STATE_COMMUNICATING_WAIT_RESPONSE: {
				state_flag = HAL_STATE_COMMUNICATING_WAIT_RESPONSE;	
				//*(io->led) = 0;
				if (current_time_ns < next_comm_attempt_time_ns) {
					io->usb_data_busy = 1;
					io->encoder_refresh = 0;
					io->in_npic_refresh = 0;
//					io->out_npic_refresh = 0;
					io->adc_refresh = 0;
//					io->dac_refresh = 0;
					io->in_exp_refresh = 0;
//					io->out_exp_refresh = 0;
					
					// --- Sezione di RICEZIONE e PARSING del Pacchetto di INPUT (Arduino -> HAL) ---
					// Il payload ricevuto sarà scritto nel buffer usb_rx_payload per l'altro thread.

					// La lunghezza del payload di input atteso è quella basata sui parametri di handshake
					// Applicazione del metodo di calcolo indicato dall'utente per tutte le sezioni.

					// Leggi dalla seriale e accumula nel serial_rx_buffer
					int bytes_read = serial_read(io->serial_fd, serial_rx_buffer + serial_rx_buffer_len, sizeof(serial_rx_buffer) - serial_rx_buffer_len);
					if (bytes_read > 0) {
						serial_rx_buffer_len += bytes_read;
					}/* else {
						break;
					}*/

					// Tenta di parsare pacchetti dal buffer
					int parsed_len;
					do {
						// Tenta di parsare un pacchetto e metti il payload direttamente in io->usb_rx_payload
						parsed_len = parse_packet(serial_rx_buffer, serial_rx_buffer_len, 
											COMM_INIT, COMM_END, io->usb_rx_payload, MAX_RX_PAYLOAD_SIZE);
						if (error_message == 1) {
							*(io->hal_comm_state) = HAL_STATE_COMMUNICATING; 
							if (verbose >= VERBOSE_ALL) {
								rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: ERROR MESSAGE IN COMMUNICATING (ASCII): '%s'\n", format_as_ascii_for_debug(serial_rx_buffer, serial_rx_buffer_len));
							}
						} else {
							if (parsed_len < 0) { // Errore di parsing o checksum
								serial_rx_buffer_len = 0; // Scarta i dati non validi
								*(io->hal_comm_state) = HAL_STATE_COMMUNICATING; // Torna all'handshake per ristabilire
								comm_parse_error++;
								break; // Esci dal loop di parsing
							} else if (parsed_len > 0) { // Pacchetto valido trovato
								// Il payload è già in io->usb_rx_payload
								if (parsed_len == io->usb_rx_payload_len) { //
	//								
									io->usb_last_read_time_ns = current_time_ns; // Aggiorna il timestamp all'ultima ricezione valida
									io->usb_mini_timeout_ns = current_time_ns; // Aggiorna il timestamp all'ultima ricezione valida
									// Rimuovi il pacchetto processato dal buffer seriale
									int packet_total_len = parsed_len + 3; // Payload + COMM_INIT + Checksum + COMM_END
									if (serial_rx_buffer_len >= packet_total_len) {
										memmove(serial_rx_buffer, serial_rx_buffer + packet_total_len, serial_rx_buffer_len - packet_total_len);
										serial_rx_buffer_len -= packet_total_len;
										
									} else {
										serial_rx_buffer_len = 0; // Consumato tutto il buffer
									}
									*(io->hal_comm_state) = HAL_STATE_COMMUNICATING; // 
									*(io->led) = 1;
									comm_valid_received++;
									io->encoder_refresh = 1;
									io->in_npic_refresh = 1;
//									io->out_npic_refresh = 1;
									io->adc_refresh = 1;
//									io->dac_refresh = 1;
									io->in_exp_refresh = 1;
//									o->out_exp_refresh = 1;
                                    usb2_current_time_ns = rtapi_get_time();
                                    usb_time_ok = 1;
									break;
								} else {
									// Lunghezza del payload inaspettata (potrebbe essere un pacchetto di errore formalizzato o corrotto)
									serial_rx_buffer_len = 0; // Scarta il buffer
									*(io->hal_comm_state) = HAL_STATE_COMMUNICATING; // Torna allo stato di handshake
									comm_invalid_received++;
									break; // Esci dal loop di parsing

								}
							} else { // parsed_len == 0, pacchetto incompleto, attendi più dati
								break; // Esci dal loop di parsing, attendi il prossimo ciclo di lettura
							}
						}
						
					} while (serial_rx_buffer_len > 0); // Continua finché ci sono dati nel buffer
					
					io->usb_data_busy = 0;

					
				
				} else {
					*(io->hal_comm_state) = HAL_STATE_COMMUNICATING;
				}
                break; // Fine del case HAL_STATE_COMMUNICATING_WAIT_RESPONSE
            }

            case HAL_STATE_ERROR: {
				state_flag = HAL_STATE_ERROR;
                // In stato di errore, il componente attende e riprova ad aprire la porta (gestito dalla Sezione 1)
                // Oppure potresti voler implementare una logica di recupero specifica.
                *(io->hal_comm_state) = HAL_STATE_HANDSHAKE; // O HAL_STATE_HANDSHAKE
                next_comm_attempt_time_ns = current_time_ns + handshake_period_ns; // Pianifica il prossimo tentativo di handshake
                io->usb_last_read_time_ns = current_time_ns; // Inizia il timeout per la risposta
                close(io->serial_fd); // Chiudi il file descriptor corrente
				io->serial_fd = -1;   // Invalida il file descriptor per forzare la riapertura
				//error_state_serial_failed = 0;
				comm_state_dbg_printed = 0;
				comm_state_reenter_printed = 0;
                usleep(100000); // Attendi 0,1s prima di riprovare
                break;
            }
        } // end switch
        //funzione per il timeout
		if ((current_time_ns - io->usb_last_read_time_ns) > response_timeout_ns) {
			*(io->hal_comm_state) = HAL_STATE_HANDSHAKE;
			io->usb_last_read_time_ns = current_time_ns; // Resetta il timeout per il prossimo tentativo
			serial_rx_buffer_len = 0; // Pulisci il buffer su timeout
			next_comm_attempt_time_ns = current_time_ns + handshake_period_ns; // Pianifica il prossimo tentativo di handshake
			if (verbose >= VERBOSE_MIN) {
				rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Communication timeout. \nReturn to handshake.\n");
			}
		}
		if ((current_time_ns >= (next_comm_attempt_time_ns-(io->loop_time_ns/2)))||(timer_overflow_4 == 1)) {
			timer_overflow_4 = 0;
			*(io->led) = 0;
		}
		

        usb_jitter_time_ns = usb_current_time_ns - usb2_current_time_ns;
        if (usb_jitter_time_ns > 0) {
			if (usb_jitter_time_ns > max_usb_time) {
				max_usb_time = usb_jitter_time_ns;
			}
			if (usb_jitter_time_ns < min_usb_time) {
				min_usb_time = usb_jitter_time_ns;
			}
		}
        
		//funzione di statistica degli errori in comunicazione
		if ((current_time_ns - next_parse_error_time_ns) > parse_error_time_ns) {
			next_parse_error_time_ns = current_time_ns + parse_error_time_ns; // Pianifica il prossimo conto delle statistiche
			float tx_total = 0;
			float tx_success = 0;
			float rx_total = 0;
			float rx_success = 0;
			float error_total = 0;
			float tx_total_delta = 0;
			float tx_success_delta = 0;
			float rx_total_delta = 0;
			float rx_success_delta = 0;
			float error_total_delta = 0;
			
			float servo_jitter_f = 0; 
			float base_jitter_f = 0; 
			float jitter_time = 0; //usb_jitter_time_ns
			float loop_current = 0; //loop_current_time_ns
			
			
			tx_total = (float)comm_valid_send + comm_invalid_send;
			if (tx_total > 0) {
				tx_success = ((float)comm_valid_send / tx_total) * 100.0;
			}
			
			rx_total = (float)comm_valid_received + comm_invalid_received + comm_parse_error;
			if (rx_total > 0) {
				rx_success = ((float)comm_valid_received / rx_total) * 100.0;
			}
			
			if ((rx_total > 0)&&(tx_total > 0)) {
				error_total = ((float)(comm_parse_error + comm_invalid_send) / (tx_total + rx_total)) * 100.0;
			}
			
			tx_total_delta = (float)(comm_valid_send-old_comm_valid_send) + (comm_invalid_send-old_comm_invalid_send);
			if (tx_total_delta > 0) {
				tx_success_delta = ((float)(comm_valid_send-old_comm_valid_send) / tx_total_delta) * 100.0;
			}
			
			rx_total_delta = (float)(comm_valid_received-old_comm_valid_received) + (comm_invalid_received-old_comm_invalid_received) + (comm_parse_error-old_comm_parse_error);
			if (rx_total_delta > 0) {
				rx_success_delta = ((float)(comm_valid_received-old_comm_valid_received) / rx_total_delta) * 100.0;
			}
			
			if ((rx_total_delta > 0)&&(tx_total_delta > 0)) {
				error_total_delta = ((float)((comm_parse_error-old_comm_parse_error) + (comm_invalid_send-old_comm_invalid_send)) / (tx_total_delta + rx_total_delta)) * 100.0;
			}
			
			int parse_diff = comm_parse_error-old_comm_parse_error;	
			if (comm_parse_error <= old_comm_parse_error) {
				parse_diff = 0;
			}	
			old_comm_valid_send = comm_valid_send;
			old_comm_invalid_send = comm_invalid_send;
			old_comm_valid_received = comm_valid_received;
			old_comm_invalid_received = comm_invalid_received;
			old_comm_parse_error = comm_parse_error;
			
			if (max_parse_diff < parse_diff) {
				max_parse_diff = parse_diff;
			}

			int32_t servo_jitter = max_servo_thread_time - min_servo_thread_time;			
			int32_t base_jitter = max_base_thread_time - min_base_thread_time;			
            usb_jitter_time_ns = max_usb_time - min_usb_time;
            loop_current_time_ns = max_usb_loop - min_usb_loop;
			// Formatta la stringa con l'ora e la data
			time(&rawtime);
			info_tempo = localtime(&rawtime);			
			strftime(time_buffer, sizeof(time_buffer), "Time_%H:%M:%S\nDate_%Y-%m-%d", info_tempo);
			*(io->hal_error_count) = error_total;  
			*(io->parse_error_percentile) = error_total_delta;
			if ((print_parse_error > 1) && (verbose >= VERBOSE_MIN)) {
				*(io->servo_jitter_pin) = servo_jitter;
				*(io->base_jitter_pin) = base_jitter;
				*(io->jitter_usb_comm) = usb_jitter_time_ns;
				*(io->jitter_usb_loop) = loop_current_time_ns;
				servo_jitter_f = (float)(servo_jitter / 1000000.0f);
				base_jitter_f = (float)(base_jitter / 1000000.0f);
				jitter_time = (float)(usb_jitter_time_ns / 1000000.0f);
				loop_current = (float)(loop_current_time_ns / 1000000.0f);
				if (error_total_delta>=5) { //5% del periodo di controllo
					rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: ERROR:\nBad USB cable communication\nOr too much Jitter\nparse error=%.1f/100\n%s\njitter.base = %.3fms\njitter.servo = %.3fms\njitter.usb.comm. =%.3fms\njitter.usb.thread =%.3fms\n", 
                                                    error_total_delta, time_buffer, base_jitter_f, servo_jitter_f,
                                                    jitter_time, loop_current);
				} else if (error_total_delta>=1) { //1% del periodo di controllo
					rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: WARNING:\nBad USB cable communication\nparse error=%.1f/100\n%s\njitter.base = %.3fms\njitter.servo = %.3fms\njitter.usb.comm. =%.3fms\njitter.usb.thread =%.3fms\n",
                                                    error_total_delta, time_buffer, base_jitter_f, servo_jitter_f,
                                                    jitter_time, loop_current);
				} else {
					rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: STATISTIC OK: parse error=%.1f/100\n%s\njitter.base = %.3fms\njitter.servo = %.3fms\njitter.usb.comm. =%.3fms\njitter.usb.thread =%.3fms\n",
                                                    error_total_delta, time_buffer, base_jitter_f, servo_jitter_f,
                                                    jitter_time, loop_current);
				}
			}
			print_parse_error++;	//la prime 2 volte che entra non invia il messaggio
			if (print_parse_error > 2) {
				print_parse_error = 2;
			}
			max_servo_thread_time = 0;
			min_servo_thread_time = 2147483647;
			max_base_thread_time = 0;
			min_base_thread_time = 2147483647;
            min_usb_loop = 2147483647;
            min_usb_time = 2147483647;
            max_usb_loop = 0;
            max_usb_time = 0;
			
//			rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder:STATISTIC: 		tx_success_ratio=%.2f. 		rx_success_ratio=%.2f. 		error_total_ratio=%.2f.\n", tx_success, rx_success, error_total);					
//			rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder:STATISTIC: 		tx_success_ratio_delta=%.2f. 		rx_success_ratio_delta=%.2f. 		error_total_ratio_delta=%.2f.\n", tx_success_delta, rx_success_delta, error_total_delta);					
//			rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: STATISTIC:	 comm_valid_send=%d. 	comm_valid_received=%d. 	comm_invalid_send=%d. 	comm_invalid_received=%d. 	comm_parse_error=%d.	parse_diff=%d. 		max_parse_diff=%d.\n", comm_valid_send, comm_valid_received, comm_invalid_send, comm_invalid_received, comm_parse_error, parse_diff, max_parse_diff);

		}	

        // Ritardo per non sovraccaricare la CPU in questo loop non-realtime
        // Questo ritardo è esterno alla logica di timeout delle comunicazioni
        // Serve a dare tempo al sistema operativo di fare altro e non bloccare il core CPU
        rtapi_delay(100000); // 100 microsecondi di ritardo minimo tra i cicli del thread USB
    } // end while (!hal_exit)

    // Se il loop termina (hal_exit è true), chiudi la porta seriale se è aperta
    if (io->serial_fd >= 0) {
        close(io->serial_fd);
        io->serial_fd = -1;
        rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: USB thread exiting. Serial port closed.\n"); // INFO -> ERR
    }

    rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: USB thread finished.\n"); // INFO -> ERR
    return;
}

/***********************************************************************
* REALTIME LOOP CALCULATIONS                     *
************************************************************************/

//npic input dai pulsanti
uint8_t npic_in_routine(hal_io_decoder_t *io, uint8_t* buffer, uint8_t length) {

	uint8_t i;
    uint8_t bytesProcessed = 0; // Contatore dei byte letti dal buffer
    uint8_t currentByte = 0;
    uint8_t current_bit = 0;
    uint8_t bitIndex;
    uint8_t bitIndex_appo;
    uint8_t bitValue;
    
	if (io->in_npic_refresh == 1) {	//questo serve per dare una specie di antirimbalzo. solo al secondo giro il pin hal viene
		io->in_npic_refresh = 0; 	//validato se è uguale al giro precedente. il pin viene aggiornato dopo un loop usb (20ms)
		switch (firmware) {
			case 255:
				// Imposta i valori specifici per il firmware 255
				bitIndex_appo = 4;
				break;

			default:
				bitIndex_appo = 8;
		}
		for (i = 0; i < io->arduino_byte_npic_input; i++) {
			currentByte = buffer[i];
			for (bitIndex = 0; bitIndex < bitIndex_appo; bitIndex++) {
				bitValue = (currentByte >> bitIndex) & 0x01;
				if (bitValue == in_bit_d[current_bit]) {					
					*(io->in_bit[current_bit]) = bitValue;
					if (io->in_bit_t[current_bit] != NULL) {
						if ((bitValue == 1)&&(in_bit_t_state[current_bit] == 0)) {
							*(io->in_bit_t[current_bit]) = !(*(io->in_bit_t[current_bit]));
							in_bit_t_state[current_bit] = 1;
						}
						if (bitValue == 0) {
							in_bit_t_state[current_bit] = 0;
						}
					}	
				} else {
					in_bit_d[current_bit] = bitValue;
				}
                
				current_bit++;
			}  
            bytesProcessed++;   
		}
		if (current_bit != input) {					
			rtapi_print_msg(RTAPI_MSG_WARN, "io_decoder: input extraction: mismatch.\n"); // INFO -> ERR
		}
	}
}

//npic output verso le spie luminose
void npic_out_routine(hal_io_decoder_t *io, uint8_t* dato, uint8_t length) {

	uint8_t i;
    uint8_t currentByte = 0;
    uint8_t current_bit = 0;
    uint8_t bitIndex;
    uint8_t bitIndex_appo;
    uint8_t bitValue;
    long current_time_ns;
    long freq_conversion;
    long freq_conversion2;
    float freq_command;
    float freq_width;

    
	if (io->out_npic_refresh == 1) {	//per non intasare i cicli i valori aggiornano ogni loop USB. il dato trasmesso è in ritardo di un loop
		io->out_npic_refresh = 0; 	
		current_time_ns = rtapi_get_time(); // Aggiorna il tempo corrente all'inizio di ogni ciclo
		if (current_time_ns < 0) {
			// Se c'è un overflow, reimposta il tempo a un valore positivo
			// per non far fallire i calcoli successivi
			current_time_ns = current_time_ns & (~0x80000000); // Maschera per il 32° bit
        }
        switch (firmware) {
			case 255:
				// Imposta i valori specifici per il firmware 255
				bitIndex_appo = 4;
				break;

			default:
				bitIndex_appo = 8;
		}
		for (i = 0; i < io->arduino_byte_npic_output; i++) {
			if (current_time_ns < io->out_blink_time_ns[current_bit]) {
				io->out_blink_time_ns[current_bit] = current_time_ns;
			}
			currentByte = 0;
			for (bitIndex = 0; bitIndex < bitIndex_appo; bitIndex++) {
				bitValue = *(io->out_bit[current_bit]);

				if (io->out_bit_blink_en[current_bit] && *(io->out_bit_blink_en[current_bit]) == 1) {	
					freq_command = 0.25;	// default e minimo 0.25Hz = 4 secondi
					freq_width = 0.5;	// default
					if (*(io->out_bit_blink_freq[current_bit]) >= freq_command) {		
						freq_command = *(io->out_bit_blink_freq[current_bit]);
						if (freq_command > 16) {
							freq_command = 16;;
						}
						freq_conversion = (long)((1.0f / freq_command) * 1000 * 1000 * 1000);
						
					}
					if (bitValue == 1) {	//io->out_blink_time_ns[current_bit]
						if ((*(io->out_bit_blink_width[current_bit]) > 0)&&(*(io->out_bit_blink_width[current_bit]) < 1)) {
							freq_width = *(io->out_bit_blink_width[current_bit]);
						}	
						freq_conversion2 = (long)(((1.0f / freq_command)*freq_width) * 1000 * 1000 * 1000);
						if (current_time_ns > (io->out_blink_time_ns[current_bit] + freq_conversion)) {	
							bitValue = 1;
							io->out_blink_time_ns[current_bit] = current_time_ns;
						} else if (current_time_ns > (io->out_blink_time_ns[current_bit] + (freq_conversion2))) {	
							bitValue = 0;
						}
					}
				}
				
				if (bitValue == 1) {
					currentByte |= (1 << bitIndex); // Imposta il bit corrente (LSB first)
				} else {
                    currentByte &= ~(1 << bitIndex);
                }				
				current_bit++;
			}  
			dato[i] = currentByte;
        }
		if (current_bit != output) {					
			rtapi_print_msg(RTAPI_MSG_WARN, "io_decoder: output insection: mismatch.\n"); // INFO -> ERR
		}
	}
}

uint8_t encoder_routine(hal_io_decoder_t *io, uint8_t* buffer, uint8_t length) {

	int encoder_idx;
    int encoder_delta_from_arduino; // Questo sarà il valore decodificato (il "delta")
    int i;
    uint8_t bytesProcessed = 0; // Contatore dei byte letti dal buffer
 
	if (io->encoder_refresh == 1) {
		io->encoder_refresh = 0; 
		
		for (i = 0; i < io->firmware_n_encoder; i++) {

            if (bytesProcessed + io->arduino_byte_encoder > length) {
                break; // Non c'è più spazio nel buffer, esci
            }

            int32_t currentDelta = 0; // Useremo un int32_t per gestire il valore del delta

            if (io->arduino_bit_encoder <= 8) {
                // Caso: Encoder a 8 bit (1 byte)
                uint8_t inputByte = buffer[bytesProcessed];
                bytesProcessed += 1;
			
                if (inputByte & 0x80) { // Controlla l'MSB (bit di segno)
                    // Se l'MSB è 1, il numero è negativo
                    io->enc_delta_sign[i] = 1;
                    currentDelta = (int32_t)(inputByte & 0x7F);
                    io->enc_delta[i] = currentDelta * 2;
                    currentDelta = currentDelta * -1;
                } else {
                    // Se l'MSB è 0, il numero è positivo
                    io->enc_delta_sign[i] = 0;
                    currentDelta = (int32_t)(inputByte & 0x7F);
                    io->enc_delta[i] = currentDelta * 2;
                }
            } else { // Presumiamo 16 bit se > 8 (2 byte)
                // Caso: Encoder a 16 bit
                // L'Arduino invia MSB-first, quindi ricostruiamo il valore a 16 bit
                uint16_t inputWord = ((uint16_t)buffer[bytesProcessed] << 8) | buffer[bytesProcessed + 1];
                bytesProcessed += 2;

                if (inputWord & 0x8000) { // Controlla l'MSB (bit di segno)
                    // Se l'MSB è 1, il numero è negativo
                    io->enc_delta_sign[i] = 1;
                    currentDelta = (int32_t)(inputWord & 0x7FFF);
                    io->enc_delta[i] = currentDelta * 2;
                    currentDelta = currentDelta * -1;
                } else {
                    // Se l'MSB è 0, il numero è positivo
                    io->enc_delta_sign[i] = 0;
                    currentDelta = (int32_t)(inputWord & 0x7FFF);
                    io->enc_delta[i] = currentDelta * 2;
                }
            }

            // Applica l'inversione se il pin HAL enc_invert è TRUE
            if (io->enc_invert[i] && *(io->enc_invert[i]) == 1) {
                currentDelta = -currentDelta;
            }

            // Aggiorna il valore cumulativo del pin s32
            *(io->enc[i]) += currentDelta;

        }
			
	}
//	genera gli impulsi per i pin di hal enc_up ed enc_down rispettivi
	for (encoder_idx = 0; encoder_idx < io->firmware_n_encoder; encoder_idx++) {
		if (io->enc_delta[encoder_idx] > 0) {
			if (io->enc_delta[encoder_idx] % 2 == 0) {
				// se il valore è pari
				if (io->enc_delta_sign[encoder_idx] == 0) { // Se il bit di segno è 0 (corrisponde a negativo)
					*(io->enc_down[encoder_idx])= 1;
				} else { // Se il bit di segno è 1 (corrisponde a positivo)
					*(io->enc_up[encoder_idx]) = 1;
				}
			} else {
				//se il valore è dispari
				*(io->enc_up[encoder_idx]) = 0;
				*(io->enc_down[encoder_idx]) = 0;
			}
			io->enc_delta[encoder_idx]--;
		} else { 
			io->enc_delta[encoder_idx] = 0;
			*(io->enc_up[encoder_idx]) = 0;
			*(io->enc_down[encoder_idx]) = 0;
		}		
	}
	
	return bytesProcessed; // Restituisce il numero totale di byte che sono stati letti
}

int32_t map(int32_t valore, int32_t min_vecchio, int32_t max_vecchio, int32_t min_nuovo, int32_t max_nuovo) {
    return (valore - min_vecchio) * (max_nuovo - min_nuovo) / (max_vecchio - min_vecchio) + min_nuovo;
}

uint8_t adc_routine(hal_io_decoder_t *io, uint8_t* buffer, uint8_t length) {
	
    int i;
    uint8_t bytesProcessed = 0; // Contatore dei byte letti dal buffer
    float_t current_value = 0; // Useremo un int32_t per gestire il valore del adc
    uint32_t input_value =0;
//    int adc_idx;
    long current_time_ns;
    long overflow_period_ns = 20 * 1000 * 1000; // Periodo di 20ms per resettare l'overflow
	int32_t joy_deadb;
	int32_t joy_offset;
	//uint32_t joy_log;
	float_t max_value = (1 << io->arduino_bit_adc) - 1;
	int32_t max_value_2 = max_value/2;
	int32_t joy_center;
	int32_t joy_center_plus;
	int32_t joy_center_minus;
	int32_t max_joy_ms;
	int32_t min_joy_ms;
	long time_plus_ns;
	long time_minus_ns;
	int joy_plus_ok = 0;
	int joy_minus_ok = 0;
	
    
    
    
	if (io->adc_refresh == 1) {
		io->adc_refresh = 0; 	    	
		for (i = 0; i < io->firmware_n_adc; i++) {        
            if (bytesProcessed + io->arduino_byte_adc > length) {
                break; // Non c'è più spazio nel buffer, esci
            }
            
            current_time_ns = rtapi_get_time(); // Aggiorna il tempo corrente all'inizio di ogni ciclo

			if (current_time_ns < 0) {
				// Se c'è un overflow, reimposta il tempo a un valore positivo
				// per non far fallire i calcoli successivi
				current_time_ns = current_time_ns & (~0x80000000); // Maschera per il 32° bit
				if (current_time_ns < io->next_adc_joy_ns[i]) {	//la prima volta che torna a 0 
					io->next_adc_joy_ns[i] = current_time_ns; // Pianifica il prossimo tentativo
				}
			}

            if (io->arduino_bit_adc <= 8) {
                // Caso: adc a 8 bit (1 byte)
                uint8_t inputByte = buffer[bytesProcessed];
                bytesProcessed += 1;
                *(io->adc_raw[i]) = (uint32_t)inputByte;
//				current_value = (float_t)inputByte;  
				input_value = (uint32_t)inputByte;

            } else { // Presumiamo 16 bit se > 8 (2 byte)
                // Caso: adc a 16 bit
                // L'Arduino invia MSB-first, quindi ricostruiamo il valore a 16 bit
                uint16_t inputWord = ((uint16_t)buffer[bytesProcessed] << 8) | buffer[bytesProcessed + 1];
                bytesProcessed += 2;
                *(io->adc_raw[i]) = (uint32_t)inputWord;
//				current_value = (float_t)inputWord;   
				input_value = (uint32_t)inputWord;
            }

			// Applica il filtro passa-basso
            io->old_adc_raw[i] = (input_value * ALPHA) + (io->old_adc_raw[i] * (1.0 - ALPHA));
			current_value = (float_t)io->old_adc_raw[i];
			
			// Applica l'inversione se il pin HAL adc_invert è TRUE
            if (io->adc_invert[i] && *(io->adc_invert[i]) == 1) {
                current_value = max_value - current_value;
            }
            
            io->adc_raw_joy[i] = current_value;
                        
            		
			if ((*(io->adc_joy_deadb[i]) > 0) &&(*(io->adc_joy_deadb[i]) < 1)) {
				joy_deadb = (int32_t)(max_value_2 * (*(io->adc_joy_deadb[i])));
			} else {
				joy_deadb = 0;
			}
			if ((*(io->adc_joy_center[i]) > -1) &&(*(io->adc_joy_center[i]) < 1)) {
				joy_center = (int32_t)(max_value_2 + (max_value_2 * (*(io->adc_joy_center[i]))));
			} else {
				joy_center = max_value_2;
			}
			max_joy_ms = 2000;
			min_joy_ms = 100;
			
			if ((*(io->adc_joy_factor[i]) >= 0) &&(*(io->adc_joy_factor[i]) <= 0.999)) {
//				max_joy_ms = (int32_t)(max_joy_ms * (1 - *(io->adc_joy_factor[i])));
				min_joy_ms = (int32_t)(min_joy_ms * (1 - *(io->adc_joy_factor[i])));
//				min_joy_ms = (int32_t)(1000*(1-*(io->adc_joy_factor[i])));
            } else {
                min_joy_ms = (int32_t)(min_joy_ms * 0.001);
			}

			joy_center_plus = joy_center + joy_deadb;
			joy_center_minus = joy_center - joy_deadb;
			
			if (io->adc_joy_valido[i] == 0) {
				if ((io->adc_raw_joy[i] <= joy_center_plus) && (io->adc_raw_joy[i] >= joy_center_minus)) {
					// Consideriamo il valore valido per la prima volta.
					// Potremmo anche aggiungere un controllo per vedere se è già nel centro.
					io->adc_joy_valido[i] = 1; 
				}	
			}		
			//io->adc_joy_valido[i] = 1;	
			
			if (io->adc_raw_joy[i] > (joy_center_plus)) {
				time_plus_ns = (long)map((io->adc_raw_joy[i] - joy_center_plus), 0, (max_value-joy_center_plus), max_joy_ms, min_joy_ms);
				time_plus_ns = time_plus_ns * 1000 * 1000;
				time_minus_ns = 0;
				joy_plus_ok = 1;
				joy_minus_ok = 0;
			} else if (io->adc_raw_joy[i] < (joy_center_minus)) {
				time_minus_ns = (long)map((joy_center_minus - io->adc_raw_joy[i]), 0, joy_center_minus, max_joy_ms, min_joy_ms);
				time_minus_ns = time_minus_ns * 1000 * 1000;
				time_plus_ns = 0;
				joy_minus_ok = 1;
				joy_plus_ok = 0;
			} else {
				joy_plus_ok = 0;	
				joy_minus_ok = 0;
				time_plus_ns = 0;
				time_minus_ns = 0;
				io->next_adc_joy_ns[i] = current_time_ns; // Pianifica il prossimo tentativo immediatamente
			}
			if ((*(io->hal_comm_state) == HAL_STATE_COMMUNICATING) || (*(io->hal_comm_state) == HAL_STATE_COMMUNICATING_WAIT_RESPONSE)) {
				if (io->adc_joy_valido[i]) {	
					if (joy_plus_ok == 1) {
						if (current_time_ns > io->next_adc_joy_ns[i]) {	
							io->next_adc_joy_ns[i] = current_time_ns + time_plus_ns; // Pianifica il prossimo tentativo
							*(io->adc_joy_pulse_up[i]) = 1;
							(*(io->adc_joy_count[i]))++;
						}
						if (current_time_ns > (io->next_adc_joy_ns[i]-(time_plus_ns/2))) {	
							*(io->adc_joy_pulse_up[i]) = 0;
						}
					} else {
						*(io->adc_joy_pulse_up[i]) = 0;	
					}
					
					if (joy_minus_ok == 1) {
						if (current_time_ns > io->next_adc_joy_ns[i]) {	
							io->next_adc_joy_ns[i] = current_time_ns + time_minus_ns; // Pianifica il prossimo tentativo
							*(io->adc_joy_pulse_down[i]) = 1;
							(*(io->adc_joy_count[i]))--;
						}
						if (current_time_ns > (io->next_adc_joy_ns[i]-(time_minus_ns/2))) {	
							*(io->adc_joy_pulse_down[i]) = 0;
						}
					} else {
						*(io->adc_joy_pulse_down[i]) = 0;	
					}
				}
			} else {
				*(io->adc_joy_pulse_up[i]) = 0;
				*(io->adc_joy_pulse_down[i]) = 0;
				io->adc_joy_valido[i] = 0;
				
			}	

            // controllo per evitare moltiplicazione negativa
			float_t scale_check;
			if (*(io->adc_scale[i]) > 0) {
                scale_check = *(io->adc_scale[i]);
            } else {
				scale_check = 1;
            }
            // Aggiorna il valore cumulativo del pin float
            *(io->adc[i]) = (current_value / max_value) * scale_check;
            
// controllo per debug			
			if (i == 0) {
				float appo_time;
				if (time_plus_ns != 0) {
					appo_time = (float)((time_plus_ns/1000)/1000);
					*(io->debug_f) = (1000 * (1/appo_time));
				} else {
					*(io->debug_f) = 0;	
				}	
				if (time_minus_ns != 0) {
					appo_time = (float)((time_minus_ns/1000)/1000);
					*(io->debug_f2) = (1000 * (1/appo_time));
				} else {
					*(io->debug_f2) = 0;	
				}
					
			}

        }
	
	}

    
	
//	for (adc_idx = 0; adc_idx < io->firmware_n_adc; adc_idx++) {
		
	//}

}

void DAC_out_routine(hal_io_decoder_t *io, uint8_t* dato, uint8_t length) {

	int i;
    uint8_t bytesWritten = 0; // Contatore dei byte letti dal buffer
    float_t current_value = 0; // Useremo un int32_t per gestire il valore del adc
    
	if (io->dac_refresh == 1) {
		io->dac_refresh = 0; 
		float_t max_value = (1 << io->arduino_bit_dac) - 1;
		for (int i = 0; i < io->firmware_n_dac; i++) {
			if ((bytesWritten + io->arduino_byte_dac) > length) { // Controlla lo spazio disponibile nel buffer
				break; // Non c'è spazio sufficiente, esce dal loop
			}
			// controllo per evitare divisione con 0 o negativa
			float_t scale_check;
			if (*(io->dac_scale[i]) > 0) {
                scale_check = *(io->dac_scale[i]);
            } else {
				scale_check = 1;
            }
			// applica la conversione per andare ad inserire il valore
            current_value = (*(io->dac[i]) / scale_check) * max_value;
			// Applica l'inversione se il pin HAL adc_invert è TRUE
            if (io->dac_invert[i] && *(io->dac_invert[i]) == 1) {
                current_value = max_value - current_value;
            }

            
			if (io->arduino_bit_dac <= 8) {
				dato[bytesWritten++] = (uint8_t)current_value; // Scrive il byte nel buffer
			} else {
				uint16_t outputWord = (uint16_t)current_value;
				dato[bytesWritten++] = (uint8_t)((outputWord >> 8) & 0xFF); // Byte alto
				dato[bytesWritten++] = (uint8_t)(outputWord & 0xFF);     // Byte basso
			}
			
		}
		
	}
	
}

uint8_t in_expansion_routine(hal_io_decoder_t *io, uint8_t* buffer, uint8_t length) {

	int i;
    uint8_t bytesWritten = 0; // Contatore dei byte letti dal buffer
    float_t current_value = 0; // Useremo un int32_t per gestire il valore
    
	if (io->in_exp_refresh == 1) {
		io->in_exp_refresh = 0; 
		// Qui implementiamo la logica basata sull'EXPANSION_CODE
		switch (io->arduino_firmware) {
			case 101: // EXPANSION_CODE = 0: "Nessuna espansione"
				// Se non c'è espansione di input o è solo un placeholder,
				// scrivi i byte ma non fare nulla.
				break;

			// --- PREPARAZIONE PER FUTURE ESPANSIONI DI OUTPUT ---
			// case 1: // Esempio: Espansione per ricevere valori da apparato esterno
			//     
			//     break;

			// case 2: // Esempio: Espansione per ADC esterno SPI/I2C
			//
			//     break;

			default:
				// Gestione per codici di espansione non riconosciuti
				// 
				// Non fare nulla sull'hardware o metti in uno stato sicuro se puoi.
				break;
		}
	}
}

void out_expansion_routine(hal_io_decoder_t *io, uint8_t* dato, uint8_t length) {

	int i;
    uint8_t bytesWritten = 0; // Contatore dei byte letti dal buffer
    float_t current_value = 0; // Useremo un int32_t per gestire il valore
    
	if (io->out_exp_refresh == 1) {
		io->out_exp_refresh = 0; 
		// Qui implementiamo la logica basata sull'EXPANSION_CODE
		switch (io->arduino_firmware) {
			case 101: // EXPANSION_CODE = 0: "Nessuna espansione"
				// Se non c'è espansione di output o è solo un placeholder,
				// leggi i byte ma non fare nulla sull'hardware.
				break;

			// --- PREPARAZIONE PER FUTURE ESPANSIONI DI OUTPUT ---
			// case 1: // Esempio: Espansione per controllo relè/LED (output digitale)
			//     
			//     break;

			// case 2: // Esempio: Espansione per DAC esterno SPI/I2C
			//
			//     break;

			default:
				// Gestione per codici di espansione non riconosciuti
				// 
				// Non fare nulla sull'hardware o metti in uno stato sicuro se puoi.
				break;
		}
	}
}

static void emit(int fd, int type, int code, int value)
{
    struct input_event ev;
    
    // Assicurati che l'inizializzazione sia sicura
    memset(&ev, 0, sizeof(ev)); 

    ev.type = type;
    ev.code = code;
    ev.value = value;
    
    // Inserisci l'evento nel kernel (questa è la parte che DEVE avvenire nel thread RT)
    // Non devi gestire qui gli errori di scrittura (per stabilità RT)
    write(fd, &ev, sizeof(ev)); 
}

void key_simulation_loop(void *arg) {
    
    hal_io_decoder_t *io = (hal_io_decoder_t *)arg;
    
    int i,j;
    uint8_t bitValue;
    uint32_t keysym;
    uint32_t keysym_modifier;
    int send_status;
    long current_time_ns;
    
    
    current_time_ns = rtapi_get_time(); // Aggiorna il tempo corrente all'inizio di ogni ciclo

	if (current_time_ns < 0) {
		// Se c'è un overflow, reimposta il tempo a un valore positivo
		// per non far fallire i calcoli successivi
		current_time_ns = current_time_ns & (~0x80000000); // Maschera per il 32° bit
		if (current_time_ns < next_key_attempt_time_ns - KEY_SCAN_DELAY_NS) {	//la prima volta che torna a 0 
			next_key_attempt_time_ns = current_time_ns + KEY_SCAN_DELAY_NS; // Pianifica il prossimo tentativo
		}
	}
    
    if (current_time_ns >= next_key_attempt_time_ns) {
		    
		for (j = 1; j < 4; j++) {
			for (i = 0; i < input; i++) {
				// FILTRA LA SCANSIONE PER TIPO (MODIFICATORE PRIMA, NORMALE DOPO)
				if (((j == KEY_TYPE_MODIFIER)||(j == KEY_TYPE_BOTH))&&(key_input_map[i].key_type == j )) {
					key_input_map[i].current_state = *(io->in_bit[i]);
					// LOGICA DI TRANSIZIONE (Down e Up)
					if (key_input_map[i].current_state != key_input_map[i].previous_state) {
						
						//keysym = key_input_map[i].keycode_value;
						keysym_modifier = key_input_map[i].modifier_value;
						
						int suppress_base_key = 0; // 0 = false, 1 = true
						
						// *** CHIAMATA KEY DOWN ***
						if (key_input_map[i].current_state == 1) {
							if (keysym_modifier != 0) {
								switch (keysym_modifier) {
									case KEY_LEFTSHIFT:
										if (shift_count == 0) {
											emit(io->uinput_fd, EV_KEY, keysym_modifier, 1);
											emit(io->uinput_fd, EV_SYN, SYN_REPORT, 0); // Non dimenticare la sincronizzazione!
										}
										shift_count++;
										break;
									case KEY_RIGHTCTRL:
										if (ctrl_count == 0) {
											emit(io->uinput_fd, EV_KEY, keysym_modifier, 1);
											emit(io->uinput_fd, EV_SYN, SYN_REPORT, 0); // Non dimenticare la sincronizzazione!
											suppress_base_key = 1;
										}
										ctrl_count++;
										break;
									case KEY_LEFTALT:
										if (l_alt_count == 0) {
											//keysym = 0;
											emit(io->uinput_fd, EV_KEY, keysym_modifier, 1);
											emit(io->uinput_fd, EV_SYN, SYN_REPORT, 0); // Non dimenticare la sincronizzazione!
										}
										l_alt_count++;
										break;
									case KEY_RIGHTALT:
										if (r_alt_count == 0) {
											emit(io->uinput_fd, EV_KEY, keysym_modifier, 1);
											emit(io->uinput_fd, EV_SYN, SYN_REPORT, 0); // Non dimenticare la sincronizzazione!
										}
										r_alt_count++;
										break;
									default:
										
										break;
										
								}

							}
							
						}
						
						// *** CHIAMATA KEY UP ***
						if (key_input_map[i].current_state == 0) {							
							if (keysym_modifier != 0) {
								switch (keysym_modifier) {
									case KEY_LEFTSHIFT:
										shift_count--;
										if (shift_count <= 0) {
											shift_count = 0;
											emit(io->uinput_fd, EV_KEY, keysym_modifier, 0);
											emit(io->uinput_fd, EV_SYN, SYN_REPORT, 0); // Non dimenticare la sincronizzazione!
										}
										break;
									case KEY_RIGHTCTRL:
										ctrl_count--;
										if (ctrl_count <= 0) {
											ctrl_count = 0;
											emit(io->uinput_fd, EV_KEY, keysym_modifier, 0);
											emit(io->uinput_fd, EV_SYN, SYN_REPORT, 0); // Non dimenticare la sincronizzazione!
										}
										break;
									case KEY_LEFTALT:
										l_alt_count--;
										if (l_alt_count <= 0) {
											l_alt_count = 0;
											emit(io->uinput_fd, EV_KEY, keysym_modifier, 0);
											emit(io->uinput_fd, EV_SYN, SYN_REPORT, 0); // Non dimenticare la sincronizzazione!
										}
										break;
									case KEY_RIGHTALT:
										r_alt_count--;
										if (r_alt_count <= 0) {
											r_alt_count = 0;
											emit(io->uinput_fd, EV_KEY, keysym_modifier, 0);
											emit(io->uinput_fd, EV_SYN, SYN_REPORT, 0); // Non dimenticare la sincronizzazione!
										}
										break;
									default:
										
										break;
										
								}
								
							}

						}

						if (j == KEY_TYPE_MODIFIER) {
							// Aggiorna lo stato precedente DOPO l'azione
							key_input_map[i].previous_state = key_input_map[i].current_state;
						}
					}
	
				}
				if (j == KEY_TYPE_NORMAL) {
					if ((key_input_map[i].key_type == KEY_TYPE_NORMAL)||(key_input_map[i].key_type == KEY_TYPE_BOTH)) {
						key_input_map[i].current_state = *(io->in_bit[i]);
						// LOGICA DI TRANSIZIONE (Down e Up)
						if (key_input_map[i].current_state != key_input_map[i].previous_state) {
							
							keysym = key_input_map[i].keycode_value;
							if (key_input_map[i].current_state == 1) {
								if (keysym != 0) {
									emit(io->uinput_fd, EV_KEY, keysym, 1);
									emit(io->uinput_fd, EV_SYN, SYN_REPORT, 0); // Non dimenticare la sincronizzazione!
								}
							}
							if (key_input_map[i].current_state == 0) {
								if (keysym != 0) {
									emit(io->uinput_fd, EV_KEY, keysym, 0);
									emit(io->uinput_fd, EV_SYN, SYN_REPORT, 0); 
								}
							}
							
							key_input_map[i].previous_state = key_input_map[i].current_state;
						}
						
					}
				}
			}
		}
		next_key_attempt_time_ns = current_time_ns + KEY_SCAN_DELAY_NS; // Pianifica il prossimo tentativo

	}
}

// Funzione per mappare il prefisso al codice KEY_ del modificatore
static int get_modifier_keycode(char prefix_char) {
    switch (prefix_char) {
        case 's': return KEY_LEFTSHIFT;
        case 'S': return KEY_LEFTSHIFT;
        
        case 'c': return KEY_RIGHTCTRL;
        case 'C': return KEY_RIGHTCTRL;
        
        case 'a': return KEY_LEFTALT;
        case 'A': return KEY_LEFTALT;
        
        case 'g': return KEY_RIGHTALT;
        case 'G': return KEY_RIGHTALT;
        
        case '0': return 0;
        case '1': return 0;
        case '2': return 0;
        case '3': return 0;
        case '4': return 0;
        case '5': return 0;
        case '6': return 0;
        case '7': return 0;
        case '8': return 0;
        case '9': return 0;
        
        default:  return -1; // Nessun modificatore
    }
}

int load_key_map(const char *filename)
{
    FILE *fp;
    char line[256];
    char temp_line[256];
    char *token;
    char pin_name[32]; 
    int keycode, index_found;
    int line_count = 0;
    int board_num, pin_num;
    int i;
    char key_config_str[32]; // Spazio per la stringa "s45"
    int modifier_key = 0; // NUOVA VARIABILE
    uint32_t base_key_code = 0; // NUOVA VARIABILE
    char prefix_char = 0; // NUOVA VARIABILE

	for (i = 0; i < MAX_INPUT; i++) {
		key_input_map[i].current_state = 0; 
		key_input_map[i].previous_state = 0;
		key_input_map[i].key_type = 0;
		key_input_map[i].keycode_value = 0;
		key_input_map[i].modifier_value = 0;
		key_input_map[i].both_loop = 0;
	}

    fp = fopen(filename, "r");
    if (fp == NULL) {
		if (verbose >= VERBOSE_USB) {
			rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Warning! Key-map file \n%s \nnot found\nThe component is loaded without keyboard functionallity", filename);
        }
        keyboard_initialized_ok = 0;
        return 0;
    } else {
		keyboard_initialized_ok = 1;
		rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: Loading custom keymap from: %s\n", filename);
    }

	if (keyboard_initialized_ok == 1) {
		while (fgets(line, sizeof(line), fp)) {
			line_count++;
			
			// Ignora righe vuote o commenti (#)
			if (line[0] == '\n' || line[0] == '#' || line[0] == '\r') continue;
			
			// Trova il primo '#' nella riga e tronca la stringa lì.
            char *comment_start = strchr(line, '#');
            if (comment_start != NULL) {
                *comment_start = '\0'; // Sostituisci '#' con terminatore di stringa
            }

			// Lavoriamo su una copia, poiché strtok modifica la stringa originale
			strncpy(temp_line, line, sizeof(temp_line) - 1);
			temp_line[sizeof(temp_line) - 1] = '\0';
			
			// 1. Estrai il primo token (Nome del Pin: in.NN-PP)
			// Delimitatori: Spazio (' ') e Tabulazione ('\t')
			token = strtok(temp_line, " \t");
			if (token == NULL) {
				rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Warning!\nKeyboard configuration file\nLine %d: Missing pin name.\n", line_count);
				continue;
			}
			strncpy(pin_name, token, sizeof(pin_name) - 1);
			pin_name[sizeof(pin_name) - 1] = '\0';

			// 2. Estrai il secondo token (Codice Tastiera)
			token = strtok(NULL, " \t\n\r"); // Aggiungi newline/carriage return come delimitatori
			if (token == NULL) {
				rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Warning!\nKeyboard configuration file\nLine %d: \nMissing keycode for pin %s.\n", line_count, pin_name);
				continue;
			}
			//keycode = atoi(token); // Converte la stringa del Keycode in intero

			// Copia il token in una variabile locale per lavorare (es. "s45")
			strncpy(key_config_str, token, sizeof(key_config_str) - 1);
			key_config_str[sizeof(key_config_str) - 1] = '\0';
			
			// 1. Controlla il prefisso
			prefix_char = key_config_str[0];
			modifier_key = get_modifier_keycode(prefix_char);

			if (modifier_key != 0) {
				if (modifier_key > 0) {
					// key_config_str + 1 punta al numero (es. "45")
					base_key_code = (uint32_t)strtol(key_config_str + 1, NULL, 10);
					keycode = 1;
				} else {
					rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Warning!\nKeyboard configuration file\nLine %d:\nIncorrect modifier!\nInserted: %c\nAllowed: s, c, a, g .\n", line_count, prefix_char);
					keycode = 0;
					continue;
				}	
			} else {
				// Nessun modificatore (es. "45"): L'intera stringa è il tasto base
				base_key_code = (uint32_t)strtol(key_config_str, NULL, 10);
				keycode = 0;
			}

			// -------------------------------------------------------------------
			// Formato: "in." (letterale) seguito da un intero (%d) - (trattino) seguito da un intero (%d)
			if (sscanf(pin_name, "in.%d-%d", &board_num, &pin_num) != 2) {
				// Errore: la stringa non corrisponde al formato atteso
				rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Warning!\nKeyboard configuration file\nInvalid pin format: \nInserted: %s.\nCorrect format: in.MM-N\nData Ignored.\n", pin_name);
				continue;
			}
			
			index_found = (board_num * 8)+ pin_num;
			key_input_map[index_found].keycode_value = base_key_code;
			if ((index_found >= input)||(pin_num > 7)) {
				// Errore: la stringa non corrisponde al formato atteso
				index_found = (input-1)/8;
				rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Warning!\nKeyboard configuration file\nInvalid pin format: \nValues outside declared limits\nMax in.%d-N\nMax in.MM-7\nInserted: %s.\nData Ignored.\n", index_found, pin_name);
				continue;
			}
			
			// Controlla se il keycode è uno dei modificatori noti.
			// Non è necessario usare tutti i modificatori, solo quelli che ti interessano.
			if (base_key_code == KEY_LEFTSHIFT || base_key_code == KEY_RIGHTSHIFT ||
				base_key_code == KEY_LEFTCTRL  || base_key_code == KEY_RIGHTCTRL  ||
				base_key_code == KEY_LEFTALT   || base_key_code == KEY_RIGHTALT) 
			{			
				key_input_map[index_found].key_type = KEY_TYPE_MODIFIER;
				key_input_map[index_found].keycode_value = base_key_code;
				key_input_map[index_found].modifier_value = base_key_code;
			} else {
				key_input_map[index_found].key_type = KEY_TYPE_NORMAL;
			}

			if (keycode == 1) {
				key_input_map[index_found].key_type = KEY_TYPE_BOTH;
				key_input_map[index_found].modifier_value = (uint32_t)modifier_key;
			}
			
			if (base_key_code > 0) {
				in_bit_keycode_value[keycode_number] = base_key_code;
				keycode_number++;
			}
			if (key_input_map[index_found].modifier_value > 0) {
				in_bit_keycode_value[keycode_number] = key_input_map[index_found].modifier_value;
				keycode_number++;
			}
				 
		}
		if (keycode_number == 0) {
			keyboard_initialized_ok = 0; 
			if (verbose >= VERBOSE_USB) {
				rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Warning!\nKeyboard configuration file empty\nNo configured pins.\nThe component is loaded without keyboard functionality\n");
			}
		}
	}
	fclose(fp);
	
    return 0; 
}

static int init_uinput(hal_io_decoder_t *addr)
{
    struct uinput_user_dev uidev;
    int i;
    int retval = 0; // Codice di ritorno locale
    
    // Variabili statiche globali popolate da load_key_map (si assumono disponibili)
    extern uint32_t in_bit_keycode_value[]; 
    extern int keycode_number;              
    
    if (uinput_chmod_cmd != NULL && strlen(uinput_chmod_cmd) > 0) {
		// 1. Apertura del file descriptor
		//system("chmod 0666 /dev/uinput");
		system(uinput_chmod_cmd);
	} else {
        rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: Automatic /dev/uinput permission change disabled by user.\n");
    }
    addr->uinput_fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
    if (addr->uinput_fd < 0) {
        //rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Errore nell'apertura di /dev/uinput. Dispositivo tastiera disabilitato (Permessi?).\n");
        if (verbose >= VERBOSE_USB) {
			rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Error opening /dev/uinput (Code %d): %s\n", errno, strerror(errno));
        }
        addr->uinput_fd = -1; 
        return -1; // Ritorna errore critico
    }
    
    rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: Opening /dev/uinput OK. Configuring keyboard...\n");

    // 2. Abilitazione del tipo di evento "Key" e del tipo "Sincronizzazione"
    if (ioctl(addr->uinput_fd, UI_SET_EVBIT, EV_KEY) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: ioctl UI_SET_EVBIT EV_KEY Error.\n");
        retval = -1;
    }
    if (ioctl(addr->uinput_fd, UI_SET_EVBIT, EV_SYN) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: ioctl UI_SET_EVBIT EV_SYN Error.\n");
        retval = -1;
    }
    
    // 3. ABILITAZIONE DINAMICA DEI KEYCODE
    for (i = 0; i < keycode_number; i++) { 
        uint32_t current_keycode = in_bit_keycode_value[i];

        if (current_keycode > 0) { 
            if (ioctl(addr->uinput_fd, UI_SET_KEYBIT, current_keycode) < 0) {
                 rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: ioctl UI_SET_KEYBIT error for keycode %d.\n", current_keycode);
                 // Non usiamo retval qui, un fallimento su un singolo tasto non è critico
            }
        }
    }
    
    // 4. Configurazione del dispositivo
    memset(&uidev, 0, sizeof(uidev));
    snprintf(uidev.name, UINPUT_MAX_NAME_SIZE, "LinuxCNC_io_decoder");
    uidev.id.bustype = BUS_USB;
    uidev.id.vendor  = 0x1A2B; 
    uidev.id.product = 0x3C4D; 
    uidev.id.version = 1;

    // 5. Scrittura della configurazione e Creazione del dispositivo
    if (write(addr->uinput_fd, &uidev, sizeof(uidev)) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Uinput write error.\n");
        retval = -1;
    }
    if (ioctl(addr->uinput_fd, UI_DEV_CREATE) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: ioctl UI_DEV_CREATE Error.\n");
        retval = -1;
    } else {
        rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: Virtual keyboard device 'LinuxCNC_io_decoder' created successfully.\n");
    }
    
    return retval;
}

// Servo thread function (HAL component function)
static void calc_io_decoder(void *arg, long period) {
    hal_io_decoder_t *io = (hal_io_decoder_t *)arg;
   
    int i;
    int bufferSize_out;
    int bytesRead;
    int bufferSize_in;

	if ((io->usb_data_busy == 0) && (*(io->hal_comm_state) == HAL_STATE_COMMUNICATING)) {		
		
		//chiama subroutine delle uscite
		bufferSize_out=io->arduino_byte_npic_output; //grandezza buffer dei npic output
		npic_out_routine(io, &io->usb_tx_payload[0],bufferSize_out); //parte dalla posizione 0 per la quantità dei npic output 
		DAC_out_routine(io, &io->usb_tx_payload[bufferSize_out],io->arduino_bytes_dac); //parte dalla posizione dopo la quantità dei npic output per la quantità dei DAC_BYTES
		bufferSize_out=bufferSize_out + io->arduino_bytes_dac;
		out_expansion_routine(io, &io->usb_tx_payload[bufferSize_out],io->arduino_out_byte_expansion); //parte dalla posizione dopo la quantità dei DAC_BYTES per la quantità dei OUT_EXPANSION_BYTES

		//chiama subroutine degli ingressi
		bytesRead=0;
		bufferSize_in=io->arduino_byte_npic_input;
		bytesRead+=npic_in_routine(io, &io->usb_rx_payload[0],bufferSize_in); //parte dalla posizione 0 per la quantità dei npic input
		bytesRead+=encoder_routine(io, &io->usb_rx_payload[bufferSize_in],io->arduino_bytes_encoder); //parte dalla posizione dopo la quantità dei npic input per la quantità byte degli encoder
		bufferSize_in=bufferSize_in + io->arduino_bytes_encoder;
		bytesRead+=adc_routine(io, &io->usb_rx_payload[bufferSize_in],io->arduino_bytes_adc); //parte dalla posizione dopo la quantità degli encoder per la quantità byte degli ADC
		bufferSize_in=bufferSize_in + io->arduino_bytes_adc;
		bytesRead+=in_expansion_routine(io, &io->usb_rx_payload[bufferSize_in],io->arduino_in_byte_expansion); //parte dalla posizione dopo la quantità degli encoder per la quantità byte degli ADC

		if (keyboard_initialized_ok == 1) {
			key_simulation_loop(io);
        }
        
    } else {
		for (i = 0; i < input; i++) { 
			*(io->in_bit[i]) = 0;
		}
	}

}


/***********************************************************************
* LOCAL FUNCTION DEFINITIONS                         *
************************************************************************/

int rtapi_app_main(void)
{
    //hal_io_decoder_t *io_decoder;
    rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: rtapi_app_main started.\n");  
	

    int n, retval, i;
    int howmany;
    int somma_canali;
    int maxchan;
    int inchan;
    int outchan;
    int minchan;

    n=0;
    
    hal_io_decoder_t *addr;
//    hal_io_decoder_t *io = &(io_decoder_array[n]);
    
    struct uinput_user_dev uidev;
    
    maxchan = MAX_CHAN;
    minchan = MIN_CHAN/2;
    inchan = MAX_INPUT;
    outchan = MAX_OUTPUT;

    switch (firmware) {
        case 255:
            // Imposta i valori specifici per il firmware 255 della versione demo/evaluation
            input = 4;
            output = 4;
            break;

        default:
                somma_canali=input+output;
				if(somma_canali > maxchan) {
					rtapi_print_msg(RTAPI_MSG_ERR,"sum of input + output must be max %d \n",maxchan);
					return -EINVAL;
				}

				if (input % 8 != 0 || input < minchan || input > inchan) {
					rtapi_print_msg(RTAPI_MSG_ERR, "input is %d and must be a multiple of 8, between %d and %d.\n",
								input, minchan, inchan);
					return -EINVAL;
				}

				if (output % 8 != 0 || output < minchan || output > outchan) {
					rtapi_print_msg(RTAPI_MSG_ERR, "output is %d and must be a multiple of 8, between %d and %d.\n",
								output, minchan, outchan);
					return -EINVAL;
				}
    }


    
    if((verbose > VERBOSE_ALL)||(verbose < VERBOSE_NULL)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"value allowed %d to %d. inserted %d \n",VERBOSE_NULL, VERBOSE_ALL, verbose);
        return -EINVAL;
    }
    
    // Controlla se HAL ha fornito un percorso valido dal file .ini
	if (keymap_file != NULL && strlen(keymap_file) > 0) {
		
		// Chiama la funzione di utility per parsificare il file.
		// Il parametro passato è il percorso letto dal file .ini.
		retval = load_key_map(keymap_file);
		
		if (retval != 0) {
			// La funzione load_key_map ha restituito un errore (es. file non trovato)
			rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Fatal error loading keycodes mapping file.\n");
			// In un componente HAL, è meglio uscire in caso di errore di configurazione critico.
			hal_exit(comp_id);
			return -1;
		}
	} else {
		rtapi_print_msg(RTAPI_MSG_WARN, "io_decoder: No path specified for 'keymap-filename'. Using default mapping (0).\n");
	}

    // have good config info, connect to the HAL
    //comp_id = hal_init("io_decoder");
    comp_id = hal_init("io_decoder");
    if (comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: ERROR: hal_init() failed\n");
        return -1;
    }

    howmany = 1;
    io_decoder_array = hal_malloc(howmany * sizeof(hal_io_decoder_t));
    if (io_decoder_array == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "io_decoder: ERROR: hal_malloc() failed\n");
        hal_exit(comp_id);
        hal_shutdown_flag = 1;
        return -1;
    }

	addr = &(io_decoder_array[0]);

    // Azzerare la memoria allocata per la struttura (buona pratica)
    memset(&(io_decoder_array[n]), 0, sizeof(hal_io_decoder_t));

    io_decoder_array[n].input = input; // Copia il valore dalla variabile statica globale 'input'
    io_decoder_array[n].output = output; // Copia il valore dalla variabile statica globale 'output'
    // Copia la stringa del nome della porta USB in sicurezza
    strncpy(io_decoder_array[n].usb_port_name, usb_port_name, sizeof(io_decoder_array[n].usb_port_name) - 1);
    io_decoder_array[n].usb_port_name[sizeof(io_decoder_array[n].usb_port_name) - 1] = '\0'; // Assicurati la terminazione null
    io_decoder_array[n].firmware = firmware; // Copia il valore dalla variabile statica globale 'firmware'

    // Passa i parametri del modulo alla struttura del componente

    switch (firmware) {
        case 101:
            // Imposta i valori specifici per il firmware 101
            io_decoder_array[n].firmware_firmware = 101; // Esempio: nuovo membro nella struct
            io_decoder_array[n].firmware_n_encoder = 2;     //poi 4 
            io_decoder_array[n].firmware_n_dac = 2;
            io_decoder_array[n].firmware_n_adc = 3;
            io_decoder_array[n].firmware_in_bit_expansion = 8;
            io_decoder_array[n].firmware_out_bit_expansion = 8;
            break;

        case 102:
            // Imposta i valori specifici per il firmware 101
            io_decoder_array[n].firmware_firmware = 102; // Esempio: nuovo membro nella struct
            io_decoder_array[n].firmware_n_encoder = 2;     //poi 4 
            io_decoder_array[n].firmware_n_dac = 2;
            io_decoder_array[n].firmware_n_adc = 1;
            io_decoder_array[n].firmware_in_bit_expansion = 8;
            io_decoder_array[n].firmware_out_bit_expansion = 8;
            break;
            
        case 255:
            // Imposta i valori specifici per il firmware 101
            io_decoder_array[n].firmware_firmware = 255; // Esempio: nuovo membro nella struct
            io_decoder_array[n].firmware_n_encoder = 1;     //poi 4 
            io_decoder_array[n].firmware_n_dac = 1;
            io_decoder_array[n].firmware_n_adc = 1;
            io_decoder_array[n].firmware_in_bit_expansion = 8;
            io_decoder_array[n].firmware_out_bit_expansion = 8;
            break;

        default:
            rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: ERROR: Firmware %d not supported.\n", firmware);
            hal_exit(comp_id);
            hal_shutdown_flag = 1;
            return -EINVAL;
    }
	
	// ***************************************************************
    // CHIAMATA ALLA SUBROUTINE UINPUT
    // ***************************************************************
    if (keyboard_initialized_ok == 1) {
		if (init_uinput(addr) < 0) {
			if (verbose >= VERBOSE_USB) {
				rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: Uinput initialization failed. Keyboard simulation will NOT work.\n");
				// Non forziamo l'uscita, permettendo al resto del decoder di funzionare
				// se l'errore non è fatale per le altre funzionalità.
			}
		}
	}

/****************************************************************************************************************
 * **************************************************************************************************************
 * da leggere file .ini prima di questo punto *******************************************************************
 * **************************************************************************************************************
 * *************************************************************************************************************/
 
    // export variables and functions for each io_decoder
    char buf[HAL_NAME_LEN + 1];
    rtapi_snprintf(buf, sizeof(buf), "io_decoder");
    retval = export_io_decoder(n, &(io_decoder_array[n]),buf);
    if (retval < 0) { // Check return value of export_io_decoder
        hal_exit(comp_id);
        hal_shutdown_flag = 1;
        return retval;
    }

    
    // ADDED: Create the USB communication thread
    io_decoder_array[n].usb_thread_id = rtapi_task_new(
        usb_thread_function,
        &(io_decoder_array[n]),
        USB_THREAD_PRIORITY,
        comp_id,
        USB_THREAD_STACKSIZE,
        RTAPI_NO_FP
    );
    

    if (io_decoder_array[n].usb_thread_id < 0) {

        rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: ERROR: Failed to create USB thread\n");
        hal_exit(comp_id);
        hal_shutdown_flag = 1;
        return -1;
    } else { // AGGIUNGI ANCHE IL MESSAGGIO DI SUCCESSO
        rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: USB thread created successfully with ID: %d\n", io_decoder_array[n].usb_thread_id);
        rtapi_task_start(io_decoder_array[n].usb_thread_id, 0);
        rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: USB Task started.\n");

    }



    rtapi_print_msg(RTAPI_MSG_INFO,"io_decoder: installed %d input and %d output\n", input, output);
    hal_ready(comp_id);

    return 0;
}

void rtapi_app_exit(void)
{
    if (io_decoder_array[0].uinput_fd >= 0) {
		rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: Destroying uinput device.\n");
		// Chiama l'ioctl di distruzione prima di chiudere il file descriptor
		ioctl(io_decoder_array[0].uinput_fd, UI_DEV_DESTROY);
		close(io_decoder_array[0].uinput_fd);
	}
    
    // ADDED: Delete the USB thread
    rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: rtapi_app_exit started.\n");
    rtapi_task_delete(io_decoder_array[0].usb_thread_id); // MODIFICATO: rtapi_thread_delete -> rtapi_task_delete
    hal_exit(comp_id);
    hal_shutdown_flag = 1;
    rtapi_print_msg(RTAPI_MSG_INFO, "io_decoder: rtapi_app_exit finished.\n");
}



/***********************************************************************
* LOCAL FUNCTION DEFINITIONS                         *
************************************************************************/

static int export_io_decoder(int num, hal_io_decoder_t * addr,char* prefix)
{
    int retval;
    int i=0;
    int j=0;
    int k=0;

	addr->usb_tx_payload_len =0; // Lunghezza effettiva del payload nel buffer tx
    addr->usb_rx_payload_len =0; // Lunghezza effettiva del payload nel buffer rx

	for (i = 0; i < MAX_RX_PAYLOAD_SIZE; i++) {
		addr->serial_rx_buffer[i]= 0;
	}
    addr->serial_rx_buffer_len =0;
    for (i = 0; i < MAX_TX_PAYLOAD_SIZE; i++) {
		addr->tx_packet_buffer[i]= 0;
	}
    

// export pins

//sezione IO_encoder##############################################

    for (i = 0; i < input; i++) {
//		rtapi_print_msg(RTAPI_MSG_ERR, "io_decoder: i = %d.\n", i);
		if (key_input_map[i].key_type == KEY_TYPE_UNUSED) {
			retval = hal_pin_bit_newf(HAL_OUT, &(addr->in_bit[i]), comp_id,
				"%s.in.%02i-%01i", prefix, j, k);
			if (retval != 0) {
				return retval;
			}
			retval = hal_pin_bit_newf(HAL_OUT, &(addr->in_bit_t[i]), comp_id,
				"%s.in.%02i-%01i.toggle", prefix, j, k);
			if (retval != 0) {
				return retval;
			}
			*(addr->in_bit_t[i]) = 0;
			
		} else { 
			retval = hal_pin_bit_newf(HAL_OUT, &(addr->in_bit[i]), comp_id,
				"%s.in.%02i-%01i-keyboard", prefix, j, k);
		}		
        k++;
        if (k == 8) {
            k=0;
            j++;
		}

		
        *(addr->in_bit[i]) = 0;
        in_bit_d[i] = 0;
        in_bit_t_state[i] = 0;
    }

    i=0;
    j=0;
    k=0;

    for (i = 0; i < output; i++) {
        retval = hal_pin_bit_newf(HAL_IN, &(addr->out_bit[i]), comp_id,
				"%s.out.%02i-%01i", prefix, j, k);
		retval = hal_pin_float_newf(HAL_IN, &(addr->out_bit_blink_freq[i]), comp_id,
				"%s.out.%02i-%01i.blink-freq", prefix, j, k);
		retval = hal_pin_bit_newf(HAL_IN, &(addr->out_bit_blink_en[i]), comp_id,
				"%s.out.%02i-%01i.blink-en", prefix, j, k);
		retval = hal_pin_float_newf(HAL_IN, &(addr->out_bit_blink_width[i]), comp_id,
				"%s.out.%02i-%01i.blink-width", prefix, j, k);		
        k++;	
        if (k == 8) {
            k=0;
            j++;
		}

        if (retval != 0) {
            return retval;
		}
        *(addr->out_bit[i]) = 0;
        *(addr->out_bit_blink_freq[i]) = 1;
        *(addr->out_bit_blink_en[i]) = 0;
        *(addr->out_bit_blink_width[i]) = 0.5;
        addr->out_blink_time_ns[i] = 0;
    }

    


//sezione encoder jog ##############################################

    for (i = 0; i < addr->firmware_n_encoder; i++) {
        
        retval = hal_pin_s32_newf(HAL_OUT, &(addr->enc[i]), comp_id, // Using array for enc
                    "%s.enc.%01i", prefix, i);
        if (retval != 0) {
            return retval;
        }
        *(addr->enc[i]) = 0;

        retval = hal_pin_bit_newf(HAL_IN, &(addr->enc_invert[i]), comp_id, // Using array for enc_invert
                    "%s.enc.%01i.invert", prefix, i);
        if (retval != 0) {
            return retval;
        }
        *(addr->enc_invert[i]) = 0;

        retval = hal_pin_bit_newf(HAL_OUT, &(addr->enc_up[i]), comp_id, // Using array for enc_up
                    "%s.enc.%01i.up", prefix, i);
        if (retval != 0) {
            return retval;
        }
        *(addr->enc_up[i]) = 0;

        retval = hal_pin_bit_newf(HAL_OUT, &(addr->enc_down[i]), comp_id, // Using array for enc_down
                    "%s.enc.%01i.down", prefix, i);
        if (retval != 0) {
            return retval;
        }
        *(addr->enc_down[i]) = 0;
        
        addr->enc_delta[i] = 0;
        addr->enc_delta_sign[i] = 0;
    }

//sezione DAC ##############################################

    for (i = 0; i < addr->firmware_n_dac; i++) {
        retval = hal_pin_float_newf(HAL_IN, &(addr->dac[i]), comp_id, // Using array for dac
                    "%s.dac.%01i", prefix, i);
        if (retval != 0) {
            return retval;
        }
        *(addr->dac[i]) = 0;

        retval = hal_pin_float_newf(HAL_IN, &(addr->dac_scale[i]), comp_id, // Using array for dac
                    "%s.dac.%01i.scale", prefix, i);
        if (retval != 0) {
            return retval;
        }
        *(addr->dac_scale[i]) = 0;
        
        retval = hal_pin_bit_newf(HAL_IN, &(addr->dac_invert[i]), comp_id, // Using array for dac
                    "%s.dac.%01i.invert", prefix, i);
        if (retval != 0) {
            return retval;
        }
        *(addr->dac_invert[i]) = 0;
    }

//sezione ADC ##############################################

    for (i = 0; i < addr->firmware_n_adc; i++) {
        retval = hal_pin_float_newf(HAL_OUT, &(addr->adc[i]), comp_id, // Using array for dac
                    "%s.adc.%01i", prefix, i);
        if (retval != 0) {
            return retval;
        }
        *(addr->adc[i]) = 0;

        retval = hal_pin_float_newf(HAL_IN, &(addr->adc_scale[i]), comp_id, // Using array for dac
                    "%s.adc.%01i.scale", prefix, i);
        if (retval != 0) {
            return retval;
        }
        *(addr->adc_scale[i]) = 0;
        
        retval = hal_pin_bit_newf(HAL_IN, &(addr->adc_invert[i]), comp_id, // Using array for dac
                    "%s.adc.%01i.invert", prefix, i);
        if (retval != 0) {
            return retval;
        }
        *(addr->adc_invert[i]) = 0;
        
        retval = hal_pin_u32_newf(HAL_OUT, &(addr->adc_raw[i]), comp_id, // Using array for dac
                    "%s.adc.%01i.raw", prefix, i);
        if (retval != 0) {
            return retval;
        }
        *(addr->adc_raw[i]) = 0;
        
        retval = hal_pin_float_newf(HAL_IN, &(addr->adc_joy_center[i]), comp_id, // Using array for dac
                    "%s.adc.%01i.joy.center", prefix, i);
        if (retval != 0) {
            return retval;
        }
        *(addr->adc_joy_center[i]) = 0;
        
        retval = hal_pin_float_newf(HAL_IN, &(addr->adc_joy_deadb[i]), comp_id, // Using array for dac
                    "%s.adc.%01i.joy.deadband", prefix, i);
        if (retval != 0) {
            return retval;
        }
        *(addr->adc_joy_deadb[i]) = 0;
        
        retval = hal_pin_float_newf(HAL_IN, &(addr->adc_joy_factor[i]), comp_id, // Using array for dac
                    "%s.adc.%01i.joy.factor", prefix, i);
        if (retval != 0) {
            return retval;
        }
        *(addr->adc_joy_factor[i]) = 0.5;
        
        retval = hal_pin_bit_newf(HAL_OUT, &(addr->adc_joy_pulse_up[i]), comp_id, // Using array for dac
                    "%s.adc.%01i.joy.pulse.up", prefix, i);
        if (retval != 0) {
            return retval;
        }
        *(addr->adc_joy_pulse_up[i]) = 0;
        
        retval = hal_pin_bit_newf(HAL_OUT, &(addr->adc_joy_pulse_down[i]), comp_id, // Using array for dac
                    "%s.adc.%01i.joy.pulse.down", prefix, i);
        if (retval != 0) {
            return retval;
        }
        *(addr->adc_joy_pulse_down[i]) = 0;

        retval = hal_pin_s32_newf(HAL_OUT, &(addr->adc_joy_count[i]), comp_id, // Using array for dac
                    "%s.adc.%01i.joy.count", prefix, i);
        if (retval != 0) {
            return retval;
        }
        *(addr->adc_joy_count[i]) = 0;
        
        addr->old_adc_raw[i] = 0;
        addr->adc_raw_joy[i] = 0;
        addr->next_adc_joy_ns[i] = 0;
        addr->adc_joy_valido[i] = 0;
          
    }

//sezione accessori##############################################

    retval = hal_pin_bit_newf(HAL_OUT, &(addr->led), comp_id,
				"%s.diag.led", prefix);
    if (retval != 0) {
		return retval;
    }
    *(addr->led) = 0;

    retval = hal_pin_float_newf(HAL_OUT, &(addr->debug_f), comp_id,
                "%s.diag.debug-f", prefix); // Use prefix directly
    if (retval != 0) {
        return retval;
    }
    *(addr->debug_f) = 0.0f;
    
    retval = hal_pin_float_newf(HAL_OUT, &(addr->debug_f2), comp_id,
                "%s.diag.debug-f2", prefix); // Use prefix directly
    if (retval != 0) {
        return retval;
    }
    *(addr->debug_f2) = 0.0f;
    
    retval = hal_pin_s32_newf(HAL_IN, &(addr->servo_thread_time), comp_id,
                "%s.diag.servo-thread-time", prefix); // Use prefix directly
    if (retval != 0) {
        return retval;
    }
    *(addr->servo_thread_time) = 0;
    
    retval = hal_pin_s32_newf(HAL_OUT, &(addr->servo_jitter_pin), comp_id,
                "%s.diag.servo-thread-jitter", prefix); // Use prefix directly
    if (retval != 0) {
        return retval;
    }
    *(addr->servo_jitter_pin) = 0;
    
    retval = hal_pin_s32_newf(HAL_IN, &(addr->base_thread_time), comp_id,
                "%s.diag.base-thread-time", prefix); // Use prefix directly
    if (retval != 0) {
        return retval;
    }
    *(addr->base_thread_time) = 0;
    
    retval = hal_pin_s32_newf(HAL_OUT, &(addr->base_jitter_pin), comp_id,
                "%s.diag.base-thread-jitter", prefix); // Use prefix directly
    if (retval != 0) {
        return retval;
    }
    *(addr->base_jitter_pin) = 0;

    // ADDED: Export HAL pins for communication state and error count
    retval = hal_pin_s32_newf(HAL_OUT, &(addr->hal_comm_state), comp_id,
                "%s.diag.comm-state", prefix); // Use prefix directly
    if (retval != 0) {
        return retval;
    }
    *(addr->hal_comm_state) = HAL_STATE_HANDSHAKE;

    retval = hal_pin_float_newf(HAL_OUT, &(addr->hal_error_count), comp_id,
                "%s.diag.error-count", prefix); // Use prefix directly
    if (retval != 0) {
        return retval;
    }
    *(addr->hal_error_count) = 0.0f;

	retval = hal_pin_float_newf(HAL_OUT, &(addr->parse_error_percentile), comp_id,
                "%s.diag.parse-error", prefix); // Use prefix directly
    if (retval != 0) {
        return retval;
    }
    *(addr->parse_error_percentile) = 0.0f;
               
	retval = hal_pin_bit_newf(HAL_OUT, &(addr->usb_connected ), comp_id,
                "%s.diag.usb-connected", prefix); // Use prefix directly
    if (retval != 0) {
        return retval;
    }
    *(addr->usb_connected ) = 0;


	retval = hal_pin_s32_newf(HAL_OUT, &(addr->jitter_usb_comm), comp_id,
                "%s.diag.usb-communication-jitter", prefix); // Use prefix directly
    if (retval != 0) {
        return retval;
    }
    *(addr->jitter_usb_comm) = 0;
				
	retval = hal_pin_s32_newf(HAL_OUT, &(addr->jitter_usb_loop), comp_id,
                "%s.diag.usb-thread-jitter", prefix); // Use prefix directly
    if (retval != 0) {
        return retval;
    }
    *(addr->jitter_usb_loop) = 0;
    
    // export function for this loop

    char appoggio[HAL_NAME_LEN + 1];
    rtapi_snprintf(appoggio, sizeof(appoggio), "%s.update", prefix);

    retval =
	hal_export_funct(appoggio, calc_io_decoder, &(io_decoder_array[num]), 0, 0,
	comp_id);
    if (retval != 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
			"io_decoder: ERROR: update funct export failed\n");
		hal_exit(comp_id);
        hal_shutdown_flag = 1;
		return -1;
    }
    return 0;
}











