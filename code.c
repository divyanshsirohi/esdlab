/*
 * ===================================================================
 * LPC1768 Air Quality Monitor (Receiver)
 * ===================================================================
 *
 * Hardware: ALS-SDA-ARMCTXM3-01 (LPC1768)
 * Purpose:  Receives air quality data from an Arduino via UART1,
 * displays PPM values on a 16x2 LCD, and controls
 * a buzzer based on air quality state.
 *
 * Expected Data Format from Arduino (UART1 @ 9600 baud):
 * "CO_PPM,AQ_PPM\n" (e.g., "25.50,120.75\n")
 *
 * --- Connections ---
 * - LPC1768 CNC Pin 1 (P0.15/TXD1) -> Arduino Pin D10 (RX)
 * - LPC1768 CNC Pin 2 (P0.16/RXD1) -> Arduino Pin D11 (TX)
 * - LPC1768 CND Pin 10 (GND)      -> Arduino GND
 * - FRC Cable: CND -> CNAD (for LCD)
 * - FRC Cable: CNA -> CNA5 (for Buzzer)
 *
 */

#include <LPC17xx.h>
#include <stdio.h> // For sscanf and sprintf

// --- Pin Definitions (ALS Board) ---
#define BUZZER   (1 << 11)     // P0.11 (via CNA -> CNA5)
#define LCD_DATA_MASK (0xF << 23)  // P0.23-P0.26 (via CND -> CNAD)
#define LCD_RS    (1 << 27)         // P0.27
#define LCD_EN    (1 << 28)         // P0.28

// --- Air Quality State Logic ---
enum AirQualityState { GOOD, MODERATE, POOR, HAZARDOUS };
enum AirQualityState currentState = GOOD;
const char *stateNames[] = {"GOOD    ", "MODERATE", "POOR    ", "HAZARDOUS"};

/*
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!
 * !!! CRITICAL: TUNE THESE VALUES !!!
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 * These PPM thresholds are ESTIMATES. You MUST calibrate these
 * by observing the values your sensors report in clean air
 * and in the presence of contaminants.
 */

// --- Hysteresis Thresholds (in PPM) ---
// "Turn On" thresholds (when quality is getting worse)
#define CO_MODERATE_ON  10.0f  // (e.g., 10 ppm)
#define CO_POOR_ON      50.0f  // (e.g., 50 ppm) - BUZZER ON
#define CO_HAZARD_ON    100.0f // (e.g., 100 ppm)

#define AQ_MODERATE_ON  20.0f  // (e.g., 20 ppm)
#define AQ_POOR_ON      75.0f  // (e.g., 75 ppm) - BUZZER ON
#define AQ_HAZARD_ON    150.0f // (e.g., 150 ppm)

// "Turn Off" thresholds (when quality is improving)
#define CO_MODERATE_OFF 45.0f  // Buzzer turns OFF
#define CO_GOOD_OFF     8.0f

#define AQ_MODERATE_OFF 70.0f  // Buzzer turns OFF
#define AQ_GOOD_OFF     15.0f

// --- Global Variables ---
volatile int data_ready = 0; // Flag set by UART interrupt
char rx_buffer[40];          // Buffer for incoming UART data
char lcdBuffer[20];          // Buffer for formatting LCD strings

// --- Simple Delay ---
void delay(unsigned int count) {
    unsigned int i;
    for (i = 0; i < count; i++);
}

// --- Timer 0 for Microsecond Delay ---
void delayUS(unsigned int us) {
    LPC_TIM0->TCR = 0x02; // reset timer
    LPC_TIM0->PR = 24;    // prescaler for 1 us tick (assumes 25 MHz PCLK)
    LPC_TIM0->TCR = 0x01; // start timer
    while (LPC_TIM0->TC < us);
    LPC_TIM0->TCR = 0x00; // stop timer
}

void initTimer0(void) {
    LPC_SC->PCONP |= (1 << 1); // power on Timer0
    LPC_TIM0->CTCR = 0x0;      // timer mode
    LPC_TIM0->PR = 0;
    LPC_TIM0->TCR = 0x02;      // reset timer
}

// --- LCD Functions ---
void lcd_pulse_enable(void) {
    LPC_GPIO0->FIOSET = LCD_EN;
    delayUS(1);
    LPC_GPIO0->FIOCLR = LCD_EN;
    delayUS(100);
}

void lcd_send_nibble(unsigned char nibble) {
    LPC_GPIO0->FIOCLR = LCD_DATA_MASK;
    LPC_GPIO0->FIOSET = ((nibble & 0x0F) << 23);
    lcd_pulse_enable();
}

void lcd_send_byte(unsigned char byte, int is_data) {
    if (is_data)
        LPC_GPIO0->FIOSET = LCD_RS;
    else
        LPC_GPIO0->FIOCLR = LCD_RS;
    lcd_send_nibble(byte >> 4);
    lcd_send_nibble(byte & 0x0F);
}

void lcd_command(unsigned char cmd) {
    lcd_send_byte(cmd, 0);
    delay(2000);
}

void lcd_data(unsigned char data) {
    lcd_send_byte(data, 1);
    delay(2000);
}

void lcd_init(void) {
    LPC_GPIO0->FIODIR |= LCD_DATA_MASK | LCD_RS | LCD_EN;
    delay(30000); // wait >15ms after power on
    lcd_command(0x33);
    lcd_command(0x32);
    lcd_command(0x28); // 4-bit, 2 line, 5x7 font
    lcd_command(0x0C); // Display ON, cursor off
    lcd_command(0x06); // Entry mode
    lcd_command(0x01); // Clear display
    delay(3000);
}

void lcd_string(char *str) {
    while (*str) {
        lcd_data(*str++);
    }
}

// --- UART1 Functions ---
void init_uart1(void) {
    LPC_SC->PCONP |= (1 << 4); // Power on UART1
    
    // Config P0.15 as TXD1 and P0.16 as RXD1
    LPC_PINCON->PINSEL0 |= (1 << 30); // P0.15 = TXD1
    LPC_PINCON->PINSEL1 |= (1 << 0);  // P0.16 = RXD1
    
    // 9600 Baud Rate @ 25MHz PCLK
    LPC_UART1->LCR = 0x83; // 8-N-1, Enable DLAB
    LPC_UART1->DLL = 162;  // (25,000,000 / (16 * 9600)) = 162.7
    LPC_UART1->DLM = 0;
    LPC_UART1->LCR = 0x03; // Disable DLAB
    
    LPC_UART1->FCR = 0x07; // Enable and reset FIFOs
    
    // Enable Receive Data Available (RDA) Interrupt
    LPC_UART1->IER = (1 << 0);
    
    NVIC_EnableIRQ(UART1_IRQn);
}

void UART1_IRQHandler(void) {
    static int rx_index = 0;
    char c;

    // While data is in the receive buffer register
    while (LPC_UART1->LSR & 0x01) {
        c = LPC_UART1->RBR;

        if (c == '\n' || c == '\r') {
            if (rx_index > 0) { // Only set flag if we got data
                rx_buffer[rx_index] = '\0'; // Null-terminate
                data_ready = 1;
                rx_index = 0;
            }
        } else if (rx_index < (sizeof(rx_buffer) - 1)) {
            rx_buffer[rx_index++] = c;
        }
    }
}

// --- State Logic Function ---
void update_system_state(float co_ppm, float aq_ppm) {
    // Check for worst-case scenario first
    if (co_ppm > CO_HAZARD_ON || aq_ppm > AQ_HAZARD_ON) {
        currentState = HAZARDOUS;
        LPC_GPIO0->FIOSET = BUZZER; // Turn Buzzer ON
    } 
    // Check for POOR state
    else if (co_ppm > CO_POOR_ON || aq_ppm > AQ_POOR_ON) {
        currentState = POOR;
        LPC_GPIO0->FIOSET = BUZZER; // Turn Buzzer ON
    }
    // Check for MODERATE state
    else if (co_ppm > CO_MODERATE_ON || aq_ppm > AQ_MODERATE_ON) {
        currentState = MODERATE;
        // Turn buzzer off only if levels drop below the OFF threshold (hysteresis)
        if (co_ppm < CO_MODERATE_OFF && aq_ppm < AQ_MODERATE_OFF) {
            LPC_GPIO0->FIOCLR = BUZZER;
        }
    }
    // Check for GOOD state (hysteresis)
    else if (co_ppm < CO_GOOD_OFF && aq_ppm < AQ_GOOD_OFF) {
        currentState = GOOD;
        LPC_GPIO0->FIOCLR = BUZZER; // Turn Buzzer OFF
    }
    // Otherwise, we are in a "hold" state (e.g., CO is 48).
    // We don't change state, and the buzzer stays as it was.
}

// --- Main Program ---
int main(void) {
    float co_ppm, aq_ppm;
    int items_parsed;

    SystemInit();
    SystemCoreClockUpdate();
    initTimer0();
    lcd_init();
    init_uart1();

    // Set Buzzer pin (P0.11) as output
    LPC_GPIO0->FIODIR |= BUZZER;
    LPC_GPIO0->FIOCLR = BUZZER; // Start with buzzer off

    lcd_command(0x80); // Line 1
    lcd_string("Air Quality Mon.");
    lcd_command(0xC0); // Line 2
    lcd_string("Warming up...."); // Arduino is in its 1-min warmup

    while (1) {
        if (data_ready) {
            data_ready = 0; // Clear the interrupt flag
            
            // Parse the PPM string: e.g., "25.50,120.75"
            items_parsed = sscanf(rx_buffer, "%f,%f", &co_ppm, &aq_ppm);

            if (items_parsed == 2) {
                // 1. Update system state & buzzer
                update_system_state(co_ppm, aq_ppm);

                // 2. Update LCD
                lcd_command(0x80); // Line 1
                // Format: "CO:xx.x AQ:xxx.x "
                sprintf(lcdBuffer, "CO:%.1f AQ:%.1f ", co_ppm, aq_ppm);
                lcd_string(lcdBuffer);

                lcd_command(0xC0); // Line 2
                sprintf(lcdBuffer, "State: %s", stateNames[currentState]);
                lcd_string(lcdBuffer);
                
            } else {
                // Data was corrupted or in wrong format
                lcd_command(0x80);
                lcd_string("Data Parse Error");
                lcd_command(0xC0);
                lcd_string("                "); // Clear line 2
            }
        }
    }
}
