/*
 * ===================================================================
 * LPC1768 Air Quality Monitor (Receiver)
 *
 * v3.1 - Corrected to parse INTEGER data from Arduino
 * - Expects data as: "190,145\n"
 * - Uses your tuned INTEGER thresholds.
 * ===================================================================
 */

#include <LPC17xx.h>
#include <stdio.h>

// --- Pin Definitions (ALS Board) ---
#define BUZZER   (1 << 11)     // P0.11 (via CNA -> CNA5)
#define LCD_DATA_MASK (0xF << 23)  // P0.23-P0.26 (via CND -> CNAD)
#define LCD_RS    (1 << 27)         // P0.27
#define LCD_EN    (1 << 28)         // P0.28

// --- Air Quality State Logic ---
enum AirQualityState { GOOD, MODERATE, POOR, HAZARDOUS };
enum AirQualityState currentState = GOOD;
const char *stateNames[] = {"GOOD    ", "MODERATE", "POOR    ", "HAZARDOUS"};

// --- INTEGER Thresholds (Based on your tuned values) ---
// MQ7 (CO) thresholds based on normal readings up to 200
#define CO_MODERATE_ON  210
#define CO_POOR_ON      250  // BUZZER ON for CO
#define CO_HAZARD_ON    300

#define CO_MODERATE_OFF 240  // Buzzer OFF for CO
#define CO_GOOD_OFF     205

// MQ135 (AQ) thresholds based on normal readings up to 150
#define AQ_MODERATE_ON  160  // AQ reading goes from 150 up to 160
#define AQ_POOR_ON      180  // BUZZER ON for AQ
#define AQ_HAZARD_ON    250  // Even more critical warning

#define AQ_MODERATE_OFF 175  // Buzzer OFF for AQ
#define AQ_GOOD_OFF     155

// --- Global Variables ---
volatile int data_ready = 0;
char rx_buffer[40];
char lcdBuffer[20];
int co_raw, aq_raw; // Switched to global int

// --- Timer-Based Delay Functions ---

void initTimer0(void) {
    uint32_t pclk;
    
    LPC_SC->PCONP |= (1 << 1); // power on Timer0

    pclk = SystemCoreClock / 4; 
    
    LPC_TIM0->CTCR = 0x0;      // timer mode
    
    // Set prescaler for 1 microsecond (1,000,000 Hz)
    LPC_TIM0->PR = (pclk / 1000000) - 1; 
                                        
    LPC_TIM0->TCR = 0x02;      // reset timer
}

void delayUS(unsigned int us) {
    LPC_TIM0->TCR = 0x02; // reset timer
    LPC_TIM0->TCR = 0x01; // start timer
    while (LPC_TIM0->TC < us);
    LPC_TIM0->TCR = 0x00; // stop timer
}

void delayMS(unsigned int ms) {
    delayUS(ms * 1000);
}

// --- LCD Functions (Now using reliable delays) ---
void lcd_pulse_enable(void) {
    LPC_GPIO0->FIOSET = LCD_EN;
    delayUS(1); // EN pulse must be >450ns
    LPC_GPIO0->FIOCLR = LCD_EN;
    delayUS(1); //
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
    
    lcd_send_nibble(byte >> 4); // Send high nibble
    lcd_send_nibble(byte & 0x0F); // Send low nibble
}

void lcd_command(unsigned char cmd) {
    lcd_send_byte(cmd, 0);
    delayUS(50); // Most commands need ~40us
}

void lcd_data(unsigned char data) {
    lcd_send_byte(data, 1);
    delayUS(50); // Data writes need ~40us
}

void lcd_init(void) {
    LPC_GPIO0->FIODIR |= LCD_DATA_MASK | LCD_RS | LCD_EN;
    
    delayMS(20); // Wait >15ms after power on
    
    lcd_send_nibble(0x03);
    delayMS(5);
    lcd_send_nibble(0x03);
    delayUS(100);
    lcd_send_nibble(0x03);
    delayUS(100);
    
    lcd_send_nibble(0x02);
    delayUS(100);
    
    lcd_command(0x28); // 4-bit, 2 line, 5x7 font
    lcd_command(0x0C); // Display ON, cursor off
    lcd_command(0x06); // Entry mode: increment cursor, no shift
    lcd_command(0x01); // Clear display
    
    delayMS(2); // Wait > 1.6ms for clear display
}

void lcd_string(char *str) {
    while (*str) {
        lcd_data(*str++);
    }
}

// --- UART1 Functions (Robust, 9600 Baud) ---
void init_uart1(void) {
    uint32_t pclk;
    uint16_t baud_divisor;
    
    LPC_SC->PCONP |= (1 << 4); // Power on UART1
    
    LPC_PINCON->PINSEL0 |= (1 << 30); // P0.15 = TXD1
    LPC_PINCON->PINSEL1 |= (1 << 0);  // P0.16 = RXD1
    
    pclk = SystemCoreClock / 4; 
    
    baud_divisor = (pclk / (16 * 9600));
    
    LPC_UART1->LCR = 0x83; // 8-N-1, Enable DLAB
    
    LPC_UART1->DLL = baud_divisor & 0xFF; // Low byte
    LPC_UART1->DLM = (baud_divisor >> 8) & 0xFF; // High byte
    
    LPC_UART1->LCR = 0x03; // Disable DLAB
    
    LPC_UART1->FCR = 0x07; // Enable and reset FIFOs
    
    LPC_UART1->IER = (1 << 0);
    
    NVIC_EnableIRQ(UART1_IRQn);
}

void UART1_IRQHandler(void) {
    static int rx_index = 0;
    char c;
    while (LPC_UART1->LSR & 0x01) {
        c = LPC_UART1->RBR;
        if (c == '\n' || c == '\r') {
            if (rx_index > 0) {
                rx_buffer[rx_index] = '\0';
                data_ready = 1;
                rx_index = 0;
            }
        } else if (rx_index < (sizeof(rx_buffer) - 1)) {
            rx_buffer[rx_index++] = c;
        }
    }
}

// --- State Logic Function (Uses Integer ADC Values) ---
void update_system_state(int co_val, int aq_val) {
    if (co_val > CO_HAZARD_ON || aq_val > AQ_HAZARD_ON) {
        currentState = HAZARDOUS;
        LPC_GPIO0->FIOSET = BUZZER;
    } 
    else if (co_val > CO_POOR_ON || aq_val > AQ_POOR_ON) {
        currentState = POOR;
        LPC_GPIO0->FIOSET = BUZZER;
    }
    else if (co_val > CO_MODERATE_ON || aq_val > AQ_MODERATE_ON) {
        currentState = MODERATE;
        if (co_val < CO_MODERATE_OFF && aq_val < AQ_MODERATE_OFF) {
            LPC_GPIO0->FIOCLR = BUZZER;
        }
    }
    else if (co_val < CO_GOOD_OFF && aq_val < AQ_GOOD_OFF) {
        currentState = GOOD;
        LPC_GPIO0->FIOCLR = BUZZER;
    }
}

// --- Main Program ---
int main(void) {
    int items_parsed;

    SystemInit();
    SystemCoreClockUpdate(); 
    
    initTimer0(); 
    
    lcd_init();
    init_uart1();

    LPC_GPIO0->FIODIR |= BUZZER;
    LPC_GPIO0->FIOCLR = BUZZER; 

    lcd_command(0x80); 
    lcd_string("Air Quality Mon.");
    lcd_command(0xC0); 
    lcd_string("Waiting for data"); // Changed message

    while (1) {
        if (data_ready) {
            data_ready = 0; 
            
            // *** THE CRITICAL CHANGE ***
            // Parse for integers (%d) not floats (%f)
            items_parsed = sscanf(rx_buffer, "%d,%d", &co_raw, &aq_raw);

            if (items_parsed == 2) {
                // 1. Update system state & buzzer
                update_system_state(co_raw, aq_raw);

                // 2. Update LCD
                lcd_command(0x80); // Line 1
                // Display integers (%d)
                // Fixed typo to show "AQ" instead of "NO"
                sprintf(lcdBuffer, "CO:%-5d AQ:%-5d", co_raw, aq_raw);
                lcd_string(lcdBuffer);

                lcd_command(0xC0); // Line 2
                sprintf(lcdBuffer, "State: %s", stateNames[currentState]);
                lcd_string(lcdBuffer);
                
            } else {
                lcd_command(0x80);
                lcd_string("Data Parse Error");
                lcd_command(0xC0);
                lcd_string("                "); 
            }
        }
    }
}
