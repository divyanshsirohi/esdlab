#include <LPC17xx.h>
#include <stdio.h>

// --- Pin Definitions based on ALS Board Manual ---

// Buzzer (via CNA -> CNA5)
[cite_start]#define BUZZER   (1 << 11)     // P0.11 (CNA Pin 8) [cite: 932]

// LCD pins (via CND -> CNAD)
[cite_start]#define LCD_DATA_MASK (0xF << 23)  // P0.23 to P0.26 as D4-D7 [cite: 768]
[cite_start]#define LCD_RS    (1 << 27)         // P0.27 [cite: 768]
[cite_start]#define LCD_EN    (1 << 28)         // P0.28 [cite: 768]

// --- UART Data ---
volatile int data_ready = 0;
char rx_buffer[40];
char lcdBuffer[20];

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
    LPC_SC->PCONP |= (1 << 1); [cite_start]// power on Timer0 [cite: 466]
    LPC_TIM0->CTCR = 0x0;
    LPC_TIM0->PR = 0;
    LPC_TIM0->TCR = 0x02;
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
    delay(30000);
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
    LPC_SC->PCONP |= (1 << 4); [cite_start]// Power on UART1 [cite: 466]
    
    // Config P0.15 as TXD1 and P0.16 as RXD1
    LPC_PINCON->PINSEL0 |= (1 << 30); // P0.15 = TXD1
    LPC_PINCON->PINSEL1 |= (1 << 0);  // P0.16 = RXD1
    
    // 9600 Baud Rate @ 25MHz PCLK
    LPC_UART1->LCR = 0x83; // 8-N-1, Enable DLAB
    LPC_UART1->DLL = 162;
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

// --- Main Program ---
int main(void) {
    int co_raw, aq_raw;
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
    lcd_string("Waiting for data");

    while (1) {
        if (data_ready) {
            data_ready = 0;

            // Parse the simplified string from Arduino
            items_parsed = sscanf(rx_buffer, "%d,%d", &co_raw, &aq_raw);

            if (items_parsed == 2) {
                // Data is valid, update LCD
                lcd_command(0x80); // Line 1
                sprintf(lcdBuffer, "CO Level: %-5d", co_raw);
                lcd_string(lcdBuffer);

                lcd_command(0xC0); // Line 2
                sprintf(lcdBuffer, "AQ Level: %-5d", aq_raw);
                lcd_string(lcdBuffer);

                // --- Buzzer Logic ---
                // NOTE: These are RAW analog values (0-1023).
                // Tune these thresholds based on your sensor's behavior.
                if (co_raw > 600 || aq_raw > 500) {
                    LPC_GPIO0->FIOSET = BUZZER; // Turn Buzzer ON
                } else {
                    LPC_GPIO0->FIOCLR = BUZZER; // Turn Buzzer OFF
                }

            } else {
                // Data is invalid
                lcd_command(0x80);
                lcd_string("Data Parse Error");
                lcd_command(0xC0);
                lcd_string("                "); // Clear line 2
            }
        }
    }
}
