/*
 * ==========================================================================
 * LPC1768 Air Quality Monitor - v1.7 (Syntax Fixed)
 * - Removed all non-ASCII characters ("smart quotes", etc.)
 * - Hysteresis logic restored
 * - Percentage calculation fixed
 * - 'const' compile error fixed
 * ==========================================================================
 */

#include <LPC17xx.h>
#include <stdio.h>
#include <string.h>

// --- Pin Definitions (ALS Board) ---
#define BUZZER          (1 << 11)
#define LCD_DATA_MASK   (0xF << 23)
#define LCD_RS          (1 << 27)
#define LCD_EN          (1 << 28)

// --- Air Quality States ---
enum AirQualityState { GOOD, MODERATE, POOR, HAZARDOUS };
enum AirQualityState currentState = GOOD;
const char *stateNames[] = {"GOOD", "MODERATE", "POOR", "HAZARD"};

// --- Thresholds now use PPM & AQI directly ---
#define CO_MODERATE_ON   25
#define CO_POOR_ON       35  // Buzzer ON
#define CO_HAZARD_ON     45

#define AQ_MODERATE_ON   80
#define AQ_POOR_ON       120 // Buzzer ON
#define AQ_HAZARD_ON     200

// Hysteresis "turn-off" thresholds
#define CO_POOR_OFF      30  // Buzzer OFF
#define AQ_POOR_OFF      110 // Buzzer OFF

// Max values for percentage display
#define CO_MAX_PPM 100  // Max PPM for 100%
#define AQI_MAX 500     // Max AQI for 100%

// --- Globals ---
volatile int data_ready = 0;
char rx_buffer[40];
char lcdBuffer[20];
int co_ppm = 0, aqi = 0, temp = 0, hum = 0;
int display_cycle = 0;

// --- Custom LCD Bar Characters ---
unsigned char bar_chars[5][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0x1F},
    {0x00,0x00,0x00,0x00,0x00,0x1F,0x1F,0x1F},
    {0x00,0x00,0x00,0x1F,0x1F,0x1F,0x1F,0x1F},
    {0x00,0x00,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}
};

// --- Timer Setup ---
void initTimer0(void) {
    LPC_SC->PCONP |= (1 << 1);
    uint32_t pclk = SystemCoreClock / 4;
    LPC_TIM0->CTCR = 0x0;
    LPC_TIM0->PR = (pclk / 1000000) - 1;  
    LPC_TIM0->TCR = 0x02;
}

void delayUS(unsigned int us) {
    LPC_TIM0->TCR = 0x02;
    LPC_TIM0->TC = 0;
    LPC_TIM0->TCR = 0x01;
    while (LPC_TIM0->TC < us);
    LPC_TIM0->TCR = 0x00;
}

void delayMS(unsigned int ms) {
    while (ms--) delayUS(1000);
}

// --- LCD Control ---
void lcd_pulse_enable(void) {
    LPC_GPIO0->FIOSET = LCD_EN;
    delayUS(1);
    LPC_GPIO0->FIOCLR = LCD_EN;
    delayUS(1);
}

void lcd_send_nibble(unsigned char nibble) {
    LPC_GPIO0->FIOCLR = LCD_DATA_MASK;
    LPC_GPIO0->FIOSET = ((nibble & 0x0F) << 23);
    lcd_pulse_enable();
}

void lcd_send_byte(unsigned char byte, int is_data) {
    if (is_data) LPC_GPIO0->FIOSET = LCD_RS;
    else LPC_GPIO0->FIOCLR = LCD_RS;

    lcd_send_nibble(byte >> 4);
    lcd_send_nibble(byte & 0x0F);
}

void lcd_command(unsigned char cmd) {
    lcd_send_byte(cmd, 0);
    delayUS(50);
}

void lcd_data(unsigned char data) {
    lcd_send_byte(data, 1);
    delayUS(50);
}

void lcd_create_char(unsigned char location, unsigned char *pattern) {
    lcd_command(0x40 | (location << 3));
    for (int i = 0; i < 8; i++) lcd_data(pattern[i]);
    lcd_command(0x80);
}

void lcd_init(void) {
    LPC_GPIO0->FIODIR |= LCD_DATA_MASK | LCD_RS | LCD_EN;
    delayMS(20);

    lcd_send_nibble(0x03); delayMS(5);
    lcd_send_nibble(0x03); delayUS(100);
    lcd_send_nibble(0x03); delayUS(100);
    lcd_send_nibble(0x02); delayUS(100);

    lcd_command(0x28);
    lcd_command(0x0C);
    lcd_command(0x06);
    lcd_command(0x01);
    delayMS(2);

    for (int i = 0; i < 5; i++) lcd_create_char(i, bar_chars[i]);
}

void lcd_string(const char *str) {
    while (*str) {
        lcd_data(*str++);
    }
}

// --- UART1 Setup ---
void init_uart1(void) {
    LPC_SC->PCONP |= (1 << 4);
    LPC_PINCON->PINSEL0 |= (1 << 30);
    LPC_PINCON->PINSEL1 |= (1 << 0);

    uint32_t pclk = SystemCoreClock / 4;
    uint16_t divisor = pclk / (16 * 9600);

    LPC_UART1->LCR = 0x83;
    LPC_UART1->DLL = divisor & 0xFF;
    LPC_UART1->DLM = (divisor >> 8) & 0xFF;
    LPC_UART1->LCR = 0x03;
    LPC_UART1->FCR = 0x07;
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
        } else if (rx_index < sizeof(rx_buffer) - 1) {
            rx_buffer[rx_index++] = c;
        }
    }
}

// --- State Machine ---
void update_system_state(int co_ppm, int aqi) {
    if (co_ppm > CO_HAZARD_ON || aqi > AQ_HAZARD_ON) {
        currentState = HAZARDOUS;
        LPC_GPIO0->FIOSET = BUZZER;
    } 
    else if (co_ppm > CO_POOR_ON || aqi > AQ_POOR_ON) {
        currentState = POOR;
        LPC_GPIO0->FIOSET = BUZZER;
    }
    else if (co_ppm > CO_MODERATE_ON || aqi > AQ_MODERATE_ON) {
        currentState = MODERATE;
        if (co_ppm < CO_POOR_OFF && aqi < AQ_POOR_OFF) {
            LPC_GPIO0->FIOCLR = BUZZER;
        }
    }
    else { // This is the GOOD state
        currentState = GOOD;
        LPC_GPIO0->FIOCLR = BUZZER;
    }
}

// --- Display Modes ---
void display_mode_1(void) {
    lcd_command(0x80);
    sprintf(lcdBuffer, "CO:%3dppm       ", co_ppm); // Fixed spacing
    lcd_string(lcdBuffer);

    lcd_command(0xC0);
    sprintf(lcdBuffer, "AQI:%3d         ", aqi); // Fixed spacing
    lcd_string(lcdBuffer);
}

void display_mode_2(void) {
    lcd_command(0x80);
    sprintf(lcdBuffer, "Status:%-8s", stateNames[currentState]);
    lcd_string(lcdBuffer);

    lcd_command(0xC0);
    switch(currentState) {
        case GOOD:     lcd_string("Air is Clean!   "); break;
        case MODERATE: lcd_string("Acceptable Air  "); break;
        case POOR:     lcd_string("Sensitive Alert!"); break;
        case HAZARDOUS:lcd_string("Seek Fresh Air! "); break;
        default:       lcd_string("Monitoring...   "); break;
    }
}

void display_mode_3(void) {
    int co_percent = (co_ppm * 100) / CO_MAX_PPM;
    int aq_percent = (aqi * 100) / AQI_MAX;   
    
    if (co_percent > 100) co_percent = 100;
    if (aq_percent > 100) aq_percent = 100;

    lcd_command(0x80);
    sprintf(lcdBuffer, "CO Level: %3d%%", co_percent);
    lcd_string(lcdBuffer);

    lcd_command(0xC0);
    sprintf(lcdBuffer, "AQ Level: %3d%%", aq_percent);
    lcd_string(lcdBuffer);
}

void display_mode_4(void) {
    lcd_command(0x80);
    sprintf(lcdBuffer, "T:%2d\xDF""C  H:%2d%%", temp, hum);
    lcd_string(lcdBuffer);

    lcd_command(0xC0);
    if (hum < 30)       lcd_string("Dry             ");
    else if (hum <=60)  lcd_string("Feels Good      ");
    else                lcd_string("Humid           ");
}

// --- Main ---
int main(void) {
    SystemInit();
    SystemCoreClockUpdate();
    initTimer0();
    lcd_init();
    init_uart1();

    LPC_GPIO0->FIODIR |= BUZZER;
    LPC_GPIO0->FIOCLR = BUZZER;

    lcd_command(0x80); lcd_string("Air Quality Mon.");
    lcd_command(0xC0); lcd_string("Initializing...");
    delayMS(1000);

    int update_counter = 0;

    while (1) {
        if (data_ready) {
            data_ready = 0;

            if (sscanf(rx_buffer, "%d,%d,%d,%d", &co_ppm, &aqi, &temp, &hum) == 4) {
                
                update_system_state(co_ppm, aqi);

                update_counter++;
                if (update_counter >= 5) {
                    update_counter = 0;
                    display_cycle = (display_cycle + 1) % 4;  
                }

                switch(display_cycle) {
                    case 0: display_mode_1(); break;
                    case 1: display_mode_2(); break;
                    case 2: display_mode_3(); break;
                    case 3: display_mode_4(); break;
                }
            } else {
                lcd_command(0x80); lcd_string("Sensor Error    ");
                lcd_command(0xC0); lcd_string("Check Connection");
            }
        }
        delayMS(100);
    }
}
