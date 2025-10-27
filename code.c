/*
 * ===================================================================
 * LPC1768 Air Quality Monitor - ENHANCED DISPLAY VERSION
 *
 * v1.2 - Improved conversion accuracy, added raw value display
 * ===================================================================
 */

#include <LPC17xx.h>
#include <stdio.h>
#include <string.h>
#include <math.h> // Include for floor()

// --- Pin Definitions (ALS Board) ---
#define BUZZER   (1 << 11)
#define LCD_DATA_MASK (0xF << 23)
#define LCD_RS    (1 << 27)
#define LCD_EN    (1 << 28)

// --- Air Quality State Logic ---
enum AirQualityState { GOOD, MODERATE, POOR, HAZARDOUS };
enum AirQualityState currentState = GOOD;
const char *stateNames[] = {"GOOD", "MODERATE", "POOR", "HAZARD"};

// --- Calibration Constants ---
#define CO_BASE_PPM 0
#define CO_MAX_PPM 1000
#define CO_RAW_MIN 190
#define CO_RAW_MAX 400

#define AQ_RAW_MIN 145
#define AQ_RAW_MAX 300
#define AQI_MIN 0
#define AQI_MAX 500

// --- INTEGER Thresholds ---
#define CO_MODERATE_ON  210
#define CO_POOR_ON      250
#define CO_HAZARD_ON    300
#define CO_MODERATE_OFF 240
#define CO_GOOD_OFF     205

#define AQ_MODERATE_ON  160
#define AQ_POOR_ON      180
#define AQ_HAZARD_ON    250
#define AQ_MODERATE_OFF 175
#define AQ_GOOD_OFF     155

// --- Global Variables ---
volatile int data_ready = 0;
char rx_buffer[40];
char lcdBuffer[20];
int co_raw, aq_raw;
int scroll_position = 0;
int display_cycle = 0; // For alternating displays

// --- Custom LCD Characters for Bar Graph ---
unsigned char bar_chars[5][8] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F}, // Empty (Index 0)
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F}, // 1 bar   (Index 1)
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F}, // 2 bars  (Index 2)
    {0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F}, // 3 bars  (Index 3)
    {0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}  // 4 bars  (Index 4)
};

// --- Timer Functions ---
void initTimer0(void) {
    uint32_t pclk;
    LPC_SC->PCONP |= (1 << 1);
    pclk = SystemCoreClock / 4;
    LPC_TIM0->CTCR = 0x0;
    LPC_TIM0->PR = (pclk / 1000000) - 1;
    LPC_TIM0->TCR = 0x02;
}

void delayUS(unsigned int us) {
    LPC_TIM0->TCR = 0x02;
    LPC_TIM0->TCR = 0x01;
    while (LPC_TIM0->TC < us);
    LPC_TIM0->TCR = 0x00;
}

void delayMS(unsigned int ms) {
    delayUS(ms * 1000);
}

// --- LCD Functions ---
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
    if (is_data)
        LPC_GPIO0->FIOSET = LCD_RS;
    else
        LPC_GPIO0->FIOCLR = LCD_RS;
    
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
    int i;
    lcd_command(0x40 | (location << 3)); // Set CGRAM address
    for (i = 0; i < 8; i++) {
        lcd_data(pattern[i]);
    }
    lcd_command(0x80); // Return to DDRAM
}

void lcd_init(void) {
    int i;
    LPC_GPIO0->FIODIR |= LCD_DATA_MASK | LCD_RS | LCD_EN;
    
    delayMS(20);
    lcd_send_nibble(0x03);
    delayMS(5);
    lcd_send_nibble(0x03);
    delayUS(100);
    lcd_send_nibble(0x03);
    delayUS(100);
    lcd_send_nibble(0x02);
    delayUS(100);
    
    lcd_command(0x28);
    lcd_command(0x0C);
    lcd_command(0x06);
    lcd_command(0x01);
    delayMS(2);
    
    // Load custom bar characters
    for (i = 0; i < 5; i++) {
        lcd_create_char(i, bar_chars[i]);
    }
}

void lcd_string(const char *str) { // Added const
    while (*str) {
        lcd_data(*str++);
    }
}

void lcd_clear_line(int line) {
    lcd_command(line == 1 ? 0x80 : 0xC0);
    lcd_string("                ");
}

// --- UART1 Functions ---
void init_uart1(void) {
    uint32_t pclk;
    uint16_t baud_divisor;
    
    LPC_SC->PCONP |= (1 << 4);
    LPC_PINCON->PINSEL0 |= (1 << 30);
    LPC_PINCON->PINSEL1 |= (1 << 0);
    
    pclk = SystemCoreClock / 4;
    baud_divisor = (pclk / (16 * 9600));
    
    LPC_UART1->LCR = 0x83;
    LPC_UART1->DLL = baud_divisor & 0xFF;
    LPC_UART1->DLM = (baud_divisor >> 8) & 0xFF;
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
        } else if (rx_index < (sizeof(rx_buffer) - 1)) {
            rx_buffer[rx_index++] = c;
        }
    }
}

// --- Conversion Functions (Using Float for Accuracy) ---
int convert_co_to_ppm(int raw_value) {
  float ppm = ((float)(raw_value - CO_RAW_MIN) * CO_MAX_PPM) / (float)(CO_RAW_MAX - CO_RAW_MIN);  
	if (raw_value <= CO_RAW_MIN) return CO_BASE_PPM;
    if (raw_value >= CO_RAW_MAX) return CO_MAX_PPM;
    
    // Use float for calculation
    
    return (int)ppm; // Convert back to int
}

int convert_aq_to_aqi(int raw_value) {
	float aqi = ((float)(raw_value - AQ_RAW_MIN) * AQI_MAX) / (float)(AQ_RAW_MAX - AQ_RAW_MIN);
    if (raw_value <= AQ_RAW_MIN) return AQI_MIN;
    if (raw_value >= AQ_RAW_MAX) return AQI_MAX;
    
   
    return (int)aqi; // Convert back to int
}

// --- Display Functions (Bar graph uses float) ---
void draw_bar_graph(int value, int max_value, int num_chars) {
    int i;
    // Use float for calculation
    float filled_float = ((float)value * num_chars * 4.0f) / (float)max_value;
    int filled = (int)floor(filled_float); // Round down
    
    for (i = 0; i < num_chars; i++) {
        int char_level = filled - (i * 4);
        if (char_level >= 4) {
            lcd_data(4); // Full bar (Index 4)
        } else if (char_level > 0) {
            lcd_data(char_level); // Partial bar (Index 1, 2, or 3)
        } else {
            lcd_data(0); // Empty bar (Index 0)
        }
    }
}

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

// **** MODIFIED FOR DEBUGGING ****
void display_mode_1(int co_ppm, int aqi, int co_raw_val, int aq_raw_val) {
    // Mode 1: Show Raw and Converted values
    lcd_command(0x80); // Line 1
    // Format: CO:Raw(PPM)
    sprintf(lcdBuffer, "CO:%d(%d)ppm ", co_raw_val, co_ppm);
    lcd_string(lcdBuffer);
    // Removed bar graph temporarily for space
    
    lcd_command(0xC0); // Line 2
    // Format: AQ:Raw(AQI)
    sprintf(lcdBuffer, "AQ:%d(%d)AQI ", aq_raw_val, aqi);
    lcd_string(lcdBuffer);
     // Removed bar graph temporarily for space
}
// *******************************


void display_mode_2(void) {
    // Mode 2: Status and health advisory
    const char *health_msg;
    
    lcd_command(0x80);
    sprintf(lcdBuffer, "Status: %-8s", stateNames[currentState]);
    lcd_string(lcdBuffer);
    
    lcd_command(0xC0);
    switch(currentState) {
        case GOOD:      health_msg = "Air is Clean!   "; break;
        case MODERATE:  health_msg = "Acceptable Air  "; break;
        case POOR:      health_msg = "Sensitive Alert!"; break;
        case HAZARDOUS: health_msg = "Seek Fresh Air! "; break;
        default:        health_msg = "Monitoring...   "; break;
    }
    lcd_string(health_msg);
}

void display_mode_3(int co_ppm, int aqi) {
    // Mode 3: Numerical display with percentages
    int co_percent = (co_ppm * 100) / CO_MAX_PPM;
    int aq_percent = (aqi * 100) / AQI_MAX;
    
    if (co_percent > 100) co_percent = 100;
    if (aq_percent > 100) aq_percent = 100;
    
    lcd_command(0x80);
    sprintf(lcdBuffer, "CO Lvl:   %3d%%", co_percent);
    lcd_string(lcdBuffer);
    
    lcd_command(0xC0);
    sprintf(lcdBuffer, "AQ Lvl:   %3d%%", aq_percent);
    lcd_string(lcdBuffer);
}

// --- Main Program ---
int main(void) {
    int items_parsed;
    int co_ppm, aqi;
    int update_counter = 0;

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
    lcd_string("Initializing... ");
    delayMS(1000); // Give UART etc. time to settle

    while (1) {
        if (data_ready) {
            data_ready = 0;
            
            items_parsed = sscanf(rx_buffer, "%d,%d", &co_raw, &aq_raw);

            if (items_parsed == 2) {
                // Convert raw values to meaningful measurements
                co_ppm = convert_co_to_ppm(co_raw);
                aqi = convert_aq_to_aqi(aq_raw);
                
                // Update system state using RAW values
                update_system_state(co_raw, aq_raw);

                // Cycle through display modes
                update_counter++;
                if (update_counter >= 5) {
                    update_counter = 0;
                    display_cycle = (display_cycle + 1) % 3;
                }

                // Display based on current mode
                switch(display_cycle) {
                    case 0:
                        // Pass raw values for debugging display
                        display_mode_1(co_ppm, aqi, co_raw, aq_raw);
                        break;
                    case 1:
                        display_mode_2();
                        break;
                    case 2:
                        display_mode_3(co_ppm, aqi);
                        break;
                }
                
            } else {
                lcd_command(0x80);
                lcd_string("Sensor Error    ");
                lcd_command(0xC0);
                lcd_string("Check Connection");
            }
        }
        
        delayMS(100); // Small delay for main loop stability
    }
}
