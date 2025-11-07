/*
 * ==========================================================================
 * LPC1768 Air Quality Monitor - v2.0 (Simulated ML)
 * - Added "ML" prediction layer
 * - Logic is based on a calculated "hazard score"
 * ==========================================================================
 */

#include <LPC17xx.h>
#include <stdio.h>
#include <string.h>
#include "aq_model.h"

// --- Pin Definitions (ALS Board) ---
#define BUZZER          (1 << 11)
#define LCD_DATA_MASK   (0xF << 23)
#define LCD_RS          (1 << 27)
#define LCD_EN          (1 << 28)

// --- Air Quality States ---
enum AirQualityState { GOOD, MODERATE, POOR, HAZARDOUS };
enum AirQualityState currentState = GOOD;
const char *stateNames[] = {"GOOD", "MODERATE", "POOR", "HAZARD"};


// *** NEW: SIMULATED "ML MODEL" PARAMETERS ***
// These "weights" and "biases" are what your "model" learned.
// (We've just invented them, but they look plausible).

// CO Model: score = (co * w1) + (temp * w2) + (hum * w3) + bias
const float CO_PPM_WEIGHT = 0.8f;
const float CO_TEMP_WEIGHT = 0.15f;
const float CO_HUM_WEIGHT = 0.1f;
const float CO_BIAS = -10.0f;

// AQI Model: score = (aqi * w1) + (temp * w2) + (hum * w3) + bias
const float AQI_VAL_WEIGHT = 0.7f;
const float AQI_TEMP_WEIGHT = -0.1f;
const float AQI_HUM_WEIGHT = 0.25f;
const float AQI_BIAS = 5.0f;


// *** MODIFIED: Thresholds are now for the SCORE, not raw PPM ***
#define CO_SCORE_MODERATE_ON   20.0f
#define CO_SCORE_POOR_ON       30.0f // Buzzer ON
#define CO_SCORE_HAZARD_ON     40.0f

#define AQI_SCORE_MODERATE_ON  70.0f
#define AQI_SCORE_POOR_ON      110.0f // Buzzer ON
#define AQI_SCORE_HAZARD_ON    180.0f

// Hysteresis "turn-off" thresholds for the SCORE
#define CO_SCORE_POOR_OFF      27.0f // Buzzer OFF
#define AQI_SCORE_POOR_OFF     100.0f // Buzzer OFF

// Max values for percentage display
#define CO_MAX_PPM 100
#define AQI_MAX 500

// --- Globals ---
volatile int data_ready = 0;
char rx_buffer[40];
char lcdBuffer[20];
int co_ppm = 0, aqi = 0, temp = 0, hum = 0;
int display_cycle = 0;

// ... (bar_chars definition remains the same) ...
unsigned char bar_chars[5][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0x1F},
    {0x00,0x00,0x00,0x00,0x00,0x1F,0x1F,0x1F},
    {0x00,0x00,0x00,0x1F,0x1F,0x1F,0x1F,0x1F},
    {0x00,0x00,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}
};

// --- Timer Setup ---
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
    LPC_TIM0->TC = 0;
    LPC_TIM0->TCR = 0x01;
    while (LPC_TIM0->TC < us);
    LPC_TIM0->TCR = 0x00;
}

void delayMS(unsigned int ms) {
    while (ms--) delayUS(1000);
}

// ... (LCD functions remain the same) ...
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
    int i; 
    lcd_command(0x40 | (location << 3));
    for (i = 0; i < 8; i++) lcd_data(pattern[i]);
    lcd_command(0x80);
}

void lcd_init(void) {
    int i; 
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
    for (i = 0; i < 5; i++) lcd_create_char(i, bar_chars[i]);
}

void lcd_string(const char *str) {
    while (*str) {
        lcd_data(*str++);
    }
}

// --- UART1 Setup ---
void init_uart1(void) {
    uint32_t pclk;
    uint16_t divisor;
    
    LPC_SC->PCONP |= (1 << 4);
    LPC_PINCON->PINSEL0 |= (1 << 30);
    LPC_PINCON->PINSEL1 |= (1 << 0);
    pclk = SystemCoreClock / 4;
    divisor = pclk / (16 * 9600);
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


// *** NEW: "ML" PREDICTION FUNCTIONS ***

/*
 * =======================================================
 * PREDICTION FUNCTION: predict_co_hazard
 * =======================================================
 * Simulates a trained linear regression model.
 * Features: 
 * - x1: CO PPM (int)
 * - x2: Temperature (int)
 * - x3: Humidity (int)
 * Output: 
 * - y: Hazard Score (float)
 * =======================================================
 */
float predict_co_hazard(int ppm, int temp_c, int hum_pct) {
    float score;
    score = (ppm * CO_PPM_WEIGHT) + 
            (temp_c * CO_TEMP_WEIGHT) + 
            (hum_pct * CO_HUM_WEIGHT) + 
            CO_BIAS;
            
    // Ensure score is not negative
    if (score < 0) score = 0;
    return score;
}

/*
 * =======================================================
 * PREDICTION FUNCTION: predict_aqi_hazard
 * =======================================================
 * Simulates a trained linear regression model.
 * Features: 
 * - x1: AQI (int)
 * - x2: Temperature (int)
 * - x3: Humidity (int)
 * Output: 
 * - y: Hazard Score (float)
 * =======================================================
 */
float predict_aqi_hazard(int aqi_val, int temp_c, int hum_pct) {
    float score;
    score = (aqi_val * AQI_VAL_WEIGHT) + 
            (temp_c * AQI_TEMP_WEIGHT) + 
            (hum_pct * AQI_HUM_WEIGHT) + 
            AQI_BIAS;
            
    // Ensure score is not negative
    if (score < 0) score = 0;
    return score;
}


// *** MODIFIED: State Machine now uses SCORES ***
void update_system_state(float co_score, float aqi_score) {
    if (co_score > CO_SCORE_HAZARD_ON || aqi_score > AQI_SCORE_HAZARD_ON) {
        currentState = HAZARDOUS;
        LPC_GPIO0->FIOSET = BUZZER;
    } 
    else if (co_score > CO_SCORE_POOR_ON || aqi_score > AQI_SCORE_POOR_ON) {
        currentState = POOR;
        LPC_GPIO0->FIOSET = BUZZER;
    }
    else if (co_score > CO_SCORE_MODERATE_ON || aqi_score > AQI_SCORE_MODERATE_ON) {
        currentState = MODERATE;
        // Hysteresis
        if (co_score < CO_SCORE_POOR_OFF && aqi_score < AQI_SCORE_POOR_OFF) {
            LPC_GPIO0->FIOCLR = BUZZER;
        }
    }
    else { // This is the GOOD state
        currentState = GOOD;
        LPC_GPIO0->FIOCLR = BUZZER;
    }
}

// --- Display Modes (Unchanged, they still show PPM/AQI) ---
// --- Display Modes (Fixed Padding) ---
void display_mode_1(void) {
    lcd_command(0x80);
    // Padded to 16 chars: "CO:XXXppm       "
    sprintf(lcdBuffer, "CO:%3dppm       ", co_ppm); 
    lcd_string(lcdBuffer);

    lcd_command(0xC0);
    // Padded to 16 chars: "AQI:XXX         "
    sprintf(lcdBuffer, "AQI:%3d         ", aqi); 
    lcd_string(lcdBuffer);
}

void display_mode_2(void) {
    lcd_command(0x80);
    // Padded to 16 chars: "Status:HAZARD   "
    sprintf(lcdBuffer, "Status:%-8s", stateNames[currentState]);
    lcd_string(lcdBuffer);

    lcd_command(0xC0);
    // These are already 16 chars, so they are fine
    switch(currentState) {
        case GOOD:     lcd_string("Air is Clean!   "); break;
        case MODERATE: lcd_string("Acceptable Air  "); break;
        case POOR:     lcd_string("Sensitive Alert!"); break;
        case HAZARDOUS:lcd_string("Seek Fresh Air! "); break;
        default:       lcd_string("Monitoring...   "); break;
    }
}

void display_mode_3(void) {
    int co_percent;
    int aq_percent;
    
    co_percent = (co_ppm * 100) / CO_MAX_PPM;
    aq_percent = (aqi * 100) / AQI_MAX;   
    
    if (co_percent > 100) co_percent = 100;
    if (aq_percent > 100) aq_percent = 100;

    lcd_command(0x80);
    // Padded to 16 chars: "CO Level: XXX%  "
    sprintf(lcdBuffer, "CO Level: %3d%%  ", co_percent);
    lcd_string(lcdBuffer);

    lcd_command(0xC0);
    // Padded to 16 chars: "AQ Level: XXX%  "
    sprintf(lcdBuffer, "AQ Level: %3d%%  ", aq_percent);
    lcd_string(lcdBuffer);
}

void display_mode_4(void) {
    lcd_command(0x80);
    // Padded to 16 chars: "T:XXCH:XX% "
    sprintf(lcdBuffer, "T:%2d\xDF""C  H:%2d%% ", temp, hum);
    lcd_string(lcdBuffer);

    lcd_command(0xC0);
    // These are already 16 chars, so they are fine
    if (hum < 30)       lcd_string("Dry             ");
    else if (hum <=60)  lcd_string("Feels Good      ");
    else                lcd_string("Humid           ");
}
// --- Main ---
int main(void) {
    int update_counter = 0;
    // *** NEW: Variables for our scores ***
    float co_hazard_score;
    float aqi_hazard_score;
    
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

    while (1) {
        if (data_ready) {
            data_ready = 0;

            if (sscanf(rx_buffer, "%d,%d,%d,%d", &co_ppm, &aqi, &temp, &hum) == 4) {
                
                // *** MODIFIED: Call prediction functions ***
                co_hazard_score = predict_co_hazard(co_ppm, temp, hum);
                aqi_hazard_score = predict_aqi_hazard(aqi, temp, hum);
                
                // Pass the *scores* to the state machine
                update_system_state(co_hazard_score, aqi_hazard_score);

                // This part is the same, it just cycles the display
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
