/*
 * ==========================================================================
 * LPC1768 Air Quality Monitor - v3.0 (Improved ML)
 * - Less strict thresholds
 * - Better ML model weights
 * - Improved hysteresis
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

// *** IMPROVED ML MODEL PARAMETERS ***
// More balanced weights that consider environmental factors properly

// CO Model: Focuses more on CO but considers temperature/humidity effects
const float CO_PPM_WEIGHT = 0.5f;      // Reduced from 0.8
const float CO_TEMP_WEIGHT = 0.05f;    // Reduced impact
const float CO_HUM_WEIGHT = 0.02f;     // Reduced impact
const float CO_BIAS = -5.0f;           // Less negative

// AQI Model: Balanced weights
const float AQI_VAL_WEIGHT = 0.4f;     // Reduced from 0.7
const float AQI_TEMP_WEIGHT = 0.03f;   // Positive now (heat increases pollution)
const float AQI_HUM_WEIGHT = 0.02f;    // Reduced from 0.25
const float AQI_BIAS = -3.0f;          // Slightly negative

// *** IMPROVED THRESHOLDS - Less Strict ***
// CO Score Thresholds
#define CO_SCORE_MODERATE_ON   30.0f   // Was 20
#define CO_SCORE_POOR_ON       50.0f   // Was 30 - Buzzer ON
#define CO_SCORE_HAZARD_ON     75.0f   // Was 40

// AQI Score Thresholds  
#define AQI_SCORE_MODERATE_ON  50.0f   // Was 70
#define AQI_SCORE_POOR_ON      90.0f   // Was 110 - Buzzer ON
#define AQI_SCORE_HAZARD_ON    150.0f  // Was 180

// Hysteresis - wider gap for stability
#define CO_SCORE_POOR_OFF      45.0f   // Was 27
#define AQI_SCORE_POOR_OFF     80.0f   // Was 100

// Display max values
#define CO_MAX_PPM 200  // Changed from 100
#define AQI_MAX 300     // Changed from 500

// --- Globals ---
volatile int data_ready = 0;
char rx_buffer[40];
char lcdBuffer[20];
int co_ppm = 0, aqi = 0, temp = 0, hum = 0;
int display_cycle = 0;

// Buzzer pattern control
int buzzer_enabled = 0;
int buzzer_counter = 0;
#define BUZZER_ON_TIME 10    // 100ms on (10 * 10ms loop delay)
#define BUZZER_OFF_TIME 10   // 100ms off (10 * 10ms loop delay)
#define BUZZER_PATTERN_TOTAL (BUZZER_ON_TIME + BUZZER_OFF_TIME)

// Custom LCD characters for bar graph
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

// *** IMPROVED ML PREDICTION FUNCTIONS ***

/*
 * =======================================================
 * PREDICTION FUNCTION: predict_co_hazard
 * =======================================================
 * Features: CO PPM, Temperature, Humidity
 * Output: Hazard Score (0-100 scale)
 * 
 * Improvements:
 * - Reduced weight on CO for less sensitivity
 * - Minor environmental factor adjustments
 * - Better baseline offset
 * =======================================================
 */
float predict_co_hazard(int ppm, int temp_c, int hum_pct) {
    float score;
    
    // Base score from CO level
    score = ppm * CO_PPM_WEIGHT;
    
    // Temperature adjustment (higher temp = slightly worse)
    score += (temp_c - 20) * CO_TEMP_WEIGHT;
    
    // Humidity adjustment (extreme humidity = slightly worse)
    int hum_deviation = (hum_pct > 60) ? (hum_pct - 60) : 0;
    score += hum_deviation * CO_HUM_WEIGHT;
    
    // Add bias
    score += CO_BIAS;
    
    // Clamp to valid range
    if (score < 0) score = 0;
    if (score > 100) score = 100;
    
    return score;
}

/*
 * =======================================================
 * PREDICTION FUNCTION: predict_aqi_hazard
 * =======================================================
 * Features: AQI, Temperature, Humidity
 * Output: Hazard Score (0-150 scale)
 * 
 * Improvements:
 * - More reasonable AQI weight
 * - Temperature increases pollution perception
 * - Humidity has minimal effect
 * =======================================================
 */
float predict_aqi_hazard(int aqi_val, int temp_c, int hum_pct) {
    float score;
    
    // Base score from AQI
    score = aqi_val * AQI_VAL_WEIGHT;
    
    // Temperature adjustment (heat makes pollution worse)
    score += (temp_c - 20) * AQI_TEMP_WEIGHT;
    
    // Humidity adjustment (minimal effect)
    score += (hum_pct - 50) * AQI_HUM_WEIGHT;
    
    // Add bias
    score += AQI_BIAS;
    
    // Clamp to valid range
    if (score < 0) score = 0;
    if (score > 150) score = 150;
    
    return score;
}

/*
 * =======================================================
 * STATE MACHINE: update_system_state
 * =======================================================
 * Uses scores with proper hysteresis to prevent flickering
 * Buzzer pattern activates only in POOR or HAZARDOUS states
 * =======================================================
 */
void update_system_state(float co_score, float aqi_score) {
    enum AirQualityState previous_state = currentState;
    
    // Determine new state based on scores
    if (co_score >= CO_SCORE_HAZARD_ON || aqi_score >= AQI_SCORE_HAZARD_ON) {
        currentState = HAZARDOUS;
    } 
    else if (co_score >= CO_SCORE_POOR_ON || aqi_score >= AQI_SCORE_POOR_ON) {
        currentState = POOR;
    }
    else if (co_score >= CO_SCORE_MODERATE_ON || aqi_score >= AQI_SCORE_MODERATE_ON) {
        currentState = MODERATE;
    }
    else {
        currentState = GOOD;
    }
    
    // Hysteresis: If transitioning from POOR to MODERATE, check OFF thresholds
    if (previous_state == POOR && currentState == MODERATE) {
        if (co_score >= CO_SCORE_POOR_OFF || aqi_score >= AQI_SCORE_POOR_OFF) {
            currentState = POOR; // Stay in POOR
        }
    }
    
    // Buzzer control - Enable pattern for POOR and HAZARDOUS only
    if (currentState == POOR || currentState == HAZARDOUS) {
        buzzer_enabled = 1;
    } else {
        buzzer_enabled = 0;
        LPC_GPIO0->FIOCLR = BUZZER; // Ensure buzzer is OFF
    }
}

/*
 * =======================================================
 * FUNCTION: update_buzzer_pattern
 * =======================================================
 * Creates a beeping pattern: ON for 100ms, OFF for 100ms
 * Call this function every loop iteration (every 100ms)
 * =======================================================
 */
void update_buzzer_pattern(void) {
    if (!buzzer_enabled) {
        return; // Do nothing if buzzer is disabled
    }
    
    buzzer_counter++;
    if (buzzer_counter >= BUZZER_PATTERN_TOTAL) {
        buzzer_counter = 0; // Reset counter
    }
    
    // Turn buzzer ON for first half of pattern, OFF for second half
    if (buzzer_counter < BUZZER_ON_TIME) {
        LPC_GPIO0->FIOSET = BUZZER;
    } else {
        LPC_GPIO0->FIOCLR = BUZZER;
    }
}

// --- Display Modes ---
void display_mode_1(void) {
    lcd_command(0x80);
    sprintf(lcdBuffer, "CO:%3dppm       ", co_ppm); 
    lcd_string(lcdBuffer);

    lcd_command(0xC0);
    sprintf(lcdBuffer, "AQI:%3d         ", aqi); 
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
    sprintf(lcdBuffer, "CO Level: %3d%%  ", co_percent);
    lcd_string(lcdBuffer);

    lcd_command(0xC0);
    sprintf(lcdBuffer, "AQ Level: %3d%%  ", aq_percent);
    lcd_string(lcdBuffer);
}

void display_mode_4(void) {
    lcd_command(0x80);
    sprintf(lcdBuffer, "T:%2d\xDF""C  H:%2d%% ", temp, hum);
    lcd_string(lcdBuffer);

    lcd_command(0xC0);
    if (hum < 30)       lcd_string("Dry             ");
    else if (hum <=60)  lcd_string("Feels Good      ");
    else                lcd_string("Humid           ");
}

// --- Main ---
int main(void) {
    int update_counter = 0;
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
    delayMS(2000);

    while (1) {
        if (data_ready) {
            data_ready = 0;

            if (sscanf(rx_buffer, "%d,%d,%d,%d", &co_ppm, &aqi, &temp, &hum) == 4) {
                
                // Calculate hazard scores using ML model
                co_hazard_score = predict_co_hazard(co_ppm, temp, hum);
                aqi_hazard_score = predict_aqi_hazard(aqi, temp, hum);
                
                // Update system state based on scores
                update_system_state(co_hazard_score, aqi_hazard_score);

                // Update display cycle every 5 readings
                update_counter++;
                if (update_counter >= 5) {
                    update_counter = 0;
                    display_cycle = (display_cycle + 1) % 4;  
                }

                // Show appropriate display mode
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
        
        // Update buzzer pattern every loop iteration
        update_buzzer_pattern();
        
        delayMS(100);
    }
}
