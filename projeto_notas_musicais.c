#include <stdio.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "neopixel.c"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "inc/ssd1306.h"

// Prototipação das funções utilizadas
void pwm_init_buzzer(uint pin);
void tocarNota(int frequency);
void definirNotaMusical();
bool debounceButton(uint pin);
void init_microphone();
void sample_mic();
float estimate_frequency(uint16_t *buffer, uint samples);
void tocarNotaDuracao(int frequency, int duracao);
void tocarSomAcerto();
void tocarSomErro();

#define BUZZER_PIN 21
#define LED_RED 13
#define LED_GREEN 11
#define BUTTON_A 5
#define BUTTON_B 6
// Pino e canal do microfone no ADC.
#define MIC_CHANNEL 2
#define MIC_PIN (26 + MIC_CHANNEL)

const uint I2C_SDA = 14;
const uint I2C_SCL = 15;

// Parâmetros e macros do ADC.
#define ADC_CLOCK_DIV 96.f
#define SAMPLES 200                   // Número de amostras do ADC.
#define ADC_ADJUST(x) (x * 3.3f / (1 << 12u) - 1.65f) 
#define ADC_MAX 3.3f
#define ADC_STEP (3.3f/5.f)

// Taxa de amostragem (em Hz) para estimar a frequência.
#define SAMPLE_RATE 10000.0f

#define abs(x) ((x < 0) ? (-x) : (x))

// Buffer e configurações do DMA.
uint16_t adc_buffer[SAMPLES];
uint dma_channel;
dma_channel_config dma_cfg;

int contador = -1;
int frequency = 0;
int lastButtonAPress = 0; // Estado anterior do botão A
int lastButtonBPress = 0; // Estado anterior do botão B
// Notas musicais: Dó, Ré, Mi, Fá, Sol, Lá, Si
const uint notas[] = {262, 294, 330, 349, 392, 440, 494};

//
// Funções para controle do buzzer via PWM
//
void pwm_init_buzzer(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.0f); // Ajusta o divisor do clock
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(pin, 0);  // Buzzer desligado inicialmente
}

void tocarNotaDuracao(int frequency, int duracao) {
    if (frequency == 0) {
        pwm_set_gpio_level(BUZZER_PIN, 0);
        return;
    }
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    uint32_t clock_freq = clock_get_hz(clk_sys);
    float clkdiv = 8.0f;  // Divisor de clock
    uint32_t top = (clock_freq / (clkdiv * frequency)) - 1;
    pwm_set_wrap(slice_num, top);
    pwm_set_clkdiv(slice_num, clkdiv);
    pwm_set_gpio_level(BUZZER_PIN, top / 2); // 50% duty cycle
    sleep_ms(duracao);
    pwm_set_gpio_level(BUZZER_PIN, 0);
}

void tocarNota(int frequency) {
    tocarNotaDuracao(frequency, 500);
}

void tocarSomAcerto() {
    tocarNotaDuracao(523, 200);  // Dó2
    sleep_ms(50);
    tocarNotaDuracao(659, 200);  // Mi2
    sleep_ms(50);
    tocarNotaDuracao(783, 200);  // Sol2
}

void tocarSomErro() {
    tocarNotaDuracao(300, 200);
    sleep_ms(50);
    tocarNotaDuracao(300, 200);
    sleep_ms(50);
    tocarNotaDuracao(300, 200);
}

void definirNotaMusical() {
    switch (contador) {
        case 0:
            printf("Nota Dó selecionada! Frequência: 262 Hz\n");
            frequency = notas[0];
            break;
        case 1:
            printf("Nota Ré selecionada! Frequência: 294 Hz\n");
            frequency = notas[1];
            break;
        case 2:
            printf("Nota Mi selecionada! Frequência: 330 Hz\n");
            frequency = notas[2];
            break;
        case 3:
            printf("Nota Fá selecionada! Frequência: 349 Hz\n");
            frequency = notas[3];
            break;
        case 4:
            printf("Nota Sol selecionada! Frequência: 392 Hz\n");
            frequency = notas[4];
            break;
        case 5:
            printf("Nota Lá selecionada! Frequência: 440 Hz\n");
            frequency = notas[5];
            break;
        case 6:
            printf("Nota Si selecionada! Frequência: 494 Hz\n");
            frequency = notas[6];
            break;
        default:
            frequency = 0;
            break;
    }
    tocarNota(frequency);
}

bool debounceButton(uint pin) {
    if (!gpio_get(pin)) {
        sleep_ms(50);
        if (!gpio_get(pin)) {
            while (!gpio_get(pin));  // Aguarda o botão ser solto
            return true;
        }
    }
    return false;
}

//
// Funções para capturar e estimar a frequência do som via microfone
//
void init_microphone() {
    adc_gpio_init(MIC_PIN);
    adc_init();
    adc_select_input(MIC_CHANNEL);
    adc_fifo_setup(true, true, 1, false, false);
    adc_set_clkdiv(ADC_CLOCK_DIV);
    
    dma_channel = dma_claim_unused_channel(true);
    dma_cfg = dma_channel_get_default_config(dma_channel);
    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&dma_cfg, false);
    channel_config_set_write_increment(&dma_cfg, true);
    channel_config_set_dreq(&dma_cfg, DREQ_ADC);
}

void sample_mic() {
    adc_fifo_drain();
    adc_run(false);
    dma_channel_configure(dma_channel, &dma_cfg,
      adc_buffer,
      &adc_hw->fifo,
      SAMPLES,
      true
    );
    adc_run(true);
    dma_channel_wait_for_finish_blocking(dma_channel);
    adc_run(false);
}

float estimate_frequency(uint16_t *buffer, uint samples) {
    int zero_crossings = 0;
    int mid = 2048; // Valor médio para um ADC de 12 bits (0 a 4095)
    for (int i = 1; i < samples; i++) {
        if ((buffer[i-1] < mid && buffer[i] >= mid) ||
            (buffer[i-1] > mid && buffer[i] <= mid)) {
            zero_crossings++;
        }
    }
    float est_freq = ((float)zero_crossings / 2.0f) * (SAMPLE_RATE / SAMPLES);
    return est_freq;
}


int main() {
    stdio_init_all();
    pwm_init_buzzer(BUZZER_PIN);
    
    gpio_init(BUTTON_A);
    gpio_set_dir(BUTTON_A, GPIO_IN);
    gpio_pull_up(BUTTON_A);

    gpio_init(BUTTON_B);
    gpio_set_dir(BUTTON_B, GPIO_IN);
    gpio_pull_up(BUTTON_B);

    gpio_init(LED_GREEN);
    gpio_set_dir(LED_GREEN, GPIO_OUT);
    
    gpio_init(LED_RED);
    gpio_set_dir(LED_RED, GPIO_OUT);
    
    init_microphone();

     // Inicialização do i2c
     i2c_init(i2c1, ssd1306_i2c_clock * 1000);
     gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
     gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
     gpio_pull_up(I2C_SDA);
     gpio_pull_up(I2C_SCL);
    // Processo de inicialização completo do OLED SSD1306
    ssd1306_init();

    // Preparar área de renderização para o display (ssd1306_width pixels por ssd1306_n_pages páginas)
    struct render_area frame_area = {
        start_column : 0,
        end_column : ssd1306_width - 1,
        start_page : 0,
        end_page : ssd1306_n_pages - 1
    };

    calculate_render_area_buffer_length(&frame_area);

    // zera o display inteiro
    uint8_t ssd[ssd1306_buffer_length];
    memset(ssd, 0, ssd1306_buffer_length);
    render_on_display(ssd, &frame_area);

restart:

// Parte do código para exibir a mensagem no display (opcional: mudar ssd1306_height para 32 em ssd1306_i2c.h)
// /**
    char *text[] = {
        "Ola! para",
        "comecar clique no",
        "  botao A", 
};

    int y = 0;
    for (uint i = 0; i < count_of(text); i++)
    {
        ssd1306_draw_string(ssd, 5, y, text[i]);
        y += 8;
    }
    render_on_display(ssd, &frame_area);


    while (1) {
        bool notePlayed = false;
        if (debounceButton(BUTTON_A)) {
            contador++;
            if (contador > 6) contador = 0;
            definirNotaMusical();
            notePlayed = true;
            gpio_put(LED_GREEN, 1);
            sleep_ms(200);
            gpio_put(LED_GREEN, 0);
        }
        if (debounceButton(BUTTON_B)) {
            contador--;
            if (contador < 0) contador = 6;
            definirNotaMusical();
            notePlayed = true;
            gpio_put(LED_RED, 1);
            sleep_ms(200);
            gpio_put(LED_RED, 0);
        }
        
        if (notePlayed) {
            sleep_ms(3000);
            printf("Ouvindo...\n");
            sample_mic();
            float captured_freq = estimate_frequency(adc_buffer, SAMPLES);
            printf("Frequência capturada: %.2f Hz\n", captured_freq);
            
            if (fabs(captured_freq - frequency) < 5.0) {
                printf("Acerto: Som similar a nota selecionada (%d Hz)\n", frequency);
                gpio_put(LED_GREEN, 1);
                tocarSomAcerto();
                gpio_put(LED_GREEN, 0);
            } else {
                printf("Erro: Som não corresponde à nota selecionada (%d Hz)\n", frequency);
                gpio_put(LED_RED, 1);
                tocarSomErro();
                gpio_put(LED_RED, 0);
            }
            sleep_ms(1000);
        }
    }
    return 0;
}
