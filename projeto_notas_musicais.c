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
// Protótipo da função para exibir mensagens no display
void exibirMensagem(const char *mensagem);

//
// Variáveis globais para o display OLED
//
struct render_area frame_area;
uint8_t ssd[1024]; // Ajuste este tamanho conforme o buffer definido em ssd1306.h

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
#define SAMPLES 2048             // Número de amostras do ADC.
#define ADC_ADJUST(x) (x * 3.3f / (1 << 12u) - 1.65f) 
#define ADC_MAX 3.3f
#define ADC_STEP (3.3f/5.f)
#define TOLERANCIA 50

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
    char mensagem[50];
    switch (contador) {
        case 0:
            sprintf(mensagem, "Nota Do selecionada! Freq: 262 Hz");
            printf("%s\n", mensagem);
            frequency = notas[0];
            break;
        case 1:
            sprintf(mensagem, "Nota Re selecionada! Freq: 294 Hz");
            printf("%s\n", mensagem);
            frequency = notas[1];
            break;
        case 2:
            sprintf(mensagem, "Nota Mi selecionada! Freq: 330 Hz");
            printf("%s\n", mensagem);
            frequency = notas[2];
            break;
        case 3:
            sprintf(mensagem, "Nota Fa selecionada! Freq: 349 Hz");
            printf("%s\n", mensagem);
            frequency = notas[3];
            break;
        case 4:
            sprintf(mensagem, "Nota Sol selecionada! Freq: 392 Hz");
            printf("%s\n", mensagem);
            frequency = notas[4];
            break;
        case 5:
            sprintf(mensagem, "Nota La selecionada! Freq: 440 Hz");
            printf("%s\n", mensagem);
            frequency = notas[5];
            break;
        case 6:
            sprintf(mensagem, "Nota Si selecionada! Freq: 494 Hz");
            printf("%s\n", mensagem);
            frequency = notas[6];
            break;
        default:
            frequency = 0;
            return;
    }
    // Exibe a mensagem no display OLED
    exibirMensagem(mensagem);
    // Toca a nota
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

/*
 * Função de estimação de frequência utilizando autocorrelação com janela de Hamming.
 */
float estimate_frequency(uint16_t *buffer, uint samples) {
    float data[samples];

    // Converter os valores ADC para float
    for (int i = 0; i < samples; i++) {
        data[i] = (float) buffer[i];
    }
    // Remover o offset DC: calcular a média e subtrair
    float mean = 0;
    for (int i = 0; i < samples; i++) {
        mean += data[i];
    }
    mean /= samples;
    for (int i = 0; i < samples; i++) {
        data[i] -= mean;
    }
    // Aplicar janela de Hamming
    for (int i = 0; i < samples; i++) {
        float w = 0.54 - 0.46 * cos(2.0 * M_PI * i / (samples - 1));
        data[i] *= w;
    }
    // Autocorrelação
    int lag_max = samples / 2;
    float best_corr = 0.0;
    int best_lag = 0;
    // Iniciar a partir de um lag mínimo (por exemplo, 20) para evitar ruídos e picos espúrios
    for (int lag = 20; lag < lag_max; lag++) {
        float sum = 0.0;
        for (int i = 0; i < samples - lag; i++) {
            sum += data[i] * data[i + lag];
        }
        if (sum > best_corr) {
            best_corr = sum;
            best_lag = lag;
        }
    }
    if (best_lag == 0) {
        return 0.0;
    }
    // Calcula a frequência fundamental
    float fundamental = SAMPLE_RATE / best_lag;
    return fundamental;
}

// Função para exibir mensagens no display OLED
void exibirMensagem(const char *mensagem) {
    memset(ssd, 0, ssd1306_buffer_length);
    ssd1306_draw_string(ssd, 0, 0, (char *)mensagem);
    render_on_display(ssd, &frame_area);
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

    // Inicialização do I2C para o OLED
    i2c_init(i2c1, ssd1306_i2c_clock * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    ssd1306_init();

    // Configuração da área de renderização do display
    frame_area.start_column = 0;
    frame_area.end_column = ssd1306_width - 1;
    frame_area.start_page = 0;
    frame_area.end_page = ssd1306_n_pages - 1;
    calculate_render_area_buffer_length(&frame_area);
    memset(ssd, 0, ssd1306_buffer_length);
    render_on_display(ssd, &frame_area);

    // Exibe mensagem inicial no display
    char *text[] = {
        "  Ola clique no!   ",
        "  botao A   ",
        "  para comecar  "
    };

    int y = 0;
    for (uint i = 0; i < (sizeof(text)/sizeof(text[0])); i++) {
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
            exibirMensagem("Ouvindo...");
            
            sample_mic();
            float captured_freq = estimate_frequency(adc_buffer, SAMPLES);
            char freqMsg[50];
            sprintf(freqMsg, "Freq capt: %.2f Hz", captured_freq);
            printf("%s\n", freqMsg);
            exibirMensagem(freqMsg);
            
            if (fabs(captured_freq - frequency) <= TOLERANCIA) {
                sprintf(freqMsg, "Acerto: %d Hz", frequency);
                printf("Acerto: Som similar a nota selecionada (%d Hz)\n", frequency);
                exibirMensagem(freqMsg);
                gpio_put(LED_GREEN, 1);
                tocarSomAcerto();
                gpio_put(LED_GREEN, 0);
            } else {
                sprintf(freqMsg, "Erro: %d Hz", frequency);
                printf("Erro: Som nao corresponde a nota selecionada (%d Hz)\n", frequency);
                exibirMensagem(freqMsg);
                gpio_put(LED_RED, 1);
                tocarSomErro();
                gpio_put(LED_RED, 0);
            }
            sleep_ms(1000);
        }
    }
    return 0;
}
