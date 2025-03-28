#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define LED_PIN 13  // LED rojo en pin 13 (tiene protección interna)
#define SIGNAL_GAIN 15  // Aumentado para mayor respuesta al hablar
#define LED_THRESHOLD 8000000  // Umbral más alto para sonidos fuertes
#define LED_MIN_THRESHOLD 200000  // Umbral mínimo para evitar ruido de fondo
#define MAX_SAMPLE_THRESHOLD 1000000  // Umbral para detectar picos de sonido
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define I2S_WS 25
#define I2S_SD 32
#define I2S_SCK 33
#define I2S_PORT I2S_NUM_0

#define SAMPLES 256
#define SAMPLING_FREQUENCY 16000
ArduinoFFT<double> FFT;

int32_t sampleBuffer[SAMPLES];
double vReal[SAMPLES];
double vImag[SAMPLES];

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);  // Configurar el LED como salida

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("Error al iniciar OLED"));
    while (1);
  }

  // Configurar I2S
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLING_FREQUENCY,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_start(I2S_PORT);
}

void loop() {
  size_t bytesRead = 0;
  if (i2s_read(I2S_PORT, sampleBuffer, sizeof(sampleBuffer), &bytesRead, 100) == ESP_OK && bytesRead > 0) {
    // Calcular el promedio y máximo de la magnitud del audio
    double totalMagnitude = 0;
    double maxSample = 0;
    for (int i = 0; i < SAMPLES; i++) {
      int32_t sample = sampleBuffer[i] >> 8;
      sample *= SIGNAL_GAIN;
      vReal[i] = (double)sample;
      vImag[i] = 0.0;
      totalMagnitude += abs(sample);
      maxSample = max(maxSample, (double)abs(sample));
    }
    
    // Encender el LED solo cuando hay sonido fuerte y cercano
    if (totalMagnitude > LED_THRESHOLD && 
        totalMagnitude > LED_MIN_THRESHOLD && 
        maxSample > MAX_SAMPLE_THRESHOLD) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }

    FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.complexToMagnitude(vReal, vImag, SAMPLES);

    display.clearDisplay();

    int numBars = 32;  // Más barras para mayor detalle
    int barWidth = 3;  // Barras más delgadas pero visibles
    int centerY = SCREEN_HEIGHT / 2;
    int maxHeight = SCREEN_HEIGHT / 2;

    for (int i = 0; i < numBars; i++) {
      int index = map(i, 0, numBars - 1, 5, SAMPLES / 4);  // Priorizamos voz (~85Hz-3kHz)
      double magnitude = vReal[index] / 500000.0;  // Escala ajustada para voz
      int barHeight = constrain((int)magnitude, 2, maxHeight);

      int xPos = i * (SCREEN_WIDTH / numBars);

      // Barras creciendo desde el centro
      display.drawLine(xPos, centerY, xPos, centerY - barHeight, SSD1306_WHITE);
      display.drawLine(xPos, centerY, xPos, centerY + barHeight, SSD1306_WHITE);
    }

    display.display();
  }

  delay(20);
}
