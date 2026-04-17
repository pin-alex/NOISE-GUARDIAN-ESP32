// ─────────────────────────────────────────────────────────────────────────────
//  Lärmampel – Noise Traffic Light
//  Waveshare ESP32-S3-Touch-LCD-1.85C V2
//
//  Grün  = leise   (smooth RMS < THRESHOLD_LOW)
//  Gelb  = mittel  (smooth RMS < THRESHOLD_HIGH)
//  Rot   = laut    (smooth RMS >= THRESHOLD_HIGH)
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <ESP_I2S.h>
#include "driver/i2c.h"
#include "es7210.h"
#include <math.h>

// ── Display QSPI Pins (verifiziert: Waveshare Display_ST77916.h) ──────────────
#define TFT_CS   21
#define TFT_SCK  40
#define TFT_D0   46
#define TFT_D1   45
#define TFT_D2   42
#define TFT_D3   41
#define TFT_RST  -1    // Software-Reset via gfx->begin()
#define TFT_BL    5    // Backlight PWM

// ── I2S + ES7210 Codec Pins (verifiziert: Waveshare 08_esp_sr.ino) ───────────
#define I2S_MCK_PIN  2    // Master Clock
#define I2S_BCK_PIN  48   // Bit Clock
#define I2S_WS_PIN   38   // Word Select / LRCLK
#define I2S_DIN_PIN  39   // Daten IN (Mic → ESP32)
#define I2S_DOUT_PIN 47   // Daten OUT (nicht verwendet)
#define I2C_SDA_PIN  11   // I2C für ES7210-Codec
#define I2C_SCL_PIN  10
#define ES7210_ADDR  0x40

// ── Audio-Einstellungen ───────────────────────────────────────────────────────
#define SAMPLE_RATE   16000
#define SAMPLE_COUNT  512      // Frames pro Messung (32 ms bei 16 kHz)
#define SMOOTH_COUNT  8        // Gleitender Mittelwert über 8 Messungen (~256 ms)

// ── Schwellenwerte (empirisch kalibrieren, siehe Kommentar in loop()) ─────────
#define THRESHOLD_LOW   0.008f   // unter diesem Wert → GRÜN
#define THRESHOLD_HIGH  0.025f   // über diesem Wert  → ROT

// ── State & Puffer ────────────────────────────────────────────────────────────
enum TrafficLight { LIGHT_NONE = -1, LIGHT_GREEN, LIGHT_YELLOW, LIGHT_RED };

static int16_t            audio_buf[SAMPLE_COUNT * 2];   // Stereo 16-bit
static float              rms_history[SMOOTH_COUNT] = {};
static uint8_t            rms_idx        = 0;
static TrafficLight       current_light  = LIGHT_NONE;   // erzwingt ersten Draw
static es7210_dev_handle_t es7210_handle = NULL;

// ── Display-Objekte (QSPI-Bus) ────────────────────────────────────────────────
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    TFT_CS, TFT_SCK, TFT_D0, TFT_D1, TFT_D2, TFT_D3);
Arduino_GFX *gfx = new Arduino_ST77916(bus, TFT_RST, 0, false, 360, 360);

// ── I2S-Objekt ────────────────────────────────────────────────────────────────
I2SClass i2s;

// ─────────────────────────────────────────────────────────────────────────────
// Initialisierung ES7210-Codec via Legacy-I2C-Treiber (erforderlich für Lib)
// ─────────────────────────────────────────────────────────────────────────────
static void init_codec()
{
    // Legacy ESP-IDF I2C Treiber initialisieren (von ES7210-Lib benötigt)
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_SDA_PIN,
        .scl_io_num       = I2C_SCL_PIN,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master           = { .clk_speed = 100000 },
        .clk_flags        = 0,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    // ES7210 Codec-Handle erstellen
    es7210_i2c_config_t i2c_conf = {
        .i2c_port = I2C_NUM_0,
        .i2c_addr = ES7210_ADDR,
    };
    ESP_ERROR_CHECK(es7210_new_codec(&i2c_conf, &es7210_handle));

    // Codec konfigurieren
    es7210_codec_config_t cfg = {
        .sample_rate_hz  = SAMPLE_RATE,
        .mclk_ratio      = 256,            // MCLK = 16000 × 256 = 4.096 MHz
        .i2s_format      = ES7210_I2S_FMT_I2S,
        .bit_width       = ES7210_I2S_BITS_16B,
        .mic_bias        = ES7210_MIC_BIAS_2V87,
        .mic_gain        = ES7210_MIC_GAIN_36DB,
        .flags           = { .tdm_enable = false },
    };
    ESP_ERROR_CHECK(es7210_config_codec(es7210_handle, &cfg));
    ESP_ERROR_CHECK(es7210_config_volume(es7210_handle, 40));
}

// ─────────────────────────────────────────────────────────────────────────────
// I2S-Interface initialisieren (Arduino ESP32 Core 3.x API)
// ─────────────────────────────────────────────────────────────────────────────
static void init_i2s()
{
    i2s.setPins(I2S_BCK_PIN, I2S_WS_PIN, I2S_DOUT_PIN, I2S_DIN_PIN, I2S_MCK_PIN);
    i2s.setTimeout(1000);
    i2s.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
}

// ─────────────────────────────────────────────────────────────────────────────
// RMS aus Stereo-I2S-Samples berechnen (nur linker Kanal = Mikrofon)
// ─────────────────────────────────────────────────────────────────────────────
static float compute_rms()
{
    size_t bytes_read = i2s.readBytes((char *)audio_buf, sizeof(audio_buf));
    if (bytes_read == 0) return 0.0f;

    size_t n = bytes_read / sizeof(int16_t);   // Anzahl int16 gesamt (L+R)
    double sum_sq = 0.0;
    for (size_t i = 0; i < n; i += 2) {        // i+=2 → nur linker Kanal
        double v = (double)audio_buf[i] / 32767.0;
        sum_sq += v * v;
    }
    return (float)sqrt(sum_sq / (n / 2));
}

// ─────────────────────────────────────────────────────────────────────────────
// Gleitender Mittelwert über SMOOTH_COUNT letzte RMS-Werte
// ─────────────────────────────────────────────────────────────────────────────
static float smooth_rms(float new_val)
{
    rms_history[rms_idx] = new_val;
    rms_idx = (rms_idx + 1) % SMOOTH_COUNT;
    float sum = 0.0f;
    for (uint8_t i = 0; i < SMOOTH_COUNT; i++) sum += rms_history[i];
    return sum / SMOOTH_COUNT;
}

// ─────────────────────────────────────────────────────────────────────────────
// Display nur bei Zustandswechsel neu einfärben
// ─────────────────────────────────────────────────────────────────────────────
static void update_display(TrafficLight state)
{
    if (state == current_light) return;
    current_light = state;

    uint16_t color;
    switch (state) {
        case LIGHT_GREEN:  color = 0x07E0; break;   // RGB565 grün
        case LIGHT_YELLOW: color = 0xFFE0; break;   // RGB565 gelb
        case LIGHT_RED:    color = 0xF800; break;   // RGB565 rot
        default:           color = 0x0000; break;
    }
    gfx->fillScreen(color);
}

// ─────────────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);

    // Backlight einschalten
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);

    // Display initialisieren
    gfx->begin();
    gfx->fillScreen(0x0000);   // Startup: schwarz

    // Mikrofon-Codec und I2S starten
    init_codec();
    init_i2s();

    Serial.println("Laermampel bereit");
}

// ─────────────────────────────────────────────────────────────────────────────
void loop()
{
    float rms    = compute_rms();
    float smooth = smooth_rms(rms);

    // Kalibrierungs-Hilfe: Zeile einkommentieren, Serial Monitor öffnen (115200)
    // Serial.printf("rms=%.5f smooth=%.5f\n", rms, smooth);

    TrafficLight state;
    if      (smooth < THRESHOLD_LOW)  state = LIGHT_GREEN;
    else if (smooth < THRESHOLD_HIGH) state = LIGHT_YELLOW;
    else                              state = LIGHT_RED;

    update_display(state);
}
