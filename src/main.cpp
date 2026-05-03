// ─────────────────────────────────────────────────────────────────────────────
//  Lärmampel – Noise Traffic Light
//  Waveshare ESP32-S3-Touch-LCD-1.85C V2
//
//  Grün  = leise   (smooth RMS < THRESHOLD_LOW)
//  Gelb  = mittel  (smooth RMS < THRESHOLD_HIGH)
//  Rot   = laut    (smooth RMS >= THRESHOLD_HIGH)
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include <ESP_I2S.h>
#include "driver/i2c_master.h"
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
#define THRESHOLD_LOW   0.30f    // unter diesem Wert → GRÜN
#define THRESHOLD_HIGH  0.70f    // über diesem Wert  → ROT

// ── State & Puffer ────────────────────────────────────────────────────────────
enum TrafficLight { LIGHT_NONE = -1, LIGHT_GREEN, LIGHT_YELLOW, LIGHT_RED };

static int16_t            audio_buf[SAMPLE_COUNT * 2];   // Stereo 16-bit
static float              rms_history[SMOOTH_COUNT] = {};
static uint8_t            rms_idx        = 0;
static TrafficLight       current_light  = LIGHT_NONE;   // erzwingt ersten Draw
static es7210_dev_handle_t es7210_handle = NULL;

// ── TCA9554PWR IO-Expander (steuert Display-Reset auf EXIO2) ─────────────────
#define TCA9554_ADDR       0x20
#define TCA9554_OUTPUT_REG 0x01
#define TCA9554_CONFIG_REG 0x03
#define EXIO_LCD_RST       1      // Bit 1 = EXIO_PIN2

// ── Display-Objekte (QSPI-Bus) ────────────────────────────────────────────────
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    TFT_CS, TFT_SCK, TFT_D0, TFT_D1, TFT_D2, TFT_D3);

// Waveshare-spezifische ST77916-Init (vendor_specific_init_new)
// Für neuere Panel-Variante (Reg 0x04 = 0x00,0x02,0x7F,0x7F).
// Komplett andere Power-, Gate-Timing- und Color-Mux-Werte als _default.
static const uint8_t st77916_waveshare_init[] = {
    BEGIN_WRITE,
    // ── Vendor Page 0x28 ─────────────────────────────────────────────────
    WRITE_C8_D8, 0xF0, 0x28,
    WRITE_C8_D8, 0xF2, 0x28,
    WRITE_C8_D8, 0x73, 0xF0,     // nur in _new
    WRITE_C8_D8, 0x7C, 0xD1,
    WRITE_C8_D8, 0x83, 0xE0,
    WRITE_C8_D8, 0x84, 0x61,
    WRITE_C8_D8, 0xF2, 0x82,
    WRITE_C8_D8, 0xF0, 0x00,
    // ── Page 1: Power / Voltage / Timing ─────────────────────────────────
    WRITE_C8_D8, 0xF0, 0x01,
    WRITE_C8_D8, 0xF1, 0x01,
    WRITE_C8_D8, 0xB0, 0x56,
    WRITE_C8_D8, 0xB1, 0x4D,
    WRITE_C8_D8, 0xB2, 0x24,
    WRITE_C8_D8, 0xB4, 0x87,
    WRITE_C8_D8, 0xB5, 0x44,
    WRITE_C8_D8, 0xB6, 0x8B,
    WRITE_C8_D8, 0xB7, 0x40,
    WRITE_C8_D8, 0xB8, 0x86,
    WRITE_C8_D8, 0xBA, 0x00,
    WRITE_C8_D8, 0xBB, 0x08,
    WRITE_C8_D8, 0xBC, 0x08,
    WRITE_C8_D8, 0xBD, 0x00,
    WRITE_C8_D8, 0xC0, 0x80,
    WRITE_C8_D8, 0xC1, 0x10,
    WRITE_C8_D8, 0xC2, 0x37,
    WRITE_C8_D8, 0xC3, 0x80,
    WRITE_C8_D8, 0xC4, 0x10,
    WRITE_C8_D8, 0xC5, 0x37,
    WRITE_C8_D8, 0xC6, 0xA9,
    WRITE_C8_D8, 0xC7, 0x41,
    WRITE_C8_D8, 0xC8, 0x01,
    WRITE_C8_D8, 0xC9, 0xA9,
    WRITE_C8_D8, 0xCA, 0x41,
    WRITE_C8_D8, 0xCB, 0x01,
    WRITE_C8_D8, 0xD0, 0x91,
    WRITE_C8_D8, 0xD1, 0x68,
    WRITE_C8_D8, 0xD2, 0x68,
    WRITE_C8_D16, 0xF5, 0x00, 0xA5,
    WRITE_C8_D8, 0xDD, 0x4F,     // nur in _new
    WRITE_C8_D8, 0xDE, 0x4F,     // nur in _new
    WRITE_C8_D8, 0xF1, 0x10,
    WRITE_C8_D8, 0xF0, 0x00,
    // ── Page 2: Gamma (andere Kurve als _default) ────────────────────────
    WRITE_C8_D8, 0xF0, 0x02,
    WRITE_C8_BYTES, 0xE0, 14,
    0xF0, 0x0A, 0x10, 0x09,
    0x09, 0x36, 0x35, 0x33,
    0x4A, 0x29, 0x15, 0x15,
    0x2E, 0x34,
    WRITE_C8_BYTES, 0xE1, 14,
    0xF0, 0x0A, 0x0F, 0x08,
    0x08, 0x05, 0x34, 0x33,
    0x4A, 0x39, 0x15, 0x15,
    0x2D, 0x33,
    // ── Page 0x10: Gate Driver ───────────────────────────────────────────
    WRITE_C8_D8, 0xF0, 0x10,
    WRITE_C8_D8, 0xF3, 0x10,
    WRITE_C8_D8, 0xE0, 0x07,
    WRITE_C8_D8, 0xE1, 0x00,
    WRITE_C8_D8, 0xE2, 0x00,
    WRITE_C8_D8, 0xE3, 0x00,
    WRITE_C8_D8, 0xE4, 0xE0,
    WRITE_C8_D8, 0xE5, 0x06,
    WRITE_C8_D8, 0xE6, 0x21,
    WRITE_C8_D8, 0xE7, 0x01,
    WRITE_C8_D8, 0xE8, 0x05,
    WRITE_C8_D8, 0xE9, 0x02,
    WRITE_C8_D8, 0xEA, 0xDA,
    WRITE_C8_D8, 0xEB, 0x00,
    WRITE_C8_D8, 0xEC, 0x00,
    WRITE_C8_D8, 0xED, 0x0F,
    WRITE_C8_D8, 0xEE, 0x00,
    WRITE_C8_D8, 0xEF, 0x00,
    WRITE_C8_D8, 0xF8, 0x00,
    WRITE_C8_D8, 0xF9, 0x00,
    WRITE_C8_D8, 0xFA, 0x00,
    WRITE_C8_D8, 0xFB, 0x00,
    WRITE_C8_D8, 0xFC, 0x00,
    WRITE_C8_D8, 0xFD, 0x00,
    WRITE_C8_D8, 0xFE, 0x00,
    WRITE_C8_D8, 0xFF, 0x00,
    // Gate source timing
    WRITE_C8_D8, 0x60, 0x40,
    WRITE_C8_D8, 0x61, 0x04,
    WRITE_C8_D8, 0x62, 0x00,
    WRITE_C8_D8, 0x63, 0x42,
    WRITE_C8_D8, 0x64, 0xD9,
    WRITE_C8_D8, 0x65, 0x00,
    WRITE_C8_D8, 0x66, 0x00,
    WRITE_C8_D8, 0x67, 0x00,
    WRITE_C8_D8, 0x68, 0x00,
    WRITE_C8_D8, 0x69, 0x00,
    WRITE_C8_D8, 0x6A, 0x00,
    WRITE_C8_D8, 0x6B, 0x00,
    WRITE_C8_D8, 0x70, 0x40,
    WRITE_C8_D8, 0x71, 0x03,
    WRITE_C8_D8, 0x72, 0x00,
    WRITE_C8_D8, 0x73, 0x42,
    WRITE_C8_D8, 0x74, 0xD8,
    WRITE_C8_D8, 0x75, 0x00,
    WRITE_C8_D8, 0x76, 0x00,
    WRITE_C8_D8, 0x77, 0x00,
    WRITE_C8_D8, 0x78, 0x00,
    WRITE_C8_D8, 0x79, 0x00,
    WRITE_C8_D8, 0x7A, 0x00,
    WRITE_C8_D8, 0x7B, 0x00,
    // Gate driver channels (0x48 base, mit 0x04 in Byte 5)
    WRITE_C8_D8, 0x80, 0x48,
    WRITE_C8_D8, 0x81, 0x00,
    WRITE_C8_D8, 0x82, 0x06,
    WRITE_C8_D8, 0x83, 0x02,
    WRITE_C8_D8, 0x84, 0xD6,
    WRITE_C8_D8, 0x85, 0x04,
    WRITE_C8_D8, 0x86, 0x00,
    WRITE_C8_D8, 0x87, 0x00,
    WRITE_C8_D8, 0x88, 0x48,
    WRITE_C8_D8, 0x89, 0x00,
    WRITE_C8_D8, 0x8A, 0x08,
    WRITE_C8_D8, 0x8B, 0x02,
    WRITE_C8_D8, 0x8C, 0xD8,
    WRITE_C8_D8, 0x8D, 0x04,
    WRITE_C8_D8, 0x8E, 0x00,
    WRITE_C8_D8, 0x8F, 0x00,
    WRITE_C8_D8, 0x90, 0x48,
    WRITE_C8_D8, 0x91, 0x00,
    WRITE_C8_D8, 0x92, 0x0A,
    WRITE_C8_D8, 0x93, 0x02,
    WRITE_C8_D8, 0x94, 0xDA,
    WRITE_C8_D8, 0x95, 0x04,
    WRITE_C8_D8, 0x96, 0x00,
    WRITE_C8_D8, 0x97, 0x00,
    WRITE_C8_D8, 0x98, 0x48,
    WRITE_C8_D8, 0x99, 0x00,
    WRITE_C8_D8, 0x9A, 0x0C,
    WRITE_C8_D8, 0x9B, 0x02,
    WRITE_C8_D8, 0x9C, 0xDC,
    WRITE_C8_D8, 0x9D, 0x04,
    WRITE_C8_D8, 0x9E, 0x00,
    WRITE_C8_D8, 0x9F, 0x00,
    WRITE_C8_D8, 0xA0, 0x48,
    WRITE_C8_D8, 0xA1, 0x00,
    WRITE_C8_D8, 0xA2, 0x05,
    WRITE_C8_D8, 0xA3, 0x02,
    WRITE_C8_D8, 0xA4, 0xD5,
    WRITE_C8_D8, 0xA5, 0x04,
    WRITE_C8_D8, 0xA6, 0x00,
    WRITE_C8_D8, 0xA7, 0x00,
    WRITE_C8_D8, 0xA8, 0x48,
    WRITE_C8_D8, 0xA9, 0x00,
    WRITE_C8_D8, 0xAA, 0x07,
    WRITE_C8_D8, 0xAB, 0x02,
    WRITE_C8_D8, 0xAC, 0xD7,
    WRITE_C8_D8, 0xAD, 0x04,
    WRITE_C8_D8, 0xAE, 0x00,
    WRITE_C8_D8, 0xAF, 0x00,
    WRITE_C8_D8, 0xB0, 0x48,
    WRITE_C8_D8, 0xB1, 0x00,
    WRITE_C8_D8, 0xB2, 0x09,
    WRITE_C8_D8, 0xB3, 0x02,
    WRITE_C8_D8, 0xB4, 0xD9,
    WRITE_C8_D8, 0xB5, 0x04,
    WRITE_C8_D8, 0xB6, 0x00,
    WRITE_C8_D8, 0xB7, 0x00,
    WRITE_C8_D8, 0xB8, 0x48,
    WRITE_C8_D8, 0xB9, 0x00,
    WRITE_C8_D8, 0xBA, 0x0B,
    WRITE_C8_D8, 0xBB, 0x02,
    WRITE_C8_D8, 0xBC, 0xDB,
    WRITE_C8_D8, 0xBD, 0x04,
    WRITE_C8_D8, 0xBE, 0x00,
    WRITE_C8_D8, 0xBF, 0x00,
    // Color mux (andere Reihenfolge als _default)
    WRITE_C8_D8, 0xC0, 0x10,
    WRITE_C8_D8, 0xC1, 0x47,
    WRITE_C8_D8, 0xC2, 0x56,
    WRITE_C8_D8, 0xC3, 0x65,
    WRITE_C8_D8, 0xC4, 0x74,
    WRITE_C8_D8, 0xC5, 0x88,
    WRITE_C8_D8, 0xC6, 0x99,
    WRITE_C8_D8, 0xC7, 0x01,
    WRITE_C8_D8, 0xC8, 0xBB,
    WRITE_C8_D8, 0xC9, 0xAA,
    WRITE_C8_D8, 0xD0, 0x10,
    WRITE_C8_D8, 0xD1, 0x47,
    WRITE_C8_D8, 0xD2, 0x56,
    WRITE_C8_D8, 0xD3, 0x65,
    WRITE_C8_D8, 0xD4, 0x74,
    WRITE_C8_D8, 0xD5, 0x88,
    WRITE_C8_D8, 0xD6, 0x99,
    WRITE_C8_D8, 0xD7, 0x01,
    WRITE_C8_D8, 0xD8, 0xBB,
    WRITE_C8_D8, 0xD9, 0xAA,
    // ── Zurück zu Page 0, finale Kommandos ───────────────────────────────
    WRITE_C8_D8, 0xF3, 0x01,
    WRITE_C8_D8, 0xF0, 0x00,
    WRITE_COMMAND_8, 0x21,        // Display Inversion On
    WRITE_C8_D8, 0x3A, 0x55,     // 16-bit color
    WRITE_COMMAND_8, 0x11,        // Sleep Out
    END_WRITE,
    DELAY, 120,
    BEGIN_WRITE,
    WRITE_COMMAND_8, 0x29,        // Display ON
    END_WRITE
};

Arduino_GFX *gfx = new Arduino_ST77916(
    bus, TFT_RST, 0, true, 360, 360,
    0, 0, 0, 0,
    st77916_waveshare_init, sizeof(st77916_waveshare_init));

// ── I2S-Objekt ────────────────────────────────────────────────────────────────
I2SClass i2s;

// ─────────────────────────────────────────────────────────────────────────────
// Hardware-Reset des Displays über TCA9554PWR IO-Expander (I2C)
// ─────────────────────────────────────────────────────────────────────────────
static void tca9554_write(uint8_t reg, uint8_t data)
{
    Wire.beginTransmission(TCA9554_ADDR);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

static void reset_display()
{
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    // Alle Pins als Output konfigurieren
    tca9554_write(TCA9554_CONFIG_REG, 0x00);

    // EXIO2 LOW → Reset aktiv
    uint8_t out = 0x00;
    out &= ~(1 << EXIO_LCD_RST);
    tca9554_write(TCA9554_OUTPUT_REG, out);
    delay(10);

    // EXIO2 HIGH → Reset loslassen
    out |= (1 << EXIO_LCD_RST);
    tca9554_write(TCA9554_OUTPUT_REG, out);
    delay(50);

    Wire.end();   // I2C freigeben für späteren IDF-Treiber (ES7210)
}

// ─────────────────────────────────────────────────────────────────────────────
// Initialisierung ES7210-Codec via IDF v5 I2C Master Treiber
// ─────────────────────────────────────────────────────────────────────────────
static bool init_codec()
{
    // Neuer IDF v5 I2C Master Treiber (kompatibel mit Arduino Core 3.x / driver_ng)
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port            = I2C_NUM_0,
        .sda_io_num          = (gpio_num_t)I2C_SDA_PIN,
        .scl_io_num          = (gpio_num_t)I2C_SCL_PIN,
        .clk_source          = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt   = 7,
        .flags               = { .enable_internal_pullup = true },
    };
    i2c_master_bus_handle_t bus_handle;
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus_handle);
    if (err != ESP_OK) {
        Serial.printf("I2C Bus Fehler: 0x%x\n", err);
        return false;
    }

    // ES7210 Codec-Handle erstellen
    es7210_i2c_config_t i2c_conf = {
        .bus_handle = bus_handle,
        .i2c_addr   = ES7210_ADDR,
    };
    err = es7210_new_codec(&i2c_conf, &es7210_handle);
    if (err != ESP_OK) {
        Serial.printf("ES7210 new_codec Fehler: 0x%x\n", err);
        return false;
    }

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
    err = es7210_config_codec(es7210_handle, &cfg);
    if (err != ESP_OK) {
        Serial.printf("ES7210 config_codec Fehler: 0x%x\n", err);
        return false;
    }
    err = es7210_config_volume(es7210_handle, 30);
    if (err != ESP_OK) {
        Serial.printf("ES7210 config_volume Fehler: 0x%x\n", err);
        return false;
    }
    return true;
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
    while (!Serial && millis() < 5000) delay(10);   // Warten bis USB-CDC verbunden (max 5s)
    Serial.println("== Laermampel startet ==");

    // Backlight einschalten
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);

    // Hardware-Reset des Displays über IO-Expander
    Serial.println("Display reset...");
    reset_display();

    // Display initialisieren
    Serial.println("Display init...");
    if (!gfx->begin()) {
        Serial.println("Display begin() FEHLGESCHLAGEN");
        while (true) delay(1000);
    }
    delay(100);
    gfx->fillScreen(0x0000);   // Startup: schwarz
    Serial.println("Display OK");

    // Mikrofon-Codec und I2S starten
    Serial.println("Codec init...");
    if (!init_codec()) {
        Serial.println("CODEC FEHLGESCHLAGEN – Display bleibt schwarz");
        gfx->fillScreen(0xF800);   // Rot als Fehlersignal
        while (true) delay(1000);   // Hier stoppen statt abstürzen
    }
    Serial.println("Codec OK");

    Serial.println("I2S init...");
    init_i2s();
    Serial.println("I2S OK");

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
