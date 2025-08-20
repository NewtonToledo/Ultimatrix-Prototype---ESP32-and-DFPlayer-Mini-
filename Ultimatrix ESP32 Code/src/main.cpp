#include <Arduino.h>
#include <HardwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "image128.h"
#include "esp_system.h"
#include "WiFi.h"
#include "esp_bt.h"
#include "soc/rtc.h"

// ——— Pines y configuración ———
#define POT_PIN         34
#define BTN_PLAY_PIN    32
#define BTN_VOLUP_PIN   33
#define BTN_VOLDN_PIN   27   // <- volumen en 27 como pediste
#define BTN_MODE_PIN    25
#define RX2_PIN         16
#define TX2_PIN         17
#define TOTAL_TRACKS    8

// nombres de canciones
const char* fileNames[TOTAL_TRACKS+1] = {
  "", "Ben 10 Intro", "Bring me to life", "Faded", "Force", "Power Rangers", "Enemy", "DBZ Opening", "Legends Never Die"
};
// duraciones en segundos
const uint16_t trackDurations[TOTAL_TRACKS+1] = {
  0, 61, 244, 212, 240, 241, 187, 111, 235
};

// ——— Modo 4: Frases en inglés ———
#define NUM_FRASES 10
const char* frasesEN[NUM_FRASES] = {
  "Hello", 
  "Excuse me", "I'm sorry", "Please", "Thank you",
  "Again", "Good job", "Bye", "Help me!", "Hero Time!"
};
const char* frasesES[NUM_FRASES] = {
  "\"Hola\"", 
  "\"Con permiso\"", "\"Lo siento\"", "\"Por favor\"", "\"Gracias\"",
  "\"Otra vez\"", "\"Buen trabajo\"", "\"Adios\"", "\"Ayudame!\"", "\"Hora de ser heroe!\""
};
const uint8_t fraseTracks[NUM_FRASES] = { 12,13,14,15,16,17,18,19,20,21 };
volatile uint8_t currentFraseIndex = 0;

// ——— Modo 5: Vocales ———
#define NUM_VOCALES 5
const char* vocales[NUM_VOCALES] = { "a","e","i","o","u" };
const uint8_t vocalTracks[NUM_VOCALES] = { 22,23,24,25,26 };
volatile uint8_t currentVocalIndex = 0;

// ——— Modo 6: Números (1–10) ———
#define NUM_NUMS 10
const char* numerosEN[NUM_NUMS] = { "One","Two","Three","Four","Five","Six","Seven","Eight","Nine","Ten" };
const uint8_t numeroTracks[NUM_NUMS] = { 27,28,29,30,31,32,33,34,35,36 };
volatile uint8_t currentNumeroIndex = 0; // 0..9 → 1..10

// ——— Modo 7: Colores ———
#define NUM_COLORES 10
const char* coloresEN[NUM_COLORES] = {
  "Red","Blue","Yellow","Green","Orange","Purple","Brown","Pink","White","Black"
};
const char* coloresES[NUM_COLORES] = {
  "\"Rojo\"","\"Azul\"","\"Amarillo\"","\"Verde\"","\"Naranja\"","\"Morado\"","\"Cafe\"","\"Rosa\"","\"Blanco\"","\"Negro\""
};
const uint8_t colorTracks[NUM_COLORES] = { 37,38,39,40,41,42,43,44,45,46 };
volatile uint8_t currentColorIndex = 0;

// pantalla SH1107 I2C
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  128
#define OLED_RESET     -1
#define I2C_ADDRESS    0x3C
Adafruit_SH1107 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// DFPlayer y sincronización
HardwareSerial       dfSerial(2);
DFRobotDFPlayerMini  player;
SemaphoreHandle_t    playerMutex;

// ——— Estado MP3 ———
volatile int     currentTrack    = 1;   // 1..TOTAL_TRACKS
volatile bool    isPlaying       = false;
uint32_t         trackStartTime  = 0;
uint32_t         trackDurationMs = 0;
bool             pauseState      = false;
uint32_t         pauseStartTime  = 0;

// ——— Icono de volumen ———
volatile bool    volFlag      = false;
volatile int     volIconType  = 0;    // 1=+Vol, 2=-Vol
uint32_t         volTime      = 0;

// ——— Modos ———
// 0 = reproductor MP3 (pot selecciona pista)
// 1 = icono omnitrix (reproduce sonido al entrar)
// 2 = grupo aliens M2 (pot cambia imagen + FX al girar)
// 3 = grupo aliens M3 (pot cambia imagen + FX al girar)
// 4 = frases EN/ES (pot selecciona frase)
// 5 = vocales (pot selecciona vocal)
// 6 = números (pot selecciona número)
// 7 = colores EN/ES (pot selecciona color)
volatile uint8_t mode = 0;

#define MODES_COUNT        8
#define MODE1_SOUND_TRACK  11

// ——— Imágenes ———
#define NUM_IMAGES 10
extern const uint8_t* const imagenesModo2[NUM_IMAGES];
extern const uint8_t* const imagenesModo3[NUM_IMAGES];
volatile uint8_t currentImageIndex = 0;

// ——— Estados de sincronización por modo (reemplazan statics) ———
volatile int8_t lastTrackIdx   = -1; // mode 0 (0..TOTAL_TRACKS-1)
volatile int8_t lastImageIdx2  = -1; // mode 2
volatile int8_t lastImageIdx3  = -1; // mode 3
volatile int8_t lastFraseIdx   = -1; // mode 4
volatile int8_t lastVocalIdx   = -1; // mode 5
volatile int8_t lastNumeroIdx  = -1; // mode 6
volatile int8_t lastColorIdx   = -1; // mode 7

// Helper de mapeo 0..4095 → 0..count-1
static inline uint8_t mapPotToIndex(int raw, int count) {
  if (raw < 0) raw = 0;
  if (raw > 4095) raw = 4095;
  int idx = (int)((uint32_t)raw * (uint32_t)count / 4096U);
  if (idx >= count) idx = count - 1;
  return (uint8_t)idx;
}

// ——— Prototipos ———
void nextTrackAuto();
void taskPot(void*);
void taskButtons(void*);
void taskDisplay(void*);

void setup() {
  setCpuFrequencyMhz(80);         // mantener 80 MHz
  WiFi.mode(WIFI_OFF);
  btStop();

  // DFPlayer
  dfSerial.begin(9600, SERIAL_8N1, RX2_PIN, TX2_PIN);
  while (!player.begin(dfSerial)) { delay(200); }
  player.volume(20);
  player.play(currentTrack);
  isPlaying       = true;
  trackStartTime  = millis();
  trackDurationMs = trackDurations[currentTrack] * 1000UL;

  // potenciómetro
  analogReadResolution(12);
  analogSetPinAttenuation(POT_PIN, ADC_11db);
  pinMode(POT_PIN, INPUT);

  // botones
  pinMode(BTN_PLAY_PIN, INPUT_PULLUP);
  pinMode(BTN_VOLUP_PIN, INPUT_PULLUP);
  pinMode(BTN_VOLDN_PIN, INPUT_PULLUP);
  pinMode(BTN_MODE_PIN, INPUT_PULLUP);

  // pantalla
  Wire.begin(21, 22);
  if (!display.begin(I2C_ADDRESS)) { while (true); }
  display.clearDisplay();
  display.display();

  // FreeRTOS
  playerMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(taskPot,      "POT", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskButtons,  "BTN", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskDisplay,  "DSP", 2048, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}

void nextTrackAuto() {
  xSemaphoreTake(playerMutex, portMAX_DELAY);
    currentTrack    = (currentTrack < TOTAL_TRACKS ? currentTrack + 1 : 1);
    player.play(currentTrack);
    isPlaying       = true;
    pauseState      = false;
    trackStartTime  = millis();
    trackDurationMs = trackDurations[currentTrack] * 1000UL;
    lastTrackIdx    = currentTrack - 1; // mantener sincronía con pot
  xSemaphoreGive(playerMutex);
}

// ——— Potenciómetro: controla modos 0,2,3,4,5,6,7 ———
void taskPot(void *pv) {
  int lastRaw = -1;
  for (;;) {
    int raw = analogRead(POT_PIN);

    if (abs(raw - lastRaw) > 60) {   // pequeña histéresis anti-ruido
      lastRaw = raw;

      // Modo 0: selector de pistas 1..TOTAL_TRACKS
      if (mode == 0) {
        uint8_t idx0 = mapPotToIndex(raw, TOTAL_TRACKS); // 0..TOTAL_TRACKS-1
        if (idx0 != lastTrackIdx) {
          uint8_t track = idx0 + 1;
          xSemaphoreTake(playerMutex, pdMS_TO_TICKS(80));
            player.play(track);
            currentTrack    = track;
            isPlaying       = true;
            pauseState      = false;
            trackStartTime  = millis();
            trackDurationMs = trackDurations[currentTrack] * 1000UL;
          xSemaphoreGive(playerMutex);
          lastTrackIdx = idx0;
        }
      }
      // Modo 2: imágenes grupo 2 + FX al cambiar
      else if (mode == 2) {
        uint8_t idx = mapPotToIndex(raw, NUM_IMAGES);
        if (idx != lastImageIdx2) {
          xSemaphoreTake(playerMutex, pdMS_TO_TICKS(80));
            player.play(10);   // efecto cambio
          xSemaphoreGive(playerMutex);
          currentImageIndex = idx;
          lastImageIdx2     = idx;
        }
      }
      // Modo 3: imágenes grupo 3 + FX al cambiar
      else if (mode == 3) {
        uint8_t idx = mapPotToIndex(raw, NUM_IMAGES);
        if (idx != lastImageIdx3) {
          xSemaphoreTake(playerMutex, pdMS_TO_TICKS(80));
            player.play(10);   // efecto cambio
          xSemaphoreGive(playerMutex);
          currentImageIndex = idx;
          lastImageIdx3     = idx;
        }
      }
      // Modo 4: frases EN/ES
      else if (mode == 4) {
        uint8_t idx = mapPotToIndex(raw, NUM_FRASES);
        if (idx != lastFraseIdx) {
          xSemaphoreTake(playerMutex, pdMS_TO_TICKS(80));
            player.play(fraseTracks[idx]);
            isPlaying = true;
          xSemaphoreGive(playerMutex);
          currentFraseIndex = idx;
          lastFraseIdx      = idx;
        }
      }
      // Modo 5: vocales
      else if (mode == 5) {
        uint8_t idx = mapPotToIndex(raw, NUM_VOCALES);
        if (idx != lastVocalIdx) {
          xSemaphoreTake(playerMutex, pdMS_TO_TICKS(80));
            player.play(vocalTracks[idx]);
            isPlaying = true;
          xSemaphoreGive(playerMutex);
          currentVocalIndex = idx;
          lastVocalIdx      = idx;
        }
      }
      // Modo 6: números
      else if (mode == 6) {
        uint8_t idx = mapPotToIndex(raw, NUM_NUMS);
        if (idx != lastNumeroIdx) {
          xSemaphoreTake(playerMutex, pdMS_TO_TICKS(80));
            player.play(numeroTracks[idx]);
            isPlaying = true;
          xSemaphoreGive(playerMutex);
          currentNumeroIndex = idx;
          lastNumeroIdx      = idx;
        }
      }
      // Modo 7: colores
      else if (mode == 7) {
        uint8_t idx = mapPotToIndex(raw, NUM_COLORES);
        if (idx != lastColorIdx) {
          xSemaphoreTake(playerMutex, pdMS_TO_TICKS(80));
            player.play(colorTracks[idx]);
            isPlaying = true;
          xSemaphoreGive(playerMutex);
          currentColorIndex = idx;
          lastColorIdx      = idx;
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(40)); // latencia baja sin saturar
  }
}

// ——— Botones ———
void taskButtons(void *pv) {
  bool lastP = HIGH, lastU = HIGH, lastD = HIGH, lastM = HIGH;
  for (;;) {
    bool m = digitalRead(BTN_MODE_PIN);
    if (m == LOW && lastM == HIGH) {
      // cambiar de modo
      uint8_t newMode = (mode + 1) % MODES_COUNT;

      xSemaphoreTake(playerMutex, portMAX_DELAY);
        mode = newMode;

        // Lee pot “ya” para entrar sincronizado en cada modo
        int rawNow = analogRead(POT_PIN);

        if      (mode == 0) {
          uint8_t idx0  = mapPotToIndex(rawNow, TOTAL_TRACKS);
          uint8_t track = idx0 + 1;
          currentTrack  = track;
          lastTrackIdx  = idx0;
          player.play(track);
          isPlaying = true;
          pauseState = false;
          trackStartTime  = millis();
          trackDurationMs = trackDurations[currentTrack] * 1000UL;
        }
        else if (mode == 1) {
          player.play(MODE1_SOUND_TRACK);
          isPlaying = true;
        }
        else if (mode == 2) {
          uint8_t idx = mapPotToIndex(rawNow, NUM_IMAGES);
          currentImageIndex = idx;
          lastImageIdx2     = idx;
          player.pause();
          player.play(9);   // sonido de entrada modo 2
          isPlaying = false;
        }
        else if (mode == 3) {
          uint8_t idx = mapPotToIndex(rawNow, NUM_IMAGES);
          currentImageIndex = idx;
          lastImageIdx3     = idx;
          player.pause();
          player.play(9);   // sonido de entrada modo 3
          isPlaying = false;
        }
        else if (mode == 4) {
          uint8_t idx = mapPotToIndex(rawNow, NUM_FRASES);
          currentFraseIndex = idx;
          lastFraseIdx      = idx;
          player.play(fraseTracks[idx]);
          isPlaying = true;
        }
        else if (mode == 5) {
          uint8_t idx = mapPotToIndex(rawNow, NUM_VOCALES);
          currentVocalIndex = idx;
          lastVocalIdx      = idx;
          player.play(vocalTracks[idx]);
          isPlaying = true;
        }
        else if (mode == 6) {
          uint8_t idx = mapPotToIndex(rawNow, NUM_NUMS);
          currentNumeroIndex = idx;
          lastNumeroIdx      = idx;
          player.play(numeroTracks[idx]);
          isPlaying = true;
        }
        else if (mode == 7) {
          uint8_t idx = mapPotToIndex(rawNow, NUM_COLORES);
          currentColorIndex = idx;
          lastColorIdx      = idx;
          player.play(colorTracks[idx]);
          isPlaying = true;
        }
      xSemaphoreGive(playerMutex);
    }
    lastM = m;

    // Controles del reproductor (sólo en modo 0)
    if (mode == 0) {
      // play/pause
      bool p = digitalRead(BTN_PLAY_PIN);
      if (p == LOW && lastP == HIGH) {
        xSemaphoreTake(playerMutex, portMAX_DELAY);
          if (isPlaying) {
            player.pause();
            isPlaying      = false;
            pauseState     = true;
            pauseStartTime = millis();
          } else {
            player.start();
            trackStartTime += millis() - pauseStartTime;
            isPlaying      = true;
            pauseState     = false;
          }
        xSemaphoreGive(playerMutex);
      }
      lastP = p;

      // volumen +
      bool u = digitalRead(BTN_VOLUP_PIN);
      if (u == LOW && lastU == HIGH) {
        xSemaphoreTake(playerMutex, portMAX_DELAY);
          player.volumeUp();
          volFlag     = true;
          volIconType = 1;
          volTime     = millis();
        xSemaphoreGive(playerMutex);
      }
      lastU = u;

      // volumen –
      bool d = digitalRead(BTN_VOLDN_PIN);
      if (d == LOW && lastD == HIGH) {
        xSemaphoreTake(playerMutex, portMAX_DELAY);
          player.volumeDown();
          volFlag     = true;
          volIconType = 2;
          volTime     = millis();
        xSemaphoreGive(playerMutex);
      }
      lastD = d;
    }

    vTaskDelay(pdMS_TO_TICKS(30));
  }
}

// ——— Display ———
void taskDisplay(void *pv) {
  const int refreshMs = 120;
  static uint32_t lastRefresh = 0;
  static int16_t  fnameX     = -100;
  static bool     fnameDirR  = true;
  static uint32_t lastMove   = 0;
  char buf[24];

  for (;;) {
    uint32_t now = millis();
    if (now - lastRefresh < (uint32_t)refreshMs) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    lastRefresh = now;

    display.clearDisplay();
    display.setTextColor(SH110X_WHITE);
    display.setTextWrap(false);

    if (mode == 0) {
      display.setTextSize(1);

      // cronómetro & auto-next
      uint32_t elapsed = pauseState
        ? (pauseStartTime - trackStartTime)
        : (now - trackStartTime);
      if (elapsed > trackDurationMs) elapsed = trackDurationMs;
      if (elapsed >= trackDurationMs) {
        // Al terminar la canción, tomar lectura del pot y reproducir la pista correspondiente
        int raw = analogRead(POT_PIN);
        int next = raw * TOTAL_TRACKS / 4096 + 1;
        next = constrain(next, 1, TOTAL_TRACKS);

        xSemaphoreTake(playerMutex, portMAX_DELAY);
          currentTrack    = next;
          player.play(currentTrack);
          isPlaying       = true;
          pauseState      = false;
          trackStartTime  = millis();
          trackDurationMs = trackDurations[currentTrack] * 1000UL;
        xSemaphoreGive(playerMutex);

        elapsed = 0;
      }


      // 1) Track #
      snprintf(buf, sizeof(buf), "Track %d", currentTrack);
      int tw = strlen(buf) * 6;
      display.setCursor(14 + (100 - tw)/2, 14);
      display.print(buf);

      // 2) Marquesina filename
      int fw      = strlen(fileNames[currentTrack]) * 6;
      int16_t lb  = 14, rb = 14 + 100 - fw;
      if (now - lastMove >= 120) {
        lastMove = now;
        if (fnameDirR) { if (++fnameX >= rb) fnameDirR = false; }
        else            { if (--fnameX <= lb) fnameDirR = true; }
      }
      display.setCursor(fnameX, 26);
      display.print(fileNames[currentTrack]);

      // 3) Cronómetro
      uint16_t m = elapsed / 60000;
      uint16_t s = (elapsed / 1000) % 60;
      snprintf(buf, sizeof(buf), "%02u:%02u", m, s);
      tw = strlen(buf) * 6;
      display.setCursor(14 + (100 - tw)/2, 38);
      display.print(buf);

      // 4) Icono ▶ / ⏸
      int cx = 64, cy = 80;
      if (isPlaying) {
        display.fillRect(cx-6, cy-6, 4, 12, SH110X_WHITE);
        display.fillRect(cx+2, cy-6, 4, 12, SH110X_WHITE);
      } else {
        display.fillTriangle(cx-6, cy-6, cx-6, cy+6, cx+6, cy, SH110X_WHITE);
      }

      // 5) Barra progreso
      int barY = cy + 12;
      display.drawRect(14, barY, 100, 3, SH110X_WHITE);
      uint32_t w = (uint64_t)elapsed * 100 / trackDurationMs;
      if (w > 0) display.fillRect(14, barY, w, 3, SH110X_WHITE);

      // 6) +Vol / -Vol
      if (volFlag && now - volTime < 1000) {
        const char* vstr = (volIconType == 1 ? "+Vol" : "-Vol");
        tw = strlen(vstr) * 6;
        display.setCursor(14 + (100 - tw)/2, barY + 10);
        display.print(vstr);
      } else {
        volFlag = false;
      }
    }
    else if (mode == 1) {
      display.drawBitmap(0, 0, omnitrix, SCREEN_WIDTH, SCREEN_HEIGHT, SH110X_WHITE);
    }
    else if (mode == 2) {
      display.drawBitmap(0, 0, imagenesModo2[currentImageIndex], SCREEN_WIDTH, SCREEN_HEIGHT, SH110X_WHITE);
    }
    else if (mode == 3) {
      display.drawBitmap(0, 0, imagenesModo3[currentImageIndex], SCREEN_WIDTH, SCREEN_HEIGHT, SH110X_WHITE);
    }
    else if (mode == 4) {
      // Frase EN (size 2)
      display.setTextSize(2);
      const char* en = frasesEN[currentFraseIndex];
      int enW = strlen(en) * 6 * 2;
      int enX = (SCREEN_WIDTH - enW) / 2; if (enX < 0) enX = 0;
      display.setCursor(enX, 28);
      display.print(en);

      // Traducción ES (size 1)
      display.setTextSize(1);
      const char* es = frasesES[currentFraseIndex];
      int esW = strlen(es) * 6;
      int esX = (SCREEN_WIDTH - esW) / 2; if (esX < 0) esX = 0;
      display.setCursor(esX, 54);
      display.print(es);

      // Indicador
      char ibuf[12];
      snprintf(ibuf, sizeof(ibuf), "%u/%u", currentFraseIndex + 1, NUM_FRASES);
      int tw = strlen(ibuf) * 6;
      display.setCursor((SCREEN_WIDTH - tw)/2, 72);
      display.print(ibuf);
    }
    else if (mode == 5) {
      // Vocal grande (size 4)
      display.setTextSize(4);
      const char* v = vocales[currentVocalIndex];
      int vW = strlen(v) * 6 * 4;
      int vX = (SCREEN_WIDTH - vW) / 2; if (vX < 0) vX = 0;
      display.setCursor(vX, 36);
      display.print(v);

      display.setTextSize(1);
      snprintf(buf, sizeof(buf), "%u/%u", currentVocalIndex + 1, NUM_VOCALES);
      int tw = strlen(buf) * 6;
      display.setCursor((SCREEN_WIDTH - tw)/2, 86);
      display.print(buf);
    }
    else if (mode == 6) {
      // Número (size 3)
      display.setTextSize(3);
      char nbuf[3];
      uint8_t n = currentNumeroIndex + 1; // 1..10
      if (n == 10) { nbuf[0] = '1'; nbuf[1] = '0'; nbuf[2] = '\0'; }
      else { nbuf[0] = '0' + n; nbuf[1] = '\0'; }
      int nW = strlen(nbuf) * 6 * 3;
      int nX = (SCREEN_WIDTH - nW) / 2; if (nX < 0) nX = 0;
      display.setCursor(nX, 24);
      display.print(nbuf);

      // Palabra en inglés entre comillas (size 1)
      display.setTextSize(1);
      snprintf(buf, sizeof(buf), "\"%s\"", numerosEN[currentNumeroIndex]);
      int w2 = strlen(buf) * 6;
      int x2 = (SCREEN_WIDTH - w2) / 2; if (x2 < 0) x2 = 0;
      display.setCursor(x2, 64);
      display.print(buf);

      // Indicador
      char ibuf[12];
      snprintf(ibuf, sizeof(ibuf), "%u/%u", currentNumeroIndex + 1, NUM_NUMS);
      int tw = strlen(ibuf) * 6;
      display.setCursor((SCREEN_WIDTH - tw)/2, 84);
      display.print(ibuf);
    }
    else if (mode == 7) {
      // Color EN (size 2) + ES (size 1), como modo 4
      display.setTextSize(2);
      const char* cen = coloresEN[currentColorIndex];
      int enW = strlen(cen) * 6 * 2;
      int enX = (SCREEN_WIDTH - enW) / 2; if (enX < 0) enX = 0;
      display.setCursor(enX, 28);
      display.print(cen);

      display.setTextSize(1);
      const char* ces = coloresES[currentColorIndex];
      int esW = strlen(ces) * 6;
      int esX = (SCREEN_WIDTH - esW) / 2; if (esX < 0) esX = 0;
      display.setCursor(esX, 54);
      display.print(ces);

      char ibuf[12];
      snprintf(ibuf, sizeof(ibuf), "%u/%u", currentColorIndex + 1, NUM_COLORES);
      int tw = strlen(ibuf) * 6;
      display.setCursor((SCREEN_WIDTH - tw)/2, 72);
      display.print(ibuf);
    }

    display.display();
  }
}
