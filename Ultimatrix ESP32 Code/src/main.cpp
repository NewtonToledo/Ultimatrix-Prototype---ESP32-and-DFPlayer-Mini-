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

#define POT_PIN         34
#define BTN_PLAY_PIN    32
#define BTN_VOLUP_PIN   33
#define BTN_VOLDN_PIN   26
#define BTN_MODE_PIN    25    
#define RX2_PIN         16
#define TX2_PIN         17
#define TOTAL_TRACKS    4

// nombres de canciones
const char* fileNames[TOTAL_TRACKS+1] = {
  "", "Ben 10 Intro", "Bring me to life", "Faded", "Force"
};
// duraciones en segundos
const uint16_t trackDurations[TOTAL_TRACKS+1] = {
  0, 61, 244, 212, 240
};

// pantalla SH1107 I2C
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  128
#define OLED_RESET     -1
#define I2C_ADDRESS    0x3C
Adafruit_SH1107 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

HardwareSerial       dfSerial(2);
DFRobotDFPlayerMini  player;
SemaphoreHandle_t    playerMutex;

volatile int     currentTrack    = 1;
volatile bool    isPlaying       = false;
uint32_t         trackStartTime  = 0;
uint32_t         trackDurationMs = 0;
bool             pauseState      = false;
uint32_t         pauseStartTime  = 0;

volatile bool    volFlag      = false;
volatile int     volIconType  = 0;    // 1=+Vol, 2=-Vol
uint32_t         volTime      = 0;

// ——— Modo de operacion ———
// 0 = reproductor MP3
// 1 = icono omnitrix
// 2 = grupo de aliens “Modo 2”
// 3 = grupo de aliens “Modo 3”
volatile uint8_t mode = 0;

#define MODES_COUNT        4
#define MODE1_SOUND_TRACK  7

#define NUM_IMAGES 10
extern const uint8_t* const imagenesModo2[NUM_IMAGES];
extern const uint8_t* const imagenesModo3[NUM_IMAGES];
volatile uint8_t currentImageIndex = 0;

void nextTrackAuto();
void taskPot(void*);
void taskButtons(void*);
void taskDisplay(void*);

void setup() {
  setCpuFrequencyMhz(80);         // reducir frecuencia a 80 MHz
  WiFi.mode(WIFI_OFF);            // apagar WiFi
  btStop();                        // apagar Bluetooth
  Serial.begin(115200);

  // DFPlayer
  dfSerial.begin(9600, SERIAL_8N1, RX2_PIN, TX2_PIN);
  while (!player.begin(dfSerial)) { delay(200); }
  player.volume(20);
  player.play(currentTrack);
  isPlaying       = true;
  trackStartTime  = millis();
  trackDurationMs = trackDurations[currentTrack] * 1000UL;

  // potenciometro
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
  xSemaphoreGive(playerMutex);
}

void taskPot(void *pv) {
  int lastRaw = -1;
  for (;;) {
    int raw = analogRead(POT_PIN);

    if (abs(raw - lastRaw) > 30) {
      lastRaw = raw;

      if (mode == 0) {
        int track = raw * TOTAL_TRACKS / 4096 + 1;
        track = constrain(track, 1, TOTAL_TRACKS);
        if (track != currentTrack) {
          xSemaphoreTake(playerMutex, pdMS_TO_TICKS(50));
            player.play(track);
            currentTrack    = track;
            isPlaying       = true;
            pauseState      = false;
            trackStartTime  = millis();
            trackDurationMs = trackDurations[currentTrack] * 1000UL;
          xSemaphoreGive(playerMutex);
        }
      }
      else if (mode == 2 || mode == 3) {
        static int8_t lastImageIndex = -1;
        uint8_t idx = raw * NUM_IMAGES / 4096;
        idx = constrain(idx, 0, NUM_IMAGES - 1);

        if (idx != lastImageIndex) {
          xSemaphoreTake(playerMutex, pdMS_TO_TICKS(50));
            player.play(6);  // Efecto de cambio
          xSemaphoreGive(playerMutex);
          lastImageIndex = idx;
        }
        currentImageIndex = idx;
      }

      // modo 1 ignorado completamente
    }

    vTaskDelay(pdMS_TO_TICKS(300));
  }
}

void taskButtons(void *pv) {
  bool lastP = HIGH, lastU = HIGH, lastD = HIGH, lastM = HIGH;
  for (;;) {
    bool m = digitalRead(BTN_MODE_PIN);
    if (m == LOW && lastM == HIGH) {
      mode = (mode + 1) % MODES_COUNT;
      xSemaphoreTake(playerMutex, portMAX_DELAY);
        if      (mode == 0) {
          // volvemos al reproductor MP3
          player.play(currentTrack);
          isPlaying = true;
        }
        else if (mode == 1) {
          player.play(MODE1_SOUND_TRACK);
          isPlaying = true;
        }
        else {
          player.pause();
          player.play(5);
          isPlaying = false;
        }
      xSemaphoreGive(playerMutex);
    }
    lastM = m;

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
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void taskDisplay(void *pv) {
  const int refreshMs = 200;   
  static uint32_t lastRefresh = 0;
  static int16_t  fnameX     = -100;
  static bool     fnameDirR  = true;
  static uint32_t lastMove   = 0;
  char buf[16];

  for (;;) {
    uint32_t now = millis();
    if (now - lastRefresh < (uint32_t)refreshMs) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    lastRefresh = now;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);

    if (mode == 0) {
      // —— UI reproductor MP3 —— 
      uint32_t elapsed = pauseState
        ? (pauseStartTime - trackStartTime)
        : (now - trackStartTime);
      if (elapsed > trackDurationMs) elapsed = trackDurationMs;
      if (elapsed >= trackDurationMs) {
        nextTrackAuto();
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
        if (fnameDirR) {
          if (++fnameX >= rb) fnameDirR = false;
        } else {
          if (--fnameX <= lb) fnameDirR = true;
        }
      }
      display.setCursor(fnameX, 26);
      display.print(fileNames[currentTrack]);

      // 3) Cronómetro MM:SS
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

      // 5) Barra de progreso
      int barY = cy + 12;
      display.drawRect(14, barY, 100, 3, SH110X_WHITE);
      uint32_t w = (uint64_t)elapsed * 100 / trackDurationMs;
      if (w > 0) display.fillRect(14, barY, w, 3, SH110X_WHITE);

      // 6) Icono +Vol / -Vol
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
      display.drawBitmap(
        0, 0,
        omnitrix,
        SCREEN_WIDTH, SCREEN_HEIGHT,
        SH110X_WHITE
      );
    }
    else if (mode == 2) {
      display.drawBitmap(
        0, 0,
        imagenesModo2[currentImageIndex],
        SCREEN_WIDTH, SCREEN_HEIGHT,
        SH110X_WHITE
      );
    }
    else { // mode == 3
      display.drawBitmap(
        0, 0,
        imagenesModo3[currentImageIndex],
        SCREEN_WIDTH, SCREEN_HEIGHT,
        SH110X_WHITE
      );
    }

    display.display();
  }
}
