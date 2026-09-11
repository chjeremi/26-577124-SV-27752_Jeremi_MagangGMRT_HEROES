/**
 * @file main.cpp
 * @brief Modul Kontrol Penggerak Utama & Mekanisme Safety Robot KRAI (GMRT 2027)
 * @author Jeremi
 * @date 2026-09-11
 * 
 * @details
 * Program ini mengimplementasikan Finite State Machine (FSM) non-blocking untuk
 * mengendalikan motor DC via H-Bridge L293D menggunakan masukan Keypad 4x4.
 * Sistem dilengkapi dengan:
 * 1. Open-Loop Fallback Redundancy jika terjadi kegagalan/timeout pada Encoder.
 * 2. Emergency Stop (E-Stop) bertipe Latching dengan prioritas eksekusi tertinggi.
 * 3. Indikator visual real-time menggunakan NeoPixel LED Strip.
 * 
 * ==============================================================================
 * KEYPAD 4x4 MAPPING & CONTROL LAYOUT
 * ==============================================================================
 * [ 1 ] : Motor Direction CW  (Clockwise)
 * [ 2 ] : Motor STOP          (Penghentian Normal)
 * [ 3 ] : Motor Direction CCW (Counter-Clockwise)
 * [ A ] : EMERGENCY STOP      (E-Stop Latch/Unlatch) -> Prioritas Utama
 * [ 5 ] : Speed UP            (+ PWM Duty Cycle via STEP_PWM)
 * [ 8 ] : Speed DOWN          (- PWM Duty Cycle via STEP_PWM)
 * 
 * ==============================================================================
 * FINITE STATE MACHINE (FSM) STATES
 * ==============================================================================
 * - STATE_NORMAL       : Pembacaan closed-loop encoder aktif. Indikator LED:
 *                        * Hijau = Putaran CW
 *                        * Biru  = Putaran CCW
 * - STATE_ENCODER_FAULT: Timeout encoder terdeteksi (>400ms).
 *                        Pindah ke Open-Loop Fallback Mode (Visual LED: Orange).
 * - STATE_ESTOP        : Sistem terkunci (latching), motor diputus seketika,
 *                        selain tombol 'A' semua input diabaikan. (Visual LED: Kedip Merah).
 * ==============================================================================
 */

#include <Keypad.h>
#include <Adafruit_NeoPixel.h>

// ==============================================================================
// 1. HARDWARE PIN DEFINITIONS
// ==============================================================================
#define PIN_ENCODER_SIGNAL  2   ///< Pin External Interrupt 0 (INT0) pembaca pulsa encoder
#define PIN_ENABLE_PWM      3   ///< Pin PWM ke Pin 1 L293D (1,2EN) untuk kontrol kecepatan[cite: 1, 2]
#define PIN_MOTOR_IN1       4   ///< Pin Arah IN1 L293D[cite: 1, 2]
#define PIN_MOTOR_IN2       5   ///< Pin Arah IN2 L293D[cite: 1, 2]

#define PIN_NEOPIXEL        A0  ///< Pin Output Sinyal Data NeoPixel Strip[cite: 1, 2]
#define NUM_LEDS            8   ///< Jumlah LED pada Strip WS2812B/NeoPixel[cite: 1, 2]

// ==============================================================================
// 2. TIMING & CONTROL CONSTANTS
// ==============================================================================
const byte ROWS = 4;            ///< Jumlah baris Keypad Matrix[cite: 1, 2]
const byte COLS = 4;            ///< Jumlah kolom Keypad Matrix[cite: 1, 2]

/// Mapping tombol matriks Keypad 4x4[cite: 1, 2]
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte rowPins[ROWS] = {13, 12, 11, 10}; ///< Pin Arduino terhubung ke baris Keypad[cite: 1, 2]
byte colPins[COLS] = {9, 8, 7, 6};     ///< Pin Arduino terhubung ke kolom Keypad[cite: 1, 2]

const unsigned long ENCODER_TIMEOUT = 400; ///< Ambang batas waktu (ms) tanpa pulsa sebelum FAULT
const unsigned long ESTOP_BLINK_INT = 250; ///< Interval kedip (ms) LED merah saat E-Stop aktif[cite: 1]
const int PWM_STEP                  = 32;  ///< Inkremen Duty Cycle PWM per step (~12.5% dari 255)

// ==============================================================================
// 3. ENUMERATIONS & GLOBAL SYSTEM VARIABLES
// ==============================================================================

/**
 * @enum SystemState
 * @brief Status utama operasi logika robot.
 */
enum SystemState {
  STATE_NORMAL,        ///< Operasi normal closed-loop
  STATE_ENCODER_FAULT, ///< Open-loop fallback mode akibat encoder stuck/hilang sinyal[cite: 1]
  STATE_ESTOP          ///< Mode darurat ter-latch[cite: 1]
};

/**
 * @enum MotorDirection
 * @brief Arah rotasi aktual motor DC.
 */
enum MotorDirection {
  MOTOR_STOP,          ///< Motor dalam kondisi diam
  MOTOR_CW,            ///< Motor berputar Searah Jarum Jam (Clockwise)
  MOTOR_CCW            ///< Motor berputar Berlawanan Arah Jarum Jam (Counter-Clockwise)
};

// State Variables
SystemState currentState  = STATE_NORMAL;
MotorDirection currentDir = MOTOR_STOP;

// Motor & Telemetry Variables
int currentPWM = 0;                         ///< Sinyal Duty Cycle PWM aktif (0 - 255)
volatile unsigned long pulseCount = 0;     ///< Counter pulsa dari ISR Interrupt (volatile untuk akses thread-safe)
unsigned long lastPulseTime       = 0;     ///< Timestamp (ms) pulsa terakhir diterima dari encoder
unsigned long lastEStopBlink     = 0;     ///< Timestamp (ms) penanda kedip LED E-Stop
bool eStopBlinkState              = false; ///< Toggle status ON/OFF LED saat E-Stop

// Object Instantiation
Keypad customKeypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
Adafruit_NeoPixel strip(NUM_LEDS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// ==============================================================================
// 4. INTERRUPT SERVICE ROUTINE (ISR)
// ==============================================================================

/**
 * @brief ISR untuk menangani sinyal pulsa dari Encoder.
 * @note Dipanggil secara otomatis oleh hardware ketika terjadi logika CHANGE pada PIN_ENCODER_SIGNAL.
 */
void encoderISR() {
  pulseCount++;
  lastPulseTime = millis();
}

// ==============================================================================
// 5. HARDWARE DRIVER FUNCTIONS
// ==============================================================================

/**
 * @brief Mengirim sinyal kontrol fisik ke IC Driver L293D.
 * @details Fungsi ini memutus sinyal daya jika sistem berada dalam E-Stop, 
 *          kecepatan 0, atau perintah STOP.[cite: 1]
 */
void updateMotorHardware() {
  // Overriding Safety Check: Matikan motor jika E-Stop aktif atau PWM = 0[cite: 1]
  if (currentState == STATE_ESTOP || currentDir == MOTOR_STOP || currentPWM == 0) {
    digitalWrite(PIN_MOTOR_IN1, LOW);
    digitalWrite(PIN_MOTOR_IN2, LOW);
    analogWrite(PIN_ENABLE_PWM, 0);
    return;
  }

  // Konfigurasi Arah Putaran H-Bridge L293D[cite: 1]
  if (currentDir == MOTOR_CW) {
    digitalWrite(PIN_MOTOR_IN1, HIGH);
    digitalWrite(PIN_MOTOR_IN2, LOW);
  } else if (currentDir == MOTOR_CCW) {
    digitalWrite(PIN_MOTOR_IN1, LOW);
    digitalWrite(PIN_MOTOR_IN2, HIGH);
  }

  // Sinyal Regulasi Kecepatan PWM[cite: 1]
  analogWrite(PIN_ENABLE_PWM, currentPWM);
}

/**
 * @brief Mewarnai seluruh piksel pada LED Strip secara simultan.
 * @param color Format warna uint32_t (contoh: strip.Color(R, G, B))
 */
void setStripColor(uint32_t color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

// ==============================================================================
// 6. VISUALIZER & DISPLAY LOGIC (NEOPIXEL)
// ==============================================================================

/**
 * @brief Mengatur pola visualisasi indikator LED berdasarkan State dan Kecepatan Motor.[cite: 1]
 */
void updateNeoPixelDisplay() {
  // Abaikan rutin visual normal jika sedang berada dalam Emergency Stop Mode[cite: 1]
  if (currentState == STATE_ESTOP) return;

  strip.clear();

  // Matikan LED jika motor berhenti
  if (currentPWM == 0 || currentDir == MOTOR_STOP) {
    strip.show();
    return;
  }

  int activeLEDs = 0;
  uint32_t color;

  // Pemetaan Jumlah LED berdasarkan nilai PWM (1 sampai 8 LED)
  activeLEDs = map(currentPWM, 0, 255, 1, NUM_LEDS);

  if (currentState == STATE_NORMAL) {
    // Mode Closed-Loop: Hijau = Clockwise, Biru = Counter-Clockwise[cite: 1]
    color = (currentDir == MOTOR_CW) ? strip.Color(0, 255, 0) : strip.Color(0, 0, 255);
  } 
  else if (currentState == STATE_ENCODER_FAULT) {
    // Mode Fallback (Open-Loop): Orange/Kuning Indikasi Sinyal Encoder Hilang[cite: 1]
    color = strip.Color(255, 140, 0);
  }

  // Visualisasi arah pergerakan baris LED[cite: 1]
  if (currentDir == MOTOR_CW) {
    for (int i = 0; i < activeLEDs; i++) {
      strip.setPixelColor(i, color);
    }
  } else if (currentDir == MOTOR_CCW) {
    for (int i = NUM_LEDS - 1; i >= (NUM_LEDS - activeLEDs); i--) {
      strip.setPixelColor(i, color);
    }
  }

  strip.show();
}

// ==============================================================================
// 7. SAFETY, FAULT DETECTION & EMERGENCY LOGIC
// ==============================================================================

/**
 * @brief Memeriksa kesehatan sinyal encoder secara non-blocking.
 * @details Jika motor diperintahkan bergerak namun tidak ada pulsa baru dalam
 *          rentang ENCODER_TIMEOUT, sistem beralih ke Fallback Mode.[cite: 1]
 */
void checkEncoderFault() {
  if (currentPWM > 0 && currentDir != MOTOR_STOP) {
    
    // Deteksi Timeout Pulsa Encoder[cite: 1]
    if (millis() - lastPulseTime > ENCODER_TIMEOUT) {
      if (currentState != STATE_ENCODER_FAULT && currentState != STATE_ESTOP) {
        currentState = STATE_ENCODER_FAULT;
        
        // Log Telemetry System State Change
        Serial.println(F("[SYSTEM WARNING] Encoder Signal Lost/Stuck!"));
        Serial.println(F("[SYSTEM MODE] Switched to OPEN-LOOP FALLBACK MODE (PWM)"));
      }
    } 
    // Recovery Otomatis jika Encoder Kembali Mengirimkan Pulsa
    else {
      if (currentState == STATE_ENCODER_FAULT) {
        currentState = STATE_NORMAL;
        
        Serial.println(F("[SYSTEM INFO] Encoder Signal Recovered."));
        Serial.println(F("[SYSTEM MODE] Switched back to CLOSED-LOOP MODE"));
      }
    }

  } else {
    // Reset timer pemantau saat motor dalam posisi STOP[cite: 1]
    lastPulseTime = millis();
    if (currentState == STATE_ENCODER_FAULT) {
      currentState = STATE_NORMAL;
    }
  }
}

/**
 * @brief Mengubah status Latch E-Stop (Toggle ON/OFF).
 * @note Tombol Emergency Stop ('A') dapat membatalkan semua aktivitas motor.[cite: 1]
 */
void toggleEStop() {
  if (currentState != STATE_ESTOP) {
    // Lock / Latch Emergency State[cite: 1]
    currentState = STATE_ESTOP;
    currentPWM   = 0;
    currentDir   = MOTOR_STOP;
    updateMotorHardware(); // Mematikan arus ke driver motor seketika[cite: 1]
    
    Serial.println(F("\n=========================================="));
    Serial.println(F(" !!! EMERGENCY STOP ACTIVATED (LOCKED) !!! "));
    Serial.println(F("==========================================\n"));
  } else {
    // Unlatch Emergency State[cite: 1]
    currentState = STATE_NORMAL;
    strip.clear();
    strip.show();
    
    Serial.println(F("\n[SYSTEM INFO] Emergency Stop Cleared. System Normal.\n"));
  }
}

/**
 * @brief Executable Routine saat E-Stop aktif. Menangani blink LED indikator tanpa delay().[cite: 1]
 */
void handleEStopRoutine() {
  updateMotorHardware(); // Memastikan hardware motor tetap dalam status mati total[cite: 1]

  // Rutin Kedip Merah Non-Blocking (250 ms)[cite: 1]
  unsigned long currentMillis = millis();
  if (currentMillis - lastEStopBlink >= ESTOP_BLINK_INT) {
    lastEStopBlink  = currentMillis;
    eStopBlinkState = !eStopBlinkState;

    if (eStopBlinkState) {
      setStripColor(strip.Color(255, 0, 0)); // Merah Menyala[cite: 1]
    } else {
      strip.clear();
      strip.show();                          // Merah Padam
    }
  }
}

// ==============================================================================
// 8. KEYPAD INPUT PARSER & CONTROLLER
// ==============================================================================

/**
 * @brief Memproses perintah dari tombol Keypad.
 * @param key Karakter tombol yang ditekan dari Keypad matrix.
 */
void handleKeypress(char key) {
  // High Priority Interruption Check[cite: 1]
  if (key == 'A') {
    toggleEStop();
    return;
  }

  // Kunci semua input tombol lain jika E-Stop dalam kondisi aktif[cite: 1]
  if (currentState == STATE_ESTOP) {
    Serial.println(F("[REJECTED] System is locked in EMERGENCY STOP mode!"));
    return;
  }

  // Eksekusi Command Kontrol[cite: 1]
  switch (key) {
    case '1': // Set Putaran Searah Jarum Jam (CW)
      currentDir = MOTOR_CW;
      if (currentPWM == 0) currentPWM = 128; // Standard Initial Speed (50%)
      Serial.println(F("[COMMAND] Direction: CLOCKWISE (CW)"));
      break;

    case '3': // Set Putaran Berlawanan Arah Jarum Jam (CCW)
      currentDir = MOTOR_CCW;
      if (currentPWM == 0) currentPWM = 128; // Standard Initial Speed (50%)
      Serial.println(F("[COMMAND] Direction: COUNTER-CLOCKWISE (CCW)"));
      break;

    case '2': // Penghentian Normal Motor
      currentDir = MOTOR_STOP;
      currentPWM = 0;
      Serial.println(F("[COMMAND] Motor STOP"));
      break;

    case '5': // Inkremen Kecepatan (+ PWM Step)
      currentPWM += PWM_STEP;
      if (currentPWM > 255) currentPWM = 255;
      Serial.print(F("[COMMAND] Speed UP -> Duty Cycle PWM: "));
      Serial.print((currentPWM * 100) / 255);
      Serial.println(F("%"));
      break;

    case '8': // Dekremen Kecepatan (- PWM Step)
      currentPWM -= PWM_STEP;
      if (currentPWM < 0) currentPWM = 0;
      if (currentPWM == 0) currentDir = MOTOR_STOP;
      Serial.print(F("[COMMAND] Speed DOWN -> Duty Cycle PWM: "));
      Serial.print((currentPWM * 100) / 255);
      Serial.println(F("%"));
      break;

    default:
      // Abaikan tombol yang tidak masuk dalam pustaka perintah
      break;
  }

  // Terapkan perubahan setting ke hardware motor[cite: 1]
  updateMotorHardware();
}

// ==============================================================================
// 9. CORE SYSTEM INITIALIZATION & MAIN LOOP
// ==============================================================================

void setup() {
  // Inisialisasi Serial Telemetry
  Serial.begin(9600);
  Serial.println(F("================================================================="));
  Serial.println(F(" HEROES GMRT 2027 - MAIN MOTION CONTROLLER & SAFETY TESTBENCH    "));
  Serial.println(F(" Developer : Jeremi-26/577124/SV/27752                          "));
  Serial.println(F("================================================================="));

  // Mode Pin Hardware
  pinMode(PIN_ENABLE_PWM, OUTPUT);
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);

  // Inisialisasi External Interrupt Encoder
  pinMode(PIN_ENCODER_SIGNAL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_SIGNAL), encoderISR, CHANGE);

  // Inisialisasi Bus Driver NeoPixel
  strip.begin();
  strip.show();
  strip.setBrightness(100); // Skala Kecerahan (0 - 255)

  // Inisialisasi Status Hardware
  updateMotorHardware();
  lastPulseTime = millis();
  
  Serial.println(F("[SYSTEM READY] Awaiting Keypad Command Inputs...\n"));
}

void loop() {
  // Pembacaan Matriks Keypad
  char key = customKeypad.getKey();

  // 1. Eksekusi Input dari User[cite: 1]
  if (key != NO_KEY) {
    handleKeypress(key);
  }

  // 2. Eksekusi State Machine Utama[cite: 1]
  if (currentState == STATE_ESTOP) {
    handleEStopRoutine();    // Rutin Keamanan E-Stop[cite: 1]
  } else {
    checkEncoderFault();     // Monitoring Kesehatan Encoder[cite: 1]
    updateNeoPixelDisplay(); // Update Indikator Visual LED[cite: 1]
  }
}