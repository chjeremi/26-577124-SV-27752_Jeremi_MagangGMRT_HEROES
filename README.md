# 🤖 Modul Kontrol Penggerak Utama & Mekanisme Safety Robot KRAI (GMRT 2027)

> **Subdivisi:** Programming & Control System — GMRT UGM  
> **Pengembang:** Jeremi Christian (26/577124/SV/27752)  
> **Platform:** Arduino UNO R3 / ATmega328P  
> **Simulasi:** Tinkercad Testbench Environment  

---

## Dokumentasi TinkerCad
https://www.tinkercad.com/things/49LmtVEqjQK-26-577124-sv-27752jeremimaganggmrtheroes?sharecode=uhqBEm5xb3hsFiuTFvvV_YjkZL9Wu9SbnyP6-YPqAF0


## Dokumentasi Video Simulasi
 **Link Video Demo (YouTube):**[Lihat Video Demo YouTube Shorts](https://youtube.com/shorts/epaCiMpIKVA?feature=share)

## 📋 Deskripsi Proyek

Proyek ini merupakan implementasi sistem kendali motor DC berikatan rapat (*closed-loop motion control*) berbasis **Finite State Machine (FSM)**. Rangkaian sistem ini dirancang untuk menangani penggerak utama robot KRAI dengan fokus pada **keandalan tinggi, kendali darurat (Emergency Stop), dan sistem keamanan redundansi (Open-Loop Fallback Mode)** apabila sensor encoder mengalami kegagalan sinyal (*fault/stuck*).

---

## 🏗️ Arsitektur Sistem & State Machine

Sistem memiliki tiga status utama (*System States*) yang dieksekusi secara non-blocking:

```text
                     ┌───────────────────────┐
                     │     STATE_NORMAL      │
                     │ (Closed-Loop Encoder) │
                     └──────────┬────────────┘
                                │
        ┌───────────────────────┴───────────────────────┐
        │ Timeout Encoder > 400ms                       │ Tekan Tombol 'A' (E-Stop)
        ▼                                               ▼
┌───────────────────────┐                       ┌───────────────────────┐
│  STATE_ENCODER_FAULT  │                       │      STATE_ESTOP      │
│  (Open-Loop Fallback) │                       │    (System Locked)    │
└──────────┬────────────┘                       └───────────┬───────────┘
           │                                                │
           │ Encoder Pulsa Pulih                            │ Tekan Tombol 'A' Kembali
           │                                                │ (Unlatch)
           └────────────────────┬───────────────────────────┘
                                │
                                ▼
                     ┌───────────────────────┐
                     │     STATE_NORMAL      │
                     └───────────────────────┘

```

1. **`STATE_NORMAL` (Closed-Loop Mode):**
   * Motor dikendalikan berdasarkan arah dan kecepatan PWM.
   * Sensor encoder memberikan pulsa balik (*feedback*) via interupsi eksternal.
   * **Indikator NeoPixel:** Hijau (CW) / Biru (CCW)[cite: 1].
2. **`STATE_ENCODER_FAULT` (Open-Loop Fallback Mode):**
   * Aktif secara otomatis jika motor diberi sinyal PWM tetapi tidak ada pulsa encoder baru selama $> 400\text{ ms}$[cite: 1].
   * Kontrol tetap berjalan menggunakan estimasi variabel PWM tanpa *feedback*[cite: 1].
   * **Indikator NeoPixel:** Orange/Kuning[cite: 1].
3. **`STATE_ESTOP` (Emergency Stop - Latching Mode):**
   * Memiliki prioritas eksekusi tertinggi. Aktif saat tombol **'A'** ditekan[cite: 1].
   * Arus ke L293D diputus seketika, motor mati total, dan seluruh input kontrol diabaikan hingga E-Stop di-unlatch[cite: 1].
   * **Indikator NeoPixel:** Kedip Merah (Interval 250ms)[cite: 1].

---

## 🎹 Mapping Keypad 4x4

| Tombol | Aksi Kontrol | Keterangan |
| :---: | :--- | :--- |
| **`1`** | **Clockwise (CW)** | Memutar motor searah jarum jam (Default PWM: 50%)[cite: 1] |
| **`2`** | **Stop Normal** | Mematikan motor secara halus[cite: 1] |
| **`3`** | **Counter-Clockwise (CCW)** | Memutar motor berlawanan arah jarum jam (Default PWM: 50%)[cite: 1] |
| **`5`** | **Speed UP** | Menambah Duty Cycle PWM sebesar $+32$ ($\approx 12.5\%$)[cite: 1] |
| **`8`** | **Speed DOWN** | Mengurangi Duty Cycle PWM sebesar $-32$ ($\approx 12.5\%$)[cite: 1] |
| **`A`** | **EMERGENCY STOP** | **Prioritas Utama:** Latch / Unlatch E-Stop[cite: 1] |

---

## 🔌 Pinout & Skematik Wiring

### 1. Arduino UNO ke Motor Driver L293D & Power Supply
* **Pin Digital 3 (PWM)** $\rightarrow$ Pin 1 L293D (`1,2EN` Enable PWM)[cite: 1, 2]
* **Pin Digital 4** $\rightarrow$ Pin 2 L293D (`IN1` Direction A)[cite: 1, 2]
* **Pin Digital 5** $\rightarrow$ Pin 7 L293D (`IN2` Direction B)[cite: 1, 2]
* **Pin 16 L293D (`VCC1`)** $\rightarrow$ 5V Arduino[cite: 2]
* **Pin 8 L293D (`VCC2`)** $\rightarrow$ Positif (+) Power Supply Eksternal 12V[cite: 1, 2]
* **Pin 4, 5, 12, 13 L293D** $\rightarrow$ Common GND (Arduino GND + Power Supply GND)[cite: 2]

### 2. Sensor Encoder & Output Motor DC
* **Motor Power (+) & (-)** $\rightarrow$ Output Pin 3 (`OUT1`) & Pin 6 (`OUT2`) L293D[cite: 1, 2]
* **Encoder Channel A Signal** $\rightarrow$ **Pin Digital 2 Arduino** (`INT0` External Interrupt)[cite: 1, 2]
* **Encoder VCC & GND** $\rightarrow$ 5V & GND Arduino[cite: 2]

### 3. Peripheral Visualizer & Input
* **Keypad 4x4 Row Pins (1–4)** $\rightarrow$ Pin Digital 13, 12, 11, 10 Arduino[cite: 1, 2]
* **Keypad 4x4 Col Pins (5–8)** $\rightarrow$ Pin Digital 9, 8, 7, 6 Arduino[cite: 1, 2]
* **NeoPixel LED Strip (DIN)** $\rightarrow$ **Pin Analog A0 Arduino**[cite: 1, 2]

---

## 🛠️ Requirements & Dependencies Library

Untuk mengompilasi kode program ini di Arduino IDE atau Tinkercad, pastikan library berikut sudah terinstal:

1. **`Keypad`** oleh Mark Stanley & Alexander Brevig (v3.1.1+)[cite: 1, 2]
2. **`Adafruit_NeoPixel`** oleh Adafruit (v1.10.0+)[cite: 1, 2]

---

## 🚀 Langkah Uji Coba Simulasi (Tinkercad)

1. Buka sirkuit simulasi pada Tinkercad Testbench.
2. Pastikan unit **Power Supply** diset pada **12.0 Volt** dan tombol sakelarnya dalam posisi **ON**[cite: 1, 2].
3. Tekan **Start Simulation**[cite: 1, 2].
4. Buka **Serial Monitor** (Baudrate: **9600**).
5. Tekan tombol **`1`** pada Keypad untuk menyalakan motor CW[cite: 1].
6. Amati indikator LED NeoPixel:
   * **Detik 0–0,4:** LED menyala **Hijau** (Closed-Loop Mode)[cite: 1].
   * **Detik >0,4:** Karena pulsa encoder diam pada simulasi, LED otomatis berubah warna menjadi **Orange** dan Serial Monitor menampilkan log `[SYSTEM WARNING] Encoder Signal Lost/Stuck!` (Open-Loop Fallback Mode)[cite: 1].
7. Tekan tombol **`A`** untuk menguji mekanisme **Emergency Stop**: motor akan mati total seketika dan LED berkedip merah[cite: 1].
8. Tekan kembali tombol **`A`** untuk membuka kuncian (*unlatch*) sistem[cite: 1].

---

## 📄 Lisensi & Hak Cipta

Pengembangan kode dan dokumentasi ini disusun untuk keperluan seleksi dan pengujian teknis **GMRT (Gadjah Mada Robotic Team) 2027**.
