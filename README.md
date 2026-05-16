# IMPLEMENTASI PID PADA PENGATURAN KECEPATAN MOTOR DC ROVER

## Deskripsi Project
Project ini merupakan implementasi kontrol PID (*Proportional Integral Derivative*) untuk mengatur kecepatan motor DC rover terhadap gangguan beban (*load disturbance*).

Program dibuat menggunakan MATLAB dan Simulink dengan fitur:

- Pemodelan Transfer Function Motor DC
- Implementasi PID dengan derivative filter
- Simulasi state-space manual
- Simulasi closed-loop system
- Analisis performa terhadap gangguan
- Analisis frekuensi dan kestabilan
- Pembuatan model Simulink otomatis
- Perbandingan respon sistem

---

# Struktur Program

Program terdiri dari beberapa bagian utama:

| Bagian | Deskripsi |
|---|---|
| Bagian 1 | Parameter motor DC |
| Bagian 2 | Transfer function motor |
| Bagian 3 | Desain kontroler PID |
| Bagian 4 | Closed-loop system |
| Bagian 5 | Setup simulasi |
| Bagian 6 | Simulasi dengan gangguan |
| Bagian 7 | Simulasi tanpa gangguan |
| Bagian 8 | Analisis performa |
| Bagian 9 | Dashboard visualisasi |
| Bagian 10 | Perbandingan respon |
| Bagian 11 | Analisis kestabilan |
| Bagian 12 | Pembuatan model Simulink |
| Bagian 13 | Simulasi Simulink |

---

# Parameter Motor DC Rover

Model motor DC menggunakan parameter berikut:

| Parameter | Nilai |
|---|---|
| Momen Inersia (J) | 0.01 kg.m² |
| Gesekan Viskos (b) | 0.1 N.m.s |
| Konstanta Motor (K) | 0.01 |
| Resistansi Armature (R) | 1 Ohm |
| Induktansi Armature (L) | 0.5 H |

---

# Model Transfer Function

Transfer function motor DC:

\[
G(s) = \frac{K}{(Js+b)(Ls+R)+K^2}
\]

dengan:

\[
J = 0.01
\]
\[
b = 0.1
\]
\[
K = 0.01
\]
\[
R = 1
\]
\[
L = 0.5
\]

---

# Kontroler PID

Kontroler PID yang digunakan:

\[
C(s) = K_p + \frac{K_i}{s} + \frac{K_d N}{s+N}
\]

Parameter PID:

| Parameter | Nilai |
|---|---|
| Kp | 100 |
| Ki | 200 |
| Kd | 10 |
| N | 100 |

Bentuk transfer function PID:

\[
C(s)=\frac{K_dNs^2+(K_pN+K_dN^2)s+K_iN}{s^2+Ns}
\]

---

# Fitur Simulasi

## 1. Simulasi Tanpa Gangguan
Sistem diuji menggunakan input step sebesar 1 rad/s untuk melihat:

- Rise Time
- Settling Time
- Overshoot
- Steady-state error

---

## 2. Simulasi Dengan Gangguan Beban

Gangguan torsi diberikan pada:

| Parameter | Nilai |
|---|---|
| Waktu mulai gangguan | 3 s |
| Waktu akhir gangguan | 7 s |
| Amplitudo gangguan | 0.3 N.m |

Tujuan simulasi:
- Menguji robustness PID
- Mengamati recovery system
- Menganalisis error akibat disturbance

---

# Visualisasi yang Dihasilkan

## Figure 1 — Dashboard PID Motor DC Rover
Berisi 6 subplot:

1. Respon kecepatan motor
2. Step response closed-loop
3. Error kecepatan
4. Sinyal kontrol PID
5. Profil gangguan beban
6. Arus jangkar motor

---

## Figure 2 — Perbandingan Respon
Membandingkan:
- Sistem tanpa gangguan
- Sistem dengan gangguan

---

## Figure 3 — Analisis Frekuensi dan Kestabilan
Meliputi:
- Diagram Bode
- Diagram Nyquist
- Pole-Zero Map
- Root Locus

---

## Figure 4 — Simulink vs State-Space
Membandingkan:
- Simulasi manual MATLAB
- Simulasi Simulink

---

# Analisis Kestabilan

Program menghitung:

- Gain Margin
- Phase Margin
- Gain Crossover Frequency
- Phase Crossover Frequency

Kriteria stabil:

- Phase Margin > 45°
- Gain Margin > 6 dB

---

# Simulink Otomatis

Program dapat membuat model Simulink otomatis dengan nama:

```text
PID_DC_Motor_Rover.slx
