# 🤖 Implementasi PID pada Pengaturan Kecepatan Motor DC Rover

![MATLAB](https://img.shields.io/badge/MATLAB-R2023a%2B-orange?style=flat-square&logo=mathworks)
![Simulink](https://img.shields.io/badge/Simulink-Included-blue?style=flat-square)
![License](https://img.shields.io/badge/License-MIT-green?style=flat-square)
![Status](https://img.shields.io/badge/Status-Active-brightgreen?style=flat-square)

Implementasi kontrol **PID (Proportional Integral Derivative)** untuk mengatur kecepatan motor DC rover terhadap gangguan beban (*load disturbance*) menggunakan MATLAB dan Simulink.

---

## 📋 Daftar Isi

- [Deskripsi](#-deskripsi)
- [Fitur](#-fitur)
- [Persyaratan](#-persyaratan)
- [Struktur Program](#-struktur-program)
- [Parameter Motor DC](#-parameter-motor-dc)
- [Model Transfer Function](#-model-transfer-function)
- [Kontroler PID](#-kontroler-pid)
- [Simulasi](#-simulasi)
- [Visualisasi](#-visualisasi)
- [Analisis Kestabilan](#-analisis-kestabilan)
- [Cara Penggunaan](#-cara-penggunaan)
- [Output Simulink](#-output-simulink)
- [Kontribusi](#-kontribusi)

---

## 📖 Deskripsi

Project ini mengimplementasikan sistem kontrol PID untuk motor DC rover. Sistem dirancang untuk mempertahankan kecepatan putar motor pada setpoint yang diinginkan meskipun terdapat gangguan beban eksternal. Program mencakup pemodelan matematis motor DC, desain kontroler PID dengan derivative filter, simulasi closed-loop, serta analisis frekuensi dan kestabilan sistem.

---

## ✨ Fitur

- ✅ Pemodelan Transfer Function Motor DC
- ✅ Implementasi PID dengan derivative filter
- ✅ Simulasi state-space manual
- ✅ Simulasi closed-loop system
- ✅ Analisis performa terhadap gangguan beban
- ✅ Analisis frekuensi dan kestabilan (Bode, Nyquist, Root Locus)
- ✅ Pembuatan model Simulink otomatis
- ✅ Dashboard visualisasi interaktif
- ✅ Perbandingan respon sistem

---

## 🛠️ Persyaratan

| Software | Versi Minimum |
|----------|--------------|
| MATLAB | R2021a |
| Simulink | R2021a |
| Control System Toolbox | R2021a |

> **Catatan:** Pastikan Control System Toolbox terinstal untuk fungsi `tf()`, `feedback()`, `bode()`, `nyquist()`, dan `margin()`.

---

## 📁 Struktur Program

```
PID_DC_Motor_Rover/
│
├── main.m                        # File utama program
├── PID_DC_Motor_Rover.slx        # Model Simulink (auto-generated)
└── README.md                     # Dokumentasi project
```

Program terdiri dari 13 bagian utama:

| Bagian | Deskripsi |
|--------|-----------|
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

## ⚙️ Parameter Motor DC

Model motor DC menggunakan parameter fisik berikut:

| Parameter | Simbol | Nilai | Satuan |
|-----------|--------|-------|--------|
| Momen Inersia | J | 0.01 | kg·m² |
| Koefisien Gesekan Viskos | b | 0.1 | N·m·s |
| Konstanta Motor | K | 0.01 | — |
| Resistansi Armature | R | 1 | Ω |
| Induktansi Armature | L | 0.5 | H |

---

## 📐 Model Transfer Function

Transfer function motor DC yang menghubungkan tegangan input dengan kecepatan angular output:

$$G(s) = \frac{K}{(Js+b)(Ls+R)+K^2}$$

Substitusi parameter menghasilkan:

$$G(s) = \frac{0.01}{0.005s^2 + 0.11s + 0.1001}$$

---

## 🎛️ Kontroler PID

Kontroler PID dengan derivative filter digunakan untuk mencapai performa yang diinginkan:

$$C(s) = K_p + \frac{K_i}{s} + \frac{K_d \cdot N}{s + N}$$

**Parameter PID:**

| Parameter | Simbol | Nilai |
|-----------|--------|-------|
| Proportional Gain | Kp | 100 |
| Integral Gain | Ki | 200 |
| Derivative Gain | Kd | 10 |
| Derivative Filter Coefficient | N | 100 |

Bentuk transfer function PID lengkap:

$$C(s) = \frac{K_d N s^2 + (K_p N + K_d N^2)s + K_i N}{s^2 + Ns}$$

---

## 🔬 Simulasi

### Simulasi Tanpa Gangguan

Sistem diuji menggunakan input step sebesar **1 rad/s** untuk mengukur:

| Metrik | Keterangan |
|--------|------------|
| Rise Time | Waktu untuk mencapai 90% setpoint |
| Settling Time | Waktu hingga error < 2% |
| Overshoot | Persentase overshoot maksimum |
| Steady-state Error | Error pada kondisi tunak |

### Simulasi Dengan Gangguan Beban

Gangguan torsi (*load disturbance*) diberikan untuk menguji robustness kontroler:

| Parameter | Nilai |
|-----------|-------|
| Waktu mulai gangguan | 3 s |
| Waktu akhir gangguan | 7 s |
| Amplitudo gangguan | 0.3 N·m |

**Tujuan simulasi:**
- Menguji kemampuan PID menolak gangguan
- Mengamati recovery system setelah gangguan
- Menganalisis error transien akibat disturbance

---

## 📊 Visualisasi

Program menghasilkan **4 figure** visualisasi:

### Figure 1 — Dashboard PID Motor DC Rover
Dashboard utama dengan 6 subplot:
1. Respon kecepatan motor (dengan dan tanpa gangguan)
2. Step response closed-loop
3. Error kecepatan terhadap waktu
4. Sinyal kontrol PID (output kontroler)
5. Profil gangguan beban
6. Arus jangkar motor

### Figure 2 — Perbandingan Respon
Perbandingan langsung antara:
- Respon sistem **tanpa gangguan**
- Respon sistem **dengan gangguan**

### Figure 3 — Analisis Frekuensi dan Kestabilan
- **Diagram Bode** — magnitude dan phase vs frekuensi
- **Diagram Nyquist** — lintasan nyquist open-loop
- **Pole-Zero Map** — lokasi pole dan zero sistem
- **Root Locus** — locus akar karakteristik

### Figure 4 — Simulink vs State-Space
Validasi silang antara:
- Simulasi manual MATLAB (state-space)
- Simulasi model Simulink

---

## 📈 Analisis Kestabilan

Program menghitung dan menampilkan parameter kestabilan berikut:

| Parameter | Simbol | Kriteria Stabil |
|-----------|--------|-----------------|
| Gain Margin | GM | > 6 dB |
| Phase Margin | PM | > 45° |
| Gain Crossover Frequency | ωgc | — |
| Phase Crossover Frequency | ωpc | — |

> Sistem dinyatakan **stabil** apabila kedua kriteria terpenuhi secara bersamaan.

---

## 🚀 Cara Penggunaan

1. **Clone atau unduh** repository ini ke direktori lokal.

2. **Buka MATLAB** dan arahkan *Current Folder* ke direktori project:
   ```matlab
   cd('path/to/PID_DC_Motor_Rover')
   ```

3. **Jalankan program utama:**
   ```matlab
   run('main.m')
   ```

4. **Tunggu** hingga seluruh simulasi selesai. Program akan:
   - Menampilkan 4 figure visualisasi
   - Mencetak hasil analisis performa di Command Window
   - Membuat file `PID_DC_Motor_Rover.slx` secara otomatis

5. **(Opsional)** Buka model Simulink yang telah dibuat:
   ```matlab
   open_system('PID_DC_Motor_Rover')
   ```

### Kustomisasi Parameter

Untuk mengubah parameter motor atau PID, edit bagian awal `main.m`:

```matlab
% Parameter Motor DC
J = 0.01;   % Momen inersia (kg.m^2)
b = 0.1;    % Gesekan viskos (N.m.s)
K = 0.01;   % Konstanta motor
R = 1;      % Resistansi armature (Ohm)
L = 0.5;    % Induktansi armature (H)

% Parameter PID
Kp = 100;
Ki = 200;
Kd = 10;
N  = 100;   % Derivative filter coefficient
```

---

## 📦 Output Simulink

Program secara otomatis membuat model Simulink dengan nama:

```
PID_DC_Motor_Rover.slx
```

Model ini dapat dibuka, diedit, dan disimulasikan langsung di Simulink untuk keperluan pengembangan lebih lanjut.

---

## 🤝 Kontribusi

Kontribusi sangat disambut! Silakan:

1. Fork repository ini
2. Buat branch fitur baru: `git checkout -b fitur/nama-fitur`
3. Commit perubahan: `git commit -m 'Menambahkan fitur baru'`
4. Push ke branch: `git push origin fitur/nama-fitur`
5. Buka Pull Request

---

## 📄 Lisensi

Project ini dilisensikan di bawah **MIT License**. Lihat file `LICENSE` untuk detail lengkap.

---

<div align="center">

Dibuat dengan ❤️ menggunakan MATLAB & Simulink

</div>
