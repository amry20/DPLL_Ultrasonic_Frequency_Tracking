# DPLL USB Serial Driver & Konfigurasi Device Manager

Folder ini berisi driver dan skrip konfigurasi agar Virtual COM Port STM32 USB-CDC terdeteksi sebagai **"DPLL USB Serial Port"** di Windows Device Manager (*Ports (COM & LPT)*).

---

## Isi Direktori

1. **`dpll_usb_serial.inf`**
   - File driver setup information resmi Windows.
   - Mengarahkan Hardware ID STM32 (`USB\VID_0483&PID_5740`) ke driver kernel bawaan Windows `usbser.sys`.
   - Mengubah nama perangkat menjadi **`DPLL USB Serial Port`**.

2. **`install_driver.bat`**
   - Skrip instalasi 1-klik dengan auto-elevasi hak Administrator (UAC).
   - Memasukkan INF ke Driver Store Windows (`pnputil /add-driver /install`) dan memperbarui Registry PnP.

3. **`set_friendly_name.ps1`**
   - Skrip PowerShell untuk mengubah nama FriendlyName secara langsung di Windows Registry.
   - Berguna untuk mengubah nama perangkat yang sedang tercolok tanpa perlu restart PC.

---

## Cara Instalasi / Penggunaan

### Cara 1: Menggunakan Installer Otomatis (Direkomendasikan)
1. Colokkan board STM32 ke port USB PC.
2. Klik kanan pada file **`install_driver.bat`**, pilih **Run as administrator** (atau cukup double-click, UAC akan meminta izin admin).
3. Skrip akan menginstal driver dan memperbarui nama perangkat.
4. Buka **Device Manager** -> **Ports (COM & LPT)** -> perangkat akan tampil sebagai:
   ```text
   DPLL USB Serial Port (COMx)
   ```

### Cara 2: Manual lewat Device Manager
1. Buka **Device Manager** (`devmgmt.msc`).
2. Cari perangkat di bawah **Ports (COM & LPT)** (misalnya saat ini bernama `USB Serial Device (COM19)`).
3. Klik kanan pada perangkat tersebut -> pilih **Update driver**.
4. Pilih **Browse my computer for drivers**.
5. Klik **Browse...** dan arahkan ke folder `driver` ini.
6. Klik **Next** dan ikuti petunjuk hingga selesai.

---

## Konfigurasi Firmware (`platformio.ini`)

Di file `platformio.ini`, pastikan flag descriptor USB berikut sudah aktif:
```ini
; Enable USB CDC (Virtual COM Port) support
-D PIO_FRAMEWORK_ARDUINO_ENABLE_CDC
-D USBCON
-D USBD_VID=0x0483
-D USBD_PID=0x5740
-D USB_MANUFACTURER="\"DPLL Project\""
-D USB_PRODUCT="\"DPLL USB Serial Port\""
-D USB_MANUFACTURER_STRING="\"DPLL Project\""
-D USB_PRODUCT_STRING="\"DPLL USB Serial Port\""
```
Setelah firmware di-flash ulang ke STM32, descriptor internal USB mikrokontroler akan mengirimkan string produk **"DPLL USB Serial Port"** saat negosiasi USB enumeration.
