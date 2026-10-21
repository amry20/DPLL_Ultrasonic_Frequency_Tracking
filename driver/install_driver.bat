@echo off
setlocal enabledelayedexpansion

:: ==============================================================================
:: DPLL Ultrasonic Frequency Tracking - USB Driver Installer
:: ==============================================================================

:: Check for Administrator privileges
net session >nul 2>&1
if %errorlevel% neq 0 (
    echo [!] Memerlukan hak akses Administrator.
    echo [!] Membuka prompt UAC...
    powershell -Command "Start-Process cmd -ArgumentList '/c \"\"%~f0\"\"' -Verb RunAs"
    exit /b
)

title DPLL USB Serial Driver Installer
color 0B
echo ==============================================================================
echo        DPLL Ultrasonic Frequency Tracking - USB Driver Setup
echo ==============================================================================
echo.

set "INF_PATH=%~dp0dpll_usb_serial.inf"

if not exist "%INF_PATH%" (
    color 0C
    echo [-] Error: File dpll_usb_serial.inf tidak ditemukan di direktori:
    echo     %~dp0
    echo.
    pause
    exit /b 1
)

echo [*] Menambahkan driver ke Windows Driver Store...
echo     File INF: "%INF_PATH%"
echo.

pnputil /add-driver "%INF_PATH%" /install

if %errorlevel% equ 0 (
    echo.
    color 0A
    echo [V] Driver INF berhasil di-install ke sistem!
) else (
    echo.
    echo [!] Pnputil selesai dengan kode: %errorlevel%
    echo     Catatan: Jika Windows 10/11 memblokir driver unsigned, Anda juga dapat
    echo     menggunakan skrip rename instan Registry.
)

echo.
echo [*] Menjalankan sinkronisasi FriendlyName perangkat...
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0set_friendly_name.ps1"

echo.
echo ==============================================================================
echo Selesai. Silakan periksa Device Manager pada kategori "Ports (COM & LPT)".
echo ==============================================================================
echo.
pause
