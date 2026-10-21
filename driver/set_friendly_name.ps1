#==============================================================================
# DPLL Ultrasonic Frequency Tracking - Fast Device Manager Renamer
# Sets FriendlyName to "DPLL USB Serial Port (COMx)" in Windows Registry
# Must be run as Administrator
#==============================================================================

$ErrorActionPreference = "Stop"

# Ensure Administrator privileges
$currentPrincipal = New-Object Security.Principal.WindowsPrincipal([Security.Principal.WindowsIdentity]::GetCurrent())
if (-not $currentPrincipal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    Write-Host "[!] Relaunching with Administrator privileges..." -ForegroundColor Yellow
    Start-Process powershell.exe -ArgumentList ("-NoProfile -ExecutionPolicy Bypass -File `"$PSCommandPath`"") -Verb RunAs
    exit
}

Write-Host "===========================================================" -ForegroundColor Cyan
Write-Host " DPLL USB Serial Port - Windows Device Manager Name Setup " -ForegroundColor Cyan
Write-Host "===========================================================" -ForegroundColor Cyan
Write-Host ""

# Find STM32 USB CDC device (VID 0483 & PID 5740)
$devices = Get-PnpDevice | Where-Object { 
    ($_.InstanceId -like "*VID_0483&PID_5740*" -or $_.HardwareID -contains "USB\VID_0483&PID_5740") -and 
    ($_.Class -eq "Ports" -or $_.FriendlyName -like "*COM*") 
}

if (-not $devices) {
    Write-Host "[?] Mencari semua perangkat USB Serial Device di kategori Ports..." -ForegroundColor Yellow
    $devices = Get-PnpDevice | Where-Object { $_.Class -eq "Ports" -and $_.FriendlyName -like "*USB Serial Device*" }
}

if (-not $devices) {
    Write-Host "[-] Perangkat STM32 USB CDC tidak ditemukan dalam keadaan terhubung." -ForegroundColor Red
    Write-Host "    Pastikan board STM32 sudah dicolok via kabel USB." -ForegroundColor Red
    pause
    exit
}

foreach ($dev in $devices) {
    $currentName = $dev.FriendlyName
    # Extract COM port number like (COM19)
    if ($currentName -match "\(COM\d+\)") {
        $comTag = $matches[0]
    } else {
        $comTag = ""
    }

    $newName = "DPLL USB Serial Port $comTag".Trim()
    $regPath = "HKLM:\SYSTEM\CurrentControlSet\Enum\$($dev.InstanceId)"

    Write-Host "[*] Instance ID : $($dev.InstanceId)" -ForegroundColor Gray
    Write-Host "[*] Nama Lama   : $currentName" -ForegroundColor Yellow
    Write-Host "[*] Nama Baru   : $newName" -ForegroundColor Green

    try {
        Set-ItemProperty -Path $regPath -Name "FriendlyName" -Value $newName -Force
        Write-Host "[+] Berhasil memperbarui Registry Windows!" -ForegroundColor Green
    } catch {
        Write-Host "[-] Gagal mengubah Registry: $_" -ForegroundColor Red
    }
    Write-Host ""
}

Write-Host "===========================================================" -ForegroundColor Cyan
Write-Host "[V] Selesai! Buka Device Manager untuk melihat perubahan." -ForegroundColor Cyan
Write-Host "    Jika belum berubah, cabut dan colokkan kembali kabel USB." -ForegroundColor Cyan
Write-Host "===========================================================" -ForegroundColor Cyan
Write-Host ""
pause
