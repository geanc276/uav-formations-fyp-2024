# Check if running as Administrator
if (-not ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole] "Administrator")) {
    Write-Warning "This script must be run as Administrator. Exiting."
    exit
}

# ---------------------------
# 2. Enable WSL and Virtual Machine Platform if needed
# ---------------------------
Write-Output "Checking WSL and Virtual Machine Platform features..."
$wslFeature = Get-WindowsOptionalFeature -Online -FeatureName Microsoft-Windows-Subsystem-Linux
if ($wslFeature.State -ne 'Enabled') {
    Write-Output "Enabling WSL feature..."
    dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
    $needsReboot = $true
} else {
    Write-Output "WSL feature already enabled."
}
$vmFeature = Get-WindowsOptionalFeature -Online -FeatureName VirtualMachinePlatform
if ($vmFeature.State -ne 'Enabled') {
    Write-Output "Enabling Virtual Machine Platform feature..."
    dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
    $needsReboot = $true
} else {
    Write-Output "Virtual Machine Platform already enabled."
}
if ($needsReboot) {
    Write-Output "System needs reboot to complete feature enablement. Please reboot and re-run this script."
    exit
}

# ---------------------------
# 3. Ensure at least one WSL distro is installed
# ---------------------------
Write-Output "Checking for any WSL distro..."
$entries = @(wsl.exe --list --quiet 2>$null | Where-Object { $_ -and $_.Trim() -ne '' })
Write-Output "WSL distros found: $($entries -join ', ')"
if ($entries.Count -gt 0) {
    Write-Output "WSL distro(s) detected ($($entries.Count)): skipping installation."
} else {
    Write-Output "No WSL distros found. Installing Ubuntu for WSL..."
    wsl --install -d Ubuntu-22.04
    Write-Output "Ubuntu installation triggered. Please reboot or wait for completion, then re-run this script."
    exit
}

# ---------------------------
# 3.5 Ensure Ubuntu WSL initial setup is complete
# ---------------------------
Write-Output "Verifying Ubuntu WSL initialization..."
$initResult = wsl -d Ubuntu-22.04 -- bash -ic "lsb_release -cs" 2>$null
if (-not $initResult) {
    Write-Output "Ubuntu WSL has not been initialized or network is unavailable."
    Write-Output "Please launch Ubuntu from the Start menu at least once, complete any first-run setup, and ensure you can run 'lsb_release -cs' and 'apt-get update'."
    Write-Output "Then re-run this script."
    exit
} else {
    Write-Output "Ubuntu initialization verified: release='$initResult'"
}

# ---------------------------
# 4. Configure WSL to mount Windows drives with correct permissions
# ---------------------------
Write-Output "Configuring WSL automount for Windows drives..."
wsl --shutdown
wsl -d Ubuntu-22.04 -- bash -ic @'
sudo tee /etc/wsl.conf <<'EOF'
[automount]
enabled = true
root = /mnt/
options = "metadata,umask=22,fmask=11"
EOF
'@
Write-Output "WSL automount configuration applied. Please restart WSL or your machine."

Write-Output "All installations have been initiated. Please review any output messages."