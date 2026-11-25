# Test lenh lai #vcb tu PC xuong Jetson, KHONG can main_master.py, ban dap hay vo lang.
#
# Script dong vai server cua main_master.py: mo TCP cong 6000, cho braincar tren Jetson ket noi, roi gui
#   #vcb:<speed>;<steer>;<dir>;;\r\n      (dung dinh dang JetsonSender.send() trong main_master.py; Jetson chi nhan goi bat dau bang '#')
# Dong thoi nghe UDP cong 9000 (kUdpTextPort): moi lenh Jetson nhan duoc, ThreadUdpServer bao nguoc ve PC
#   @ref:<speed>;;   @steer:<steer>;;     -> thay 2 dong nay khop voi lenh gui = Jetson da nhan dung.
# (@speed = toc do do tu Nucleo, @imu = IMU: co khi Nucleo dang noi serial.)
#
# Truoc khi chay:
#   - TAT main_master.py (trung cong 6000).
#   - IP may PC phai bang kPcIp trong jetson/main.cpp, hoac chay tren Jetson: ./braincar <ip PC> <ip PC>
#   - Jetson o state DEFAULT: chi nhan lenh, banh KHONG quay (an toan). Bam 3 (MANUAL) tren Jetson thi banh moi quay
#     theo lenh: ke xe len cho banh khong cham dat.
#
# Chay (PowerShell, tai thu muc repo):
#   powershell -ExecutionPolicy Bypass -File .\test_vcb.ps1                          # 0.3 m/s, lai 10 do, tien, 5 s
#   powershell -ExecutionPolicy Bypass -File .\test_vcb.ps1 -Speed 0.5 -Steer -15 -Dir 1 -Seconds 10
#   powershell -ExecutionPolicy Bypass -File .\test_vcb.ps1 -Seconds 0              # gui mai, Ctrl+C de dung
# Ket thuc (het gio hoac Ctrl+C) luon gui lenh dung #vcb:0.000;0.000;0;;

param(
    [double]$Speed    = 0.3,    # m/s (main_master: 0 .. 1.3 tien, 0 .. 0.8 lui)
    [double]$Steer    = 10.0,   # do, -20 .. 20 (STEER_MIN/MAX cua main_4.py)
    [int]   $Dir      = 1,      # 1 tien, -1 lui, 0 N
    [double]$Seconds  = 5,      # thoi gian gui lenh; 0 = gui mai toi khi Ctrl+C
    [int]   $Hz       = 20,     # tan so gui; phai > 4 Hz vi Jetson tu dung xe neu 300 ms khong co lenh
    [int]   $TcpPort  = 6000,   # --tcp-port cua main_master.py / kTcpServerPort
    [int]   $TextPort = 9000    # kUdpTextPort trong jetson/main.cpp
)

$ErrorActionPreference = 'Stop'
# So phai viet dau cham: may Windows tieng Viet mac dinh viet "0,300" va Jetson se bo ca goi
$inv = [System.Globalization.CultureInfo]::InvariantCulture

function Format-Vcb([double]$s, [double]$st, [int]$d) {
    return [string]::Format($inv, "#vcb:{0:F3};{1:F3};{2};;`r`n", $s, $st, $d)
}

# Doc het goi UDP Jetson bao ve, giu gia tri moi nhat cua tung loai (@ref, @steer, @speed, @imu)
function Read-Echo($udp, $latest) {
    $ep = New-Object System.Net.IPEndPoint([System.Net.IPAddress]::Any, 0)
    while ($udp.Available -gt 0) {
        $text = [System.Text.Encoding]::ASCII.GetString($udp.Receive([ref]$ep)).Trim()
        if ($text -match '^@(\w+):(.*?);*$') {
            $latest[$Matches[1]] = $Matches[2]
            $latest['_count'] = 1 + [int]$latest['_count']
        }
    }
}

function Format-Echo($latest) {
    if (-not $latest['_count']) { return 'Jetson chua bao gi ve UDP ' + $TextPort }
    $parts = foreach ($k in 'ref', 'steer', 'speed', 'imu') { if ($latest.ContainsKey($k)) { "@${k}:" + $latest[$k] } }
    return 'Jetson bao: ' + ($parts -join '  ') + "  ($($latest['_count']) goi)"
}

$listener = $null; $client = $null; $udp = $null
try {
    $udp = New-Object System.Net.Sockets.UdpClient($TextPort)
    $listener = New-Object System.Net.Sockets.TcpListener([System.Net.IPAddress]::Any, $TcpPort)
    $listener.Start()

    $ips = (Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue |
            Where-Object { $_.IPAddress -notlike '127.*' -and $_.IPAddress -notlike '169.254.*' }).IPAddress -join ', '
    Write-Host "IP may nay: $ips"
    Write-Host "Cho Jetson ket noi TCP cong $TcpPort ... (Ctrl+C de thoat)"
    $waitSw = [System.Diagnostics.Stopwatch]::StartNew()
    while (-not $listener.Pending()) {
        Start-Sleep -Milliseconds 100
        if ($waitSw.Elapsed.TotalSeconds -ge 10) {
            Write-Host '  van cho... Jetson dang chay braincar chua? kPcIp co dung IP may nay khong? Firewall co chan khong?'
            $waitSw.Restart()
        }
    }
    $client = $listener.AcceptTcpClient()
    $client.NoDelay = $true
    $stream = $client.GetStream()
    Write-Host "Jetson da ket noi tu $($client.Client.RemoteEndPoint)"

    $cmd = Format-Vcb $Speed $Steer $Dir
    $bytes = [System.Text.Encoding]::ASCII.GetBytes($cmd)
    Write-Host ("Gui {0} lan/s: {1}" -f $Hz, $cmd.Trim())

    $latest = @{}
    $period = 1.0 / $Hz
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $nextSend = 0.0; $nextPrint = 0.5; $sent = 0
    while ($Seconds -le 0 -or $sw.Elapsed.TotalSeconds -lt $Seconds) {
        $t = $sw.Elapsed.TotalSeconds
        if ($t -ge $nextSend) {
            $stream.Write($bytes, 0, $bytes.Length)   # Jetson ngat ket noi -> loi -> xuong finally
            $sent++
            $nextSend += $period
        }
        Read-Echo $udp $latest
        if ($t -ge $nextPrint) {
            Write-Host ("[{0,5:F1} s] da gui {1,4} lenh | {2}" -f $t, $sent, (Format-Echo $latest))
            $nextPrint += 0.5
        }
        Start-Sleep -Milliseconds 5
    }
}
finally {
    if ($client -and $client.Connected) {
        try {
            $stop = [System.Text.Encoding]::ASCII.GetBytes((Format-Vcb 0 0 0))
            $client.GetStream().Write($stop, 0, $stop.Length)
            Write-Host 'Da gui lenh dung: #vcb:0.000;0.000;0;;'
            Start-Sleep -Milliseconds 300
            if ($udp) { $end = @{}; Read-Echo $udp $end; Write-Host (Format-Echo $end) }
        } catch { Write-Host 'Khong gui duoc lenh dung (Jetson da ngat). Jetson tu dung xe sau 300 ms khong co lenh.' }
    }
    if ($client)   { $client.Close() }
    if ($listener) { $listener.Stop() }
    if ($udp)      { $udp.Close() }
}
