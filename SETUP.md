# Setting up the two Windows computers

There are two computers:

| | Master (station) computer | Client (wall) computer |
|---|---|---|
| Screens | The Frame 32" (lying flat) | 3 × Uperfect 15.6", portrait |
| Plugged in | RPLIDAR C1 (USB), keyboard | nothing else |
| Runs | `server.py` + Chrome showing `/station` | 3 Chrome windows showing `/wall` |
| Needs this repo | **yes** | no, Chrome only (a copy doesn't hurt) |

The wall screens load their pages from the master computer over the network, so the client computer doesn't run any of this code. It only needs Chrome and the master's address.

Commands go in **PowerShell**. Where a step says *(admin)*, right-click PowerShell and choose **Run as administrator**.

---

## 0. Network (do this first)

1. Connect both computers to your router, preferably by cable.
2. On the master, run `ipconfig` and note the **IPv4 Address**, e.g. `192.168.1.10`.
3. In the router's settings, reserve that address for the master. This is usually called "DHCP reservation" or "static lease". The address then never changes.

This guide uses `192.168.1.10` below. Swap in your own.

---

## 1. Master (station) computer

### 1.1 Install the software
```powershell
winget install -e --id Python.Python.3.12
winget install -e --id Git.Git
winget install -e --id Google.Chrome
```
Close and reopen PowerShell, then:
```powershell
py -m pip install pyserial
```

### 1.2 Get the project
```powershell
cd $HOME
git clone https://github.com/hellonun/dc-installation.git
cd dc-installation
```
To get updates later, run `git pull` inside that folder.

### 1.3 Thai keyboard
Settings › Time & language › Language & region › **Add a language** › Thai (ไทย). Switch between English and Thai with **Win + Space**.

### 1.4 Check the LiDAR
Plug the C1 in through its adapter board, then:
```powershell
py lidar.py --list
py lidar.py --info
```
`--list` should show a **CP210x** port such as `COM3`. If no port appears, install the **CP210x Universal Windows Driver** from silabs.com, then unplug the C1 and plug it back in. `--info` should end with `health: Good`.

### 1.5 Let the wall computer connect *(admin)*
```powershell
netsh advfirewall firewall add rule name="DreamCon station 8080" dir=in action=allow protocol=TCP localport=8080
```
The first time `server.py` runs, Windows may ask about Python and networks. Allow **Private networks**.

### 1.6 Never sleep, never cut the USB *(admin)*
```powershell
powercfg /change standby-timeout-ac 0
powercfg /change hibernate-timeout-ac 0
powercfg /change monitor-timeout-ac 0
powercfg /setacvalueindex SCHEME_CURRENT 2a737441-1930-4402-8d77-b2bebba308a3 48e6b7a6-50f5-4782-a5d4-53bb8f07e226 0
powercfg /setactive SCHEME_CURRENT
```
The fourth line turns off "USB selective suspend", so Windows never cuts power to the LiDAR.

Also turn these off in Settings:
- **Notifications:** Settings › System › Notifications.
- **Updates during the show:** Settings › Windows Update › **Pause updates**.

### 1.7 Screen
Settings › System › Display: The Frame at **1920 × 1080**, **Scale 100%**. Turn the TV's own picture processing off too (Local Dimming, Contrast Enhancer, Energy Saving), as covered earlier, to avoid the lighter strip during the animation.

### 1.8 Calibrate the LiDAR (once on site, again if the sensor or screen moves)
1. Start the server without the sensor:
   ```powershell
   py server.py --nolidar
   ```
2. In Chrome, open `http://localhost:8080/station?calibrate=1` and press **F11** for full screen on The Frame.
3. In a **second** PowerShell window, in the same folder, run:
   ```powershell
   py lidar.py --calibrate
   ```
   Keep the brick off and touch targets 1–6 in order. It saves the calibration to `lidar_config.json`.
4. Stop the server with **Ctrl+C**.

### 1.9 Start the station (every day)
PowerShell window 1:
```powershell
cd $HOME\dc-installation
py server.py
```
Leave it open. It prints each saved message.

PowerShell window 2 opens Chrome full screen on The Frame:
```powershell
& "C:\Program Files\Google\Chrome\Application\chrome.exe" --kiosk --app=http://localhost:8080/station --user-data-dir="$env:LOCALAPPDATA\dc-station" --no-first-run --disable-session-crashed-bubble --disable-features=Translate --noerrdialogs
```
- **Exit:** Alt+F4 closes the full-screen window.
- **The Frame isn't the main display:** add `--window-position=X,0`, where X is the left edge of The Frame in Display settings (e.g. `1920`).
- **No sensor:** use `py server.py --nolidar` and open `/station?debug=1` instead. F2 toggles the brick and the mouse acts as a finger.

### 1.10 Messages
- **CSV:** `data\messages.csv` opens in Excel with Thai intact.
- **Backups:** an hourly copy goes into `data\backups\`.
- **Git:** neither file is in git. Copy them off at the end of each day, e.g. to a USB stick.

---

## 2. Client (wall) computer (3 screens)

### 2.1 Install Chrome
```powershell
winget install -e --id Google.Chrome
```

### 2.2 Screens
Settings › System › Display:
1. Click **Identify** and note which number is which monitor.
2. For each Uperfect, set **Display orientation: Portrait**, **1080 × 1920**, **Scale 100%**.
3. Drag them side by side in the order they stand, left to right, top edges aligned.
4. Note each one's left edge. With three portrait screens side by side, that's **0**, **1080** and **2160**. If a laptop screen sits on the left, add its width.

Then apply the same power settings as the master, section 1.6, without the USB line.

### 2.3 Check the connection
With `server.py` running on the master, open this in Chrome:
`http://192.168.1.10:8080/wall`
You should see the scrolling bricks. If you don't, check the master's firewall rule (1.5) and that both computers are on the same router.

### 2.4 Start the 3 screens (every day)
Change `192.168.1.10` and the `--window-position` values to match yours. Run one line per screen:
```powershell
$chrome = "C:\Program Files\Google\Chrome\Application\chrome.exe"
$flags  = "--kiosk --no-first-run --disable-session-crashed-bubble --disable-features=Translate --noerrdialogs"

Start-Process $chrome "--app=http://192.168.1.10:8080/wall?screen=1 --window-position=0,0    --user-data-dir=`"$env:LOCALAPPDATA\dc-wall1`" $flags"
Start-Process $chrome "--app=http://192.168.1.10:8080/wall?screen=2 --window-position=1080,0 --user-data-dir=`"$env:LOCALAPPDATA\dc-wall2`" $flags"
Start-Process $chrome "--app=http://192.168.1.10:8080/wall?screen=3 --window-position=2160,0 --user-data-dir=`"$env:LOCALAPPDATA\dc-wall3`" $flags"
```
- **Separate profiles:** each window has its own `--user-data-dir`. That's what lets three full-screen Chrome windows run at once.
- **Different starting points:** `screen=1/2/3` starts each screen somewhere else in the list, so they don't look identical.
- **Exit:** click a screen and press Alt+F4.

**Start the master first.** If the client starts before the master is up, Chrome shows an error page. Press **F5** on that screen once the master is running. The start scripts we'll add later will do this automatically.

---

## Daily checklist
1. Turn on the master and run `py server.py`, then the Chrome line in 1.9.
2. Turn on the client and run the three lines in 2.4.
3. Place a brick, pick a topic, type, Send. Check that the message appears on the wall screens.
4. At the end of the day, copy `data\messages.csv` off the master.
