# Dream Con brick station (browser version)

Setting up the Windows computers: see **SETUP.md**. The old TouchDesigner test is in `legacy/touchdesigner/` (not used).

```
RPLIDAR C1 ──USB──> server.py ──> Chrome full screen: /station   (The Frame)
                      │  saves data/messages.csv
                      └──network──> Chrome full screen: /wall     (3 × Uperfect, wall computer)
```

Only one extra package is needed: `pip install pyserial`. Optional: `pip install pyautogui`, so the station window takes the keyboard back if someone clicks on another screen (see *Keyboard focus*). Everything else, fonts included, is in this folder, so it works offline.

## Run it (Mac for now, Windows later: same commands)
```
cd ~/gitroot/dc-installation
python server.py
```
Then open **http://localhost:8080/station** in Chrome and make it full screen on The Frame. Stop the server with Ctrl+C.

Without the sensor: `python server.py --nolidar` and open **http://localhost:8080/station?debug=1**
- **F2** puts the brick on and takes it off
- **mouse click** on a topic button = a finger
- the green line at the bottom shows what the page is receiving

## The flow
1. **Invite.** Put a brick on the spot. The screen goes darker.
2. **Choose a topic.** One question: "คุณอยากเห็นการเปลี่ยนแปลงอะไรในเรื่อง ___ / What change would you like to see in ___?". Until someone taps, the underlined topic cycles through the 4 topics every 3.2 s and the matching button's outline lights up yellow. Tap a topic, then start typing on the keyboard.
3. **Write your message.** The same question stays at the top with the chosen topic. The answer always starts with a fixed "อยากเห็น", the same size as the question, with the cursor right after it; they type the rest (up to 80 characters), then Enter. The prefix is saved with the message and shown on the wall. Esc goes back to the topics.
4. **Embedding.** Direction B, drawn in, with no dashed outline.
5. **Thank you.** It asks them to lift the brick off. Once the brick is gone, it's back to step 1.

If the brick is taken away during steps 2–3, it goes back to the start. If nobody types for 90 s on the message screen, it goes back to the topics.

## Messages
- **CSV:** `data/messages.csv` has time, topic and message, and opens in Excel with Thai intact. Each message is written the moment Enter is pressed.
- **Backups:** an hourly copy goes into `data/backups/`.
- **Server down:** if the station page can't reach the server, the message waits in the browser and is sent when the server is back.

## Wall screens (the second computer)
Open `http://<station-computer-address>:8080/wall?screen=1`, then `screen=2` and `screen=3`, one per monitor, full screen. Each one starts at a different point in the list. A new message appears next with a warm glow. If the network drops, each screen keeps scrolling what it already has.

On a Mac, find the address in System Settings › Network. With your router, set a fixed (reserved) address for the station computer.

## Calibration (only if the sensor or the screen moves)
1. Stop `server.py`, then start it again without the sensor: `python server.py --nolidar`.
2. Open `http://localhost:8080/station?calibrate=1` full screen on The Frame.
3. In a second Terminal window, run `python lidar.py --calibrate`, and touch targets 1–6 in order. Keep the brick off.
4. Stop the `--nolidar` server and start `python server.py` normally.

`python lidar.py --raw` shows what the sensor sees.

## Brick colour (camera)
The OBSBOT Meet 2 looks at the side of the brick from table height. When the LiDAR sees a brick, the station waits 0.25 s, averages the camera for 0.7 s while the screen is still neutral, and matches the colour to the calibration. The rectangle under the brick and the running ring then take the brick's colour, the message card in the animation is that colour, and the wall shows the brick in that colour with its generation:

| colour | id | generation |
|---|---|---|
| reddish | `red` | Gen Alpha · Gen Z |
| orange-ish (beige) | `orange` | Gen Y (Millennials) |
| greenish | `green` | Gen X · Boomers |

Names, generations and the on-screen colours are in `web/bricks.json`. If the match isn't sure it tries twice more, then stays neutral (white ring, default brick on the wall).

**Lock the camera first** (OBSBOT Center app): exposure manual, white balance manual (e.g. 5000 K), HDR off, auto-framing / tracking off, zoom fixed. Restart the computer once and check the settings are still there.

**Calibrate** (again whenever the camera, the lights or the screen brightness change):
1. Run `python server.py` and open `http://localhost:8080/station?colorcal=1`. Allow the camera (Chrome remembers it for this address).
2. Pick the OBSBOT in the list if it isn't already.
3. Drag a box over the **upper** part of the brick's side; the bottom edge picks up the screen's light.
4. Put a reddish brick down and press **1**. Turn it, swap in another reddish brick, press 1 again: 3–5 samples. Then orange-ish with **2**, greenish with **3**.
5. Put bricks down one at a time and check the live guess. The numbers are distances: smaller is closer, above 32 counts as no match. If two colours are too close, add more samples of both.

It saves to `color_cal.json` next to `server.py`. In `?debug=1`, **F3** cycles through the colours without a camera, and the green line shows the colour and the distances.

## Keyboard focus
Typing only reaches the station if its Chrome window has the keyboard. If someone clicks on another screen, the station page tells `server.py`, which clicks once on the middle of the station window to take the keyboard back (needs `pip install pyautogui`; on a Mac, allow Terminal under System Settings › Privacy & Security › Accessibility). If the click lands in the wrong place, set `"refocus_xy": [x, y]` in `lidar_config.json`. Turn it off with `python server.py --norefocus`. With `?debug=1`, a yellow note top right shows when the window has lost the keyboard.

Best fix for the show: run only the station window on the master computer (the wall screens on the other computer), in Chrome kiosk mode, with no mouse plugged in.

## Settings
- **Topics and questions:** `web/topics.json`
- **Sensor, screen size, brick detection:** `lidar_config.json`
- **Screen colour and the darkening:** `BG_BASE` / `DARKEN` near the top of the script in `web/station.html`
