# Dream Con brick station (browser version)

Setting up the Windows computers: see **SETUP.md**. The old TouchDesigner test is in `legacy/touchdesigner/` (not used).

```
RPLIDAR C1 ──USB──> server.py ──> Chrome full screen: /station   (The Frame)
                      │  saves data/messages.csv
                      └──network──> Chrome full screen: /wall     (3 × Uperfect, wall computer)
```

Only one extra package is needed: `pip install pyserial`. Everything else, fonts included, is in this folder, so it works offline.

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
2. **Choose a topic.** Until someone taps, the heading cycles through the 4 topics every 3.2 s ("อยากเสนออะไรในเรื่อง … / What would you propose for …?"), each with a preview of the exact question asked next, and the matching button's outline lights up yellow. Tap a topic, then start typing on the keyboard.
3. **Write your message.** Up to 80 characters, then Enter. Esc goes back to the topics.
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

## Settings
- **Topics and questions:** `web/topics.json`
- **Sensor, screen size, brick detection:** `lidar_config.json`
- **Screen colour and the darkening:** `BG_BASE` / `DARKEN` near the top of the script in `web/station.html`
