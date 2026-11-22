# LiDAR touch table (v2): 4 buttons + brick detection

Samsung The Frame 32" lying flat, RPLIDAR C1 at the bottom-centre, laser plane a few mm above the glass.

```
RPLIDAR C1 --USB--> lidar_bridge.py --OSC :7000--> TouchDesigner (lidar_touch_test)
```

## Screen size
The Frame 32" has a 31.5" picture (16:9), which is about **697 × 392 mm**. The 727.3 × 426.8 mm figure is the outside of the frame. Measure the lit picture once, and if it's different, change `screen_w_mm` / `screen_h_mm` in `lidar_config.json`.

## 1. TouchDesigner
Paste `build_touch_test.py` into your Text DAT, replacing the old text, and choose Run Script. It replaces the old component. Then open `window` full screen on The Frame.

No sensor? Run `python lidar_bridge.py --sim` to see taps and the brick switch on and off.

## 2. Calibrate (each time the sensor or screen moves)
```
python lidar_bridge.py --calibrate
```
- Keep the brick **off** the screen, and keep people away from the table edge for the first few seconds.
- Touch the middle of each button from left to right (4 touches), one fingertip, and press Enter after each.
- It works out the sensor position and angle, and prints how far off each button is. Under about 15 mm is good.

## 3. Run
```
python lidar_bridge.py --monitor
```
Touching a button selects it. Putting the brick in the middle shows "Brick detected" after about 0.6 s, and taking it away switches back after about 1 s.

## Tuning (`lidar_config.json`)
| key | what it does |
|---|---|
| `brick_zone` | where the brick counts, as u0 v0 u1 v1 (0–1 of the picture, v=0 at the sensor side) |
| `brick_min_mm` | how wide the brick face the sensor sees must be (default 50 mm) |
| `finger_max_mm` | anything wider than this is not a finger (so hands, arms and the brick never press buttons) |
| `brick_on_s` / `brick_off_s` | how long before "present" / "absent" switches |

Diagnostics: `python lidar_bridge.py --raw` shows every object the sensor sees, with its width.
