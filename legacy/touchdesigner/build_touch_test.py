# =============================================================================
#  RPLIDAR C1 touch test  -  TouchDesigner network builder (v2: 4 buttons + brick)
# =============================================================================
#  1. In TouchDesigner, create a Text DAT (e.g. inside /project1).
#  2. Paste this whole file into it.
#  3. Right-click the DAT > Run Script   (or select it and press Ctrl+R / Cmd+R).
#
#  It builds a component called "lidar_touch_test" next to the Text DAT:
#    - a 1280x720 UI (Non-Commercial limit) styled like the reference screen
#    - 4 buttons, one selected at a time: tap to select, tap again to switch off
#    - OSC In DAT on port 7000 listening to lidar_bridge.py
#    - mouse clicks work too, so you can test without the sensor
#    - orange dots show where the LiDAR sees fingers (debug)
#    - a Window COMP that shows the UI stretched to the 32" screen
#
#  Running it again deletes and rebuilds the component.
# =============================================================================

import sys

W, H = 1280, 720                 # render resolution (Non-Commercial max 1280x1280)
DISPLAY_W, DISPLAY_H = 1920, 1080  # window size on the 32" screen
OSC_PORT = 7000

# Font with Thai glyphs. Swap for e.g. 'IBM Plex Sans Thai' or 'Anuphan' if installed.
FONT = 'Sukhumvit Set' if sys.platform == 'darwin' else 'Leelawadee UI'

TOPICS = [  # (Thai, English label, english word for the title)
    ('การศึกษา',  'Education',   'education'),
    ('เกษตรกรรม', 'Agriculture', 'agriculture'),
    ('กฎหมาย',   'Law',         'law'),
    ('เศรษฐกิจ',  'Economy',     'economy'),
]   # 4 buttons. Edit the labels freely; keep 4 entries to match lidar_bridge.py

# Layout in pixels, origin top-left of the 1280x720 frame (traced from the reference).
BTN_H, BTN_GAP, BTN_MARGIN, BTN_CY = 82, 16, 58, 632
BTN_W = (W - 2 * BTN_MARGIN - (len(TOPICS) - 1) * BTN_GAP) / len(TOPICS)
GREY  = (0.123, 0.131, 0.147)   # background
BRICK = (0.604, 0.369, 0.341)
WHITE = (1.0, 1.0, 1.0)
DARK  = (0.20, 0.20, 0.22)
DOT   = (1.0, 0.55, 0.10)

warnings = []


# --------------------------------------------------------------------------- helpers
def setp(o, name, value):
    p = getattr(o.par, name, None)
    if p is None:
        warnings.append('missing par %s.%s' % (o.name, name))
        return
    try:
        p.val = value
    except Exception as e:
        warnings.append('%s.%s: %s' % (o.name, name, e))


def set_pixels(o, name):
    """Unit menus: first entry is 'Pixels'."""
    p = getattr(o.par, name, None)
    if p is None:
        warnings.append('missing par %s.%s' % (o.name, name))
        return
    try:
        p.menuIndex = 0
    except Exception as e:
        warnings.append('%s.%s: %s' % (o.name, name, e))


def set_rgb(o, prefix, rgb):
    for c, v in zip('rgb', rgb):
        setp(o, prefix + c, v)


host = me.parent()
old = host.op('lidar_touch_test')
if old:
    old.destroy()
root = host.create(baseCOMP, 'lidar_touch_test')
root.nodeX, root.nodeY = me.nodeX + 300, me.nodeY

chain = []          # TOP chain, each one composites over the previous
col = {'x': 0}


def place(o, row=0):
    o.nodeX = col['x'] * 160
    o.nodeY = -row * 140
    col['x'] += 1


def add(o):
    if chain:
        o.inputConnectors[0].connect(chain[-1])
        setp(o, 'compoverinput', True)
    chain.append(o)
    place(o)
    return o


def rect(name, cx, cy, w, h, fill=WHITE, fill_a=1.0, border=WHITE, border_a=0.0, bw=0, radius=0):
    o = root.create(rectangleTOP, name)
    for u in ('sizeunit', 'centerunit', 'borderwidthunit', 'cornerradiusunit'):
        set_pixels(o, u)
    setp(o, 'sizex', w); setp(o, 'sizey', h)
    setp(o, 'centerx', cx - W / 2); setp(o, 'centery', H / 2 - cy)
    set_rgb(o, 'fillcolor', fill); setp(o, 'fillalpha', fill_a)
    set_rgb(o, 'border', border); setp(o, 'borderalpha', border_a)
    setp(o, 'borderwidth', bw)
    setp(o, 'cornerradius', radius)
    setp(o, 'bgalpha', 0)
    return add(o)


def text(name, s, x, y, size, color=WHITE, bold=False, alignx='center', alpha=1.0):
    """x,y = anchor in top-left pixel coords. alignx 'left' -> x is the left edge."""
    o = root.create(textTOP, name)
    setp(o, 'text', s)
    setp(o, 'font', FONT)
    setp(o, 'bold', bold)
    setp(o, 'fontautosize', 'nofit')
    set_pixels(o, 'fontsizexunit'); set_pixels(o, 'fontsizeyunit')
    setp(o, 'fontsizex', size); setp(o, 'fontsizey', size)
    setp(o, 'alignx', alignx); setp(o, 'aligny', 'center')
    set_pixels(o, 'positionunit')
    setp(o, 'position1', x if alignx == 'left' else x - W / 2)
    setp(o, 'position2', H / 2 - y)
    set_rgb(o, 'fontcolor', color); setp(o, 'fontalpha', alpha)
    setp(o, 'bgalpha', 0)
    return add(o)


# --------------------------------------------------------------------------- picture
bg = root.create(constantTOP, 'bg')
setp(bg, 'outputresolution', 'custom')
setp(bg, 'resolutionw', W); setp(bg, 'resolutionh', H)
set_rgb(bg, 'color', GREY); setp(bg, 'alpha', 1)
add(bg)

text('hdr_th', 'รัฐธรรมนูญในฝันของคุณ', 58, 61, 19, bold=True, alignx='left')
text('hdr_en', 'YOUR DREAM CONSTITUTION', 58, 93, 13, alignx='left', alpha=0.85)

text('title_th', 'คุณอยากเห็นอะไรใน ' + TOPICS[0][0], 640, 200, 50, bold=True)
text('title_en', 'What do you wish for in  %s ?' % TOPICS[0][2], 640, 266, 27)

# Brick spot: outline + faint red placeholder when empty, bright outline when the brick is on it
# Drawn at real size: brick footprint 215 x 75 mm on a 697.3 mm wide picture.
SCREEN_W_MM = 697.3                    # visible picture width (same as lidar_config.json)
BRICK_MM = (215.0, 75.0)               # footprint on the glass (20 mm tall)
BRICK_MARGIN_MM = 8.0                  # outline drawn this far outside the brick
PX_PER_MM = W / SCREEN_W_MM
BRICK_CX, BRICK_CY = 640, 388
bw_px, bh_px = BRICK_MM[0] * PX_PER_MM, BRICK_MM[1] * PX_PER_MM
m_px = BRICK_MARGIN_MM * PX_PER_MM
rect('brick_frame', BRICK_CX, BRICK_CY, bw_px + 2 * m_px, bh_px + 2 * m_px,
     fill=WHITE, fill_a=0.0, border_a=0.8, bw=2, radius=4)
rect('brick', BRICK_CX, BRICK_CY, bw_px, bh_px, fill=BRICK, fill_a=0.35)
text('brick_label', 'วางอิฐตรงนี้ · Place the brick here',
     BRICK_CX, BRICK_CY + bh_px / 2 + m_px + 24, 16, alpha=0.9)

# Outline animation: a glowing segment that runs around the brick outline when it's detected
TRAIL_N, TRAIL_DOT, TRAIL_STEP, TRAIL_LOOP_S = 30, 8, 5, 1.8   # dots, dot px, px between dots, s per lap
for k in range(TRAIL_N):
    rect('trail%d' % k, 0, 0, TRAIL_DOT, TRAIL_DOT, fill=WHITE, fill_a=0.0, radius=TRAIL_DOT / 2)
OUTLINE = (BRICK_CX - W / 2, H / 2 - BRICK_CY, bw_px + 2 * m_px, bh_px + 2 * m_px)  # centre x,y (TD coords), w, h

text('choose', 'เลือกหัวข้อ - Choose a topic', 640, 560, 16, alpha=0.9)

BTN_UV = []   # hit boxes in u,v (0..1, v up) for the logic
for i, (th, en, _) in enumerate(TOPICS):
    cx = BTN_MARGIN + BTN_W / 2 + i * (BTN_W + BTN_GAP)
    rect('btn%d_bg' % i, cx, BTN_CY, BTN_W, BTN_H, fill=WHITE, fill_a=0.0,
         border_a=0.85, bw=2, radius=14)
    text('btn%d_th' % i, th, cx, BTN_CY - 11, 23, bold=True)
    text('btn%d_en' % i, en, cx, BTN_CY + 18, 14)
    BTN_UV.append(((cx - BTN_W / 2) / W, 1 - (BTN_CY + BTN_H / 2) / H,
                   (cx + BTN_W / 2) / W, 1 - (BTN_CY - BTN_H / 2) / H))

for k in range(3):  # LiDAR touch debug dots
    rect('dot%d' % k, 0, 0, 26, 26, fill=DOT, fill_a=0.0, radius=13)

text('status', 'LiDAR: waiting for data', 1222, 30, 13, alignx='center', alpha=0.9)
st = root.op('status')
setp(st, 'alignx', 'right'); setp(st, 'position1', -58)

out = root.create(nullTOP, 'out')
add(out)

# --------------------------------------------------------------------------- panel + window
ui = root.create(containerCOMP, 'ui')
setp(ui, 'w', DISPLAY_W); setp(ui, 'h', DISPLAY_H)
setp(ui, 'top', out.path)
ui.nodeX, ui.nodeY = out.nodeX + 200, 0

win = root.create(windowCOMP, 'window')
setp(win, 'winop', ui.path)
setp(win, 'winw', DISPLAY_W); setp(win, 'winh', DISPLAY_H)
setp(win, 'borders', False)
setp(win, 'size', 'fill')          # Opening Size: Fill the display
setp(win, 'closeescape', True)      # Esc closes the window
setp(win, 'justifyoffsetto', 'specifydisplay')
setp(win, 'display', 1)            # monitor index 1 (the 2nd display; the main one is 0)
win.nodeX, win.nodeY = out.nodeX + 400, 0

# --------------------------------------------------------------------------- OSC in
cb = root.create(textDAT, 'osc_callbacks')
cb.text = '''# Receives touches from lidar_bridge.py
def onReceiveOSC(dat, rowIndex, message, byteData, timeStamp, address, args, peer):
    r = parent()
    if address == '/lidar/touches':
        a = [float(x) for x in args]
        r.store('lidar_touches', [(a[i], a[i + 1]) for i in range(0, len(a) - 1, 2)])
        r.store('lidar_last', absTime.seconds)
    elif address == '/lidar/brick':
        a = [float(x) for x in args]
        r.store('lidar_brick', a[0] > 0.5 if a else False)
        r.store('lidar_last', absTime.seconds)
    elif address == '/lidar/hz':
        r.store('lidar_hz', float(args[0]) if args else 0.0)
        r.store('lidar_last', absTime.seconds)
    return
'''
osc = root.create(oscinDAT, 'osc_in')
setp(osc, 'port', OSC_PORT)
setp(osc, 'callbacks', cb.path)
setp(osc, 'clamp', True); setp(osc, 'maxlines', 20)
osc.nodeX, osc.nodeY = 0, -300
cb.nodeX, cb.nodeY = 0, -440

# --------------------------------------------------------------------------- logic
logic = root.create(executeDAT, 'logic')
logic.text = '''# Toggle logic: runs every frame. Mouse + LiDAR touches.
TOPICS = %r
BTN = %r          # (u0, v0, u1, v1) per button, v up
W, H = %d, %d
WHITE, DARK, DOT = %r, %r, %r
PRESS_S   = 0.06   # finger must stay this long to count (filters noise)
RELEASE_S = 0.25   # finger must be gone this long before the button can toggle again
STALE_S   = 0.5    # no OSC for this long -> treat LiDAR as disconnected

def _rgb(o, prefix, c):
    setattr(o.par, prefix + 'r', c[0]); setattr(o.par, prefix + 'g', c[1]); setattr(o.par, prefix + 'b', c[2])

def apply_visuals(r, st, order):
    for i, s in enumerate(st):
        on = s['on']
        r.op('btn%%d_bg' %% i).par.fillalpha = 1.0 if on else 0.0
        for n in ('th', 'en'):
            _rgb(r.op('btn%%d_%%s' %% (i, n)), 'fontcolor', DARK if on else WHITE)
    i = order[-1] if order else 0
    r.op('title_th').par.text = 'คุณอยากเห็นอะไรใน ' + TOPICS[i][0]
    r.op('title_en').par.text = 'What do you wish for in  %%s ?' %% TOPICS[i][2]

def apply_brick(r, present):
    r.op('brick_frame').par.borderwidth = 3 if present else 2
    r.op('brick_frame').par.borderalpha = 1.0 if present else 0.8
    r.op('brick_frame').par.fillalpha = 0.25 if present else 0.0
    r.op('brick').par.fillalpha = 1.0 if present else 0.35
    r.op('brick_label').par.text = ('พบอิฐแล้ว · Brick detected' if present
                                    else 'วางอิฐตรงนี้ · Place the brick here')

def reset():
    r = parent()
    r.unstore('btn'); r.unstore('order'); r.unstore('brick_shown')

def onFrameStart(frame):
    r = parent()
    now = absTime.seconds
    st = r.fetch('btn', None)
    order = r.fetch('order', None)
    if st is None:
        st = [{'on': False, 'down': False, 'since': None, 'seen': 0.0} for _ in BTN]
        order = []
        r.store('btn', st); r.store('order', order)
        apply_visuals(r, st, order)

    live = now - r.fetch('lidar_last', -99.0) < STALE_S
    lidar = list(r.fetch('lidar_touches', []))[:1] if live else []   # single user: first touch only

    ui = r.op('ui')
    mouse = []
    if ui.panel.lselect:
        mouse = [(float(ui.panel.u), float(ui.panel.v))]

    changed = False
    for i, (u0, v0, u1, v1) in enumerate(BTN):
        s = st[i]
        hit_l = any(u0 <= u <= u1 and v0 <= v <= v1 for u, v in lidar)
        hit_m = any(u0 <= u <= u1 and v0 <= v <= v1 for u, v in mouse)
        if hit_l or hit_m:
            s['seen'] = now
            if s['since'] is None:
                s['since'] = now
            if not s['down'] and (hit_m or now - s['since'] >= PRESS_S):
                s['down'] = True
                new_on = not s['on']          # tap the selected one again to switch it off
                for o in st:                  # only one button selected at a time
                    o['on'] = False
                s['on'] = new_on
                order[:] = [i] if new_on else []
                changed = True
        else:
            s['since'] = None
            if s['down'] and now - s['seen'] >= RELEASE_S:
                s['down'] = False
    if changed:
        apply_visuals(r, st, order)

    # debug dots: only touches that land on a button are shown, everything else is ignored
    shown = [(u, v) for u, v in lidar
             if any(u0 <= u <= u1 and v0 <= v <= v1 for u0, v0, u1, v1 in BTN)]
    for k in range(3):
        d = r.op('dot%%d' %% k)
        if k < len(shown):
            u, v = shown[k]
            d.par.centerx = (u - 0.5) * W
            d.par.centery = (v - 0.5) * H
            d.par.fillalpha = 0.9
        elif d.par.fillalpha.eval() != 0.0:
            d.par.fillalpha = 0.0

    # brick
    brick = bool(r.fetch('lidar_brick', False)) if live else False
    if brick != r.fetch('brick_shown', None):
        r.store('brick_shown', brick)
        apply_brick(r, brick)
    animate_outline(r, brick)

    # status line (only touch the TOP when the text changes)
    if live:
        msg = 'LiDAR  %%.1f Hz   %%d touch   brick: %%s' %% (r.fetch('lidar_hz', 0.0), len(shown), 'yes' if brick else 'no')
    else:
        msg = 'LiDAR: no data (is lidar_bridge.py running?)'
    stx = r.op('status')
    if stx.par.text.eval() != msg:
        stx.par.text = msg
    return
''' % (TOPICS, BTN_UV, W, H, WHITE, DARK, DOT)
logic.text += '''

# ---- brick outline animation -------------------------------------------------
OUTLINE = %r
TRAIL_N, TRAIL_STEP, TRAIL_LOOP_S = %d, %d, %r

def _perimeter_point(s):
    """s = distance in px along the outline, clockwise from the top-left corner."""
    cx, cy, w, h = OUTLINE
    s %%= 2 * (w + h)
    left, right, top, bottom = cx - w / 2, cx + w / 2, cy + h / 2, cy - h / 2
    if s < w:            return left + s, top
    s -= w
    if s < h:            return right, top - s
    s -= h
    if s < w:            return right - s, bottom
    s -= w
    return left, bottom + s

def animate_outline(r, on):
    was_on = r.fetch('trail_on', False)
    if not on:
        if was_on:
            for k in range(TRAIL_N):
                r.op('trail%%d' %% k).par.fillalpha = 0.0
            r.store('trail_on', False)
        return
    if not was_on:
        r.store('trail_on', True)
        r.store('trail_t0', absTime.seconds)
    cx, cy, w, h = OUTLINE
    perim = 2 * (w + h)
    t = absTime.seconds - r.fetch('trail_t0', 0.0)
    head = (t / TRAIL_LOOP_S) * perim
    fade_in = min(1.0, t / 0.3)
    for k in range(TRAIL_N):
        x, y = _perimeter_point(head - k * TRAIL_STEP)
        d = r.op('trail%%d' %% k)
        d.par.centerx = x
        d.par.centery = y
        d.par.fillalpha = fade_in * (1.0 - k / TRAIL_N) ** 1.5
''' % (OUTLINE, TRAIL_N, TRAIL_STEP, TRAIL_LOOP_S)
setp(logic, 'framestart', True)
setp(logic, 'active', True)
logic.nodeX, logic.nodeY = 200, -300

# --------------------------------------------------------------------------- done
note = root.create(textDAT, 'README')
note.text = ('LiDAR touch test\n'
             '- Open the full-screen window: select "window" and pulse Open in Window,\n'
             '  or run: op("%s").par.winopen.pulse()\n'
             '- Click buttons with the mouse, or run lidar_bridge.py (OSC port %d).\n'
             '- Reset all toggles: op("%s").module.reset()\n') % (win.path, OSC_PORT, logic.path)
note.nodeX, note.nodeY = 400, -300

print('[lidar_touch_test] built at', root.path)
if warnings:
    print('[lidar_touch_test] %d parameter warnings (build still usable):' % len(warnings))
    for w in warnings:
        print('   ', w)
