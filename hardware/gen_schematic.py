#!/usr/bin/env python3
"""Generate the Galdr CNC carrier-board KiCad 10 project (schematic + project + footprint).

Connectivity model (validated against kicad-cli 10.0.3, 0 ERC violations): components are placed on the 1.27 mm
grid at one of four orthogonal rotations. A placed pin's schematic coordinate is
    x = ox + px*cos(th) + py*sin(th)
    y = oy + px*sin(th) - py*cos(th)
where (px, py) is the library connection point (symbol space, +y up) and th the instance rotation. Local nets are
drawn with real wires + junctions; inter-block buses (per-axis STEP/DIR, motor phases, limits, spindle, rails) use
global labels; power rails use power-port symbols driven by PWR_FLAGs. The custom symbols (ESP32 dev-board socket,
BTT TMC2209, dual op-amp) are read from the hand-maintained galdr.kicad_sym and embedded verbatim -- this script
never rewrites that library.
"""
import os, re, uuid, math

HERE = os.path.dirname(os.path.abspath(__file__))
SYMDIR = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols"
SYMLIB = os.path.join(HERE, "galdr.kicad_sym")
S = 1.27  # connection grid (mm)
def snap(v): return round(v / S) * S
def U(): return str(uuid.uuid4())
ROOT = U()
PROJ = "galdr-carrier"

# ---- stock-symbol pin connection geometry (symbol space, +y up; verified from the .kicad_sym libs) ----
PINGEO = {
    "Device:R": {"1": (0, 3.81), "2": (0, -3.81)},
    "Device:C": {"1": (0, 3.81), "2": (0, -3.81)},
    "Device:C_Polarized": {"1": (0, 3.81), "2": (0, -3.81)},
    "Device:LED": {"1": (-3.81, 0), "2": (3.81, 0)},            # 1=K 2=A
    "Device:D": {"1": (-3.81, 0), "2": (3.81, 0)},              # 1=K 2=A
    "Device:D_Schottky": {"1": (-3.81, 0), "2": (3.81, 0)},     # 1=K 2=A
    "Device:D_TVS": {"1": (-3.81, 0), "2": (3.81, 0)},          # 1=A1 2=A2
    "Device:Fuse": {"1": (0, 3.81), "2": (0, -3.81)},
    "Device:Q_PMOS": {"D": (2.54, 5.08), "G": (-5.08, 0), "S": (2.54, -5.08)},
    "Device:Q_NMOS": {"D": (2.54, 5.08), "G": (-5.08, 0), "S": (2.54, -5.08)},
    "Device:R_Potentiometer": {"1": (0, 3.81), "2": (3.81, 0), "3": (0, -3.81)},   # 2 = wiper
    "Isolator:PC817": {"1": (-7.62, 2.54), "2": (-7.62, -2.54), "3": (7.62, -2.54), "4": (7.62, 2.54)},
    "Connector:Barrel_Jack": {"1": (7.62, 2.54), "2": (7.62, -2.54)},
    "Connector_Generic:Conn_01x03": {"1": (-5.08, 2.54), "2": (-5.08, 0), "3": (-5.08, -2.54)},
    "Connector_Generic:Conn_01x04": {"1": (-5.08, 2.54), "2": (-5.08, 0), "3": (-5.08, -2.54), "4": (-5.08, -5.08)},
    "Connector_Generic:Conn_01x06": {"1": (-5.08, 5.08), "2": (-5.08, 2.54), "3": (-5.08, 0),
                                     "4": (-5.08, -2.54), "5": (-5.08, -5.08), "6": (-5.08, -7.62)},
    "power:GND": {"1": (0, 0)}, "power:+3V3": {"1": (0, 0)}, "power:+5V": {"1": (0, 0)},
    "power:+12V": {"1": (0, 0)}, "power:PWR_FLAG": {"1": (0, 0)},
}
POWER_NETS = {"GND": "power:GND", "+3V3": "power:+3V3", "+5V": "power:+5V", "+12V": "power:+12V"}

# ------------------------------------------------------------------ stock symbol extraction
def _extent(s, i):
    d = 0
    for k in range(i, len(s)):
        if s[k] == "(":
            d += 1
        elif s[k] == ")":
            d -= 1
            if d == 0:
                return s[i:k + 1]
    raise SystemExit("unbalanced parens")

def extract(lib, name):
    s = open(f"{SYMDIR}/{lib}.kicad_sym").read()
    i = s.find(f'(symbol "{name}"')
    if i < 0:
        raise SystemExit(f"symbol not found: {lib}:{name}")
    return _extent(s, i).replace(f'(symbol "{name}"', f'(symbol "{lib}:{name}"', 1)

# ------------------------------------------------------------------ custom symbols (read from galdr.kicad_sym)
# The hand-maintained library is the source of truth; we embed each used symbol verbatim (renamed Galdr:<n>) and
# parse its pin connection points so the wire generator lands exactly on every pin.
_LIBTXT = open(SYMLIB).read()

def custom_symbol(name):
    i = _LIBTXT.find(f'(symbol "{name}"')
    if i < 0:
        raise SystemExit(f"custom symbol not found in galdr.kicad_sym: {name}")
    body = _extent(_LIBTXT, i)
    sym = body.replace(f'(symbol "{name}"', f'(symbol "Galdr:{name}"', 1)
    geo = {}
    for m in re.finditer(r'\(pin\s+\w+\s+\w+\s*\(at (-?[0-9.]+) (-?[0-9.]+) (\d+)\)\s*\(length (-?[0-9.]+)\)'
                         r'.*?\(number "([^"]+)"', body, re.S):
        px, py, _ang, _ln, num = m.groups()
        geo[num] = (float(px), float(py))
    return sym, geo

CUSTOM_SYMS, CUSTOM_GEO = {}, {}
for _n in ("LB_ESP32S3", "BTT_TMC2209", "OpAmp_Dual_DIP8"):
    _sym, _geo = custom_symbol(_n)
    CUSTOM_SYMS[f"Galdr:{_n}"] = _sym
    CUSTOM_GEO[f"Galdr:{_n}"] = _geo

# ------------------------------------------------------------------ sheet/body management
SHEETS = {}
SHEET_OBJ = U()          # uuid of the (sheet) object on the carrier page that references page 2
CHILD_UUID = U()         # file uuid of the page-2 schematic
body, used_libs, CUR_PATH = [], set(), f"/{ROOT}"

def sheet(name, file, uuid_, path, title, page):
    global body, used_libs, CUR_PATH
    SHEETS[name] = {"body": [], "libs": set(), "path": path, "file": file, "uuid": uuid_, "title": title, "page": page}
    body, used_libs, CUR_PATH = SHEETS[name]["body"], SHEETS[name]["libs"], path

# ------------------------------------------------------------------ geometry + primitives
def pinpt(ox, oy, th, px, py):
    """Schematic coordinate of a library pin (px,py) for an instance at (ox,oy) rotated th degrees."""
    r = math.radians(th)
    x = ox + px * math.cos(r) + py * math.sin(r)
    y = oy + px * math.sin(r) - py * math.cos(r)
    return snap(x), snap(y)

def place(lib_id, ref, value, fp, ox, oy, th=0, ref_xy=None, val_xy=None):
    """Emit a component instance; return {pin_number: (sx, sy)} of its connection points. No labels attached.
    ref_xy/val_xy override the reference/value text positions (absolute mm) -- use for tall ICs so the text clears
    the pin field."""
    ox, oy = snap(ox), snap(oy)
    used_libs.add(lib_id)
    # Default text placement keeps the reference/value clear of the body: stacked above/below for horizontal
    # passives (th 90/270), offset to the right for vertical ones.
    if ref_xy:
        rx, ry = ref_xy
    elif th in (90, 270):
        rx, ry = ox, oy - 3.3
    else:
        rx, ry = ox + 3.0, oy - 1.4
    if val_xy:
        vx, vy = val_xy
    elif th in (90, 270):
        vx, vy = ox, oy + 3.3
    else:
        vx, vy = ox + 3.0, oy + 1.4
    body.append(f'''	(symbol (lib_id "{lib_id}") (at {ox} {oy} {th}) (unit 1)
		(exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no) (uuid "{U()}")
		(property "Reference" "{ref}" (at {snap(rx)} {snap(ry)} 0) (effects (font (size 1.016 1.016)) (justify left)))
		(property "Value" "{value}" (at {snap(vx)} {snap(vy)} 0) (effects (font (size 1.016 1.016)) (justify left)))
		(property "Footprint" "{fp}" (at {ox} {oy} 0) (effects (font (size 1.016 1.016)) (hide yes)))
		(instances (project "{PROJ}" (path "{CUR_PATH}" (reference "{ref}") (unit 1)))))''')
    geo = CUSTOM_GEO.get(lib_id) or PINGEO[lib_id]
    return {num: pinpt(ox, oy, th, px, py) for num, (px, py) in geo.items()}

pwr_ct = [0]
def pwr(net, sx, sy):
    """Place a power-port symbol at (sx,sy)."""
    sx, sy = snap(sx), snap(sy)
    used_libs.add(POWER_NETS[net])
    pwr_ct[0] += 1
    body.append(f'''	(symbol (lib_id "{POWER_NETS[net]}") (at {sx} {sy} 0) (unit 1)
		(exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no) (uuid "{U()}")
		(property "Reference" "#PWR{pwr_ct[0]:03d}" (at {sx} {sy + 5.08} 0) (effects (font (size 1.016 1.016)) (hide yes)))
		(property "Value" "{net}" (at {sx} {sy + 2.54} 0) (effects (font (size 1.016 1.016))))
		(instances (project "{PROJ}" (path "{CUR_PATH}" (reference "#PWR{pwr_ct[0]:03d}") (unit 1)))))''')

flg_ct = [0]
def flag(net, sx, sy):
    """PWR_FLAG + coincident power port: drives the rail so ERC sees it as sourced."""
    sx, sy = snap(sx), snap(sy)
    used_libs.add("power:PWR_FLAG")
    flg_ct[0] += 1
    body.append(f'''	(symbol (lib_id "power:PWR_FLAG") (at {sx} {sy} 0) (unit 1)
		(exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no) (uuid "{U()}")
		(property "Reference" "#FLG{flg_ct[0]:03d}" (at {sx} {sy - 2.54} 0) (effects (font (size 1.016 1.016)) (hide yes)))
		(property "Value" "PWR_FLAG" (at {sx} {sy + 2.54} 0) (effects (font (size 1.016 1.016))))
		(instances (project "{PROJ}" (path "{CUR_PATH}" (reference "#FLG{flg_ct[0]:03d}") (unit 1)))))''')
    pwr(net, sx, sy)

def glabel(net, sx, sy, ang=0):
    sx, sy = snap(sx), snap(sy)
    just = "left" if ang == 0 else "right"
    body.append(f'	(global_label "{net}" (shape bidirectional) (at {sx} {sy} {ang}) '
                f'(effects (font (size 1.016 1.016)) (justify {just})) (uuid "{U()}"))')

def llabel(net, sx, sy, ang=0):
    sx, sy = snap(sx), snap(sy)
    just = "left" if ang in (0, 90) else "right"
    body.append(f'	(label "{net}" (at {sx} {sy} {ang}) '
                f'(effects (font (size 1.016 1.016)) (justify {just} bottom)) (uuid "{U()}"))')

def nc(sx, sy):
    sx, sy = snap(sx), snap(sy)
    body.append(f'	(no_connect (at {sx} {sy}) (uuid "{U()}"))')

def junction(sx, sy):
    sx, sy = snap(sx), snap(sy)
    body.append(f'	(junction (at {sx} {sy}) (diameter 0) (color 0 0 0 0) (uuid "{U()}"))')

def seg(x1, y1, x2, y2):
    """One axis-aligned wire segment."""
    x1, y1, x2, y2 = snap(x1), snap(y1), snap(x2), snap(y2)
    if x1 != x2 and y1 != y2:
        raise SystemExit(f"non-orthogonal wire ({x1},{y1})->({x2},{y2})")
    if (x1, y1) == (x2, y2):
        return
    body.append(f'	(wire (pts (xy {x1} {y1}) (xy {x2} {y2})) '
                f'(stroke (width 0) (type default)) (uuid "{U()}"))')

def wire(*pts):
    """Polyline of axis-aligned segments through the given points."""
    for a, b in zip(pts, pts[1:]):
        seg(a[0], a[1], b[0], b[1])

def elbow(a, b, hfirst=True):
    """L-route from a to b with one corner."""
    cx, cy = (b[0], a[1]) if hfirst else (a[0], b[1])
    wire(a, (cx, cy), b)

# ===================================================================== PAGE 1: CARRIER
sheet("carrier", "galdr-carrier.kicad_sch", ROOT, f"/{ROOT}", "Galdr CNC Carrier Board", 1)

# Footprint library ids. THT-ONLY for this project (no SMD): axial/disc/radial passives, DIP/TO packages.
FP_R = "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal"
FP_C = "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm"
FP_CP = "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm"
FP_CP10 = "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm"
FP_LED = "LED_THT:LED_D5.0mm"
FP_D_SIG = "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal"
FP_D_TVS = "Diode_THT:D_DO-15_P10.16mm_Horizontal"
FP_D_SCH = "Diode_THT:D_DO-201AD_P15.24mm_Horizontal"
FP_FUSE = "Fuse:Fuseholder_Clip-5x20mm_Littelfuse_111_Inline_P20.00x5.00mm_D1.05mm_Horizontal"
FP_FET = "Package_TO_SOT_THT:TO-220-3_Vertical"
FP_MOSTO92 = "Package_TO_SOT_THT:TO-92_Inline"
FP_OPA = "Package_DIP:DIP-8_W7.62mm"
FP_OPTO = "Package_DIP:DIP-4_W7.62mm"
FP_POT = "Potentiometer_THT:Potentiometer_Bourns_3296W_Vertical"
FP_JST3 = "Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical"
FP_JST6 = "Connector_JST:JST_XH_B6B-XH-A_1x06_P2.50mm_Vertical"
FP_TERM4 = "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1-4-3.81_1x04_P3.81mm_Horizontal"
FP_HDR4 = "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical"
FP_SOCK22 = "Connector_PinSocket_2.54mm:PinSocket_1x22_P2.54mm_Vertical"
FP_JACK = "Connector_BarrelJack:BarrelJack_CUI_PJ-102AH_Horizontal"
FP_TMC = "Galdr:TMC2209_StepStick_Socket"
FP_ESP = ""   # dev-board mounts on two 1x22 socket strips; assign in the PCB editor (no single stock footprint).

# ---- rail flags (drive each rail once so ERC sees a source) ------------------
flag("+12V", 40, 40)
flag("+5V", 64, 40)
flag("+3V3", 88, 40)
flag("GND", 112, 40)

# ---- ESP32-S3 dev-board socket (the signal hub) -----------------------------
# Unified 2x22 socket symbol; left column = pins 1..22, right column = 23..44. Each signal pin gets a short stub
# wire + a global label (so it reads as terminated, not floating); power pins get power ports; spares no_connect.
EOX, EOY = 150, 112
ej = place("Galdr:LB_ESP32S3", "J1", "ESP32-S3 DevBoard", FP_ESP, EOX, EOY,
           ref_xy=(EOX - 13, EOY - 36), val_xy=(EOX - 13, EOY - 33.5))
# pin number -> net  ('' => power port handled below, None entries => no_connect)
ESP_SIG = {  # left column
    "4": "Z_STEP", "5": "X_DIR", "6": "Y_DIR", "7": "Z_DIR", "8": "SPIN_DIR", "9": "FHOLD",
    "10": "CYCSTART", "11": "A_STEP", "12": "STEP_EN", "15": "TMC_UART", "16": "X_LIM",
    "17": "Y_LIM", "18": "Z_LIM", "19": "SPIN_PWM", "20": "SPIN_EN",
    # right column
    "26": "X_STEP", "27": "Y_STEP", "28": "A_DIR", "40": "PROBE"}
ESP_PWR = {"1": "+3V3", "2": "+3V3", "21": "+5V", "22": "GND", "23": "GND", "43": "GND", "44": "GND"}
ESP_NC = {"3", "13", "14", "24", "25", "29", "30", "31", "32", "33", "34", "35", "36", "37", "38", "39", "41", "42"}
for num, (sx, sy) in ej.items():
    left = sx < EOX
    dx = -7.62 if left else 7.62
    end = (snap(sx + dx), sy)
    if num in ESP_PWR:
        seg(sx, sy, *end)
        pwr(ESP_PWR[num], *end)
    elif num in ESP_SIG:
        seg(sx, sy, *end)
        glabel(ESP_SIG[num], end[0], end[1], 0 if left else 180)
    else:
        nc(sx, sy)

# ---- TMC2209 driver blocks: X Y Z A ----------------------------------------
# Each driver: STEP/DIR/EN/UART via labels (to the ESP32), VIO/GND/MS-straps via power ports, a VM decoupling cap
# wired directly to the VM pin (+12 V), and the four coil pins labelled out to a motor terminal block below.
AXES = [("X", {"2": "GND", "3": "GND"}),      # MS1,MS2 address straps
        ("Y", {"2": "+3V3", "3": "GND"}),
        ("Z", {"2": "GND", "3": "+3V3"}),
        ("A", {"2": "+3V3", "3": "+3V3"})]
for i, (ax, ms) in enumerate(AXES):
    ox = 250 + i * 80
    oy = 100
    t = place("Galdr:BTT_TMC2209", f"U{i+1}", "BTT TMC2209 V1.3", FP_TMC, ox, oy,
              ref_xy=(ox - 7, oy - 20.5), val_xy=(ox - 7, oy - 18))
    # left side: control signals + straps (stub left, then label/port)
    seg(*t["1"], t["1"][0] - 2.54, t["1"][1]); glabel("STEP_EN", t["1"][0] - 2.54, t["1"][1], 180)   # EN
    for strap_num in ("2", "3"):                                                                       # MS1/MS2
        seg(*t[strap_num], t[strap_num][0] - 2.54, t[strap_num][1]); pwr(ms[strap_num], t[strap_num][0] - 2.54, t[strap_num][1])
    # RX (4) + TX (5) both join the single-wire UART bus
    seg(*t["4"], t["4"][0] - 2.54, t["4"][1]); glabel("TMC_BUS", t["4"][0] - 2.54, t["4"][1], 180)
    seg(*t["5"], t["5"][0] - 2.54, t["5"][1]); glabel("TMC_BUS", t["5"][0] - 2.54, t["5"][1], 180)
    seg(*t["6"], t["6"][0] - 2.54, t["6"][1]); pwr("GND", t["6"][0] - 2.54, t["6"][1])                 # CLK -> GND
    seg(*t["7"], t["7"][0] - 2.54, t["7"][1]); glabel(f"{ax}_STEP", t["7"][0] - 2.54, t["7"][1], 180)
    seg(*t["8"], t["8"][0] - 2.54, t["8"][1]); glabel(f"{ax}_DIR", t["8"][0] - 2.54, t["8"][1], 180)
    # right side: GND/VIO ports + coil labels
    seg(*t["9"], t["9"][0] + 2.54, t["9"][1]); pwr("GND", t["9"][0] + 2.54, t["9"][1])                 # GND
    seg(*t["10"], t["10"][0] + 2.54, t["10"][1]); pwr("+3V3", t["10"][0] + 2.54, t["10"][1])           # VIO
    seg(*t["15"], t["15"][0] + 2.54, t["15"][1]); pwr("GND", t["15"][0] + 2.54, t["15"][1])            # GND
    for num, coil in (("11", "B2"), ("12", "B1"), ("13", "A1"), ("14", "A2")):
        seg(*t[num], t[num][0] + 5.08, t[num][1]); glabel(f"{ax}_{coil}", t[num][0] + 5.08, t[num][1], 0)
    # VM decoupling cap on the +12 V rail beside the driver: wired straight off the VM pin (well clear of the coil
    # labels), +12 V port on the VM node, cap low side to GND.
    vm = t["16"]
    cx = vm[0] + 21.0
    c = place("Device:C", f"C{i+1}", "100n", FP_C, cx, oy - 11.43)   # vertical cap, pin1 top aligns with VM y
    seg(vm[0], vm[1], c["1"][0], c["1"][1])                          # VM -> cap high side (same y, above coils)
    junction(c["1"][0], c["1"][1])
    seg(c["1"][0], c["1"][1], c["1"][0], c["1"][1] - 2.54); pwr("+12V", c["1"][0], c["1"][1] - 2.54)
    seg(c["2"][0], c["2"][1], c["2"][0], c["2"][1] + 2.54); pwr("GND", c["2"][0], c["2"][1] + 2.54)
    # motor terminal block below the driver; coil phases labelled in.
    mt = place("Connector_Generic:Conn_01x04", f"J_MOT{i+1}", f"MOTOR_{ax}", FP_TERM4, ox - 4, oy + 34)
    for num, coil in (("1", "A1"), ("2", "A2"), ("3", "B1"), ("4", "B2")):
        seg(*mt[num], mt[num][0] - 2.54, mt[num][1]); glabel(f"{ax}_{coil}", mt[num][0] - 2.54, mt[num][1], 180)

# ---- TMC UART: series resistor to the bus + idle pull-up to +3V3 -------------
# R_UART converts the half-duplex single-wire UART; R_UPU idles the bus high. Both tie to the TMC_BUS node.
ru = place("Device:R", "R_UART", "1k", FP_R, 220, 56, 90)     # horizontal: pin2 left, pin1 right
seg(*ru["2"], ru["2"][0] - 2.54, ru["2"][1]); glabel("TMC_UART", ru["2"][0] - 2.54, ru["2"][1], 180)
rp = place("Device:R", "R_UPU", "20k", FP_R, 220, 48)         # vertical: pin1 top -> +3V3, pin2 bottom -> bus
seg(*rp["1"], rp["1"][0], rp["1"][1] - 2.54); pwr("+3V3", rp["1"][0], rp["1"][1] - 2.54)
# TMC_BUS node: join R_UART pin1 (right) and R_UPU pin2 (bottom) and emit a global label.
busx = ru["1"][0] + 2.54
seg(ru["1"][0], ru["1"][1], busx, ru["1"][1])
seg(rp["2"][0], rp["2"][1], rp["2"][0], ru["1"][1])
seg(rp["2"][0], ru["1"][1], busx, ru["1"][1])
junction(rp["2"][0], ru["1"][1])
glabel("TMC_BUS", busx, ru["1"][1], 0)

# ---- opto-isolated inputs: X / Y / Z limits + PROBE ------------------------
# Per channel: a field connector drives the PC817 LED through a current-limit resistor (anti-parallel signal diode
# for reverse protection); the phototransistor pulls a +3V3 node that a series/cap RC debounces into the GPIO net.
def opto_channel(ox, oy, gpio_net, ref):
    oc = place("Isolator:PC817", f"OK_{ref}", "PC817", FP_OPTO, ox + 30, oy)
    leda, sig, col, gnd3 = oc["1"], oc["2"], oc["4"], oc["3"]
    # field connector (rotated to face into the cluster); sig (pin2) aligned to the lower 'sig' rail.
    jl = place("Connector_Generic:Conn_01x03", f"J_LIM_{ref}", f"LIM_{ref}", FP_JST3, ox, oy + 2.54, 180)
    # LED current-limit resistor feeding the upper 'leda' rail from +12 V.
    rl = place("Device:R", f"R_LED_{ref}", "680", FP_R, ox + 10, oy - 2.54, 270)   # pin1 left=+12V, pin2 right=leda
    seg(*rl["1"], rl["1"][0] - 2.54, rl["1"][1]); pwr("+12V", rl["1"][0] - 2.54, rl["1"][1])
    # anti-parallel signal diode across the LED (K=leda rail, A=sig rail).
    dl = place("Device:D", f"D_LED_{ref}", "1N4148", FP_D_SIG, ox + 17, oy, 90)    # K top, A bottom
    # leda rail (y = oy-2.54): R_LED.2 -> D_LED.K tap -> PC817.leda
    seg(rl["2"][0], rl["2"][1], leda[0], leda[1])
    seg(dl["1"][0], dl["1"][1], dl["1"][0], leda[1]); junction(dl["1"][0], leda[1])
    # sig rail (y = oy+2.54): J_LIM.sig -> D_LED.A tap -> PC817.sig
    seg(jl["2"][0], jl["2"][1], sig[0], sig[1])
    seg(dl["2"][0], dl["2"][1], dl["2"][0], sig[1]); junction(dl["2"][0], sig[1])
    # connector power/ground + opto emitter ground
    seg(*jl["1"], jl["1"][0], jl["1"][1] + 2.54); pwr("+12V", jl["1"][0], jl["1"][1] + 2.54)
    seg(*jl["3"], jl["3"][0] - 2.54, jl["3"][1]); pwr("GND", jl["3"][0] - 2.54, jl["3"][1])
    seg(*gnd3, gnd3[0], gnd3[1] + 2.54); pwr("GND", gnd3[0], gnd3[1] + 2.54)
    # output: col node pulled up to +3V3, then R/C debounce into the GPIO net.
    colx = col[0] + 6.0
    seg(col[0], col[1], colx, col[1])                          # col -> trunk x
    rpu = place("Device:R", f"R_PU_{ref}", "10k", FP_R, colx, oy - 9)   # vertical: pin2 bottom -> col, pin1 -> +3V3
    seg(rpu["2"][0], rpu["2"][1], colx, col[1]); junction(colx, col[1])
    seg(*rpu["1"], rpu["1"][0], rpu["1"][1] - 2.54); pwr("+3V3", rpu["1"][0], rpu["1"][1] - 2.54)
    rdb = place("Device:R", f"R_DEB_{ref}", "1k", FP_R, colx + 7, col[1], 90)      # pin2 left=col, pin1 right=gpio
    seg(colx, col[1], rdb["2"][0], rdb["2"][1])
    gpiox = rdb["1"][0] + 3.0
    seg(rdb["1"][0], rdb["1"][1], gpiox, col[1])
    cdb = place("Device:C", f"C_DEB_{ref}", "100n", FP_C, gpiox, oy + 1)           # pin1 top -> gpio, pin2 -> GND
    seg(cdb["1"][0], cdb["1"][1], gpiox, col[1]); junction(gpiox, col[1])
    seg(*cdb["2"], cdb["2"][0], cdb["2"][1] + 2.54); pwr("GND", cdb["2"][0], cdb["2"][1] + 2.54)
    glabel(gpio_net, gpiox, col[1], 0)

for i, (ref, gpio) in enumerate([("X", "X_LIM"), ("Y", "Y_LIM"), ("Z", "Z_LIM"), ("PROBE", "PROBE")]):
    opto_channel(40 + i * 135, 190, gpio, ref)

# ---- power input: 12 V jack -> fuse -> reverse-protect PFET -> +12V rail -> buck -> +5V ----
# A +12 V top rail and GND bottom rail; the TVS, bulk + bypass caps and power LED hang between them, the buck
# module taps the rails and its output is OR-ed onto +5V through a Schottky.
PRY, PGY = 300, 316          # +12V rail y, GND rail y
RAIL_X1 = 168                # +12V rail runs to the buck tap
# Rails are drawn last (after the tap x-coords are known) so each ends exactly on a tap, leaving no loose stub.
# -- jack + fuse + PFET feeding the rail --
jp = place("Connector:Barrel_Jack", "J_PWR", "DC_12V_5.5x2.5", FP_JACK, 40, 300)
seg(*jp["2"], jp["2"][0], PGY); junction(jp["2"][0], PGY)     # jack sleeve -> GND rail
f1 = place("Device:Fuse", "F1", "3A", FP_FUSE, 54, 297.46, 270)   # pin1 VIN_RAW left, pin2 VIN_F right
elbow(jp["1"], f1["1"])
q1 = place("Device:Q_PMOS", "Q1", "FQP27P06", FP_FET, 68, 300)    # D top=VIN_F, S bottom=+12V, G left=GND
elbow(f1["2"], q1["D"], hfirst=False)
seg(q1["S"][0], q1["S"][1], q1["S"][0], PRY); junction(q1["S"][0], PRY)   # PFET source -> +12V rail
seg(*q1["G"], q1["G"][0] - 2.54, q1["G"][1]); pwr("GND", q1["G"][0] - 2.54, q1["G"][1])
# -- shunt parts between the rails --
def shunt(x, lib, ref, val, fp, th=90):
    c = place(lib, ref, val, fp, x, (PRY + PGY) / 2, th)
    seg(c["1"][0], c["1"][1], x, PRY); junction(x, PRY)
    seg(c["2"][0], c["2"][1], x, PGY); junction(x, PGY)
shunt(96, "Device:D_TVS", "D_TVS", "P6KE16A", FP_D_TVS)
shunt(108, "Device:C_Polarized", "C_IN1", "470u/35V", FP_CP, 0)
shunt(118, "Device:C", "C_IN2", "100n", FP_C, 0)
# power LED: R_PLED (+12V -> PLED_A) + D_PLED (PLED_A -> GND)
rpl = place("Device:R", "R_PLED", "4k7", FP_R, 130, 304)         # vertical: pin1 top, pin2 bottom
seg(rpl["1"][0], rpl["1"][1], 130, PRY); junction(130, PRY)
dpl = place("Device:LED", "D_PLED", "PWR", FP_LED, 130, 312, 270)  # A top -> PLED_A, K bottom -> GND
seg(rpl["2"][0], rpl["2"][1], dpl["2"][0], dpl["2"][1])
seg(dpl["1"][0], dpl["1"][1], 130, PGY); junction(130, PGY)
# -- buck module + Schottky OR onto +5V --
ub = place("Connector_Generic:Conn_01x04", "U_BUCK", "MP1584_12to5", FP_HDR4, 162, 305)  # pins left: 1+12V 2GND 3+5V_BUCK 4GND
elbow(ub["1"], (RAIL_X1, PRY)); junction(RAIL_X1, PRY)
seg(ub["2"][0], ub["2"][1], ub["2"][0] - 4, ub["2"][1]); seg(ub["2"][0] - 4, ub["2"][1], ub["2"][0] - 4, PGY); junction(ub["2"][0] - 4, PGY)
seg(ub["4"][0], ub["4"][1], ub["4"][0] - 6, ub["4"][1]); seg(ub["4"][0] - 6, ub["4"][1], ub["4"][0] - 6, PGY); junction(ub["4"][0] - 6, PGY)
db = place("Device:D_Schottky", "D_BUCK", "1N5822", FP_D_SCH, 150, 326, 180)  # A left=+5V_BUCK, K right=+5V
elbow(ub["3"], db["2"])
seg(*db["1"], db["1"][0] + 3, db["1"][1]); glabel("+5V", db["1"][0] + 3, db["1"][1], 0)
c5 = place("Device:C_Polarized", "C_5V", "10u/16V", FP_CP10, 162, 326)  # pin1 +5V top, pin2 GND
seg(c5["1"][0], c5["1"][1], c5["1"][0], db["1"][1]); seg(c5["1"][0], db["1"][1], db["1"][0] + 3, db["1"][1]); junction(c5["1"][0], db["1"][1])
seg(c5["2"][0], c5["2"][1], c5["2"][0], 332); pwr("GND", c5["2"][0], 332)
# rails: span exactly from the first tap (PFET source / jack sleeve) to the last (buck), so no endpoint is loose.
seg(q1["S"][0], PRY, RAIL_X1, PRY)            # +12V rail
seg(jp["2"][0], PGY, ub["2"][0] - 4, PGY)     # GND rail

# ---- aux / spindle interface headers -----------------------------------------
# J_SPIN carries the spindle conditioning signals + 12 V to the daughterboard (page 2); J_AUX exposes feed-hold /
# cycle-start. Both are pin headers; signals leave as global labels (buses to the ESP32 / page 2).
sp = place("Connector_Generic:Conn_01x06", "J_SPIN", "SPINDLE_IF", FP_JST6, 470, 150)
for num, net in (("1", "SPIN_PWM"), ("2", "SPIN_EN"), ("3", "SPIN_DIR"), ("4", "+12V"), ("5", "GND"), ("6", "GND")):
    seg(*sp[num], sp[num][0] - 2.54, sp[num][1])
    if net in POWER_NETS:
        pwr(net, sp[num][0] - 2.54, sp[num][1])
    else:
        glabel(net, sp[num][0] - 2.54, sp[num][1], 180)
au = place("Connector_Generic:Conn_01x04", "J_AUX", "AUX_CTRL", FP_HDR4, 470, 180)
for num, net in (("1", "FHOLD"), ("2", "CYCSTART"), ("3", "+3V3"), ("4", "GND")):
    seg(*au[num], au[num][0] - 2.54, au[num][1])
    if net in POWER_NETS:
        pwr(net, au[num][0] - 2.54, au[num][1])
    else:
        glabel(net, au[num][0] - 2.54, au[num][1], 180)

# ===================================================================== PAGE 2: SPINDLE 0-10 V CONDITIONING
# Separate daughterboard (DOC-07), mates J_SPIN over a 6-wire cable. Recovers the GPIO13 PWM to a DC level (2-stage
# RC), scales it to 0-10 V with a non-inverting op-amp (calibration trimmer), and relays EN/DIR to the WS55-220
# through open-drain MOSFETs. +12 V and GND are global nets driven on the carrier, so they bridge the cable.
sheet("spindle_cond", "galdr-spindle-cond.kicad_sch", CHILD_UUID, f"/{ROOT}/{SHEET_OBJ}", "Spindle 0-10 V Conditioning", 2)

# inter-board input connector (mates J_SPIN); pins face right into the circuit.
ci = place("Connector_Generic:Conn_01x06", "J_CIN", "FROM_CARRIER", FP_JST6, 30, 74, 180)
ci_net = {"1": "SPIN_PWM", "2": "SPIN_EN", "3": "SPIN_DIR", "4": "+12V", "5": "GND", "6": "GND"}
for num, net in ci_net.items():
    seg(*ci[num], ci[num][0] + 2.54, ci[num][1])
    if net in POWER_NETS:
        pwr(net, ci[num][0] + 2.54, ci[num][1])
    else:
        glabel(net, ci[num][0] + 2.54, ci[num][1], 0)
# Note: +12 V and GND arrive on this sheet via J_CIN as global nets driven by the carrier's PWR_FLAGs (page 1);
# no flag is added here (a second flag on the same global rail would be a duplicate power source).

# PWM -> DC: two-stage RC low-pass into SP_DC.
rl1 = place("Device:R", "R_LP1", "10k", FP_R, 52, 66, 90)        # pin2 left=SPIN_PWM, pin1 right=SP_LP1
seg(*rl1["2"], rl1["2"][0] - 2.54, rl1["2"][1]); glabel("SPIN_PWM", rl1["2"][0] - 2.54, rl1["2"][1], 180)
rl2 = place("Device:R", "R_LP2", "10k", FP_R, 66, 66, 90)        # pin2 left=SP_LP1, pin1 right=SP_DC
seg(rl1["1"][0], rl1["1"][1], rl2["2"][0], rl2["2"][1])          # SP_LP1 node
cl1 = place("Device:C", "C_LP1", "1u", FP_C, rl1["1"][0], 74)    # SP_LP1 -> GND
seg(cl1["1"][0], cl1["1"][1], cl1["1"][0], rl1["1"][1]); junction(cl1["1"][0], rl1["1"][1])
seg(cl1["2"][0], cl1["2"][1], cl1["2"][0], cl1["2"][1] + 2.54); pwr("GND", cl1["2"][0], cl1["2"][1] + 2.54)
cl2 = place("Device:C", "C_LP2", "1u", FP_C, rl2["1"][0], 74)    # SP_DC -> GND
seg(cl2["1"][0], cl2["1"][1], cl2["1"][0], rl2["1"][1]); junction(cl2["1"][0], rl2["1"][1])
seg(cl2["2"][0], cl2["2"][1], cl2["2"][0], cl2["2"][1] + 2.54); pwr("GND", cl2["2"][0], cl2["2"][1] + 2.54)
seg(rl2["1"][0], rl2["1"][1], rl2["1"][0] + 3, rl2["1"][1]); llabel("SP_DC", rl2["1"][0] + 3, rl2["1"][1], 0)

# non-inverting amp: gain 1 + (R_FB + RV_CAL)/R_G; trim RV_CAL for 10.0 V at full duty. Channel B parked as a
# grounded unity follower. The feedback divider is a vertical stack to the left; SP_SVRAW / SP_FB / SP_DC bridge it
# to the op-amp pins (all channel-A pins are on the symbol's left edge).
oa = place("Galdr:OpAmp_Dual_DIP8", "U_OA", "LMC6482IN", FP_OPA, 120, 80,
           ref_xy=(120 - 6, 80 - 9), val_xy=(120 - 6, 80 - 6.5))
seg(*oa["8"], oa["8"][0] + 2.54, oa["8"][1]); pwr("+12V", oa["8"][0] + 2.54, oa["8"][1])   # V+
seg(*oa["4"], oa["4"][0] - 2.54, oa["4"][1]); pwr("GND", oa["4"][0] - 2.54, oa["4"][1])     # V-
seg(*oa["5"], oa["5"][0] + 2.54, oa["5"][1]); pwr("GND", oa["5"][0] + 2.54, oa["5"][1])     # IN+B -> GND
seg(oa["6"][0], oa["6"][1], oa["6"][0] + 2.54, oa["6"][1])                                   # IN-B / OUTB park
seg(oa["7"][0], oa["7"][1], oa["7"][0] + 2.54, oa["7"][1])
seg(oa["6"][0] + 2.54, oa["6"][1], oa["7"][0] + 2.54, oa["7"][1])
seg(*oa["3"], oa["3"][0] - 2.54, oa["3"][1]); llabel("SP_DC", oa["3"][0] - 2.54, oa["3"][1], 180)   # IN+A
seg(*oa["2"], oa["2"][0] - 2.54, oa["2"][1]); llabel("SP_FB", oa["2"][0] - 2.54, oa["2"][1], 180)   # IN-A
seg(*oa["1"], oa["1"][0] - 2.54, oa["1"][1]); llabel("SP_SVRAW", oa["1"][0] - 2.54, oa["1"][1], 180)  # OUTA
# feedback divider stack (x = 100): SP_SVRAW - R_FB - SP_FBT - RV_CAL - SP_FB - R_G - GND
FBX = 100
rfb = place("Device:R", "R_FB", "15k", FP_R, FBX, 68)            # pin1 top SP_SVRAW, pin2 bottom SP_FBT
seg(rfb["1"][0], rfb["1"][1], rfb["1"][0], rfb["1"][1] - 2.54); llabel("SP_SVRAW", rfb["1"][0], rfb["1"][1] - 2.54, 90)
rv = place("Device:R_Potentiometer", "RV_CAL", "10k", FP_POT, FBX, 80)   # 1 top SP_FBT, 3 bottom SP_FB, 2 wiper SP_FB
seg(rfb["2"][0], rfb["2"][1], rv["1"][0], rv["1"][1])           # SP_FBT
rg = place("Device:R", "R_G", "10k", FP_R, FBX, 92)             # pin1 top SP_FB, pin2 bottom GND
seg(rv["3"][0], rv["3"][1], rg["1"][0], rg["1"][1])             # SP_FB
seg(rv["2"][0], rv["2"][1], rv["2"][0], rv["3"][1]); seg(rv["2"][0], rv["3"][1], rv["3"][0], rv["3"][1]); junction(rv["3"][0], rv["3"][1])  # wiper -> SP_FB
llabel("SP_FB", rg["1"][0], rg["1"][1], 90)
seg(rg["2"][0], rg["2"][1], rg["2"][0], rg["2"][1] + 2.54); pwr("GND", rg["2"][0], rg["2"][1] + 2.54)

# output to the WS55-220 SV terminal: series isolation R_OS + smoothing C_OS.
ros = place("Device:R", "R_OS", "100", FP_R, 150, 76, 90)       # pin2 left=SP_SVRAW, pin1 right=SP_SV
seg(*ros["2"], ros["2"][0] - 2.54, ros["2"][1]); llabel("SP_SVRAW", ros["2"][0] - 2.54, ros["2"][1], 180)
cos = place("Device:C", "C_OS", "100n", FP_C, ros["1"][0], 84)  # SP_SV -> GND
seg(cos["1"][0], cos["1"][1], cos["1"][0], ros["1"][1]); junction(cos["1"][0], ros["1"][1])
seg(cos["2"][0], cos["2"][1], cos["2"][0], cos["2"][1] + 2.54); pwr("GND", cos["2"][0], cos["2"][1] + 2.54)
seg(ros["1"][0], ros["1"][1], ros["1"][0] + 3, ros["1"][1]); glabel("SP_SV", ros["1"][0] + 3, ros["1"][1], 0)

# EN / DIR open-drain drivers (gate HIGH -> drain pulls the WS55-220 terminal to GND).
def driver(ox, sig, gate_net, drv_net, rg_ref, rpd_ref, q_ref):
    rge = place("Device:R", rg_ref, "100", FP_R, ox, 120, 90)   # pin2 left=sig, pin1 right=gate
    seg(*rge["2"], rge["2"][0] - 2.54, rge["2"][1]); glabel(sig, rge["2"][0] - 2.54, rge["2"][1], 180)
    qn = place("Device:Q_NMOS", q_ref, "2N7000", FP_MOSTO92, ox + 12, 120)   # G left, D top, S bottom
    seg(rge["1"][0], rge["1"][1], qn["G"][0], qn["G"][1])       # gate node
    rpd = place("Device:R", rpd_ref, "100k", FP_R, qn["G"][0], 130)   # gate pulldown to GND
    seg(rpd["1"][0], rpd["1"][1], qn["G"][0], qn["G"][1]); junction(qn["G"][0], qn["G"][1])
    seg(rpd["2"][0], rpd["2"][1], rpd["2"][0], rpd["2"][1] + 2.54); pwr("GND", rpd["2"][0], rpd["2"][1] + 2.54)
    seg(*qn["S"], qn["S"][0], qn["S"][1] + 2.54); pwr("GND", qn["S"][0], qn["S"][1] + 2.54)
    seg(*qn["D"], qn["D"][0], qn["D"][1] - 2.54); glabel(drv_net, qn["D"][0], qn["D"][1] - 2.54, 90)
driver(58, "SPIN_EN", "SP_GE", "SP_ENDRV", "R_GE", "R_PDE", "Q_EN")
driver(96, "SPIN_DIR", "SP_GD", "SP_DIRDRV", "R_GD", "R_PDD", "Q_DIR")

# output terminal block to the WS55-220.
ws = place("Connector_Generic:Conn_01x04", "J_WS", "TO_WS55-220", FP_TERM4, 175, 120)
for num, net in (("1", "SP_SV"), ("2", "SP_ENDRV"), ("3", "SP_DIRDRV"), ("4", "GND")):
    seg(*ws[num], ws[num][0] - 2.54, ws[num][1])
    if net in POWER_NETS:
        pwr(net, ws[num][0] - 2.54, ws[num][1])
    else:
        glabel(net, ws[num][0] - 2.54, ws[num][1], 180)

# op-amp bypass + board-powered indicator.
cby = place("Device:C", "C_BYP", "100n", FP_C, 134, 96)         # +12V -> GND near the op-amp
seg(cby["1"][0], cby["1"][1], cby["1"][0], cby["1"][1] - 2.54); pwr("+12V", cby["1"][0], cby["1"][1] - 2.54)
seg(cby["2"][0], cby["2"][1], cby["2"][0], cby["2"][1] + 2.54); pwr("GND", cby["2"][0], cby["2"][1] + 2.54)
rpl2 = place("Device:R", "R_PL", "4k7", FP_R, 40, 120)          # +12V -> SP_PLED
seg(rpl2["1"][0], rpl2["1"][1], rpl2["1"][0], rpl2["1"][1] - 2.54); pwr("+12V", rpl2["1"][0], rpl2["1"][1] - 2.54)
dpl2 = place("Device:LED", "D_PL", "PWR", FP_LED, 40, 128, 270)  # A top SP_PLED, K bottom GND
seg(rpl2["2"][0], rpl2["2"][1], dpl2["2"][0], dpl2["2"][1])
seg(dpl2["1"][0], dpl2["1"][1], dpl2["1"][0], dpl2["1"][1] + 2.54); pwr("GND", dpl2["1"][0], dpl2["1"][1] + 2.54)

# ===================================================================== collision self-check + EMIT
def _check_collisions(sheet_body, sheet_name):
    import collections
    txt = "\n".join(sheet_body)
    pts = collections.defaultdict(set)
    for m in re.finditer(r'\((?:global_label|label) "([^"]+)" .*?\(at (-?[\d.]+) (-?[\d.]+)', txt):
        pts[(round(float(m.group(2)), 2), round(float(m.group(3)), 2))].add(m.group(1))
    for m in re.finditer(r'\(symbol \(lib_id "power:(GND|\+3V3|\+5V|\+12V)"\) \(at (-?[\d.]+) (-?[\d.]+)', txt):
        pts[(round(float(m.group(2)), 2), round(float(m.group(3)), 2))].add(m.group(1))
    bad = {p: n for p, n in pts.items() if len(n) > 1}
    if bad:
        for p, n in bad.items():
            print(f"  COLLISION ({sheet_name}) at", p, "->", n)
        raise SystemExit("net coordinate collisions detected; adjust placement")

def lib_symbols_block(libs):
    blocks = []
    for lid in sorted(libs):
        blocks.append(CUSTOM_SYMS[lid] if lid.startswith("Galdr:") else extract(*lid.split(":", 1)))
    return "\n".join("\n".join("\t" + ln if ln.strip() else ln for ln in b.splitlines()) for b in blocks)

for _name, _s in SHEETS.items():
    _check_collisions(_s["body"], _name)

# (sheet) object on the carrier page referencing page 2.
sub = SHEETS.get("spindle_cond")
sheet_obj = ""
if sub:
    sheet_obj = f'''	(sheet (at 470 44) (size 40 16) (fields_autoplaced yes)
		(stroke (width 0.1524) (type solid)) (fill (color 0 0 0 0.0000)) (uuid "{SHEET_OBJ}")
		(property "Sheetname" "{sub['title']}" (at 470 43.36 0) (effects (font (size 1.27 1.27)) (justify left bottom)))
		(property "Sheetfile" "{sub['file']}" (at 470 60.8 0) (effects (font (size 1.27 1.27)) (justify left top)))
		(instances (project "{PROJ}" (path "/{ROOT}" (page "2")))))'''

carrier = SHEETS["carrier"]
sheet_instances = f'''	(sheet_instances
		(path "/" (page "1")){chr(10) + chr(9) + chr(9) + f'(path "/{SHEET_OBJ}" (page "2"))' if sub else ""})'''
sch = f'''(kicad_sch
	(version 20260306)
	(generator "galdr-gen")
	(generator_version "10.0")
	(uuid "{ROOT}")
	(paper "A2")
	(title_block
		(title "Galdr CNC Carrier Board")
		(company "Galdr")
		(comment 1 "ESP32-S3 (Lonely Binary) + 4x BTT TMC2209 V1.3 + opto limits"))
	(lib_symbols
{lib_symbols_block(carrier['libs'])}
	)
{chr(10).join(carrier['body'])}
{sheet_obj}
{sheet_instances}
	(embedded_fonts no)
)
'''
open(os.path.join(HERE, "galdr-carrier.kicad_sch"), "w").write(sch)

if sub:
    sub_sch = f'''(kicad_sch
	(version 20260306)
	(generator "galdr-gen")
	(generator_version "10.0")
	(uuid "{CHILD_UUID}")
	(paper "A3")
	(title_block
		(title "Galdr Spindle 0-10 V Conditioning")
		(company "Galdr")
		(comment 1 "PWM->0-10 V op-amp + EN/DIR open-drain for the WS55-220 (DOC-07)"))
	(lib_symbols
{lib_symbols_block(sub['libs'])}
	)
{chr(10).join(sub['body'])}
	(embedded_fonts no)
)
'''
    open(os.path.join(HERE, sub['file']), "w").write(sub_sch)

# project file
_pages = f'[\n    ["{ROOT}", "1"]' + (f',\n    ["{SHEET_OBJ}", "2"]' if sub else "") + "\n  ]"
open(os.path.join(HERE, "galdr-carrier.kicad_pro"), "w").write('''{
  "board": {"3dviewports": [], "design_settings": {}, "layer_presets": [], "viewports": []},
  "boards": [],
  "cvpcb": {"equivalence_files": []},
  "libraries": {"pinned_footprint_libs": [], "pinned_symbol_libs": []},
  "meta": {"filename": "galdr-carrier.kicad_pro", "version": 1},
  "net_settings": {"classes": []},
  "pcbnew": {"page_layout_descr_file": ""},
  "schematic": {"legacy_lib_list": [], "meta": {"version": 1}},
  "sheets": ''' + _pages + ''',
  "text_variables": {}
}
''')

# project-local lib tables
open(os.path.join(HERE, "sym-lib-table"), "w").write(
    '(sym_lib_table\n  (version 7)\n  (lib (name "Galdr")(type "KiCad")(uri "${KIPRJMOD}/galdr.kicad_sym")(options "")(descr "Galdr custom symbols"))\n)\n')
open(os.path.join(HERE, "fp-lib-table"), "w").write(
    '(fp_lib_table\n  (version 7)\n  (lib (name "Galdr")(type "KiCad")(uri "${KIPRJMOD}/galdr.pretty")(options "")(descr "Galdr custom footprints"))\n)\n')

# custom footprint: 16-pad stepstick socket (Pololu breakout, DIAG no longer broken out)
os.makedirs(os.path.join(HERE, "galdr.pretty"), exist_ok=True)
pol = open("/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints/Module.pretty/"
           "Pololu_Breakout-16_15.2x20.3mm.kicad_mod").read()
pol = pol.replace("Pololu_Breakout-16_15.2x20.3mm", "TMC2209_StepStick_Socket", 1)
open(os.path.join(HERE, "galdr.pretty", "TMC2209_StepStick_Socket.kicad_mod"), "w").write(pol)

print("generated galdr-carrier.kicad_sch" + (" + galdr-spindle-cond.kicad_sch" if sub else " (page 1 only)"))
for _name, _s in SHEETS.items():
    print(f"  sheet '{_name}' (page {_s['page']}): {len(_s['body'])} body items, libs={len(_s['libs'])}")
print(f"power ports={pwr_ct[0]} flags={flg_ct[0]}")
