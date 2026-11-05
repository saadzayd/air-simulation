# simulator.py
import pygame, math, sys, csv, os, random 
import threading
import time
import math

pygame.init()

# ---------------- إعدادات عامة ----------------
WIDTH, HEIGHT = 1200, 680
MAP_IMAGE = "map.jpeg"         # ضع صورة خريطة هنا أو اتركها غير موجودة
AIRPORTS_FILE = "airports.csv"  # ملف ourairports (اختياري)

# حدود الخريطة الجغرافية (عدّلها حسب منطقتك/صورتك)
LAT_MIN, LAT_MAX = 32.426, 37.090
LON_MIN, LON_MAX =-1.681, 8.624

# ---------------- معلمات التفادي والتنبؤ ----------------
# ---------------- معلمات التفادي والتنبؤ ----------------
AVOID_DIST   = 12.0    # ≈ 5 NM: مسافة بدء التفادي (بكسل)
CLEAR_DIST   = 18.0    # ≈ 7.5 NM: مسافة الرجوع للمسار الطبيعي
AVOID_ANGLE  = 30.0    # زاوية افتراضية للتفادي (درجة)
TURN_RATE_DPS = 20.0   # معدل دوران واقعي (درجة/ثانية)

# محاكاة/تنبؤ
LOOKAHEAD = 20.0       # نافذة النظر (ثوانٍ) لحساب TCPA
SIM_TIME = 20.0        # زمن محاكاة السيناريو (ثوانٍ)
SIM_STEPS = 30         # خطوات محاكاة السيناريو
MIN_IMPROVEMENT = 2.0  # الحد الأدنى للتحسّن بالمقارنة (≈ 1 NM)
AVOID_ANGLE_HEAD_ON = 30.0      # التفادي يميناً وجهاً لوجه
AVOID_ANGLE_CROSSING = 20.0     # التفادي عند تقاطع المسارات
AVOID_COOLDOWN_HEAD_ON = 2.0    # ثواني قبل العودة للوضع NORMAL
AVOID_COOLDOWN_CROSSING = 1.5

# إعدادات السرعة الافتراضية (بكسل/ثانية)
DEFAULT_SPEED = 80.0
spawn_timer = 0.0
weather_timer = 0.0
aircraft_lock = threading.Lock() 
# ---------------- شاشة Pygame ----------------
screen = pygame.display.set_mode((WIDTH, HEIGHT))
aircraft_img = pygame.image.load("aircraft.png").convert_alpha()
aircraft_img = pygame.transform.scale(aircraft_img, (20, 20))
pygame.display.set_caption("Air Traffic Simulator (Improved ATC Logic)")
clock = pygame.time.Clock()
font_small = pygame.font.SysFont(None, 18)
# خطوط للـ sidebar
font_large = pygame.font.SysFont(None, 20)
font_bold  = pygame.font.SysFont(None, 22, bold=True)
weather_zones = [
    {
        "x1": 100, "y1": 100, "x2": 400, "y2": 400,
        "wind_dx": 0.0, "wind_dy": 0.0,
        "turbulence": 1.0,
        "visibility": 0.7,         # 0=ضعيفة، 1=جيدة
        "fog_density": 0.3,
        "rain_intensity": 1,
        "snow_intensity": 0,
        "storm": False,
        "lightning_timer": 0,
        "clouds": [],
        "temperature": 15.0,
        "pressure": 1013
    }
]
# ==================== توليد الغيوم ====================
def init_weather_clouds():
    for zone in weather_zones:
        if "clouds" not in zone:
            zone["clouds"] = []
        width = zone["x2"] - zone["x1"]
        height = zone["y2"] - zone["y1"]
        zone["clouds"].clear()
        for _ in range(25):  # عدد غيوم أكثر
            zone["clouds"].append({
                "x": random.randint(0, width),
                "y": random.randint(0, height),
                "r": random.randint(30, 70),
                "alpha": random.randint(90, 180)
            })

def get_weather_at(x, y):
    for zone in weather_zones:
        if zone["x1"] <= x <= zone["x2"] and zone["y1"] <= y <= zone["y2"]:
            return zone
    return None

# ---------------- دوال مساعدة ----------------
def norm_angle_deg(a):
    a = a % 360.0
    if a < 0: a += 360.0
    return a

def shortest_diff_deg(a, b):
    d = (b - a + 180.0) % 360.0 - 180.0
    return d

def geo_to_screen(lat, lon):
    x = int((lon - LON_MIN) / (LON_MAX - LON_MIN) * WIDTH)
    y = int(HEIGHT - (lat - LAT_MIN) / (LAT_MAX - LAT_MIN) * HEIGHT)
    return x, y

# --------- دوال مساعدة لتصنيف الحالة حسب Annex 2 ---------
def rel_bearing(from_ac, to_ac):
    """زاوية الهدف بالنسبة لاتجاه الطائرة (درجات) ضمن [-180, +180]."""
    ang_to = math.degrees(math.atan2(to_ac["y"] - from_ac["y"], to_ac["x"] - from_ac["x"]))
    return shortest_diff_deg(from_ac["heading"], norm_angle_deg(ang_to))

def is_head_on(a, b, tol=15.0):
    """وجهاً لوجه ~ الهدف أمام كلٍ منهما ضمن ±tol درجة، مع اقتراب."""
    rb_a = rel_bearing(a, b)
    rb_b = rel_bearing(b, a)
    closing = True  # يمكن تحسينه بـ TCPA، لكن لدينا حساب TCPA أصلاً أدناه
    return abs(rb_a) <= tol and abs(rb_b) <= tol and closing

def is_overtaking(overtaker, overtaken):
    """
    تعريف Annex 2: التجاوز عندما تأتي من الخلف ضمن مخروط 70° خلفي.
    أي أن الطائرة المتجاوزة تقع بالنسبة للمتجاوَزة في قطاع خلفي: |bearing| >= 110°.
    """
    rb_to_overtaker = rel_bearing(overtaken, overtaker)   # موقع المتجاوزة بالنسبة للمتجاوَزة
    return abs(rb_to_overtaker) >= 110.0  # 180-70 = 110

def is_converging(a, b):
    """تقاطع إذا لم يكن وجهاً لوجه ولا تجاوز، وكل طرف يرى الآخر في المقدّمة (أمام الجناح)."""
    if is_head_on(a, b) or is_overtaking(a, b) or is_overtaking(b, a):
        return False
    rb_a = rel_bearing(a, b)
    rb_b = rel_bearing(b, a)
    # أمام الجناح تقريباً: |bearing| < 110°
    return abs(rb_a) < 110.0 and abs(rb_b) < 110.0

def category_priority(ac):
    """
    أولوية الفئات (اختياري): بالون > شراعية > منطاد > محرك.
    استخدم ac.get("category") إن وُجدت: 'BALLOON','GLIDER','AIRSHIP','AIRPLANE'
    الأصغر قيمة = أعلى أولوية.
    """
    order = {"BALLOON": 0, "GLIDER": 1, "AIRSHIP": 2, "AIRPLANE": 3}
    return order.get(ac.get("category", "AIRPLANE"), 3)

def set_avoid(ac, heading, cooldown):
    ac["mode"] = "AVOID"
    ac["avoid_heading"] = norm_angle_deg(heading)
    ac["avoid_cooldown"] = cooldown

# ---------------- تحميل المطارات (اختياري) ----------------
def load_airports(filename):
    airports = []
    if not os.path.exists(filename):
        # إذا لم يوجد ملف، نعيد بعض مطارات تجريبية داخل الخريطة
        return [
            {"id":"APT1","name":"Sample A","x":150,"y":300},
            {"id":"APT2","name":"Sample B","x":650,"y":250},
            {"id":"APT3","name":"Sample C","x":400,"y":500},
        ]
    with open(filename, encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                lat = float(row["latitude_deg"]); lon = float(row["longitude_deg"])
                ident = row.get("ident", row.get("icao", "UNK"))
                name = row.get("name","")
                x, y = geo_to_screen(lat, lon)
                if 0 <= x <= WIDTH and 0 <= y <= HEIGHT:
                    airports.append({"id": ident, "name": name, "x": x, "y": y})
            except Exception:
                continue
    return airports

# ---------------- تحميل الخريطة ----------------
try:
    background = pygame.image.load(MAP_IMAGE)
    background = pygame.transform.scale(background, (WIDTH, HEIGHT))
except Exception:
    background = None

# ---------------- كائنات المحاكاة ----------------
airports = load_airports(AIRPORTS_FILE)

aircraft = []           # قائمة الطائرات
selected_airports = []  # فهارس المطارات المختارة (بالمؤشر)
selected_ac = None  

def aircraft_thread(ac):
    dt = 0.1
    while True:
        # تحريك الطائرة
        arrived = move_aircraft(ac, dt)
        if arrived:
            
            with aircraft_lock:
                if ac in aircraft:
                    aircraft.remove(ac)
            break

        # كشف التعارضات لجميع الطائرات
        with aircraft_lock:
            check_conflicts(dt)

        # تحديث كولداون تجنب الاصطدام
        if ac["mode"] == "AVOID":
            ac["avoid_cooldown"] = max(0.0, ac.get("avoid_cooldown",0.0) - dt)

        time.sleep(dt)


def spawn_aircraft(start_idx, dest_idx, speed=None):
    start = airports[start_idx]; dest = airports[dest_idx]
    ac = {
        "x": float(start["x"]),
        "y": float(start["y"]),
        "dest": (float(dest["x"]), float(dest["y"])),
        "speed": 5,
        "heading": 0.0,
        "normal_heading": 0.0,
        "mode": "NORMAL",        # NORMAL / AVOID
        "avoid_heading": 0.0,
        "avoid_cooldown": 0.0,
        "color": (255, 0, 0),
        "path": [],
        "altitude": 3000.0,     # ← ارتفاع ابتدائي (قدم)
        "control": "AUTO" ,
        "planned_path": [(100, 200), (400, 250), (700, 500)],   # ← البداية والنهاية
 
    }
    # set initial heading toward dest
    dx = ac["dest"][0] - ac["x"]; dy = ac["dest"][1] - ac["y"]
    ac["heading"] = ac["normal_heading"] = norm_angle_deg(math.degrees(math.atan2(dy, dx)))
    aircraft.append(ac)

     # إنشاء Thread للطائرة
    t = threading.Thread(target=aircraft_thread, args=(ac,))
    t.daemon = True  # يغلق مع البرنامج
    t.start()

# ---------------- رسم helpers ----------------
def draw_weather():
    for zone in weather_zones:
        # --- الغيوم ---
        width = zone["x2"] - zone["x1"]
        height = zone["y2"] - zone["y1"]
        cloud_surface = pygame.Surface((width, height), pygame.SRCALPHA)

        for cloud in zone["clouds"]:
            intensity = int((1 - zone["visibility"]) * 180)
            base_alpha = min(255, cloud["alpha"] + intensity)
            for r_offset in range(20):
                radius = cloud["r"] - r_offset * 5
                if radius > 0:
                    alpha = max(30, base_alpha - r_offset * 40)
                    color = (200, 200, 255, alpha)
                    pygame.draw.circle(
                        cloud_surface,
                        color,
                        (int(cloud["x"]), int(cloud["y"])),
                        radius
                    )

        screen.blit(cloud_surface, (zone["x1"], zone["y1"]))

        # --- الضباب ---
        if zone.get("fog_density", 0) > 0:
            fog_alpha = int(zone["fog_density"] * 120)
            fog = pygame.Surface((width, height), pygame.SRCALPHA)
            fog.fill((200, 200, 200, fog_alpha))
            screen.blit(fog, (zone["x1"], zone["y1"]))

        # --- المطر ---
        if zone.get("rain_intensity", 0) > 0 and zone["temperature"] > 0:
            for _ in range(20 * zone["rain_intensity"]):
                rx = random.randint(zone["x1"], zone["x2"])
                ry = random.randint(zone["y1"], zone["y2"])
                pygame.draw.line(
                    screen, (150, 150, 255, 160),
                    (rx, ry), (rx + int(zone["wind_dx"]*3), ry + 12), 1
                )

        # --- الثلج ---
        if zone.get("snow_intensity", 0) > 0 and zone["temperature"] <= 0:
            for _ in range(15 * zone["snow_intensity"]):
                sx = random.randint(zone["x1"], zone["x2"])
                sy = random.randint(zone["y1"], zone["y2"])
                pygame.draw.circle(screen, (255, 255, 255, 200), (sx, sy), 2)

        # --- البرق ---
        if zone.get("storm", False):
            zone["lightning_timer"] -= 1
            if zone["lightning_timer"] <= 0 and random.random() < 0.01:
                zone["lightning_timer"] = 5
            if zone["lightning_timer"] > 0:
                flash = pygame.Surface((width, height), pygame.SRCALPHA)
                flash.fill((255, 255, 255, 80))
                screen.blit(flash, (zone["x1"], zone["y1"]))

def draw_dashed_line(surface, color, start_pos, end_pos, width=2, dash_len=10, gap_len=6):
    x1, y1 = start_pos; x2, y2 = end_pos
    dx, dy = x2 - x1, y2 - y1
    dist = math.hypot(dx, dy)
    if dist == 0: return
    vx, vy = dx / dist, dy / dist
    pos = 0.0
    while pos < dist:
        seg_end = min(pos + dash_len, dist)
        sx, sy = x1 + vx * pos,     y1 + vy * pos
        ex, ey = x1 + vx * seg_end, y1 + vy * seg_end
        pygame.draw.line(surface, color, (sx, sy), (ex, ey), width)
        pos += dash_len + gap_len

def planned_polyline(ac):
    pts = [(ac["x"], ac["y"])]
    if ac["mode"] == "AVOID" and ac.get("avoid_cooldown", 0) > 0:
        t = ac["avoid_cooldown"]
        r = math.radians(ac["avoid_heading"])
        p1 = (ac["x"] + ac["speed"] * t * math.cos(r),
              ac["y"] + ac["speed"] * t * math.sin(r))
        pts.append(p1)
    pts.append(ac["dest"])
    return pts

# ---------------- CPA / TCPA / DCPA ----------------
def compute_tcpa_dcpa(a, b):
    rx = b["x"] - a["x"]; ry = b["y"] - a["y"]
    vax = a["speed"] * math.cos(math.radians(a["heading"]))
    vay = a["speed"] * math.sin(math.radians(a["heading"]))
    vbx = b["speed"] * math.cos(math.radians(b["heading"]))
    vby = b["speed"] * math.sin(math.radians(b["heading"]))
    rvx = vbx - vax; rvy = vby - vay
    rv2 = rvx*rvx + rvy*rvy
    if rv2 < 1e-9:
        return float('inf'), math.hypot(rx, ry)
    tcpa = -(rx*rvx + ry*rvy) / rv2
    closest_x = rx + rvx * tcpa
    closest_y = ry + rvy * tcpa
    dcpa = math.hypot(closest_x, closest_y)
    return tcpa, dcpa

# ---------------- محاكاة قصيرة المدى مع معدل دوران ----------------
def simulate_min_distance_with_turns(a, b, target_ha, target_hb, sim_time=SIM_TIME, steps=SIM_STEPS):
    dt_sim = sim_time / steps
    ax, ay = a["x"], a["y"]; bx, by = b["x"], b["y"]
    ha, hb = a["heading"], b["heading"]
    va, vb = a["speed"], b["speed"]
    min_d = float('inf')
    for _ in range(steps):
        max_turn = TURN_RATE_DPS * dt_sim
        ha = norm_angle_deg(ha + math.copysign(min(abs(shortest_diff_deg(ha, target_ha)), max_turn), shortest_diff_deg(ha, target_ha)))
        hb = norm_angle_deg(hb + math.copysign(min(abs(shortest_diff_deg(hb, target_hb)), max_turn), shortest_diff_deg(hb, target_hb)))
        ax += va * dt_sim * math.cos(math.radians(ha))
        ay += va * dt_sim * math.sin(math.radians(ha))
        bx += vb * dt_sim * math.cos(math.radians(hb))
        by += vb * dt_sim * math.sin(math.radians(hb))
        d = math.hypot(ax - bx, ay - by)
        if d < min_d: min_d = d
    return min_d

# ---------------- رسم الطائرة ----------------
def draw_aircraft(ac):
    plan_pts = planned_polyline(ac)
    for i in range(len(plan_pts) - 1):
        draw_dashed_line(screen, (0, 160, 0), plan_pts[i], plan_pts[i+1], width=2, dash_len=10, gap_len=6)
    if len(ac["path"]) > 1:
        pygame.draw.lines(screen, (120, 120, 120), False, ac["path"], 2)
    x, y, hdg = ac["x"], ac["y"], ac["heading"]
    # تدوير الصورة حسب الاتجاه
    rotated_img = pygame.transform.rotate(aircraft_img, -hdg)

    # اجعل مركز الصورة هو موقع الطائرة
    rect = rotated_img.get_rect(center=(x, y))

    # عرض الصورة على الشاشة
    screen.blit(rotated_img, rect.topleft)

def draw_sidebar(ac):
    # لوحة على اليسار بمظهر جذاب
    panel_w, panel_h = 300, 300
    pad_x, pad_y = 12, 12
    panel_surf = pygame.Surface((panel_w, panel_h), pygame.SRCALPHA)
    # خلفية شبه شفافة داكنة
    panel_surf.fill((10, 14, 25, 200))
    # حد أبيض خفيف مع انحناء
    pygame.draw.rect(panel_surf, (255,255,255,30), (0,0,panel_w, panel_h), border_radius=12, width=1)

    # عنوان
    title = font_bold.render("Aircraft • معلومات الطائرة", True, (235,235,240))
    panel_surf.blit(title, (pad_x, pad_y))

    # رمز طيارة بسيط (مثل أيقونة)
    pygame.draw.polygon(panel_surf, (120,180,255), [(panel_w-46, 18),(panel_w-28, 26),(panel_w-46,34)])
    pygame.draw.line(panel_surf, (120,180,255), (panel_w-40,26),(panel_w-18,26),2)

    # بيانات (label + value)
    rows = [
        ("Speed", f"{ac['speed']:.1f} px/s"),
        ("Altitude", f"{ac['altitude']:.0f} ft"),
        ("Heading", f"{ac['heading']:.1f}°"),
        ("Mode", ac["mode"]),
        ("Control", ac["control"]),
        ("Coords", f"{ac['x']:.1f}, {ac['y']:.1f}"),
    ]

    y = pad_y + 36
    for label_text, val_text in rows:
        lab = font_small.render(label_text, True, (170,180,200))
        val = font_large.render(val_text, True, (245,245,250))
        panel_surf.blit(lab, (pad_x, y))
        panel_surf.blit(val, (pad_x + 110, y-2))
        y += 28

    # مثال لشريط سرعة بصري (progress bar style) — يظهر نسبياً
    # احسب نسبة سرعة (قابلة للتعديل حسب مدى مقبول)
    max_speed = max(200.0, ac.get("speed", 1.0) * 2.0)
    speed_frac = min(1.0, ac["speed"] / max_speed)
    bar_x, bar_y = pad_x, panel_h - 40
    bar_w, bar_h = panel_w - 2*pad_x, 14
    # خلفية الشريط
    pygame.draw.rect(panel_surf, (60,60,70), (bar_x, bar_y, bar_w, bar_h), border_radius=6)
    # تعبئة الشريط
    pygame.draw.rect(panel_surf, (90,200,140), (bar_x, bar_y, int(bar_w*speed_frac), bar_h), border_radius=6)

    # تسمية الشريط
    lbl = font_small.render("Speed bar", True, (160,170,185))
    panel_surf.blit(lbl, (bar_x, bar_y-18))

    # النهائية: ارجع اللوحة لترسم في المكان المطلوب
    screen.blit(panel_surf, (10, 12))


# ---------------- حركة الطائرة (محدثة) ----------------
def move_aircraft(ac, dt):
    if ac["control"] == "AUTO":
        dx = ac["dest"][0] - ac["x"]; dy = ac["dest"][1] - ac["y"]
        dist_to_dest = math.hypot(dx, dy)
        ac["normal_heading"] = norm_angle_deg(math.degrees(math.atan2(dy, dx)))
        if ac["mode"] == "AVOID":
            commanded = ac["avoid_heading"]
        else:
            commanded = ac["normal_heading"]

        delta = shortest_diff_deg(ac["heading"], commanded)
        max_step = TURN_RATE_DPS * dt
        if abs(delta) <= max_step:
            ac["heading"] = commanded
        else:
            ac["heading"] = norm_angle_deg(ac["heading"] + math.copysign(max_step, delta))
    # MANUAL → ما نغيّرش heading إلا لو المستخدم عدله
    rad = math.radians(ac["heading"])
    ac["x"] += ac["speed"] * dt * math.cos(rad)
    ac["y"] += ac["speed"] * dt * math.sin(rad)

    ac["path"].append((ac["x"], ac["y"]))

    if ac["control"] == "AUTO" and dist_to_dest < 10:
        return True
    return False


# ---------------- كشف التعارض وتطبيق حلول ATC ----------------
def check_conflicts(dt):
    n = len(aircraft)
    for i in range(n):
        for j in range(i+1, n):
            a, b = aircraft[i], aircraft[j]
            tcpa, dcpa = compute_tcpa_dcpa(a, b)

            # حالة عدم وجود تهديد: فك وضع التفادي
            if not (0 <= tcpa <= LOOKAHEAD and dcpa < AVOID_DIST):
                if a["mode"] == "AVOID" and a.get("avoid_cooldown", 0.0) <= 0.0:
                    a["mode"] = "NORMAL"
                if b["mode"] == "AVOID" and b.get("avoid_cooldown", 0.0) <= 0.0:
                    b["mode"] = "NORMAL"
                continue

            # تحديد نوع التهديد
            heading_diff = abs(shortest_diff_deg(a["heading"], b["heading"]))

            # 1. حالة وجهاً لوجه
            if heading_diff > 150:
                # كلا الطائرتين تتخذان إجراء وقائي بالانعطاف لليمين
                ha = norm_angle_deg(a["heading"] + AVOID_ANGLE_HEAD_ON)
                hb = norm_angle_deg(b["heading"] + AVOID_ANGLE_HEAD_ON)
                if a["mode"] != "AVOID":
                    a["mode"] = "AVOID"; a["avoid_heading"] = ha; a["avoid_cooldown"] = AVOID_COOLDOWN_HEAD_ON
                if b["mode"] != "AVOID":
                    b["mode"] = "AVOID"; b["avoid_heading"] = hb; b["avoid_cooldown"] = AVOID_COOLDOWN_HEAD_ON
            
            # 2. حالة التقاطع - تطبيق قاعدة إعطاء الطريق
            else:
                angle_ab = math.degrees(math.atan2(b["y"] - a["y"], b["x"] - a["x"]))
                diff_a = shortest_diff_deg(a["heading"], angle_ab)
                
                give_way_a = False
                give_way_b = False

                if diff_a < 0: # b على يمين a
                    give_way_a = True
                elif diff_a > 0: # b على يسار a
                    give_way_b = True
                
                # إعطاء الأولوية للطائرة الأسرع لعدم تغيير مسارها
                if give_way_a and give_way_b:
                    if a["speed"] > b["speed"]:
                        give_way_b = False
                    else:
                        give_way_a = False

                if give_way_a and a["mode"] != "AVOID":
                    cur_min = simulate_min_distance_with_turns(a, b, a["heading"], b["heading"])
                    best_min = cur_min; best_ha = None
                    for d in (-AVOID_ANGLE_CROSSING, AVOID_ANGLE_CROSSING):
                        cand_ha = norm_angle_deg(a["heading"] + d)
                        mnd = simulate_min_distance_with_turns(a, b, cand_ha, b["heading"])
                        if mnd > best_min + MIN_IMPROVEMENT:
                            best_min = mnd; best_ha = cand_ha
                    if best_ha is not None:
                        a["mode"] = "AVOID"; a["avoid_heading"] = best_ha; a["avoid_cooldown"] = AVOID_COOLDOWN_CROSSING
                    else:
                        a["mode"] = "AVOID"; a["avoid_heading"] = norm_angle_deg(a["heading"] + AVOID_ANGLE_CROSSING); a["avoid_cooldown"] = AVOID_COOLDOWN_CROSSING

                elif give_way_b and b["mode"] != "AVOID":
                    cur_min = simulate_min_distance_with_turns(a, b, a["heading"], b["heading"])
                    best_min = cur_min; best_hb = None
                    for d in (-AVOID_ANGLE_CROSSING, AVOID_ANGLE_CROSSING):
                        cand_hb = norm_angle_deg(b["heading"] + d)
                        mnd = simulate_min_distance_with_turns(a, b, a["heading"], cand_hb)
                        if mnd > best_min + MIN_IMPROVEMENT:
                            best_min = mnd; best_hb = cand_hb
                    if best_hb is not None:
                        b["mode"] = "AVOID"; b["avoid_heading"] = best_hb; b["avoid_cooldown"] = AVOID_COOLDOWN_CROSSING
                    else:
                        b["mode"] = "AVOID"; b["avoid_heading"] = norm_angle_deg(b["heading"] + AVOID_ANGLE_CROSSING); b["avoid_cooldown"] = AVOID_COOLDOWN_CROSSING
    for ac in aircraft:
     wx = get_weather_at(ac["x"], ac["y"])
     if not wx:
        continue  # الطائرة ليست داخل أي طقس

    # نحدد خطورة الطقس
     bad_weather = wx.get("storm", False) or wx.get("visibility", 1.0) < 0.4 or wx.get("turbulence", 0) > 2.5

     if bad_weather:
        if ac["mode"] != "AVOID":
            # مركز الطقس
            cx = (wx["x1"] + wx["x2"]) / 2
            cy = (wx["y1"] + wx["y2"]) / 2

            # زاوية من الطائرة إلى مركز الطقس
            angle_to_wx = math.degrees(math.atan2(cy - ac["y"], cx - ac["x"]))
            diff = shortest_diff_deg(ac["heading"], angle_to_wx)

            # نحدد الاتجاه لتفادي الطقس
            if wx.get("storm", False):
                # عاصفة → انعطاف أعرض
                avoid_angle = 45 if diff <= 0 else -45
                cooldown = 15.0
            else:
                # ضباب/اضطراب → انحراف أصغر
                avoid_angle = 20 if diff <= 0 else -20
                cooldown = 8.0

            ac["mode"] = "AVOID"
            ac["avoid_heading"] = norm_angle_deg(ac["heading"] + avoid_angle)
            ac["avoid_cooldown"] = cooldown
            ac["avoid_reason"] = "WEATHER"

     else:
        # رجوع للوضع الطبيعي
        if ac["mode"] == "AVOID" and ac.get("avoid_reason") == "WEATHER":
            ac["mode"] = "NORMAL"
            ac.pop("avoid_reason", None)

def spawn_random_aircraft():
    # نختار مطار البداية والنهاية عشوائياً
    start_airport = random.choice(airports)
    end_airport = random.choice(airports)
    while end_airport == start_airport:  # نتأكد أنها ليست نفس المطار
        end_airport = random.choice(airports)
    
    speed = random.uniform(100, 200)  # سرعة عشوائية للطائرة
    spawn_aircraft(airports.index(start_airport), airports.index(end_airport), speed=speed)

def spawn_random_weather():
    x1 = random.randint(0, 700)
    y1 = random.randint(0, 500)
    width = random.randint(150, 250)
    height = random.randint(150, 250)

    temp = random.uniform(-5, 30)
    zone = {
        "x1": x1,
        "y1": y1,
        "x2": x1 + width,
        "y2": y1 + height,
        "wind_dx": 0.0,
        "wind_dy": 0.0,
        "turbulence": random.uniform(0.5, 3.0),
        "visibility": random.uniform(0.3, 1.0),
        "fog_density": random.uniform(0.0, 0.7),
        "rain_intensity": random.randint(0, 2),
        "snow_intensity": random.randint(0, 1),
        "storm": random.random() < 0.2,
        "lightning_timer": 0,
        "clouds": [],
        "temperature": temp,
        "pressure": random.uniform(990, 1025),
        "lifetime": random.randint(500, 1500)  # عمر المنطقة (عدد التحديثات)
    }

    for _ in range(25):
        zone["clouds"].append({
            "x": random.randint(0, width),
            "y": random.randint(0, height),
            "r": random.randint(25, 60),
            "alpha": random.randint(80, 160)
        })

    weather_zones.append(zone)


def update_weather(dt):
    # تحديث الطقس + إزالة المناطق المنتهية
    zones_to_remove = []

    for zone in weather_zones:
        # قلل العمر
        if "lifetime" in zone:
            zone["lifetime"] -= 1
            if zone["lifetime"] <= 0:
                zones_to_remove.append(zone)
                continue

        # --- تحديث الرياح والضغط والحرارة ---
        wind_speed = random.uniform(5, 20)  # عقدة
        wind_dir = random.uniform(0, 360)   # درجات
        rad = math.radians(wind_dir)
        zone["wind_dx"] = math.cos(rad) * (wind_speed / 10.0)
        zone["wind_dy"] = math.sin(rad) * (wind_speed / 10.0)

        zone["temperature"] += random.uniform(-0.05, 0.05)
        zone["pressure"] += random.uniform(-0.2, 0.2)

        # --- تحريك الغيوم ---
        for cloud in zone["clouds"]:
            cloud["x"] += zone["wind_dx"] * 5 * dt + random.uniform(-0.3, 0.3)
            cloud["y"] += zone["wind_dy"] * 5 * dt + random.uniform(-0.3, 0.3)

            width = zone["x2"] - zone["x1"]
            height = zone["y2"] - zone["y1"]

            if cloud["x"] > width: cloud["x"] = 0
            if cloud["y"] > height: cloud["y"] = 0
            if cloud["x"] < 0: cloud["x"] = width
            if cloud["y"] < 0: cloud["y"] = height

    # إزالة المناطق المنتهية
    for zone in zones_to_remove:
        weather_zones.remove(zone)

# ---------------- الحلقة الرئيسية ----------------
running = True
while running:
    dt = clock.tick(60) / 1000.0  # زمن الإطار بالثواني
    spawn_timer += dt
    weather_timer += dt  # dt = الوقت بين الفريمات
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.MOUSEBUTTONDOWN:
            mx,my = pygame.mouse.get_pos()
            for i, ap in enumerate(airports):
                if math.hypot(mx - ap["x"], my - ap["y"]) < 8:
                    if len(selected_airports) < 2:
                        selected_airports.append(i)

        # لو ضغطت على طيارة
            for ac in aircraft:
               if math.hypot(mx - ac["x"], my - ac["y"]) < 12:
                  selected_ac = ac
                  print("✈️ Selected aircraft!")


        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_RETURN and len(selected_airports) == 2:
                spawn_aircraft(selected_airports[0], selected_airports[1], speed=DEFAULT_SPEED)
                selected_airports = []

            if selected_ac:
              if event.key == pygame.K_UP:
                selected_ac["speed"] += 10.0
              if event.key == pygame.K_DOWN:
                selected_ac["speed"] = max(10.0, selected_ac["speed"] - 10.0)


            # Ctrl + Right/Left adjusts AVOID_ANGLE
            if event.key == pygame.K_RIGHT and pygame.key.get_mods() & pygame.KMOD_CTRL:
                AVOID_ANGLE = min(60.0, AVOID_ANGLE + 5.0); print("AVOID_ANGLE =", AVOID_ANGLE)
            if event.key == pygame.K_LEFT and pygame.key.get_mods() & pygame.KMOD_CTRL:
                AVOID_ANGLE = max(5.0, AVOID_ANGLE - 5.0); print("AVOID_ANGLE =", AVOID_ANGLE)

            if selected_ac:
               if selected_ac["control"] == "MANUAL":
                  if event.key == pygame.K_UP:
                    selected_ac["speed"] += 10.0
                  if event.key == pygame.K_DOWN:
                    selected_ac["speed"] = max(10.0, selected_ac["speed"] - 10.0)
                  if event.key == pygame.K_s:
                      selected_ac["altitude"] += 500
                  if event.key == pygame.K_w:
                     selected_ac["altitude"] = max(0, selected_ac["altitude"] - 500)
                  if event.key == pygame.K_LEFT:
                     selected_ac["heading"] = norm_angle_deg(selected_ac["heading"] - 10)
                  if event.key == pygame.K_RIGHT:
                     selected_ac["heading"] = norm_angle_deg(selected_ac["heading"] + 10)
          
    # مفتاح M للتبديل بين AUTO / MANUAL
            if event.key == pygame.K_m:
                if selected_ac["control"] == "AUTO":
                     selected_ac["control"] = "MANUAL"
                     print("Switched to MANUAL control")
                else:
                   selected_ac["control"] = "AUTO"
                   print("Switched to AUTOPILOT")
    # ---------------- توليد طائرات عشوائية ----------------
    if spawn_timer >= 5.0:  # كل 5 ثوانٍ تولد طائرة جديدة
        start_ap = random.choice(airports)
        end_ap = random.choice(airports)
        while end_ap == start_ap:
            end_ap = random.choice(airports)
        speed = random.uniform(100, 200)
        spawn_aircraft(airports.index(start_ap), airports.index(end_ap), speed=speed)
        spawn_timer = 0.0

    if weather_timer >= 5.0:  # كل 10 ثوانٍ تولد منطقة طقس جديدة
         spawn_random_weather()
         weather_timer = 0.0

    # 1) نخفض الكولداون لكل الطائرات
    for ac in aircraft:
        if ac.get("avoid_cooldown", 0.0) > 0.0:
            ac["avoid_cooldown"] = max(0.0, ac["avoid_cooldown"] - dt)

    # 2) كشف التعارضات (يصدر أوامر AVOID إن لزم)
    check_conflicts(dt)

    # 3) حرك الطائرات
    for ac in aircraft[:]:
        if move_aircraft(ac, dt):
            aircraft.remove(ac)
            print("✈️ الطائرة وصلت المطار وحُذفت من المحاكاة")

    # ---------------- الرسم ----------------
    if background:
        screen.blit(background, (0, 0))
    else:
        screen.fill((240, 240, 240))

    # 🌥️ رسم الطقس
    draw_weather()
    update_weather(dt)
    
    # المطارات
    for i, ap in enumerate(airports):
        color = (0, 255, 0) if i in selected_airports else (0, 0, 255)
        pygame.draw.circle(screen, color, (ap["x"], ap["y"]), 6)
        label = font_small.render(ap["id"], True, (0, 0, 0))
        screen.blit(label, (ap["x"] + 8, ap["y"] - 8))

    # الطائرات
    for ac in aircraft:
        draw_aircraft(ac)
    if selected_ac:
     if selected_ac:
       draw_sidebar(selected_ac)


    pygame.display.flip()

pygame.quit()
sys.exit()
