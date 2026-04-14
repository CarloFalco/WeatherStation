"""
generate_docs.py  –  WeatherStation PDF documentation generator
================================================================
Generates a self-contained PDF API reference with:
  - Clickable Table of Contents
  - PDF outline sidebar (bookmarks tree)
  - Clickable external URLs
  - Internal cross-references

Usage:
    python generate_docs.py
    python generate_docs.py --out my.pdf
"""

import os
import re
import sys
import argparse
from datetime import date

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import cm
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_LEFT, TA_CENTER, TA_JUSTIFY
from reportlab.lib import colors
from reportlab.platypus import (
    BaseDocTemplate, PageTemplate, Frame,
    Paragraph, Spacer, PageBreak,
    Table, TableStyle, HRFlowable,
    KeepTogether, Flowable,
)
from reportlab.lib.colors import HexColor

# ---------------------------------------------------------------------------
# Project paths
# ---------------------------------------------------------------------------
SCRIPT_DIR  = os.path.abspath(os.path.dirname(__file__))
INCLUDE_DIR = os.path.join(SCRIPT_DIR, "include")
SRC_DIR     = os.path.join(SCRIPT_DIR, "src")
VERSION_H   = os.path.join(INCLUDE_DIR, "version.h")
DOCS_DIR    = os.path.join(SCRIPT_DIR, "docs")

# ---------------------------------------------------------------------------
# Colours
# ---------------------------------------------------------------------------
C_NAVY       = HexColor("#1e3a5f")
C_BLUE       = HexColor("#2563eb")
C_BLUE_LIGHT = HexColor("#dbeafe")
C_TEAL       = HexColor("#0e7490")
C_GREY_DARK  = HexColor("#374151")
C_GREY_MID   = HexColor("#6b7280")
C_GREY_LIGHT = HexColor("#f3f4f6")
C_WHITE      = colors.white
C_GREEN      = HexColor("#16a34a")
C_LINK       = HexColor("#1d4ed8")

PAGE_W, PAGE_H = A4
MARGIN = 2 * cm

# ===========================================================================
# Bookmark flowable
# Inserts an invisible anchor into the PDF flow AND registers an outline entry
# ===========================================================================
class Bookmark(Flowable):
    """Zero-height flowable that creates a named PDF destination + outline entry."""

    def __init__(self, key, title, level=0):
        Flowable.__init__(self)
        self.key   = key
        self.title = title
        self.level = level   # 0 = chapter, 1 = section, 2 = subsection
        self.width = self.height = 0

    def wrap(self, available_width, available_height):
        return 0, 0

    def draw(self):
        self.canv.bookmarkPage(self.key)
        self.canv.addOutlineEntry(self.title, self.key, self.level, closed=(self.level == 0))


def bm(key, title, level=0):
    """Shorthand: return a Bookmark flowable."""
    return Bookmark(key, title, level)


# ===========================================================================
# Helpers
# ===========================================================================
def slug(text):
    """Make a safe bookmark key from arbitrary text."""
    return re.sub(r"[^a-zA-Z0-9_]", "_", text).lower()


def read_define(filepath, name, default="?"):
    pattern = re.compile(
        r'^\s*#define\s+' + re.escape(name) + r'\s+"?([^"\n]+)"?'
    )
    try:
        with open(filepath, encoding="utf-8", errors="ignore") as f:
            for line in f:
                m = pattern.match(line)
                if m:
                    return m.group(1).strip().strip('"')
    except FileNotFoundError:
        pass
    return default


_DOXBLOCK = re.compile(r'/\*\*\s*(.*?)\s*\*/', re.DOTALL)
_BRIEF    = re.compile(r'@brief\s+(.+?)(?=\n\s*\*\s*@|\n\s*\*/)', re.DOTALL)
_PARAM    = re.compile(r'@param\s+(\w+)\s+(.+?)(?=\n\s*\*\s*@|\n\s*\*/)', re.DOTALL)
_RETURN   = re.compile(r'@return\s+(.+?)(?=\n\s*\*\s*@|\n\s*\*/)', re.DOTALL)
_FILE     = re.compile(r'@file\s+(\S+)')
_NOTE     = re.compile(r'@note\s+(.+?)(?=\n\s*\*\s*@|\n\s*\*/)', re.DOTALL)


def clean(text):
    lines = []
    for line in text.splitlines():
        line = re.sub(r'^\s*\*\s?', '', line)
        lines.append(line.strip())
    return ' '.join(l for l in lines if l)


def parse_file(filepath):
    result = {"path": filepath,
              "filename": os.path.basename(filepath),
              "file_brief": "", "file_note": "", "entities": []}
    try:
        src = open(filepath, encoding="utf-8", errors="ignore").read()
    except FileNotFoundError:
        return result

    for raw in _DOXBLOCK.findall(src):
        if _FILE.search(raw):
            br = _BRIEF.search(raw)
            nt = _NOTE.search(raw)
            result["file_brief"] = clean(br.group(1)) if br else ""
            result["file_note"]  = clean(nt.group(1)) if nt else ""
            continue
        brief   = clean(_BRIEF.search(raw).group(1)) if _BRIEF.search(raw) else ""
        if not brief:
            brief = clean(raw.split('\n')[0])
        params  = [(m.group(1), clean(m.group(2))) for m in _PARAM.finditer(raw)]
        returns = clean(_RETURN.search(raw).group(1)) if _RETURN.search(raw) else ""
        notes   = clean(_NOTE.search(raw).group(1))  if _NOTE.search(raw)   else ""
        if brief:
            result["entities"].append({"brief": brief, "params": params,
                                        "returns": returns, "notes": notes})
    return result


def collect_files():
    docs = []
    for base, label in [(INCLUDE_DIR, "include"), (SRC_DIR, "src")]:
        for root, _, files in os.walk(base):
            for fname in sorted(files):
                if fname.endswith(('.h', '.cpp')):
                    full = os.path.join(root, fname)
                    info = parse_file(full)
                    info["label"]    = label
                    info["rel_path"] = os.path.relpath(full, SCRIPT_DIR)
                    info["anchor"]   = slug(info["rel_path"])
                    docs.append(info)
    return docs


# ===========================================================================
# Styles
# ===========================================================================
def build_styles():
    S = {}

    def s(name, **kw):
        S[name] = ParagraphStyle(name, **kw)

    s("cover_title",  fontSize=30, textColor=C_WHITE,      fontName="Helvetica-Bold",  alignment=TA_CENTER, spaceAfter=8)
    s("cover_sub",    fontSize=13, textColor=HexColor("#bfdbfe"), fontName="Helvetica", alignment=TA_CENTER, spaceAfter=4)
    s("cover_meta_k", fontSize=9.5,textColor=C_NAVY,       fontName="Helvetica-Bold",  leading=14)
    s("cover_meta_v", fontSize=9.5,textColor=C_GREY_DARK,  fontName="Helvetica",       leading=14)

    s("h1",  fontSize=17, textColor=C_NAVY, fontName="Helvetica-Bold",  spaceBefore=16, spaceAfter=5,  leading=21)
    s("h2",  fontSize=13, textColor=C_BLUE, fontName="Helvetica-Bold",  spaceBefore=12, spaceAfter=4,  leading=16)
    s("h3",  fontSize=11, textColor=C_TEAL, fontName="Helvetica-Bold",  spaceBefore=8,  spaceAfter=3,  leading=14)

    s("body",       fontSize=9.5, textColor=C_GREY_DARK, fontName="Helvetica", alignment=TA_JUSTIFY, leading=14, spaceAfter=4)
    s("body_small", fontSize=8.5, textColor=C_GREY_MID,  fontName="Helvetica", leading=12, spaceAfter=3)
    s("brief",      fontSize=9.5, textColor=C_GREY_DARK, fontName="Helvetica-Oblique", leading=13, spaceAfter=3)
    s("filepath",   fontSize=8.5, textColor=C_TEAL,      fontName="Courier-Bold", spaceBefore=10, spaceAfter=2)

    # TOC styles with link colour
    s("toc_h1", fontSize=11, fontName="Helvetica-Bold", textColor=C_NAVY,      spaceBefore=5, spaceAfter=1)
    s("toc_h2", fontSize=9.5,fontName="Helvetica",      textColor=C_GREY_DARK, leftIndent=14, spaceAfter=1)
    s("toc_h3", fontSize=8.5,fontName="Helvetica",      textColor=C_GREY_MID,  leftIndent=28, spaceAfter=1)

    # Link style (used inline via <a href=...>)
    s("link",   fontSize=9.5, textColor=C_LINK, fontName="Helvetica", leading=14)

    return S


# ===========================================================================
# Page template with header / footer
# ===========================================================================
def make_doc(output_path, version):
    def header_footer(canvas, doc):
        canvas.saveState()
        # Header
        canvas.setFillColor(C_NAVY)
        canvas.rect(MARGIN, PAGE_H - MARGIN - 0.4*cm,
                    PAGE_W - 2*MARGIN, 0.4*cm, fill=1, stroke=0)
        canvas.setFont("Helvetica-Bold", 7)
        canvas.setFillColor(C_WHITE)
        canvas.drawString(MARGIN + 4, PAGE_H - MARGIN - 0.28*cm,
                          "WeatherStation Firmware Documentation")
        canvas.drawRightString(PAGE_W - MARGIN - 4, PAGE_H - MARGIN - 0.28*cm,
                               "v{}".format(version))
        # Footer
        canvas.setFillColor(C_GREY_LIGHT)
        canvas.rect(MARGIN, MARGIN - 0.1*cm, PAGE_W - 2*MARGIN, 0.35*cm, fill=1, stroke=0)
        canvas.setFont("Helvetica", 7)
        canvas.setFillColor(C_GREY_MID)
        canvas.drawString(MARGIN + 4, MARGIN + 0.05*cm,
                          "© {} Carlo Falco – ESP32-S3 WeatherStation".format(date.today().year))
        canvas.drawRightString(PAGE_W - MARGIN - 4, MARGIN + 0.05*cm,
                               "Page {}".format(doc.page))
        canvas.restoreStore = canvas.restoreState
        canvas.restoreState()

    frame    = Frame(MARGIN, MARGIN + 0.5*cm,
                     PAGE_W - 2*MARGIN, PAGE_H - 2*MARGIN - 1*cm, id="main")
    template = PageTemplate(id="main", frames=[frame], onPage=header_footer)
    doc      = BaseDocTemplate(
        output_path, pagesize=A4,
        leftMargin=MARGIN, rightMargin=MARGIN,
        topMargin=MARGIN + 0.5*cm, bottomMargin=MARGIN + 0.5*cm,
        title="WeatherStation v{} – API Documentation".format(version),
        author="Carlo Falco",
        subject="ESP32-S3 Weather Station Firmware",
    )
    doc.addPageTemplates([template])
    return doc


# ===========================================================================
# Utility: standard table style
# ===========================================================================
def std_table(data, col_widths, header_color=None):
    hc = header_color or C_NAVY
    t  = Table(data, colWidths=col_widths, repeatRows=1)
    t.setStyle(TableStyle([
        ("BACKGROUND",    (0, 0), (-1,  0), hc),
        ("ROWBACKGROUNDS",(0, 1), (-1, -1), [C_WHITE, C_GREY_LIGHT]),
        ("TOPPADDING",    (0, 0), (-1, -1), 5),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 5),
        ("LEFTPADDING",   (0, 0), (-1, -1), 6),
        ("RIGHTPADDING",  (0, 0), (-1, -1), 6),
        ("VALIGN",        (0, 0), (-1, -1), "MIDDLE"),
        ("LINEBELOW",     (0, 0), (-1, -1), 0.3, HexColor("#e5e7eb")),
    ]))
    return t


# ===========================================================================
# Cover page
# ===========================================================================
def cover_page(story, S, version, description):
    story.append(Spacer(1, 1.5*cm))

    # Title block
    title_data = [[Paragraph("WeatherStation", S["cover_title"])],
                  [Paragraph("ESP32-S3 Firmware Documentation", S["cover_sub"])]]
    title_tbl  = Table(title_data, colWidths=[PAGE_W - 2*MARGIN])
    title_tbl.setStyle(TableStyle([
        ("BACKGROUND",    (0, 0), (-1, -1), C_NAVY),
        ("BACKGROUND",    (0, 1), (-1,  1), C_BLUE),
        ("TOPPADDING",    (0, 0), (-1,  0), 26),
        ("BOTTOMPADDING", (0, 0), (-1,  0), 8),
        ("TOPPADDING",    (0, 1), (-1,  1), 10),
        ("BOTTOMPADDING", (0, 1), (-1,  1), 10),
    ]))
    story.append(title_tbl)
    story.append(Spacer(1, 0.8*cm))

    # Meta table
    GH_URL   = "https://github.com/CarloFalco/WeatherStation"
    meta_kv  = [
        ("Version",     "v{}".format(version)),
        ("Description", description),
        ("MCU",         "ESP32-S3-DevKitC-1"),
        ("Framework",   "Arduino (PlatformIO)"),
        ("Generated",   date.today().strftime("%d %B %Y")),
        ("Author",      "Carlo Falco"),
        ("Repository",  '<a href="{url}" color="#1d4ed8">{url}</a>'.format(url=GH_URL)),
    ]
    hs = ParagraphStyle("mk", fontSize=9.5, fontName="Helvetica-Bold", textColor=C_NAVY)
    vs = ParagraphStyle("mv", fontSize=9.5, fontName="Helvetica",      textColor=C_GREY_DARK)
    meta_data = [[Paragraph(k, hs), Paragraph(v, vs)] for k, v in meta_kv]
    meta_tbl  = Table(meta_data, colWidths=[4*cm, PAGE_W - 2*MARGIN - 4*cm])
    meta_tbl.setStyle(TableStyle([
        ("BACKGROUND",    (0, 0), (0, -1), C_BLUE_LIGHT),
        ("BACKGROUND",    (1, 0), (1, -1), C_GREY_LIGHT),
        ("TOPPADDING",    (0, 0), (-1,-1), 6),
        ("BOTTOMPADDING", (0, 0), (-1,-1), 6),
        ("LEFTPADDING",   (0, 0), (-1,-1), 10),
        ("LINEBELOW",     (0, 0), (-1,-2), 0.5, C_WHITE),
        ("VALIGN",        (0, 0), (-1,-1), "MIDDLE"),
    ]))
    story.append(meta_tbl)
    story.append(Spacer(1, 1*cm))

    # Sensor summary
    hs2 = ParagraphStyle("sh", fontSize=8.5, fontName="Helvetica-Bold", textColor=C_WHITE)
    cs2 = ParagraphStyle("sc", fontSize=8.5, fontName="Helvetica",      textColor=C_GREY_DARK)
    sensors = [
        ["Sensor",     "Interface",         "Measurement"],
        ["BME280",     "I2C 0x76",           "Temperature / Humidity / Pressure"],
        ["AS5600",     "I2C 0x36",           "Wind direction (magnetic encoder)"],
        ["Rain gauge", "GPIO19 reed switch", "Rainfall accumulation"],
        ["Anemometer", "GPIO20 reed switch", "Wind speed"],
        ["PMSA003I",   "UART RX=17 TX=18",  "PM1.0, PM2.5, PM10"],
        ["CCS811",     "I2C 0x5A",           "eCO2, TVOC"],
        ["MICS6814",   "ADC GPIO5/6/7",      "CO, NO2, NH3"],
        ["INA3221",    "I2C 0x40",           "Voltage, Current (3 channels)"],
    ]
    s_data = []
    for i, row in enumerate(sensors):
        st = hs2 if i == 0 else cs2
        s_data.append([Paragraph(c, st) for c in row])
    cw = [(PAGE_W - 2*MARGIN) * f for f in [0.21, 0.27, 0.52]]
    story.append(Paragraph("Hardware Overview", S["h2"]))
    story.append(std_table(s_data, cw))
    story.append(PageBreak())


# ===========================================================================
# Table of Contents  – clickable entries
# ===========================================================================
def toc_page(story, S, api_files):
    story.append(bm("toc", "Table of Contents", level=0))
    story.append(Paragraph("Table of Contents", S["h1"]))
    story.append(HRFlowable(width="100%", thickness=1, color=C_NAVY, spaceAfter=10))

    sections = [
        ("sec_arch",    "1.  Architecture Overview", [
            ("sec_boot",   "1.1  Boot Sequence"),
            ("sec_module", "1.2  Module Map"),
        ]),
        ("sec_hw",      "2.  Hardware Reference", [
            ("sec_gpio",   "2.1  GPIO Pinout"),
            ("sec_i2c",    "2.2  I2C Addresses"),
        ]),
        ("sec_cfg",     "3.  Configuration Reference (config.ini)", [
            ("sec_cfg_wifi",  "3.1  [wifi]"),
            ("sec_cfg_mqtt",  "3.2  [mqtt]"),
            ("sec_cfg_ota",   "3.3  [ota]"),
            ("sec_cfg_samp",  "3.4  [sampling]"),
        ]),
        ("sec_api",     "4.  API Reference", [
            (info["anchor"], info["rel_path"])
            for info in api_files
            if info["file_brief"] or info["entities"]
        ]),
        ("sec_release", "5.  Versioning & Release Process", [
            ("sec_roadmap",   "5.1  Roadmap"),
            ("sec_checklist", "5.2  Release Checklist"),
        ]),
    ]

    h1s = ParagraphStyle("toc1_lnk", fontSize=11, fontName="Helvetica-Bold",
                          textColor=C_LINK, spaceBefore=5, spaceAfter=1)
    h2s = ParagraphStyle("toc2_lnk", fontSize=9.5, fontName="Helvetica",
                          textColor=C_LINK, leftIndent=14, spaceAfter=1)
    h3s = ParagraphStyle("toc3_lnk", fontSize=8.5, fontName="Helvetica",
                          textColor=C_GREY_MID, leftIndent=28, spaceAfter=1)

    for anchor, title, children in sections:
        story.append(Paragraph(
            u'\u25b8  <a href="#{}" color="#1d4ed8">{}</a>'.format(anchor, title), h1s))
        for child_anchor, child_title in children:
            story.append(Paragraph(
                u'\u25e6  <a href="#{}" color="#2563eb">{}</a>'.format(
                    child_anchor, child_title), h2s))

    story.append(PageBreak())


# ===========================================================================
# 1. Architecture
# ===========================================================================
def arch_section(story, S):
    story.append(bm("sec_arch", "1. Architecture Overview", level=0))
    story.append(Paragraph("1.  Architecture Overview", S["h1"]))
    story.append(HRFlowable(width="100%", thickness=1, color=C_NAVY, spaceAfter=8))

    # 1.1 Boot sequence
    story.append(bm("sec_boot", "1.1 Boot Sequence", level=1))
    story.append(Paragraph("1.1  Boot Sequence", S["h2"]))

    hs = ParagraphStyle("bsh", fontSize=9, fontName="Helvetica-Bold", textColor=C_WHITE)
    ts = ParagraphStyle("bst", fontSize=9, fontName="Helvetica-Bold", textColor=C_NAVY)
    ds = ParagraphStyle("bsd", fontSize=8.5,fontName="Helvetica",     textColor=C_GREY_DARK)

    steps = [
        ("1", "Serial + LittleFS init",   "400 ms USB CDC settle delay; format-on-fail"),
        ("2", "Provisioning check",
             'Reset pin held >= 3 s  OR  /config.ini missing  →  '
             '<a href="#sec_cfg" color="#1d4ed8">see config section</a>'),
        ("3", "Captive-portal wizard",    "Soft-AP + DNS + HTTP; blocks until restart"),
        ("4", "AppConfig::load()",
             'Parses /config.ini; falls back to provisioning on error  '
             '(<a href="#src_config_appconfig_h" color="#1d4ed8">AppConfig API</a>)'),
        ("5", "FreeRTOS tasks",           "Increment 1: WiFiTask, SleepTask (placeholder)"),
    ]
    step_data = [[Paragraph(s, hs), Paragraph(t, ts), Paragraph(d, ds)]
                 for s, t, d in steps]
    cw = [0.7*cm, 4.8*cm, PAGE_W - 2*MARGIN - 5.5*cm]
    t  = Table(step_data, colWidths=cw)
    t.setStyle(TableStyle([
        ("BACKGROUND",    (0, 0), (0, -1), C_BLUE),
        ("BACKGROUND",    (1, 0), (-1,-1), C_BLUE_LIGHT),
        ("ROWBACKGROUNDS",(1, 0), (-1,-1), [C_BLUE_LIGHT, C_WHITE]),
        ("ALIGN",         (0, 0), (0, -1), "CENTER"),
        ("VALIGN",        (0, 0), (-1,-1), "MIDDLE"),
        ("TOPPADDING",    (0, 0), (-1,-1), 5),
        ("BOTTOMPADDING", (0, 0), (-1,-1), 5),
        ("LEFTPADDING",   (0, 0), (-1,-1), 6),
        ("LINEBELOW",     (0, 0), (-1,-2), 0.3, C_WHITE),
    ]))
    story.append(t)
    story.append(Spacer(1, 10))

    # 1.2 Module map
    story.append(bm("sec_module", "1.2 Module Map", level=1))
    story.append(Paragraph("1.2  Module Map", S["h2"]))

    ps = ParagraphStyle("mp", fontSize=8.5, fontName="Courier",      textColor=C_TEAL)
    cs = ParagraphStyle("mc", fontSize=9,   fontName="Helvetica-Bold",textColor=C_NAVY)
    xs = ParagraphStyle("mx", fontSize=8.5, fontName="Helvetica",     textColor=C_GREY_DARK)
    mhs= ParagraphStyle("mh", fontSize=8.5, fontName="Helvetica-Bold",textColor=C_WHITE)

    modules = [
        ("src/config/",       "AppConfig",    "INI parser / serialiser singleton",
         "src_config_appconfig_h"),
        ("src/provisioning/", "Provisioning", "Soft-AP, DNS, HTTP captive portal",
         "src_provisioning_provisioning_h"),
        ("src/power/",        "PowerManager", "MOSFET rails, deep-sleep (Increment 1)", ""),
        ("src/connectivity/", "WiFiManager",  "Wi-Fi connect / reconnect (Increment 1)", ""),
        ("src/sensors/",      "Sensor*",      "Per-sensor driver classes (Increment 2)", ""),
        ("src/ota/",          "OtaManager",   "GitHub Releases OTA (Increment 4)", ""),
    ]
    mod_data = [[Paragraph("Path", mhs), Paragraph("Class", mhs),
                 Paragraph("Responsibility", mhs)]]
    for path, cls, desc, anchor in modules:
        cls_text = ('<a href="#{}" color="#1d4ed8">{}</a>'.format(anchor, cls)
                    if anchor else cls)
        mod_data.append([Paragraph(path, ps), Paragraph(cls_text, cs),
                          Paragraph(desc, xs)])
    cw2 = [5.2*cm, 3.5*cm, PAGE_W - 2*MARGIN - 8.7*cm]
    story.append(std_table(mod_data, cw2))
    story.append(PageBreak())


# ===========================================================================
# 2. Hardware reference
# ===========================================================================
def hw_section(story, S):
    story.append(bm("sec_hw", "2. Hardware Reference", level=0))
    story.append(Paragraph("2.  Hardware Reference", S["h1"]))
    story.append(HRFlowable(width="100%", thickness=1, color=C_NAVY, spaceAfter=8))

    story.append(bm("sec_gpio", "2.1 GPIO Pinout", level=1))
    story.append(Paragraph("2.1  GPIO Pinout", S["h2"]))

    hs = ParagraphStyle("gh", fontSize=8.5, fontName="Helvetica-Bold", textColor=C_WHITE)
    cs = ParagraphStyle("gc", fontSize=8.5, fontName="Helvetica",      textColor=C_GREY_DARK)
    ps = ParagraphStyle("gp", fontSize=8.5, fontName="Courier-Bold",   textColor=C_NAVY)

    gpio_rows = [
        ["Signal",              "GPIO",    "Direction", "Notes"],
        ["I2C SDA",             "GPIO8",   "Bidirect",  "BME280, AS5600, CCS811, INA3221"],
        ["I2C SCL",             "GPIO9",   "Output",    "Shared I2C clock"],
        ["Rain gauge",          "GPIO19",  "Input",     "Reed switch, active LOW, PULLUP"],
        ["Anemometer",          "GPIO20",  "Input",     "Reed switch, active LOW, PULLUP"],
        ["PM25 UART RX",        "GPIO17",  "Input",     "Sensor TX to ESP RX"],
        ["PM25 UART TX",        "GPIO18",  "Output",    "ESP TX to Sensor RX"],
        ["CCS811 WAKE",         "GPIO41",  "Output",    "Drive LOW before I2C access"],
        ["MICS6814 CO",         "GPIO5",   "ADC In",    "ADC1 Channel 4"],
        ["MICS6814 NO2",        "GPIO6",   "ADC In",    "ADC1 Channel 5"],
        ["MICS6814 NH3",        "GPIO7",   "ADC In",    "ADC1 Channel 6"],
        ["5V MOSFET enable",    "GPIO11",  "Output",    "Active LOW (P-ch MOSFET)"],
        ["3.3V MOSFET enable",  "GPIO4",   "Output",    "Active LOW (P-ch MOSFET)"],
        ["Reset / config pin",  "GPIO0",   "Input",     "Hold LOW >= 3 s for factory reset"],
        ["Built-in LED",        "GPIO97",  "Output",    "Active HIGH"],
    ]
    gpio_data = []
    for i, row in enumerate(gpio_rows):
        if i == 0:
            gpio_data.append([Paragraph(c, hs) for c in row])
        else:
            gpio_data.append([Paragraph(row[0], cs), Paragraph(row[1], ps),
                               Paragraph(row[2], cs), Paragraph(row[3], cs)])
    cw = [4.8*cm, 2.2*cm, 2.4*cm, PAGE_W - 2*MARGIN - 9.4*cm]
    story.append(std_table(gpio_data, cw))
    story.append(Spacer(1, 10))

    story.append(bm("sec_i2c", "2.2 I2C Addresses", level=1))
    story.append(Paragraph("2.2  I2C Addresses", S["h2"]))

    lib_urls = {
        "BME280":  "https://github.com/adafruit/Adafruit_BME280_Library",
        "AS5600":  "https://github.com/RobTillaart/AS5600",
        "CCS811":  "https://github.com/adafruit/Adafruit_CCS811",
        "INA3221": "https://github.com/wollewald/INA3221_WE",
    }
    ls = ParagraphStyle("ll", fontSize=8.5, fontName="Helvetica", textColor=C_GREY_DARK)
    i2c_data = [[Paragraph("Device", hs), Paragraph("Address", hs),
                  Paragraph("Library", hs)]]
    for device, addr in [("BME280","0x76"),("AS5600","0x36"),
                          ("CCS811","0x5A"),("INA3221","0x40")]:
        url     = lib_urls[device]
        lib_txt = '<a href="{}" color="#1d4ed8">{}</a>'.format(url, url.split("/")[-1])
        as2 = ParagraphStyle("ia2", fontSize=8.5, fontName="Courier-Bold", textColor=C_TEAL)
        i2c_data.append([Paragraph(device, ls), Paragraph(addr, as2),
                          Paragraph(lib_txt, ls)])
    cw2 = [3*cm, 2.5*cm, PAGE_W - 2*MARGIN - 5.5*cm]
    story.append(std_table(i2c_data, cw2))
    story.append(PageBreak())


# ===========================================================================
# 3. Configuration reference
# ===========================================================================
def cfg_section(story, S):
    story.append(bm("sec_cfg", "3. Configuration Reference", level=0))
    story.append(Paragraph("3.  Configuration Reference  (config.ini)", S["h1"]))
    story.append(HRFlowable(width="100%", thickness=1, color=C_NAVY, spaceAfter=8))
    story.append(Paragraph(
        'The runtime configuration is stored at <font name="Courier">/config.ini</font> '
        'on LittleFS and is written by the '
        '<a href="#src_provisioning_provisioning_h" color="#1d4ed8">Provisioning wizard</a>. '
        'All parameters are exposed through the '
        '<a href="#src_config_appconfig_h" color="#1d4ed8">AppConfig singleton</a>.',
        S["body"]))
    story.append(Spacer(1, 6))

    hs = ParagraphStyle("ch", fontSize=8.5, fontName="Helvetica-Bold", textColor=C_WHITE)
    ks = ParagraphStyle("ck", fontSize=8.5, fontName="Courier-Bold",   textColor=C_TEAL)
    ts = ParagraphStyle("ct", fontSize=8.5, fontName="Courier",        textColor=C_GREY_MID)
    ds = ParagraphStyle("cd", fontSize=8.5, fontName="Courier",        textColor=C_NAVY)
    xs = ParagraphStyle("cx", fontSize=8.5, fontName="Helvetica",      textColor=C_GREY_DARK)

    sections = [
        ("sec_cfg_wifi",  "[wifi]",      C_NAVY, [
            ("ssid",       "string",  "",                    "Wi-Fi network SSID (required)"),
            ("password",   "string",  "",                    "Password; empty = open network"),
        ]),
        ("sec_cfg_mqtt",  "[mqtt]",      C_TEAL, [
            ("host",       "string",  "",                    "Broker hostname or IP (required)"),
            ("port",       "uint16",  "1883",                "TCP port"),
            ("user",       "string",  "",                    "Username (optional)"),
            ("password",   "string",  "",                    "Password (optional)"),
            ("client_id",  "string",  "weather-station-01",  "Unique MQTT client ID"),
            ("base_topic", "string",  "weather/station01",   "Root topic prefix"),
        ]),
        ("sec_cfg_ota",   "[ota]",       C_NAVY, [
            ("github_repo",  "string", "CarloFalco/WeatherStation",
             '<a href="https://github.com/CarloFalco/WeatherStation" '
             'color="#1d4ed8">GitHub repo</a> used for OTA firmware download'),
            ("github_token", "string", "",
             "Personal Access Token (leave empty for public repos)"),
        ]),
        ("sec_cfg_samp",  "[sampling]",  C_TEAL, [
            ("bme280_s",  "uint32", "60",  "BME280 read interval (s, min 5)"),
            ("rain_s",    "uint32", "60",  "Rain gauge window (s, min 10)"),
            ("wind_s",    "uint32", "10",  "Anemometer sample (s, min 5)"),
            ("pm25_s",    "uint32", "120", "PM2.5 interval (s, min 10)"),
            ("ccs811_s",  "uint32", "120", "CCS811 interval (s, min 10)"),
            ("mics_s",    "uint32", "120", "MICS6814 interval (s, min 10)"),
            ("ina_s",     "uint32", "60",  "INA3221 interval (s, min 5)"),
            ("publish_s", "uint32", "300", "MQTT publish interval (s)"),
            ("sleep_s",   "uint32", "300", "Deep-sleep duration (s); 0 = disabled"),
        ]),
    ]

    for anchor, sec_name, hdr_color, keys in sections:
        story.append(bm(anchor, sec_name, level=1))
        story.append(Paragraph(sec_name, S["h2"]))
        hdr = ParagraphStyle("hh", fontSize=8.5, fontName="Helvetica-Bold",
                              textColor=C_WHITE)
        tbl_data = [[Paragraph("Key", hdr), Paragraph("Type", hdr),
                     Paragraph("Default", hdr), Paragraph("Description", hdr)]]
        for k, t, d, desc in keys:
            tbl_data.append([Paragraph(k, ks), Paragraph(t, ts),
                              Paragraph(d, ds), Paragraph(desc, xs)])
        cw = [3.5*cm, 2*cm, 4*cm, PAGE_W - 2*MARGIN - 9.5*cm]
        tbl = Table(tbl_data, colWidths=cw)
        tbl.setStyle(TableStyle([
            ("BACKGROUND",    (0, 0), (-1, 0), hdr_color),
            ("ROWBACKGROUNDS",(0, 1), (-1,-1), [C_WHITE, C_GREY_LIGHT]),
            ("TOPPADDING",    (0, 0), (-1,-1), 4),
            ("BOTTOMPADDING", (0, 0), (-1,-1), 4),
            ("LEFTPADDING",   (0, 0), (-1,-1), 5),
            ("LINEBELOW",     (0, 0), (-1,-1), 0.3, HexColor("#e5e7eb")),
            ("VALIGN",        (0, 0), (-1,-1), "MIDDLE"),
        ]))
        story.append(tbl)
        story.append(Spacer(1, 6))
    story.append(PageBreak())


# ===========================================================================
# 4. API reference
# ===========================================================================
def api_section(story, S, files):
    story.append(bm("sec_api", "4. API Reference", level=0))
    story.append(Paragraph("4.  API Reference", S["h1"]))
    story.append(HRFlowable(width="100%", thickness=1, color=C_NAVY, spaceAfter=8))
    story.append(Paragraph(
        "Extracted automatically from "
        '<font name="Courier">/** ... */</font>'
        " Doxygen-style comments in the source files.",
        S["body"]))
    story.append(Spacer(1, 6))

    ph = ParagraphStyle("ph", fontSize=8.5, fontName="Helvetica-Bold", textColor=C_WHITE)
    pn = ParagraphStyle("pn", fontSize=8.5, fontName="Courier-Bold",   textColor=C_NAVY)
    pv = ParagraphStyle("pv", fontSize=8.5, fontName="Helvetica",      textColor=C_GREY_DARK)

    for info in files:
        anchor = info["anchor"]
        # Always register bookmark destination so links from other sections resolve
        story.append(bm(anchor, info["rel_path"], level=1))

        if not info["file_brief"] and not info["entities"]:
            continue

        items = []
        rp    = info["rel_path"].replace("\\", "/")
        items.append(Paragraph(rp, S["filepath"]))

        if info["file_brief"]:
            items.append(Paragraph(info["file_brief"], S["brief"]))
        if info["file_note"]:
            items.append(Paragraph("Note: {}".format(info["file_note"]),
                                    S["body_small"]))
        items.append(HRFlowable(width="100%", thickness=0.5,
                                 color=C_BLUE_LIGHT, spaceBefore=2, spaceAfter=4))

        for ent in info["entities"]:
            items.append(Paragraph(u"\u25b8 {}".format(ent["brief"]), S["h3"]))
            if ent["params"]:
                pd = [[Paragraph("Parameter", ph), Paragraph("Description", ph)]]
                for pname, pdesc in ent["params"]:
                    pd.append([Paragraph(pname, pn), Paragraph(pdesc, pv)])
                pt = Table(pd, colWidths=[4*cm, PAGE_W - 2*MARGIN - 4*cm])
                pt.setStyle(TableStyle([
                    ("BACKGROUND",    (0, 0), (-1, 0), C_TEAL),
                    ("ROWBACKGROUNDS",(0, 1), (-1,-1), [C_WHITE, C_GREY_LIGHT]),
                    ("TOPPADDING",    (0, 0), (-1,-1), 3),
                    ("BOTTOMPADDING", (0, 0), (-1,-1), 3),
                    ("LEFTPADDING",   (0, 0), (-1,-1), 5),
                    ("LINEBELOW",     (0, 0), (-1,-1), 0.3, HexColor("#e5e7eb")),
                ]))
                items.append(Spacer(1, 2))
                items.append(pt)
            if ent["returns"]:
                items.append(Paragraph(
                    "<b>Returns:</b> {}".format(ent["returns"]), S["body_small"]))
            if ent["notes"]:
                items.append(Paragraph(
                    "<b>Note:</b> {}".format(ent["notes"]), S["body_small"]))
            items.append(Spacer(1, 4))

        story.append(KeepTogether(items[:4]))
        story.extend(items[4:])

    story.append(PageBreak())


# ===========================================================================
# 5. Release process
# ===========================================================================
def release_section(story, S):
    story.append(bm("sec_release", "5. Versioning & Release Process", level=0))
    story.append(Paragraph("5.  Versioning &amp; Release Process", S["h1"]))
    story.append(HRFlowable(width="100%", thickness=1, color=C_NAVY, spaceAfter=8))
    story.append(Paragraph(
        'This project follows <a href="https://semver.org" color="#1d4ed8">Semantic Versioning</a>. '
        'Version constants are defined in '
        '<a href="#include_version_h" color="#1d4ed8">include/version.h</a> '
        'and consumed at build time by '
        '<font name="Courier">post_build.py</font> and '
        '<font name="Courier">generate_docs.py</font>.',
        S["body"]))
    story.append(Spacer(1, 8))

    story.append(bm("sec_roadmap", "5.1 Roadmap", level=1))
    story.append(Paragraph("5.1  Roadmap", S["h2"]))

    GH = "https://github.com/CarloFalco/WeatherStation"
    hs = ParagraphStyle("rh", fontSize=8.5, fontName="Helvetica-Bold", textColor=C_WHITE)
    ts = ParagraphStyle("rt", fontSize=8.5, fontName="Courier-Bold",   textColor=C_NAVY)
    cs = ParagraphStyle("rc", fontSize=8.5, fontName="Helvetica",      textColor=C_GREY_DARK)
    ok = ParagraphStyle("ro", fontSize=8.5, fontName="Helvetica-Bold", textColor=C_GREEN)
    pl = ParagraphStyle("rp", fontSize=8.5, fontName="Helvetica",      textColor=C_GREY_MID)

    releases = [
        ("v0.1.0", "Increment 0 – Scaffold & provisioning",       True),
        ("v0.2.0", "Increment 1 – FreeRTOS, deep-sleep, Wi-Fi, RTC", False),
        ("v0.3.0", "Increment 2 – Sensor drivers",                False),
        ("v0.4.0", "Increment 3 – MQTT JSON publishing",          False),
        ("v0.5.0", "Increment 4 – OTA via GitHub Releases API",   False),
    ]
    roadmap = [[Paragraph("Tag", hs), Paragraph("Milestone", hs),
                Paragraph("Status", hs)]]
    for tag, desc, done in releases:
        tag_url  = "{}/releases/tag/{}".format(GH, tag)
        tag_txt  = '<a href="{}" color="#1d4ed8">{}</a>'.format(tag_url, tag) if done else tag
        stat_txt = "Released" if done else "Planned"
        st_s     = ok if done else pl
        roadmap.append([Paragraph(tag_txt, ts), Paragraph(desc, cs),
                         Paragraph(stat_txt, st_s)])
    cw = [2.5*cm, PAGE_W - 2*MARGIN - 5.5*cm, 3*cm]
    story.append(std_table(roadmap, cw))
    story.append(Spacer(1, 10))

    story.append(bm("sec_checklist", "5.2 Release Checklist", level=1))
    story.append(Paragraph("5.2  Release Checklist", S["h2"]))

    checklist = [
        'Update <a href="#include_version_h" color="#1d4ed8">include/version.h</a>: '
        'FW_VERSION_MAJOR / MINOR / PATCH',
        'Add entry in <font name="Courier">CHANGELOG.md</font> under the new version tag',
        'Run <font name="Courier">pio run</font> – '
        'post_build.py writes version.txt, versioned .bin and PDF',
        'Commit: <font name="Courier">git add . &amp;&amp; git commit -m "chore: bump to vX.Y.Z"</font>',
        'Tag: <font name="Courier">git tag -a vX.Y.Z -m "Release vX.Y.Z"</font>',
        'Push: <font name="Courier">git push origin main &amp;&amp; git push origin vX.Y.Z</font>',
        'GitHub Actions builds firmware + PDF and creates the '
        '<a href="{}/releases" color="#1d4ed8">GitHub Release</a> automatically'.format(GH),
    ]
    for item in checklist:
        story.append(Paragraph(u"\u2610  {}".format(item), S["body"]))


# ===========================================================================
# Main
# ===========================================================================
def main():
    parser = argparse.ArgumentParser(description="Generate WeatherStation PDF docs")
    parser.add_argument("--out", default=None)
    args = parser.parse_args()

    version     = read_define(VERSION_H, "FW_VERSION",      "0.1.0")
    description = read_define(VERSION_H, "FW_VERSION_DESC", "ESP32-S3 Weather Station")

    os.makedirs(DOCS_DIR, exist_ok=True)
    output_path = args.out or os.path.join(
        DOCS_DIR, "WeatherStation_docs_v{}.pdf".format(version))

    print("[generate_docs] Firmware version : {}".format(version))
    print("[generate_docs] Output           : {}".format(output_path))

    files = collect_files()
    print("[generate_docs] Source files found: {}".format(len(files)))

    S     = build_styles()
    story = []

    api_files = [f for f in files if f["file_brief"] or f["entities"]]

    cover_page(story, S, version, description)
    toc_page(story, S, api_files)
    arch_section(story, S)
    hw_section(story, S)
    cfg_section(story, S)
    api_section(story, S, files)
    release_section(story, S)

    doc = make_doc(output_path, version)
    doc.build(story)

    size_kb = os.path.getsize(output_path) / 1024
    print("[generate_docs] PDF generated    : {:.1f} KB".format(size_kb))
    print("[generate_docs] Done -> {}".format(output_path))


if __name__ == "__main__":
    main()
