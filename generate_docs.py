"""
generate_docs.py  –  WeatherStation PDF documentation generator
================================================================
Generates a self-contained PDF API reference for the WeatherStation
firmware directly from the source files.

No external tools required (Doxygen, LaTeX, Graphviz …).
Uses only reportlab, which is a pure-Python library.

Usage
-----
    python generate_docs.py                  # output: docs/WeatherStation_docs_vX.Y.Z.pdf
    python generate_docs.py --out my.pdf     # custom output path

Integration with PlatformIO post-build
---------------------------------------
Call from post_build.py after copying the firmware binary:
    import subprocess, sys
    subprocess.run([sys.executable,
                    os.path.join(PROJECT_DIR, "generate_docs.py")], check=False)
"""

import os
import re
import sys
import argparse
from datetime import date

# ---------------------------------------------------------------------------
# reportlab imports
# ---------------------------------------------------------------------------
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import cm
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_LEFT, TA_CENTER, TA_JUSTIFY
from reportlab.lib import colors
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, PageBreak,
    Table, TableStyle, HRFlowable, Preformatted,
    KeepTogether,
)
from reportlab.platypus.tableofcontents import TableOfContents
from reportlab.lib.colors import HexColor

# ---------------------------------------------------------------------------
# Project paths
# ---------------------------------------------------------------------------
SCRIPT_DIR   = os.path.abspath(os.path.dirname(__file__))
INCLUDE_DIR  = os.path.join(SCRIPT_DIR, "include")
SRC_DIR      = os.path.join(SCRIPT_DIR, "src")
VERSION_H    = os.path.join(INCLUDE_DIR, "version.h")
DOCS_DIR     = os.path.join(SCRIPT_DIR, "docs")

# ---------------------------------------------------------------------------
# Colour palette (dark-blue engineering theme)
# ---------------------------------------------------------------------------
C_NAVY      = HexColor("#1e3a5f")
C_BLUE      = HexColor("#2563eb")
C_BLUE_LIGHT= HexColor("#dbeafe")
C_TEAL      = HexColor("#0e7490")
C_GREY_DARK = HexColor("#374151")
C_GREY_MID  = HexColor("#6b7280")
C_GREY_LIGHT= HexColor("#f3f4f6")
C_WHITE     = colors.white
C_BLACK     = colors.black
C_RED       = HexColor("#dc2626")
C_GREEN     = HexColor("#16a34a")

PAGE_W, PAGE_H = A4
MARGIN = 2 * cm

# ---------------------------------------------------------------------------
# Helper – read #define from a header file
# ---------------------------------------------------------------------------
def read_define(filepath, name, default="?"):
    pattern = re.compile(rf'^\s*#define\s+{re.escape(name)}\s+"?([^"\n]+)"?')
    try:
        with open(filepath, encoding="utf-8", errors="ignore") as f:
            for line in f:
                m = pattern.match(line)
                if m:
                    return m.group(1).strip()
    except FileNotFoundError:
        pass
    return default

# ---------------------------------------------------------------------------
# Helper – extract Doxygen-style /** … */ block comments + the entity name
# ---------------------------------------------------------------------------
_DOXBLOCK = re.compile(
    r'/\*\*\s*(.*?)\s*\*/',          # /** ... */
    re.DOTALL
)
_BRIEF    = re.compile(r'@brief\s+(.+?)(?=\n\s*\*\s*@|\n\s*\*/)', re.DOTALL)
_PARAM    = re.compile(r'@param\s+(\w+)\s+(.+?)(?=\n\s*\*\s*@|\n\s*\*/)', re.DOTALL)
_RETURN   = re.compile(r'@return\s+(.+?)(?=\n\s*\*\s*@|\n\s*\*/)', re.DOTALL)
_FILE     = re.compile(r'@file\s+(\S+)')
_NOTE     = re.compile(r'@note\s+(.+?)(?=\n\s*\*\s*@|\n\s*\*/)', re.DOTALL)

def clean(text):
    """Strip leading * and extra whitespace from a doxygen block."""
    lines = []
    for line in text.splitlines():
        line = re.sub(r'^\s*\*\s?', '', line)
        lines.append(line.strip())
    return ' '.join(l for l in lines if l)

def parse_file(filepath):
    """Return a dict with file-level and entity-level doc info."""
    result = {"path": filepath, "filename": os.path.basename(filepath),
              "file_brief": "", "file_note": "", "entities": []}
    try:
        src = open(filepath, encoding="utf-8", errors="ignore").read()
    except FileNotFoundError:
        return result

    blocks = _DOXBLOCK.findall(src)
    for raw in blocks:
        fb = _FILE.search(raw)
        if fb:
            br = _BRIEF.search(raw)
            nt = _NOTE.search(raw)
            result["file_brief"] = clean(br.group(1)) if br else ""
            result["file_note"]  = clean(nt.group(1)) if nt else ""
            continue

        brief = clean(_BRIEF.search(raw).group(1)) if _BRIEF.search(raw) else ""
        if not brief:
            brief = clean(raw.split('\n')[0])
        params  = [(m.group(1), clean(m.group(2))) for m in _PARAM.finditer(raw)]
        returns = clean(_RETURN.search(raw).group(1)) if _RETURN.search(raw) else ""
        notes   = clean(_NOTE.search(raw).group(1))  if _NOTE.search(raw)   else ""
        if brief:
            result["entities"].append({
                "brief": brief, "params": params,
                "returns": returns, "notes": notes,
            })
    return result

# ---------------------------------------------------------------------------
# Collect all source/header files to document
# ---------------------------------------------------------------------------
def collect_files():
    docs = []
    search_dirs = [
        (INCLUDE_DIR, "include"),
        (SRC_DIR,     "src"),
    ]
    for base, label in search_dirs:
        for root, _, files in os.walk(base):
            for fname in sorted(files):
                if fname.endswith(('.h', '.cpp')):
                    full = os.path.join(root, fname)
                    rel  = os.path.relpath(full, SCRIPT_DIR)
                    info = parse_file(full)
                    info["label"] = label
                    info["rel_path"] = rel
                    docs.append(info)
    return docs

# ---------------------------------------------------------------------------
# Styles
# ---------------------------------------------------------------------------
def build_styles():
    base = getSampleStyleSheet()
    S = {}

    S["cover_title"] = ParagraphStyle("cover_title",
        fontSize=32, textColor=C_WHITE, fontName="Helvetica-Bold",
        alignment=TA_CENTER, spaceAfter=8)
    S["cover_sub"] = ParagraphStyle("cover_sub",
        fontSize=14, textColor=HexColor("#bfdbfe"), fontName="Helvetica",
        alignment=TA_CENTER, spaceAfter=4)
    S["cover_meta"] = ParagraphStyle("cover_meta",
        fontSize=10, textColor=HexColor("#93c5fd"), fontName="Helvetica",
        alignment=TA_CENTER, spaceAfter=2)

    S["h1"] = ParagraphStyle("h1",
        fontSize=18, textColor=C_NAVY, fontName="Helvetica-Bold",
        spaceBefore=18, spaceAfter=6, leading=22)
    S["h2"] = ParagraphStyle("h2",
        fontSize=13, textColor=C_BLUE, fontName="Helvetica-Bold",
        spaceBefore=12, spaceAfter=4, leading=16)
    S["h3"] = ParagraphStyle("h3",
        fontSize=11, textColor=C_TEAL, fontName="Helvetica-Bold",
        spaceBefore=8, spaceAfter=3, leading=14)

    S["body"] = ParagraphStyle("body",
        fontSize=9.5, textColor=C_GREY_DARK, fontName="Helvetica",
        alignment=TA_JUSTIFY, leading=14, spaceAfter=4)
    S["body_small"] = ParagraphStyle("body_small",
        fontSize=8.5, textColor=C_GREY_MID, fontName="Helvetica",
        leading=12, spaceAfter=3)
    S["label"] = ParagraphStyle("label",
        fontSize=8, textColor=C_GREY_MID, fontName="Helvetica-Bold",
        spaceAfter=1, spaceBefore=6)
    S["code"] = ParagraphStyle("code",
        fontSize=8, fontName="Courier", textColor=C_NAVY,
        backColor=C_GREY_LIGHT, leading=11)
    S["filepath"] = ParagraphStyle("filepath",
        fontSize=8.5, fontName="Courier-Bold", textColor=C_TEAL,
        spaceBefore=10, spaceAfter=2)
    S["brief"] = ParagraphStyle("brief",
        fontSize=9.5, fontName="Helvetica-Oblique", textColor=C_GREY_DARK,
        leading=13, spaceAfter=3)

    S["toc_h1"] = ParagraphStyle("toc_h1",
        fontSize=11, fontName="Helvetica-Bold", textColor=C_NAVY,
        spaceBefore=4, spaceAfter=1)
    S["toc_h2"] = ParagraphStyle("toc_h2",
        fontSize=9.5, fontName="Helvetica", textColor=C_GREY_DARK,
        leftIndent=14, spaceAfter=1)

    return S

# ---------------------------------------------------------------------------
# Page template with header/footer
# ---------------------------------------------------------------------------
def make_doc(output_path: str, version: str):
    from reportlab.platypus import BaseDocTemplate, PageTemplate, Frame

    def header_footer(canvas, doc):
        canvas.saveState()
        # Header bar
        canvas.setFillColor(C_NAVY)
        canvas.rect(MARGIN, PAGE_H - MARGIN - 0.4*cm,
                    PAGE_W - 2*MARGIN, 0.4*cm, fill=1, stroke=0)
        canvas.setFont("Helvetica-Bold", 7)
        canvas.setFillColor(C_WHITE)
        canvas.drawString(MARGIN + 4, PAGE_H - MARGIN - 0.28*cm,
                          "WeatherStation Firmware Documentation")
        canvas.drawRightString(PAGE_W - MARGIN - 4, PAGE_H - MARGIN - 0.28*cm,
                               f"v{version}")
        # Footer
        canvas.setFillColor(C_GREY_LIGHT)
        canvas.rect(MARGIN, MARGIN - 0.1*cm,
                    PAGE_W - 2*MARGIN, 0.35*cm, fill=1, stroke=0)
        canvas.setFont("Helvetica", 7)
        canvas.setFillColor(C_GREY_MID)
        canvas.drawString(MARGIN + 4, MARGIN + 0.05*cm,
                          f"© {date.today().year} Carlo Falco – ESP32-S3 WeatherStation")
        canvas.drawRightString(PAGE_W - MARGIN - 4, MARGIN + 0.05*cm,
                               f"Page {doc.page}")
        canvas.restoreState()

    frame = Frame(
        MARGIN, MARGIN + 0.5*cm,
        PAGE_W - 2*MARGIN,
        PAGE_H - 2*MARGIN - 1*cm,
        id='normal'
    )
    template = PageTemplate(id='main', frames=[frame], onPage=header_footer)

    doc = BaseDocTemplate(
        output_path,
        pagesize=A4,
        leftMargin=MARGIN, rightMargin=MARGIN,
        topMargin=MARGIN + 0.5*cm, bottomMargin=MARGIN + 0.5*cm,
        title=f"WeatherStation v{version} – API Documentation",
        author="Carlo Falco",
        subject="ESP32-S3 Weather Station Firmware",
    )
    doc.addPageTemplates([template])
    return doc

# ---------------------------------------------------------------------------
# Cover page
# ---------------------------------------------------------------------------
def cover_page(story, S, version, description):
    # Navy rectangle background
    from reportlab.platypus import FrameBreak
    from reportlab.platypus import Image

    story.append(Spacer(1, 1.5*cm))

    # Blue header block (simulated with a Table)
    header_data = [[
        Paragraph("🌦️  WeatherStation", S["cover_title"]),
    ]]
    header_table = Table(header_data, colWidths=[PAGE_W - 2*MARGIN])
    header_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,-1), C_NAVY),
        ('TOPPADDING',    (0,0), (-1,-1), 28),
        ('BOTTOMPADDING', (0,0), (-1,-1), 28),
        ('LEFTPADDING',   (0,0), (-1,-1), 20),
        ('RIGHTPADDING',  (0,0), (-1,-1), 20),
        ('ROUNDEDCORNERS', [8]),
    ]))
    story.append(header_table)
    story.append(Spacer(1, 0.6*cm))

    # Subtitle band
    sub_data = [[Paragraph("API &amp; Firmware Reference Documentation", S["cover_sub"])]]
    sub_table = Table(sub_data, colWidths=[PAGE_W - 2*MARGIN])
    sub_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,-1), C_BLUE),
        ('TOPPADDING',    (0,0), (-1,-1), 10),
        ('BOTTOMPADDING', (0,0), (-1,-1), 10),
    ]))
    story.append(sub_table)
    story.append(Spacer(1, 1.2*cm))

    # Meta info table
    meta = [
        ["Version",     f"v{version}"],
        ["Description", description],
        ["MCU",         "ESP32-S3-DevKitC-1"],
        ["Framework",   "Arduino (PlatformIO)"],
        ["Date",        date.today().strftime("%d %B %Y")],
        ["Author",      "Carlo Falco"],
        ["Repository",  "https://github.com/CarloFalco/WeatherStation"],
    ]
    meta_style = ParagraphStyle("meta_val", fontSize=9.5,
                                fontName="Helvetica", textColor=C_GREY_DARK)
    meta_key_style = ParagraphStyle("meta_key", fontSize=9.5,
                                    fontName="Helvetica-Bold", textColor=C_NAVY)
    table_data = [[Paragraph(k, meta_key_style), Paragraph(v, meta_style)]
                  for k, v in meta]
    meta_table = Table(table_data, colWidths=[4*cm, PAGE_W - 2*MARGIN - 4*cm])
    meta_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,-1), C_GREY_LIGHT),
        ('BACKGROUND', (0,0), (0,-1), C_BLUE_LIGHT),
        ('TOPPADDING',    (0,0), (-1,-1), 6),
        ('BOTTOMPADDING', (0,0), (-1,-1), 6),
        ('LEFTPADDING',   (0,0), (-1,-1), 10),
        ('LINEBELOW', (0,0), (-1,-2), 0.5, C_WHITE),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
    ]))
    story.append(meta_table)
    story.append(Spacer(1, 1.5*cm))

    # Sensor summary
    sensors = [
        ["Sensor",          "Interface",      "Measurement"],
        ["BME280",          "I²C  0x76",      "Temperature · Humidity · Pressure"],
        ["AS5600",          "I²C  0x36",      "Wind direction (magnetic encoder)"],
        ["Rain gauge",      "GPIO19 (reed)",   "Rainfall (tipping bucket)"],
        ["Anemometer",      "GPIO20 (reed)",   "Wind speed"],
        ["PMSA003I",        "UART RX=17 TX=18","PM1.0 · PM2.5 · PM10"],
        ["CCS811",          "I²C  0x5A",      "eCO₂ · TVOC"],
        ["MICS6814",        "ADC GPIO5/6/7",   "CO · NO₂ · NH₃"],
        ["INA3221",         "I²C  0x40",      "Voltage · Current (3 ch)"],
    ]
    hdr_s = ParagraphStyle("tbl_hdr", fontSize=8.5,
                           fontName="Helvetica-Bold", textColor=C_WHITE)
    cel_s = ParagraphStyle("tbl_cel", fontSize=8.5,
                           fontName="Helvetica", textColor=C_GREY_DARK)
    tbl_data = []
    for i, row in enumerate(sensors):
        s = hdr_s if i == 0 else cel_s
        tbl_data.append([Paragraph(c, s) for c in row])
    col_w = [(PAGE_W - 2*MARGIN) * f for f in [0.22, 0.26, 0.52]]
    sensor_table = Table(tbl_data, colWidths=col_w)
    sensor_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), C_NAVY),
        ('BACKGROUND', (0,1), (-1,-1), C_GREY_LIGHT),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [C_WHITE, C_GREY_LIGHT]),
        ('TOPPADDING',    (0,0), (-1,-1), 5),
        ('BOTTOMPADDING', (0,0), (-1,-1), 5),
        ('LEFTPADDING',   (0,0), (-1,-1), 6),
        ('LINEBELOW', (0,0), (-1,-1), 0.3, HexColor("#e5e7eb")),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
    ]))
    story.append(Paragraph("Hardware Overview", S["h2"]))
    story.append(sensor_table)
    story.append(PageBreak())

# ---------------------------------------------------------------------------
# Table of contents (manual, simple)
# ---------------------------------------------------------------------------
def toc_page(story, S, sections):
    story.append(Paragraph("Table of Contents", S["h1"]))
    story.append(HRFlowable(width="100%", thickness=1, color=C_NAVY,
                            spaceAfter=10))
    for title, subsections in sections:
        story.append(Paragraph(f"• {title}", S["toc_h1"]))
        for sub in subsections:
            story.append(Paragraph(f"◦  {sub}", S["toc_h2"]))
    story.append(PageBreak())

# ---------------------------------------------------------------------------
# Architecture section
# ---------------------------------------------------------------------------
def arch_section(story, S):
    story.append(Paragraph("1. Architecture Overview", S["h1"]))
    story.append(HRFlowable(width="100%", thickness=1, color=C_NAVY,
                            spaceAfter=8))

    story.append(Paragraph("1.1  Boot Sequence", S["h2"]))
    boot_steps = [
        ("1", "Serial init + LittleFS mount", "400 ms USB CDC settle delay"),
        ("2", "Provisioning check", "Reset pin held ≥ 3 s  OR  /config.ini missing"),
        ("3", "Captive-portal wizard", "Soft-AP + DNS + HTTP server (blocks until restart)"),
        ("4", "AppConfig::load()", "Parses /config.ini; falls back to provisioning on error"),
        ("5", "FreeRTOS tasks", "Increment 1: WiFiTask, SleepTask (placeholder)"),
    ]
    step_data = [[Paragraph(s, ParagraphStyle("s", fontSize=9, fontName="Helvetica-Bold",
                                              textColor=C_WHITE)),
                  Paragraph(t, ParagraphStyle("t", fontSize=9, fontName="Helvetica-Bold",
                                              textColor=C_NAVY)),
                  Paragraph(d, ParagraphStyle("d", fontSize=8.5, fontName="Helvetica",
                                              textColor=C_GREY_DARK))]
                 for s, t, d in boot_steps]
    step_table = Table(step_data, colWidths=[0.7*cm, 5*cm, PAGE_W-2*MARGIN-5.7*cm])
    step_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (0,-1), C_BLUE),
        ('BACKGROUND', (1,0), (-1,-1), C_BLUE_LIGHT),
        ('ROWBACKGROUNDS', (1,0), (-1,-1), [C_BLUE_LIGHT, C_WHITE]),
        ('ALIGN',  (0,0), (0,-1), 'CENTER'),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('TOPPADDING',    (0,0), (-1,-1), 5),
        ('BOTTOMPADDING', (0,0), (-1,-1), 5),
        ('LEFTPADDING',   (0,0), (-1,-1), 6),
        ('LINEBELOW', (0,0), (-1,-2), 0.3, C_WHITE),
    ]))
    story.append(step_table)
    story.append(Spacer(1, 10))

    story.append(Paragraph("1.2  Module Map", S["h2"]))
    modules = [
        ("src/config/",        "AppConfig",    "INI parser / serialiser singleton"),
        ("src/provisioning/",  "Provisioning", "Soft-AP, DNS, HTTP captive portal"),
        ("src/power/",         "PowerManager", "MOSFET rails, deep-sleep (Increment 1)"),
        ("src/connectivity/",  "WiFiManager",  "Wi-Fi connect / reconnect (Increment 1)"),
        ("src/sensors/",       "Sensor*",       "Per-sensor driver classes (Increment 2)"),
        ("src/ota/",           "OtaManager",   "GitHub Releases OTA (Increment 4)"),
    ]
    path_s = ParagraphStyle("ps", fontSize=8.5, fontName="Courier", textColor=C_TEAL)
    cls_s  = ParagraphStyle("cs", fontSize=9,   fontName="Helvetica-Bold", textColor=C_NAVY)
    desc_s = ParagraphStyle("ds", fontSize=8.5, fontName="Helvetica", textColor=C_GREY_DARK)
    hdr_s2 = ParagraphStyle("hs", fontSize=8.5, fontName="Helvetica-Bold", textColor=C_WHITE)

    mod_data = [[Paragraph("Path", hdr_s2),
                 Paragraph("Class", hdr_s2),
                 Paragraph("Responsibility", hdr_s2)]]
    for p, c, d in modules:
        mod_data.append([Paragraph(p, path_s), Paragraph(c, cls_s), Paragraph(d, desc_s)])
    mod_cw = [5.5*cm, 3.5*cm, PAGE_W - 2*MARGIN - 9*cm]
    mod_table = Table(mod_data, colWidths=mod_cw)
    mod_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), C_NAVY),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [C_WHITE, C_GREY_LIGHT]),
        ('TOPPADDING',    (0,0), (-1,-1), 5),
        ('BOTTOMPADDING', (0,0), (-1,-1), 5),
        ('LEFTPADDING',   (0,0), (-1,-1), 6),
        ('LINEBELOW', (0,0), (-1,-1), 0.3, HexColor("#e5e7eb")),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
    ]))
    story.append(mod_table)
    story.append(PageBreak())

# ---------------------------------------------------------------------------
# Pin / address reference section
# ---------------------------------------------------------------------------
def pinout_section(story, S):
    story.append(Paragraph("2. Hardware Reference", S["h1"]))
    story.append(HRFlowable(width="100%", thickness=1, color=C_NAVY, spaceAfter=8))

    story.append(Paragraph("2.1  GPIO Pinout", S["h2"]))
    gpio_rows = [
        ["Signal",              "GPIO",      "Direction", "Notes"],
        ["I²C SDA",             "GPIO8",     "Bidirect",  "BME280, AS5600, CCS811, INA3221"],
        ["I²C SCL",             "GPIO9",     "Output",    "Shared I²C clock"],
        ["Rain gauge",          "GPIO19",    "Input",     "Reed switch, active LOW, PULLUP"],
        ["Anemometer",          "GPIO20",    "Input",     "Reed switch, active LOW, PULLUP"],
        ["PM25 UART RX",        "GPIO17",    "Input",     "Sensor TX → ESP RX"],
        ["PM25 UART TX",        "GPIO18",    "Output",    "ESP TX → Sensor RX"],
        ["CCS811 WAKE",         "GPIO41",    "Output",    "Drive LOW before I²C access"],
        ["MICS6814 CO",         "GPIO5",     "ADC In",    "ADC1 Channel 4"],
        ["MICS6814 NO₂",        "GPIO6",     "ADC In",    "ADC1 Channel 5"],
        ["MICS6814 NH₃",        "GPIO7",     "ADC In",    "ADC1 Channel 6"],
        ["5 V MOSFET enable",   "GPIO11",    "Output",    "Active LOW (P-ch MOSFET)"],
        ["3.3 V MOSFET enable", "GPIO4",     "Output",    "Active LOW (P-ch MOSFET)"],
        ["Reset/config pin",    "GPIO0",     "Input",     "Hold LOW ≥ 3 s → factory reset"],
        ["Built-in LED",        "GPIO97",    "Output",    "Active HIGH"],
    ]
    hdr_s = ParagraphStyle("gh", fontSize=8.5, fontName="Helvetica-Bold", textColor=C_WHITE)
    cel_s = ParagraphStyle("gc", fontSize=8.5, fontName="Helvetica",      textColor=C_GREY_DARK)
    pin_s = ParagraphStyle("gp", fontSize=8.5, fontName="Courier-Bold",   textColor=C_NAVY)
    gpio_data = []
    for i, row in enumerate(gpio_rows):
        if i == 0:
            gpio_data.append([Paragraph(c, hdr_s) for c in row])
        else:
            gpio_data.append([
                Paragraph(row[0], cel_s),
                Paragraph(row[1], pin_s),
                Paragraph(row[2], cel_s),
                Paragraph(row[3], cel_s),
            ])
    cw = [5*cm, 2.2*cm, 2.4*cm, PAGE_W - 2*MARGIN - 9.6*cm]
    gpio_table = Table(gpio_data, colWidths=cw)
    gpio_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), C_NAVY),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [C_WHITE, C_GREY_LIGHT]),
        ('TOPPADDING',    (0,0), (-1,-1), 4),
        ('BOTTOMPADDING', (0,0), (-1,-1), 4),
        ('LEFTPADDING',   (0,0), (-1,-1), 5),
        ('LINEBELOW', (0,0), (-1,-1), 0.3, HexColor("#e5e7eb")),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
    ]))
    story.append(gpio_table)
    story.append(Spacer(1, 10))

    story.append(Paragraph("2.2  I²C Addresses", S["h2"]))
    i2c_rows = [
        ["Device",   "Address", "Library"],
        ["BME280",   "0x76",    "Adafruit BME280 Library"],
        ["AS5600",   "0x36",    "robtillaart/AS5600"],
        ["CCS811",   "0x5A",    "Adafruit CCS811 Library"],
        ["INA3221",  "0x40",    "wollewald/INA3221_WE"],
    ]
    i2c_data = []
    for i, row in enumerate(i2c_rows):
        s = hdr_s if i == 0 else cel_s
        addr_s = ParagraphStyle("addr", fontSize=8.5, fontName="Courier-Bold",
                                textColor=C_TEAL if i > 0 else C_WHITE)
        i2c_data.append([Paragraph(row[0], s),
                         Paragraph(row[1], addr_s),
                         Paragraph(row[2], s)])
    cw2 = [3.5*cm, 2.5*cm, PAGE_W - 2*MARGIN - 6*cm]
    i2c_table = Table(i2c_data, colWidths=cw2)
    i2c_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), C_NAVY),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [C_WHITE, C_GREY_LIGHT]),
        ('TOPPADDING',    (0,0), (-1,-1), 5),
        ('BOTTOMPADDING', (0,0), (-1,-1), 5),
        ('LEFTPADDING',   (0,0), (-1,-1), 6),
        ('LINEBELOW', (0,0), (-1,-1), 0.3, HexColor("#e5e7eb")),
    ]))
    story.append(i2c_table)
    story.append(PageBreak())

# ---------------------------------------------------------------------------
# Configuration reference section
# ---------------------------------------------------------------------------
def config_section(story, S):
    story.append(Paragraph("3. Configuration Reference  (config.ini)", S["h1"]))
    story.append(HRFlowable(width="100%", thickness=1, color=C_NAVY, spaceAfter=8))

    sections_cfg = [
        ("[wifi]", [
            ("ssid",     "string",  "",      "Wi-Fi network SSID (required)"),
            ("password", "string",  "",      "Wi-Fi password; empty for open networks"),
        ]),
        ("[mqtt]", [
            ("host",       "string",  "",                   "Broker hostname or IP (required)"),
            ("port",       "uint16",  "1883",               "TCP port"),
            ("user",       "string",  "",                   "Username (optional)"),
            ("password",   "string",  "",                   "Password (optional)"),
            ("client_id",  "string",  "weather-station-01", "Unique MQTT client ID"),
            ("base_topic", "string",  "weather/station01",  "Root topic prefix"),
        ]),
        ("[ota]", [
            ("github_repo",  "string", "CarloFalco/WeatherStation", "GitHub repository for OTA"),
            ("github_token", "string", "",                          "PAT for private repos"),
        ]),
        ("[sampling]", [
            ("bme280_s",  "uint32", "60",  "BME280 read interval (s, min 5)"),
            ("rain_s",    "uint32", "60",  "Rain gauge window (s, min 10)"),
            ("wind_s",    "uint32", "10",  "Anemometer sample (s, min 5)"),
            ("pm25_s",    "uint32", "120", "PM2.5 read interval (s, min 10)"),
            ("ccs811_s",  "uint32", "120", "CCS811 read interval (s, min 10)"),
            ("mics_s",    "uint32", "120", "MICS6814 read interval (s, min 10)"),
            ("ina_s",     "uint32", "60",  "INA3221 read interval (s, min 5)"),
            ("publish_s", "uint32", "300", "MQTT publish interval (s)"),
            ("sleep_s",   "uint32", "300", "Deep-sleep duration (s); 0 = disabled"),
        ]),
    ]

    for sec_name, keys in sections_cfg:
        story.append(Paragraph(sec_name, S["h2"]))
        hdr_s = ParagraphStyle("ch", fontSize=8.5, fontName="Helvetica-Bold", textColor=C_WHITE)
        key_s = ParagraphStyle("ck", fontSize=8.5, fontName="Courier-Bold",   textColor=C_TEAL)
        typ_s = ParagraphStyle("ct", fontSize=8.5, fontName="Courier",        textColor=C_GREY_MID)
        def_s = ParagraphStyle("cd", fontSize=8.5, fontName="Courier",        textColor=C_NAVY)
        dsc_s = ParagraphStyle("cx", fontSize=8.5, fontName="Helvetica",      textColor=C_GREY_DARK)

        tbl = [[Paragraph("Key", hdr_s), Paragraph("Type", hdr_s),
                Paragraph("Default", hdr_s), Paragraph("Description", hdr_s)]]
        for k, t, d, desc in keys:
            tbl.append([Paragraph(k, key_s), Paragraph(t, typ_s),
                        Paragraph(d, def_s),  Paragraph(desc, dsc_s)])
        cw = [3.5*cm, 2*cm, 4*cm, PAGE_W - 2*MARGIN - 9.5*cm]
        t = Table(tbl, colWidths=cw)
        t.setStyle(TableStyle([
            ('BACKGROUND', (0,0), (-1,0), C_TEAL),
            ('ROWBACKGROUNDS', (0,1), (-1,-1), [C_WHITE, C_GREY_LIGHT]),
            ('TOPPADDING',    (0,0), (-1,-1), 4),
            ('BOTTOMPADDING', (0,0), (-1,-1), 4),
            ('LEFTPADDING',   (0,0), (-1,-1), 5),
            ('LINEBELOW', (0,0), (-1,-1), 0.3, HexColor("#e5e7eb")),
            ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ]))
        story.append(t)
        story.append(Spacer(1, 6))
    story.append(PageBreak())

# ---------------------------------------------------------------------------
# API section – parsed from source files
# ---------------------------------------------------------------------------
def api_section(story, S, files):
    story.append(Paragraph("4. API Reference", S["h1"]))
    story.append(HRFlowable(width="100%", thickness=1, color=C_NAVY, spaceAfter=8))
    story.append(Paragraph(
        "This section is extracted automatically from Doxygen-style "
        "<font name='Courier' size='9'>/** ... */</font> comments in the source files.",
        S["body"]))
    story.append(Spacer(1, 6))

    for info in files:
        if not info["file_brief"] and not info["entities"]:
            continue

        items = []
        items.append(Paragraph(info["rel_path"], S["filepath"]))
        if info["file_brief"]:
            items.append(Paragraph(info["file_brief"], S["brief"]))
        if info["file_note"]:
            items.append(Paragraph(f"Note: {info['file_note']}", S["body_small"]))
        items.append(HRFlowable(width="100%", thickness=0.5, color=C_BLUE_LIGHT,
                                spaceBefore=2, spaceAfter=4))

        for ent in info["entities"]:
            ent_items = []
            ent_items.append(Paragraph(f"▸ {ent['brief']}", S["h3"]))
            if ent["params"]:
                param_rows = [[
                    Paragraph("Parameter",   ParagraphStyle("ph", fontSize=8, fontName="Helvetica-Bold", textColor=C_WHITE)),
                    Paragraph("Description", ParagraphStyle("pd", fontSize=8, fontName="Helvetica-Bold", textColor=C_WHITE)),
                ]]
                for pname, pdesc in ent["params"]:
                    param_rows.append([
                        Paragraph(pname, ParagraphStyle("pn", fontSize=8, fontName="Courier-Bold", textColor=C_NAVY)),
                        Paragraph(pdesc, ParagraphStyle("pv", fontSize=8, fontName="Helvetica",    textColor=C_GREY_DARK)),
                    ])
                pt = Table(param_rows, colWidths=[4*cm, PAGE_W - 2*MARGIN - 4*cm])
                pt.setStyle(TableStyle([
                    ('BACKGROUND', (0,0), (-1,0), C_TEAL),
                    ('ROWBACKGROUNDS', (0,1), (-1,-1), [C_WHITE, C_GREY_LIGHT]),
                    ('TOPPADDING',    (0,0), (-1,-1), 3),
                    ('BOTTOMPADDING', (0,0), (-1,-1), 3),
                    ('LEFTPADDING',   (0,0), (-1,-1), 5),
                    ('LINEBELOW', (0,0), (-1,-1), 0.3, HexColor("#e5e7eb")),
                ]))
                ent_items.append(Spacer(1, 2))
                ent_items.append(pt)
            if ent["returns"]:
                ent_items.append(Paragraph(
                    f"<b>Returns:</b> {ent['returns']}",
                    S["body_small"]))
            if ent["notes"]:
                ent_items.append(Paragraph(
                    f"<b>Note:</b> {ent['notes']}",
                    S["body_small"]))
            items.extend(ent_items)
            items.append(Spacer(1, 4))

        story.append(KeepTogether(items[:4]))  # keep filename + first entity together
        story.extend(items[4:])

    story.append(PageBreak())

# ---------------------------------------------------------------------------
# Versioning & release process section
# ---------------------------------------------------------------------------
def release_section(story, S):
    story.append(Paragraph("5. Versioning & Release Process", S["h1"]))
    story.append(HRFlowable(width="100%", thickness=1, color=C_NAVY, spaceAfter=8))

    story.append(Paragraph(
        "The project follows Semantic Versioning (SemVer). "
        "Version constants are defined in <font name='Courier'>include/version.h</font> "
        "and read at build time by <font name='Courier'>post_build.py</font>.",
        S["body"]))
    story.append(Spacer(1, 8))

    roadmap = [
        ["Tag",    "Milestone",     "Status"],
        ["v0.1.0", "Increment 0 – Scaffold & provisioning wizard", "✅ Released"],
        ["v0.2.0", "Increment 1 – FreeRTOS, deep-sleep, Wi-Fi, RTC", "🚧 Planned"],
        ["v0.3.0", "Increment 2 – Sensor drivers",                  "🚧 Planned"],
        ["v0.4.0", "Increment 3 – MQTT JSON publishing",            "🚧 Planned"],
        ["v0.5.0", "Increment 4 – OTA via GitHub Releases API",     "🚧 Planned"],
    ]
    hdr_s = ParagraphStyle("rh", fontSize=8.5, fontName="Helvetica-Bold", textColor=C_WHITE)
    tag_s = ParagraphStyle("rt", fontSize=8.5, fontName="Courier-Bold",   textColor=C_NAVY)
    cel_s = ParagraphStyle("rc", fontSize=8.5, fontName="Helvetica",      textColor=C_GREY_DARK)
    ok_s  = ParagraphStyle("ro", fontSize=8.5, fontName="Helvetica-Bold", textColor=C_GREEN)
    pl_s  = ParagraphStyle("rp", fontSize=8.5, fontName="Helvetica",      textColor=C_GREY_MID)

    tbl = []
    for i, row in enumerate(roadmap):
        if i == 0:
            tbl.append([Paragraph(c, hdr_s) for c in row])
        else:
            st_s = ok_s if "Released" in row[2] else pl_s
            tbl.append([Paragraph(row[0], tag_s),
                        Paragraph(row[1], cel_s),
                        Paragraph(row[2], st_s)])
    cw = [2.5*cm, PAGE_W - 2*MARGIN - 5.5*cm, 3*cm]
    t = Table(tbl, colWidths=cw)
    t.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), C_NAVY),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [C_WHITE, C_GREY_LIGHT]),
        ('TOPPADDING',    (0,0), (-1,-1), 5),
        ('BOTTOMPADDING', (0,0), (-1,-1), 5),
        ('LEFTPADDING',   (0,0), (-1,-1), 6),
        ('LINEBELOW', (0,0), (-1,-1), 0.3, HexColor("#e5e7eb")),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
    ]))
    story.append(t)
    story.append(Spacer(1, 10))

    story.append(Paragraph("5.1  Release checklist", S["h2"]))
    checklist = [
        "Update FW_VERSION_MAJOR / MINOR / PATCH in <font name='Courier'>include/version.h</font>",
        "Add an entry in <font name='Courier'>CHANGELOG.md</font> under the new version tag",
        "Run <font name='Courier'>pio run</font> – post_build.py writes <font name='Courier'>version.txt</font> "
        "and <font name='Courier'>WeatherStation_vX.Y.Z.bin</font>",
        "Commit: <font name='Courier'>git add . && git commit -m \"chore: bump to vX.Y.Z\"</font>",
        "Tag: <font name='Courier'>git tag -a vX.Y.Z -m \"Release vX.Y.Z\"</font>",
        "Push: <font name='Courier'>git push origin main && git push origin vX.Y.Z</font>",
        "GitHub Actions builds the firmware and creates the release automatically",
    ]
    for step in checklist:
        story.append(Paragraph(f"☐  {step}", S["body"]))

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Generate WeatherStation PDF docs")
    parser.add_argument("--out", default=None,
                        help="Output PDF path (default: docs/WeatherStation_docs_vX.Y.Z.pdf)")
    args = parser.parse_args()

    # Read version
    version     = read_define(VERSION_H, "FW_VERSION",   "0.1.0")
    description = read_define(VERSION_H, "FW_VERSION_DESC",
                              "ESP32-S3 Weather Station Firmware")

    # Output path
    os.makedirs(DOCS_DIR, exist_ok=True)
    output_path = args.out or os.path.join(
        DOCS_DIR, f"WeatherStation_docs_v{version}.pdf")

    print(f"[generate_docs] Firmware version : {version}")
    print(f"[generate_docs] Output           : {output_path}")

    # Collect source files
    files = collect_files()
    print(f"[generate_docs] Source files found: {len(files)}")

    # Build story
    S = build_styles()
    story = []

    toc_sections = [
        ("Architecture Overview",       ["Boot Sequence", "Module Map"]),
        ("Hardware Reference",          ["GPIO Pinout", "I²C Addresses"]),
        ("Configuration Reference",     ["[wifi]", "[mqtt]", "[ota]", "[sampling]"]),
        ("API Reference",               [f["rel_path"] for f in files
                                         if f["file_brief"] or f["entities"]]),
        ("Versioning & Release Process",["Roadmap", "Release checklist"]),
    ]

    cover_page(story, S, version, description)
    toc_page(story, S, toc_sections)
    arch_section(story, S)
    pinout_section(story, S)
    config_section(story, S)
    api_section(story, S, files)
    release_section(story, S)

    # Build PDF
    doc = make_doc(output_path, version)
    doc.build(story)

    size_kb = os.path.getsize(output_path) / 1024
    print(f"[generate_docs] PDF generated    : {size_kb:.1f} KB")
    print(f"[generate_docs] Done → {output_path}")

if __name__ == "__main__":
    main()
