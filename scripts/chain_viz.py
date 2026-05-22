#!/usr/bin/env python3
"""Generate draw.io chain diagram from a receiver CSV config.

Reads the CSV (with stage_type and product_url columns) produced by
component_optimizer.py and generates a dynamic chain diagram.

Usage:
    python3 scripts/chain_viz.py [data_input/20ghz/receiver.csv]
"""

import csv
import sys
import xml.sax.saxutils as saxutils

# ── Geometry ──────────────────────────────────────────────────────────
STAGE_W = 110     # width of each stage block
STAGE_H = 70      # height
STAGE_GAP = 40    # horizontal gap between stages
TOP_Y = 280       # top Y for stage row
ANT_X = 80        # antenna X
ANT_Y = 310       # antenna Y

# Colours per stage type
COLOURS = {
    "Switch":  "#d5e8d4",   # green
    "BPF":     "#dae8fc",   # blue
    "LNA":     "#f8cecc",   # red/pink
    "Mixer":   "#ffe6cc",   # orange
    "Limiter": "#e1d5e7",   # purple
    "":        "#f5f5f5",   # grey fallback
}

STRAIGHT = "endArrow=none;html=1;rounded=0;"
NO_ARROW = "endArrow=none;"


def esc(text):
    return saxutils.escape(str(text), {'"': '&quot;', "'": '&apos;'})


def gen_diagram(csv_path: str, output_path: str):
    # ── Read CSV ──────────────────────────────────────────────────────
    stages = []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get("chain", "").strip() == "baseband_rx":
                stages.append(row)

    if not stages:
        print("No baseband_rx chain found!")
        return

    cells = []
    cid = [0]

    def cell(content, style, x, y, w, h):
        cid[0] += 1
        cid_str = f"c{cid[0]}"
        cells.append(
            f'  <mxCell id="{cid_str}" value="{esc(content)}" style="{style}" '
            f'vertex="1" parent="1">\n'
            f'    <mxGeometry x="{x}" y="{y}" width="{w}" height="{h}" as="geometry" />\n'
            f'  </mxCell>'
        )
        return cid_str

    def edge(src, dst, style=""):
        cid[0] += 1
        eid = f"e{cid[0]}"
        cells.append(
            f'  <mxCell id="{eid}" style="{style}" edge="1" parent="1" '
            f'source="{src}" target="{dst}">\n'
            f'    <mxGeometry relative="1" as="geometry" />\n'
            f'  </mxCell>'
        )

    def text_node(val, x, y, w, h, align="center"):
        return cell(
            val,
            f"text;html=1;whiteSpace=wrap;strokeColor=none;fillColor=none;"
            f"align={align};verticalAlign=middle;rounded=0;",
            x, y, w, h,
        )

    def edge_points(x1, y1, x2, y2, style=""):
        cid[0] += 1
        eid = f"e{cid[0]}"
        cells.append(
            f'  <mxCell id="{eid}" value="" style="{style}" edge="1" parent="1">\n'
            f'    <mxGeometry height="50" relative="1" width="50" as="geometry">\n'
            f'      <mxPoint x="{x1}" y="{y1}" as="sourcePoint" />\n'
            f'      <mxPoint x="{x2}" y="{y2}" as="targetPoint" />\n'
            f'    </mxGeometry>\n'
            f'  </mxCell>'
        )
        return eid

    def make_antenna():
        mx = ANT_X + 30
        my_top = ANT_Y - 30
        my_base = ANT_Y + 20
        arm = 20
        edge_points(mx, my_base, mx, my_top, STRAIGHT + NO_ARROW)
        edge_points(mx, my_top, mx - arm, my_top - arm, STRAIGHT + NO_ARROW)
        edge_points(mx, my_top, mx + arm, my_top - arm, STRAIGHT + NO_ARROW)
        text_node("Antenna\n40 dBi", mx - 25, my_top - 40, 50, 35)
        return cell("", "ellipse;strokeColor=none;fillColor=none;", mx - 2, my_base - 2, 4, 4)

    def stage_shape(stage_type: str) -> str:
        if stage_type == "LNA":
            return "triangle;whiteSpace=wrap;html=1;"
        elif stage_type == "Mixer":
            return "triangle;whiteSpace=wrap;html=1;rotation=-90;"
        else:
            return "rounded=1;whiteSpace=wrap;html=1;"

    def stage_style(stage_type: str) -> str:
        colour = COLOURS.get(stage_type, COLOURS[""])
        style = stage_shape(stage_type)
        style += f"fillColor={colour};strokeColor=#000000;overflow=hidden;"
        return style

    # ── Build diagram ─────────────────────────────────────────────────
    ant_anchor = make_antenna()
    stage_ids = []
    prev_id = ant_anchor

    mixer_count = 0

    for i, s in enumerate(stages):
        stype = s.get("stage_type", "").strip()
        part = s.get("part_number", "").strip()
        gain = s.get("gain_db", "").strip()
        nf = s.get("nf_db", "").strip()
        url = s.get("product_url", "").strip() or s.get("datasheet_url", "").strip()
        name = s.get("name", f"stage_{i}").strip()

        x = ANT_X + 80 + i * (STAGE_W + STAGE_GAP)
        y = TOP_Y

        if stype == "Mixer":
            # Draw mixer triangle at slightly lower Y
            h_mix = 55
            mix_id = cell("", stage_style(stype), x, y + 15, STAGE_W, h_mix)

            # LO ellipse above mixer
            lo_x = x + STAGE_W / 2 - 30
            lo_y = y - 70
            lo_text = f"LO {mixer_count + 1}"
            lo_id = cell(
                lo_text,
                "ellipse;whiteSpace=wrap;html=1;aspect=fixed;",
                lo_x, lo_y, 60, 60,
            )
            # Edge LO → mixer
            edge(lo_id, mix_id, STRAIGHT)

            # Part label next to mixer
            link_text = f'<a href="{esc(url)}">{esc(part)}</a>' if url else esc(part)
            link_id = cell(
                link_text,
                "text;html=1;whiteSpace=wrap;strokeColor=none;fillColor=none;"
                "align=center;verticalAlign=middle;rounded=0;",
                x + 15, y + 22, STAGE_W - 30, 20,
            )

            # Specs below
            spec_parts = []
            if gain:
                spec_parts.append(f"G={gain}dB")
            if nf:
                spec_parts.append(f"NF={nf}dB")
            if spec_parts:
                text_node("  ".join(spec_parts), x, y + h_mix + 10, STAGE_W, 20)

            mixer_count += 1

            # Connect from previous
            if prev_id:
                edge(prev_id, mix_id, "endArrow=block;html=1;rounded=0;")
            prev_id = mix_id
            stage_ids.append(mix_id)
        else:
            # Regular block or triangle
            if stype == "LNA":
                h = STAGE_H
                bid = cell("", stage_style(stype), x, y, STAGE_W, h)
                # Part number inside
                link_text = f'<a href="{esc(url)}">{esc(part)}</a>' if url else esc(part)
                link_id = cell(
                    link_text,
                    "text;html=1;whiteSpace=wrap;strokeColor=none;fillColor=none;"
                    "align=center;verticalAlign=middle;rounded=0;",
                    x + 5, y + 15, STAGE_W - 10, 20,
                )
                # Specs below
                spec_parts = []
                if gain:
                    spec_parts.append(f"G={gain}dB")
                if nf:
                    spec_parts.append(f"NF={nf}dB")
                if spec_parts:
                    text_node("  ".join(spec_parts), x, y + h + 5, STAGE_W, 20)
            else:
                # Passive block (Switch, BPF, Limiter)
                h = STAGE_H
                bid = cell("", stage_style(stype), x, y, STAGE_W, h)
                # Part number inside
                link_text = f'<a href="{esc(url)}">{esc(part)}</a>' if url else esc(part)
                link_id = cell(
                    link_text,
                    "text;html=1;whiteSpace=wrap;strokeColor=none;fillColor=none;"
                    "align=center;verticalAlign=middle;rounded=0;",
                    x + 5, y + 10, STAGE_W - 10, 25,
                )
                # Specs below
                spec_parts = []
                if gain:
                    spec_parts.append(f"IL={abs(float(gain)):.1f}dB" if float(gain) < 0 else f"G={gain}dB")
                if nf:
                    spec_parts.append(f"NF={nf}dB")
                if spec_parts:
                    text_node("  ".join(spec_parts), x, y + h + 5, STAGE_W, 20)

            # Add stage type label above
            text_node(stype, x, y - 22, STAGE_W, 20)

            if prev_id:
                edge(prev_id, bid, "endArrow=block;html=1;rounded=0;")
            prev_id = bid
            stage_ids.append(bid)

    # ── ADC at end ────────────────────────────────────────────────────
    last_x = ANT_X + 80 + len(stages) * (STAGE_W + STAGE_GAP)
    adc_id = cell(
        "ADC",
        "rounded=1;whiteSpace=wrap;html=1;",
        last_x, TOP_Y, STAGE_W, STAGE_H,
    )
    if prev_id:
        edge(prev_id, adc_id, "endArrow=block;html=1;rounded=0;")

    # ── Title ─────────────────────────────────────────────────────────
    title = f"Receiver Chain — {len(stages)} stages — Optimised via Digi-Key API"
    text_node(title, ANT_X, TOP_Y - 80, last_x + STAGE_W - ANT_X, 30, "center")

    # ── Serialize ─────────────────────────────────────────────────────
    xml = f"""<?xml version="1.0" encoding="UTF-8"?>
<mxfile host="app.diagrams.net" agent="Mozilla/5.0">
  <diagram name="Page-1" id="chain-diagram">
    <mxGraphModel grid="1" page="1" gridSize="10" guides="1" tooltips="1"
      connect="1" arrows="1" fold="1" pageScale="1"
      pageWidth="{last_x + STAGE_W + 100}" pageHeight="700" math="0" shadow="0">
      <root>
        <mxCell id="0" />
        <mxCell id="1" parent="0" />
{chr(10).join(cells)}
      </root>
    </mxGraphModel>
  </diagram>
</mxfile>"""

    with open(output_path, "w") as f:
        f.write(xml)
    print(f"Wrote {output_path}  ({len(stages)} stages)")


if __name__ == "__main__":
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "data_input/20ghz/receiver.csv"
    output_path = "docs/receiver_chain.drawio"
    gen_diagram(csv_path, output_path)
