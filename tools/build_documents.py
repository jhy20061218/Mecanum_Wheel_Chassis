"""JHY: build submission documents from the checked-in Markdown sources.

Requires python-docx. Run from anywhere; output remains in ../docs.
"""
from pathlib import Path
import re
from docx import Document
from docx.shared import Inches, Pt, RGBColor
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT, WD_CELL_VERTICAL_ALIGNMENT

ROOT = Path(__file__).resolve().parents[1]


def font_style(style, size, bold=False):
    style.font.name = 'Microsoft YaHei'
    style.font.size = Pt(size)
    style.font.bold = bold
    style.font.color.rgb = RGBColor(0, 0, 0)
    rpr = style.element.get_or_add_rPr()
    fonts = rpr.find(qn('w:rFonts'))
    if fonts is None:
        fonts = OxmlElement('w:rFonts'); rpr.insert(0, fonts)
    for key in ('ascii', 'hAnsi', 'eastAsia'):
        fonts.set(qn('w:' + key), 'Microsoft YaHei')


def inline(paragraph, text):
    for part in re.split(r'(\*\*.+?\*\*|`[^`]+`)', text):
        if part.startswith('**'):
            run = paragraph.add_run(part[2:-2]); run.bold = True
        elif part.startswith('`'):
            run = paragraph.add_run(part[1:-1]); run.font.name = 'Consolas'
            run.font.size = Pt(9.5)
        else:
            paragraph.add_run(re.sub(r'\[([^\]]+)\]\([^)]+\)', r'\1', part))


def table(doc, rows):
    count = max(map(len, rows))
    tab = doc.add_table(rows=1, cols=count)
    tab.alignment = WD_TABLE_ALIGNMENT.CENTER
    tab.autofit = False
    width = 6.75 / count
    if count == 2: widths = [2.05, 4.7]
    elif count == 3: widths = [1.55, 2.1, 3.1]
    elif count == 4: widths = [1.65, 1.6, 1.7, 1.8]
    elif count == 5: widths = [1.55, 1.15, 1.25, 1.65, 1.15]
    else: widths = [width] * count
    for col, w in zip(tab.columns, widths): col.width = Inches(w)
    for idx, row in enumerate(rows):
        cells = tab.rows[0].cells if idx == 0 else tab.add_row().cells
        row_properties = tab.rows[idx]._tr.get_or_add_trPr()
        no_split = OxmlElement('w:cantSplit'); row_properties.append(no_split)
        for j, cell in enumerate(cells):
            cell.width = Inches(widths[j]); cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
            p = cell.paragraphs[0]
            p.paragraph_format.space_after = Pt(4); p.paragraph_format.space_before = Pt(4)
            p.paragraph_format.line_spacing = 1.1
            inline(p, row[j] if j < len(row) else '')
            for run in p.runs:
                run.font.size = Pt(9.5)
                if idx == 0: run.bold = True
            if idx == 0:
                shade = OxmlElement('w:shd'); shade.set(qn('w:fill'), 'DCE6F1'); cell._tc.get_or_add_tcPr().append(shade)
            tcpr = cell._tc.get_or_add_tcPr()
            borders = OxmlElement('w:tcBorders')
            for side in ('top', 'left', 'bottom', 'right'):
                edge = OxmlElement('w:' + side); edge.set(qn('w:val'), 'single')
                edge.set(qn('w:sz'), '4'); edge.set(qn('w:color'), 'D9D9D9'); borders.append(edge)
            tcpr.append(borders)
            margin = OxmlElement('w:tcMar')
            for side in ('top', 'left', 'bottom', 'right'):
                item = OxmlElement('w:' + side); item.set(qn('w:w'), '90'); item.set(qn('w:type'), 'dxa'); margin.append(item)
            tcpr.append(margin)
        if idx == 0:
            repeat = OxmlElement('w:tblHeader'); tab.rows[0]._tr.get_or_add_trPr().append(repeat)
    doc.add_paragraph().paragraph_format.space_after = Pt(2)


def native_equation(doc, text):
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    math = OxmlElement('m:oMath')
    run = OxmlElement('m:r'); value = OxmlElement('m:t'); value.text = text
    run.append(value); math.append(run); p._p.append(math)
    return p


def create(source):
    doc = Document()
    # The stock Word template can place a blue bottom rule on Title/Subtitle.
    # Clear inherited borders and all theme colors explicitly.
    for style in doc.styles:
        if style.element.pPr is not None:
            for border in list(style.element.pPr.findall(qn('w:pBdr'))):
                style.element.pPr.remove(border)
    sec = doc.sections[0]
    sec.page_width = Inches(8.5); sec.page_height = Inches(11)
    sec.top_margin = Inches(.72); sec.bottom_margin = Inches(.72)
    sec.left_margin = Inches(.85); sec.right_margin = Inches(.85)
    font_style(doc.styles['Normal'], 11)
    doc.styles['Normal'].paragraph_format.space_after = Pt(6)
    doc.styles['Normal'].paragraph_format.line_spacing = 1.22
    for name, size in [('Title', 22), ('Subtitle', 11), ('Heading 1', 15), ('Heading 2', 12), ('Heading 3', 11)]:
        font_style(doc.styles[name], size, name.startswith('Heading'))
        doc.styles[name].paragraph_format.space_before = Pt(12 if name.startswith('Heading') else 0)
        doc.styles[name].paragraph_format.space_after = Pt(6)
    doc.core_properties.author = 'JHY'
    doc.core_properties.last_modified_by = 'JHY'
    doc.core_properties.subject = '麦克纳姆轮底盘预测与模拟课程设计'
    lines = source.read_text(encoding='utf-8-sig').splitlines()
    i = 0; in_code = False; equation = False; math_code = False
    while i < len(lines):
        line = lines[i].rstrip(); i += 1
        if line.startswith('```'):
            if not in_code:
                # The design's formula blocks remain editable Word equations.
                math_code = line.strip() == '```text'
            in_code = not in_code; continue
        if in_code:
            if not line: continue
            if math_code:
                p = native_equation(doc, line)
                p.paragraph_format.keep_with_next = i < len(lines) and not lines[i].startswith('```')
                continue
            p = doc.add_paragraph()
            p.paragraph_format.space_after = Pt(1)
            p.paragraph_format.line_spacing = 1.0
            run = p.add_run(line); run.font.name = 'Consolas'; run.font.size = Pt(8.5)
            continue
        if line.strip() == '$$': equation = not equation; continue
        if equation:
            native_equation(doc, line); continue
        if not line or re.fullmatch(r'\s*[-*_]{3,}\s*', line): continue
        if line.startswith('|'):
            rows = []
            i -= 1
            while i < len(lines) and lines[i].startswith('|'):
                content = [c.strip() for c in lines[i].strip().strip('|').split('|')]
                if not all(re.fullmatch(r'[: -]+', c or '-') for c in content): rows.append(content)
                i += 1
            table(doc, rows); continue
        image = re.fullmatch(r'!\[([^\]]*)\]\(([^)]+)\)', line)
        if image:
            path = source.parent / image.group(2)
            p = doc.add_paragraph(); p.alignment = WD_ALIGN_PARAGRAPH.CENTER
            p.add_run().add_picture(str(path.resolve()), width=Inches(6.45))
            if image.group(1):
                p = doc.add_paragraph(image.group(1)); p.alignment = WD_ALIGN_PARAGRAPH.CENTER
                p.paragraph_format.space_after = Pt(9)
                for r in p.runs: r.font.size = Pt(9)
            continue
        h = re.match(r'^(#{1,4})\s+(.*)', line)
        if h:
            level = len(h.group(1))
            text = h.group(2).replace('`', '').replace('**', '')
            doc.add_paragraph(text, 'Title' if level == 1 else 'Heading ' + str(min(level - 1, 3)))
            if level == 1:
                doc.core_properties.title = text
                doc.add_paragraph('JHY    自动化专业课程设计    2026年10月26日', 'Subtitle')
            continue
        bullet = re.match(r'^\s*([-*]|\d+[.、)])\s+(.*)', line)
        if bullet:
            p = doc.add_paragraph(style='List Bullet' if bullet.group(1) in '-*' else 'List Number')
            inline(p, bullet.group(2)); continue
        p = doc.add_paragraph(); inline(p, line.removeprefix('> '))
    out = source.with_suffix('.docx')
    for p in doc.paragraphs:
        if p._p.pPr is not None:
            for border in list(p._p.pPr.findall(qn('w:pBdr'))):
                p._p.pPr.remove(border)
    doc.save(out)
    print(out)


if __name__ == '__main__':
    for filename in ('设计文档.md', '实验工作报告.md'):
        source = ROOT / 'docs' / filename
        if not source.exists(): raise FileNotFoundError(source)
        create(source)
