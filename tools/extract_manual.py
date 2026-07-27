"""Extract per-section help HTML from the LEparagliding manual.

Reads manual.en.html (saved from
https://www.laboratoridenvol.com/leparagliding/manual.en.html), slices the
content of each data-file section (anchors 6.1 .. 6.37), converts the
Netscape/LibreOffice tag soup into the HTML subset supported by Qt's rich
text engine, remaps text colors so they stay readable on the Studio's dark
theme, rewrites images to qrc:/manual/images/... and downloads the preview
images that the manual actually displays inline.

Outputs into <repo>/resources/manual/: sections/sectionNN.html, images/*,
and manual.qrc.
"""

import colorsys
import os
import re
import sys
import urllib.parse
import urllib.request
from html.parser import HTMLParser

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(REPO, "resources", "manual")
BASE_URL = "https://www.laboratoridenvol.com/leparagliding/"
MANUAL_URL = BASE_URL + "manual.en.html"
# Cached download of the manual page; gitignored, delete to refresh.
MANUAL_CACHE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "manual.en.html"
)

SECTION_NUMBERS = list(range(1, 13)) + list(range(15, 38))


def load_manual():
    if not os.path.exists(MANUAL_CACHE):
        request = urllib.request.Request(
            MANUAL_URL,
            headers={"User-Agent": "Mozilla/5.0 (LEparagliding Studio build)"},
        )
        print("downloading", MANUAL_URL)
        with urllib.request.urlopen(request, timeout=60) as response:
            raw = response.read()
        with open(MANUAL_CACHE, "wb") as fh:
            fh.write(raw)
    with open(MANUAL_CACHE, "rb") as fh:
        raw = fh.read()
    try:
        return raw.decode("utf-8")
    except UnicodeDecodeError:
        return raw.decode("latin-1")


def parse_style(style):
    props = {}
    for part in style.split(";"):
        if ":" in part:
            key, value = part.split(":", 1)
            props[key.strip().lower()] = value.strip().lower()
    return props


NAMED_COLORS = {
    "black": (0, 0, 0),
    "white": (255, 255, 255),
    "red": (255, 0, 0),
    "green": (0, 128, 0),
    "blue": (0, 0, 255),
    "gray": (128, 128, 128),
    "grey": (128, 128, 128),
    "windowtext": (0, 0, 0),
    "maroon": (128, 0, 0),
    "navy": (0, 0, 128),
    "teal": (0, 128, 128),
    "purple": (128, 0, 128),
    "orange": (255, 165, 0),
}


def parse_color(value):
    value = value.strip().lower()
    if value in NAMED_COLORS:
        return NAMED_COLORS[value]
    match = re.match(r"rgb\(\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\)", value)
    if match:
        return tuple(int(component) for component in match.groups())
    match = re.match(r"#([0-9a-f]{6})$", value)
    if match:
        hexval = match.group(1)
        return tuple(int(hexval[i : i + 2], 16) for i in (0, 2, 4))
    match = re.match(r"#([0-9a-f]{3})$", value)
    if match:
        hexval = match.group(1)
        return tuple(int(ch * 2, 16) for ch in hexval)
    return None


def theme_color(value):
    """Map a manual text color to one readable on the dark theme.

    Returns a #rrggbb string, or None when the color should be dropped so
    the text inherits the default light foreground (black/gray text).
    """
    rgb = parse_color(value)
    if rgb is None:
        return None
    hue, lightness, saturation = colorsys.rgb_to_hls(*(c / 255.0 for c in rgb))
    if saturation < 0.15:
        return None
    lightness = max(lightness, 0.75)
    red, green, blue = colorsys.hls_to_rgb(hue, lightness, saturation)
    return "#{:02x}{:02x}{:02x}".format(
        round(red * 255), round(green * 255), round(blue * 255)
    )


def local_image_name(src):
    name = os.path.basename(urllib.parse.unquote(src))
    return name.replace(" ", "_")


class QtRichTextConverter(HTMLParser):
    """Convert manual tag soup to the subset QTextBrowser renders well."""

    BLOCK_TAGS = {"div", "p"}
    SKIP_CONTENT_TAGS = {"style", "script", "title"}
    PASSTHROUGH_TAGS = {"b", "i", "u", "ul", "ol", "li", "hr"}

    # Default foreground of the Studio dark theme (mainwindow stylesheet);
    # used to reset nested "color: black" spans inside colored text.
    DEFAULT_FOREGROUND = "#e6edf7"

    def __init__(self, images, internal_links=False):
        super().__init__(convert_charrefs=True)
        self.out = []
        self.stack = []  # (source_tag, [emitted_tags], color_spans)
        self.skip = 0
        self.color_depth = 0
        self.images = images  # dict: local name -> site-relative src
        # Full-page mode: keep <a name> anchors and turn same-page links
        # into #fragment links so QTextBrowser navigates inside the popup.
        self.internal_links = internal_links

    # --- emit helpers -------------------------------------------------
    def emit(self, text):
        if not self.skip:
            self.out.append(text)

    def push(self, source_tag, emitted, color_spans=0):
        self.stack.append((source_tag, emitted, color_spans))
        self.color_depth += color_spans

    # --- formatting ---------------------------------------------------
    def format_ops(self, tag, attrs):
        attrs = dict(attrs)
        props = parse_style(attrs.get("style", ""))
        ops = []
        family = props.get("font-family", "")
        if tag == "font" and "face" in attrs:
            family += " " + attrs["face"].lower()
        if "courier" in family or "mono" in family:
            ops.append(("code", "<code>", "</code>"))
        if props.get("font-weight", "").startswith("bold"):
            ops.append(("b", "<b>", "</b>"))
        if props.get("font-style", "") == "italic":
            ops.append(("i", "<i>", "</i>"))
        if "underline" in props.get("text-decoration", ""):
            ops.append(("u", "<u>", "</u>"))
        color = attrs.get("color") if tag == "font" else props.get("color")
        if color and parse_color(color) is not None:
            mapped = theme_color(color)
            if mapped is None and self.color_depth > 0:
                # black/gray inside colored text resets to the default text
                # color; without an explicit span it would inherit the color.
                mapped = self.DEFAULT_FOREGROUND
            if mapped:
                ops.append(
                    ("span", '<span style="color:%s;">' % mapped, "</span>")
                )
        return ops

    # --- parser callbacks ----------------------------------------------
    def handle_starttag(self, tag, attrs):
        if tag in self.SKIP_CONTENT_TAGS:
            self.skip += 1
            self.push(tag, [])
            return
        if self.skip:
            self.push(tag, [])
            return
        attrs_dict = dict(attrs)

        if tag == "br":
            self.emit("<br>\n")
        elif tag == "hr":
            self.emit("<hr>")
        elif tag == "meta":
            pass
        elif tag == "img":
            self.handle_img(attrs_dict)
        elif tag == "a":
            href = attrs_dict.get("href")
            if href:
                absolute = urllib.parse.urljoin(MANUAL_URL, href)
                page, _, fragment = absolute.partition("#")
                if self.internal_links and fragment and page == MANUAL_URL:
                    self.emit('<a href="#%s">' % fragment)
                else:
                    self.emit('<a href="%s">' % absolute)
                self.push(tag, ["</a>"])
            elif self.internal_links and attrs_dict.get("name"):
                # Qt's rich-text parser drops empty name anchors; a
                # zero-width space keeps them addressable for navigation.
                self.emit('<a name="%s">&#8203;</a>' % attrs_dict["name"])
                self.push(tag, [])
            else:
                self.push(tag, [])
        elif tag in self.BLOCK_TAGS or tag == "center":
            props = parse_style(attrs_dict.get("style", ""))
            pieces = ["<div"]
            if tag == "center" or props.get("text-align") == "center":
                pieces.append(' align="center"')
            margin = props.get("margin-left", "")
            match = re.match(r"(\d+)px", margin)
            if match:
                pieces.append(' style="margin-left:%spx;"' % match.group(1))
            pieces.append(">")
            self.emit("".join(pieces))
            self.push(tag, ["</div>\n"])
        elif tag == "table":
            self.emit('\n<table border="1" cellspacing="0" cellpadding="5">')
            self.push(tag, ["</table>\n"])
        elif tag in ("tr", "td", "th"):
            kept = "".join(
                ' %s="%s"' % (key, value)
                for key, value in attrs_dict.items()
                if key in ("align", "valign", "colspan", "rowspan")
            )
            self.emit("\n<%s%s>" % (tag, kept))
            self.push(tag, ["</%s>" % tag])
        elif tag in self.PASSTHROUGH_TAGS:
            self.emit("<%s>" % tag)
            self.push(tag, ["</%s>" % tag])
        elif tag in ("span", "font"):
            ops = self.format_ops(tag, attrs)
            for _, open_html, _ in ops:
                self.emit(open_html)
            self.push(
                tag,
                [close for _, _, close in reversed(ops)],
                sum(1 for op in ops if op[0] == "span"),
            )
        else:
            # big/small/tbody/body/... : structure only, no output
            self.push(tag, [])

    def handle_img(self, attrs):
        src = attrs.get("src", "")
        if not src or src.startswith("http"):
            return
        name = local_image_name(src)
        self.images[name] = urllib.parse.unquote(src)
        width = height = None
        props = parse_style(attrs.get("style", ""))
        for source in (props, attrs):
            if width is None and re.match(r"^(\d+)", str(source.get("width", ""))):
                width = int(re.match(r"^(\d+)", str(source["width"])).group(1))
            if height is None and re.match(
                r"^(\d+)", str(source.get("height", ""))
            ):
                height = int(re.match(r"^(\d+)", str(source["height"])).group(1))
        if width and width > 700:
            if height:
                height = round(height * 700 / width)
            width = 700
        size = ""
        if width:
            size += ' width="%d"' % width
        if height:
            size += ' height="%d"' % height
        self.emit('<img src="qrc:/manual/images/%s"%s>' % (name, size))

    def handle_startendtag(self, tag, attrs):
        if tag in ("br", "hr", "img", "meta"):
            self.handle_starttag(tag, attrs)

    def handle_endtag(self, tag):
        if tag in ("br", "hr", "img", "meta"):
            return
        for index in range(len(self.stack) - 1, -1, -1):
            if self.stack[index][0] == tag:
                while len(self.stack) > index:
                    popped_tag, emitted, color_spans = self.stack.pop()
                    self.color_depth -= color_spans
                    if popped_tag in self.SKIP_CONTENT_TAGS:
                        self.skip -= 1
                    for close_html in emitted:
                        self.emit(close_html)
                return
        # stray end tag: ignore

    def handle_data(self, data):
        if self.skip:
            return
        text = (
            data.replace("&", "&amp;")
            .replace("<", "&lt;")
            .replace(">", "&gt;")
            .replace(" ", "&nbsp;")
        )
        self.emit(text)

    def result(self):
        while self.stack:
            popped_tag, emitted, color_spans = self.stack.pop()
            self.color_depth -= color_spans
            if popped_tag in self.SKIP_CONTENT_TAGS:
                self.skip -= 1
            for close_html in emitted:
                self.out.append(close_html)
        html = "".join(self.out)
        # Drop empty formatting pairs left by the span-heavy source. The
        # "a href" form keeps empty <a name="x"></a> target anchors alive.
        empty = re.compile(
            r"<(b|i|u|code|a href[^>]*|span[^>]*|div[^>]*)>(\s*)"
            r"</(b|i|u|code|a|span|div)>"
        )

        def strip_if_matching(match):
            open_tag = re.match(r"[a-z]+", match.group(1)).group(0)
            if open_tag == match.group(3):
                return match.group(2)
            return match.group(0)

        previous = None
        while previous != html:
            previous = html
            html = empty.sub(strip_if_matching, html)
        # No more than two consecutive line breaks.
        html = re.sub(r"(<br>\s*){3,}", "<br>\n<br>\n", html)
        return html.strip()


def split_sections(document):
    anchors = []
    for match in re.finditer(r'<a name="(6\.\d+|8)"', document):
        anchors.append((match.group(1), match.start()))
    ranges = {}
    for index, (name, start) in enumerate(anchors):
        if name == "8":
            continue
        number = int(name.split(".")[1])
        end = anchors[index + 1][1] if index + 1 < len(anchors) else len(document)
        ranges[number] = (start, end)
    return ranges


def download_images(images):
    os.makedirs(os.path.join(OUT_DIR, "images"), exist_ok=True)
    for name, relative in sorted(images.items()):
        target = os.path.join(OUT_DIR, "images", name)
        if os.path.exists(target) and os.path.getsize(target) > 0:
            continue
        url = BASE_URL + urllib.parse.quote(relative)
        request = urllib.request.Request(
            url, headers={"User-Agent": "Mozilla/5.0 (LEparagliding Studio build)"}
        )
        print("downloading", url)
        with urllib.request.urlopen(request, timeout=60) as response:
            data = response.read()
        with open(target, "wb") as fh:
            fh.write(data)


def write_qrc(section_files, images):
    lines = ["<RCC>", '    <qresource prefix="/manual">']
    lines.append("        <file>manual_full.html</file>")
    for filename in sorted(section_files):
        lines.append("        <file>sections/%s</file>" % filename)
    for name in sorted(images):
        lines.append("        <file>images/%s</file>" % name)
    lines += ["    </qresource>", "</RCC>", ""]
    with open(os.path.join(OUT_DIR, "manual.qrc"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines))


def main():
    document = load_manual()
    ranges = split_sections(document)
    missing = [n for n in SECTION_NUMBERS if n not in ranges]
    if missing:
        sys.exit("missing manual anchors for sections: %s" % missing)

    os.makedirs(os.path.join(OUT_DIR, "sections"), exist_ok=True)
    images = {}
    section_files = []
    for number in SECTION_NUMBERS:
        start, end = ranges[number]
        converter = QtRichTextConverter(images)
        converter.feed(document[start:end])
        html = converter.result()
        filename = "section%02d.html" % number
        path = os.path.join(OUT_DIR, "sections", filename)
        with open(path, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(html + "\n")
        section_files.append(filename)
        print("section %2d: %6d chars, images so far: %d" % (number, len(html), len(images)))

    # Complete manual for the in-app Help popup: everything from the page
    # body (intro chapters, all sections, appendices, figure index) with
    # working in-document anchor navigation.
    body_start = document.find("<body")
    converter = QtRichTextConverter(images, internal_links=True)
    converter.feed(document[body_start:])
    full_html = converter.result()
    with open(
        os.path.join(OUT_DIR, "manual_full.html"),
        "w",
        encoding="utf-8",
        newline="\n",
    ) as fh:
        fh.write(full_html + "\n")
    print("full manual: %d chars, images: %d" % (len(full_html), len(images)))

    download_images(images)
    write_qrc(section_files, images)
    print("done: %d sections, %d images" % (len(section_files), len(images)))


if __name__ == "__main__":
    main()
