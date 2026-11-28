// agmm_ui.hpp - shared flat panel kit for the AGMM modules.
// Everything is flat fills and hairline strokes: no gradients, glows, scanlines or fake logos.
//
// -----------------------------------------------------------------------------
// OVERVIEW
// -----------------------------------------------------------------------------
// This header is the shared "UI kit" used by every AGMM module panel. It bundles
// the common drawing primitives (text, panels, bezels, screens, plates, headers,
// meters, chips) and the light widgets, so each module's widget.cpp only needs
// to lay out its own controls on top of a consistent look.
//
// Design rules enforced here:
//   - Flat fills and 1px hairline strokes only.
//   - A fixed dark palette (bg / text / dim / accent / violet / warm).
//   - A single monospaced font (ShareTechMono) loaded from Rack's system assets.
//   - No text smaller than 7px, since Rack scales panels down on zoom-out.
//
// The functions are all inline so this header can be included from multiple
// translation units without violating the ODR.
// -----------------------------------------------------------------------------
#pragma once
#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <algorithm>

namespace agmm
{

    // NanoVG text alignment presets: horizontal position + vertical centring.
    constexpr int ALIGN_L = NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE;
    constexpr int ALIGN_C = NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE;
    constexpr int ALIGN_R = NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE;

    // ---- palette ----

    // Panel background gradient start colour (top of the panel).
    inline NVGcolor bgTop() { return nvgRGB(0x23, 0x26, 0x2d); }

    // Panel background gradient end colour (bottom of the panel).
    inline NVGcolor bgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }

    // Default body text colour (light grey-blue).
    inline NVGcolor textCol() { return nvgRGB(0xc9, 0xd1, 0xdc); }

    // Muted colour for secondary labels and inactive states.
    inline NVGcolor dim() { return nvgRGB(0x8a, 0x95, 0xa4); }

    // Primary accent colour (cyan) used for highlights and active elements.
    inline NVGcolor accent() { return nvgRGB(0x3d, 0xe0, 0xff); }

    // Brighter accent variant for emphasis.
    inline NVGcolor bright() { return nvgRGB(0x8a, 0xee, 0xff); }

    // Secondary accent (violet) for alternate signal paths / lights.
    inline NVGcolor violet() { return nvgRGB(0xb0, 0x5a, 0xff); }

    // Warm accent (orange) for warning / alternate states.
    inline NVGcolor warm() { return nvgRGB(0xff, 0x9c, 0x6a); }

    // Returns a copy of `c` with its alpha channel replaced by `a` (0..1).
    inline NVGcolor withA(NVGcolor c, float a)
    {
        c.a = a;
        return c;
    }

    // ---- text ----

    // Loads the AGMM monospace font and makes it the current NanoVG font face.
    // Returns false if there is no window yet or the font asset is missing,
    // in which case callers should skip drawing text.
    inline bool setFont(NVGcontext *vg)
    {
        if (!APP || !APP->window)
            return false;
        std::shared_ptr<window::Font> font =
            APP->window->loadFont(asset::system("res/fonts/ShareTechMono-Regular.ttf"));
        if (!font)
            return false;
        nvgFontFaceId(vg, font->handle);
        return true;
    }

    // Draws string `s` at (x, y) with the given pixel size, colour, alignment
    // and letter spacing. Size is clamped to a minimum of 7px because smaller
    // text is unreadable once Rack scales the panel down.
    inline void text(NVGcontext *vg, float x, float y, const char *s, float size, NVGcolor col,
                     int align = ALIGN_C, float spacing = 0.5f)
    {
        if (!setFont(vg))
            return;
        nvgFontSize(vg, std::max(size, 7.f));
        nvgTextLetterSpacing(vg, spacing);
        nvgTextAlign(vg, align);
        nvgFillColor(vg, col);
        nvgText(vg, x, y, s, NULL);
    }

    // Measures the rendered width in pixels of string `s` at the given size and
    // letter spacing. Useful for laying out chips and headers whose contents
    // vary at runtime.
    inline float textWidth(NVGcontext *vg, const char *s, float size, float spacing = 0.5f)
    {
        if (!setFont(vg))
            return 0.f;
        nvgFontSize(vg, std::max(size, 7.f));
        nvgTextLetterSpacing(vg, spacing);
        float b[4];
        nvgTextBounds(vg, 0.f, 0.f, s, NULL, b);
        return b[2] - b[0];
    }

    // ---- panel ----

    // Paints the module background: a flat vertical gradient, the module title
    // centred near the top (y = 15) and the "AGMM" maker tag near the bottom
    // (y = H - 13).
    inline void panelBase(NVGcontext *vg, float W, float H, const char *title)
    {
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, W, H);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, H, bgTop(), bgBottom()));
        nvgFill(vg);
        text(vg, W / 2.f, 15.f, title, 12.f, textCol(), ALIGN_C, 3.f);
        text(vg, W / 2.f, H - 13.f, "AGMM", 9.f, dim(), ALIGN_C, 3.f);
    }

    // Draws a dark recessed frame around a display. The display area itself is
    // the rectangle (x, y, w, h); the bezel extends 4px beyond it on all sides.
    inline void bezel(NVGcontext *vg, float x, float y, float w, float h)
    {
        nvgBeginPath(vg);
        nvgRoundedRect(vg, x - 4.f, y - 4.f, w + 8.f, h + 8.f, 5.f);
        nvgFillColor(vg, nvgRGB(0x0b, 0x0c, 0x0e));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    // Fills a flat near-black display background at (x, y, w, h).
    inline void screen(NVGcontext *vg, float x, float y, float w, float h)
    {
        nvgBeginPath(vg);
        nvgRect(vg, x, y, w, h);
        nvgFillColor(vg, nvgRGB(0x05, 0x08, 0x0c));
        nvgFill(vg);
    }

    // Draws the translucent dark frame used to host a plot/graph, with a faint
    // cyan hairline border.
    inline void plotFrame(NVGcontext *vg, float x, float y, float w, float h)
    {
        nvgBeginPath(vg);
        nvgRect(vg, x, y, w, h);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    // Draws a rounded control plate centred horizontally on `cx` with its top
    // edge at `top`. When `tint` is true the plate is drawn in accent cyan,
    // which is used to mark output jacks.
    inline void plate(NVGcontext *vg, float cx, float top, float w, float h, bool tint = false)
    {
        nvgBeginPath(vg);
        nvgRoundedRect(vg, cx - w / 2.f, top, w, h, 5.f);
        nvgFillColor(vg, tint ? nvgRGBA(0x3d, 0xe0, 0xff, 22) : nvgRGBA(0, 0, 0, 55));
        nvgFill(vg);
        nvgStrokeColor(vg, tint ? nvgRGBA(0x3d, 0xe0, 0xff, 60) : nvgRGBA(255, 255, 255, 22));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    // A plate with a text label centred 10px below its top edge. `tint` follows
    // the same meaning as in plate(); `labelSize` controls the label font size.
    inline void cell(NVGcontext *vg, float cx, float top, float w, float h, const char *label,
                     bool tint = false, float labelSize = 8.f)
    {
        plate(vg, cx, top, w, h, tint);
        text(vg, cx, top + 10.f, label, labelSize, dim(), ALIGN_C, 0.5f);
    }

    // Draws a centred section title with horizontal hairlines on either side.
    // The hairlines stop a measured distance from the text (based on its
    // rendered width) so they never touch the letters. `margin` is the gap
    // from the panel edges to the start/end of the lines.
    inline void sectionHeader(NVGcontext *vg, float W, float y, const char *txt, float margin = 12.f)
    {
        const float cx = W / 2.f;
        float gap = textWidth(vg, txt, 8.f, 2.f) / 2.f + 6.f;
        text(vg, cx, y, txt, 8.f, accent(), ALIGN_C, 2.f);
        nvgBeginPath(vg);
        nvgMoveTo(vg, margin, y);
        nvgLineTo(vg, cx - gap, y);
        nvgMoveTo(vg, cx + gap, y);
        nvgLineTo(vg, W - margin, y);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 70));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    // Vertical level meter at (x, y, w, h): a faint track plus a flat fill that
    // rises from the bottom in proportion to `level` (clamped to 0..1). The
    // fill turns red once the level exceeds 0.9 as a clip warning.
    inline void meter(NVGcontext *vg, float x, float y, float w, float h, float level)
    {
        level = std::min(std::max(level, 0.f), 1.f);
        nvgBeginPath(vg);
        nvgRect(vg, x, y, w, h);
        nvgFillColor(vg, nvgRGBA(255, 255, 255, 14));
        nvgFill(vg);
        float fh = level * h;
        if (fh > 0.5f)
        {
            nvgBeginPath(vg);
            nvgRect(vg, x, y + h - fh, w, fh);
            nvgFillColor(vg, level > 0.9f ? nvgRGB(0xff, 0x50, 0x50) : accent());
            nvgFill(vg);
        }
    }

    // Small status chip (e.g. REC / PLAY) at (x, y, w, h). When `on` it is
    // filled and outlined in `col` with matching text; when off it shows only a
    // faint outline and dim text. Text must fit the chip width: use textWidth()
    // when choosing `w`.
    inline void chip(NVGcontext *vg, float x, float y, float w, float h, const char *txt, bool on, NVGcolor col)
    {
        nvgBeginPath(vg);
        nvgRoundedRect(vg, x, y, w, h, 3.f);
        if (on)
        {
            nvgFillColor(vg, withA(col, 0.25f));
            nvgFill(vg);
            nvgStrokeColor(vg, withA(col, 0.9f));
        }
        else
        {
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
        }
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
        text(vg, x + w / 2.f, y + h / 2.f + 0.5f, txt, 7.f, on ? col : dim(), ALIGN_C, 0.5f);
    }

    // ---- lights ----

    // Cyan LED widget, templated on the base light widget class so it can be
    // used as a plain light or as a multi-light / momentary variant.
    template <typename TBase = GrayModuleLightWidget>
    struct TCyanLight : TBase
    {
        // Registers the cyan base colour with the underlying light widget.
        TCyanLight() { this->addBaseColor(nvgRGB(0x3d, 0xe0, 0xff)); }
    };
    using CyanLight = TCyanLight<>;

    // Violet LED widget, templated the same way as TCyanLight.
    template <typename TBase = GrayModuleLightWidget>
    struct TVioletLight : TBase
    {
        // Registers the violet base colour with the underlying light widget.
        TVioletLight() { this->addBaseColor(nvgRGB(0xb0, 0x5a, 0xff)); }
    };
    using VioletLight = TVioletLight<>;

} // namespace agmm