// agmm_ui.hpp - shared flat panel kit for the AGMM modules.
// Everything is flat fills and hairline strokes: no gradients, glows, scanlines or fake logos.
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

    constexpr int ALIGN_L = NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE;
    constexpr int ALIGN_C = NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE;
    constexpr int ALIGN_R = NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE;

    // ---- palette ----
    inline NVGcolor bgTop() { return nvgRGB(0x23, 0x26, 0x2d); }
    inline NVGcolor bgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }
    inline NVGcolor textCol() { return nvgRGB(0xc9, 0xd1, 0xdc); }
    inline NVGcolor dim() { return nvgRGB(0x8a, 0x95, 0xa4); }
    inline NVGcolor accent() { return nvgRGB(0x3d, 0xe0, 0xff); }
    inline NVGcolor bright() { return nvgRGB(0x8a, 0xee, 0xff); }
    inline NVGcolor violet() { return nvgRGB(0xb0, 0x5a, 0xff); }
    inline NVGcolor warm() { return nvgRGB(0xff, 0x9c, 0x6a); }
    inline NVGcolor withA(NVGcolor c, float a)
    {
        c.a = a;
        return c;
    }

    // ---- text ----
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

    // Minimum size is 7 px: smaller text is unreadable in Rack.
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
    // Flat background, title centred at y = 15, maker tag centred at y = H - 13.
    inline void panelBase(NVGcontext *vg, float W, float H, const char *title)
    {
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, W, H);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, H, bgTop(), bgBottom()));
        nvgFill(vg);
        text(vg, W / 2.f, 15.f, title, 12.f, textCol(), ALIGN_C, 3.f);
        text(vg, W / 2.f, H - 13.f, "AGMM", 9.f, dim(), ALIGN_C, 3.f);
    }

    // Dark recessed frame around a display (display itself is x,y,w,h).
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

    // Flat display background, local coordinates.
    inline void screen(NVGcontext *vg, float x, float y, float w, float h)
    {
        nvgBeginPath(vg);
        nvgRect(vg, x, y, w, h);
        nvgFillColor(vg, nvgRGB(0x05, 0x08, 0x0c));
        nvgFill(vg);
    }

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

    // Control plate centred on cx, top edge at `top`. tint = output jacks.
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

    // Plate with its label centred 10 px below the top edge.
    inline void cell(NVGcontext *vg, float cx, float top, float w, float h, const char *label,
                     bool tint = false, float labelSize = 8.f)
    {
        plate(vg, cx, top, w, h, tint);
        text(vg, cx, top + 10.f, label, labelSize, dim(), ALIGN_C, 0.5f);
    }

    // Section title; the hairlines stop a measured distance from the text.
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

    // Vertical level meter: track + flat fill (red near the top).
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

    // Status chip (e.g. REC / PLAY). Text must fit: use textWidth when sizing w.
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
    template <typename TBase = GrayModuleLightWidget>
    struct TCyanLight : TBase
    {
        TCyanLight() { this->addBaseColor(nvgRGB(0x3d, 0xe0, 0xff)); }
    };
    using CyanLight = TCyanLight<>;

    template <typename TBase = GrayModuleLightWidget>
    struct TVioletLight : TBase
    {
        TVioletLight() { this->addBaseColor(nvgRGB(0xb0, 0x5a, 0xff)); }
    };
    using VioletLight = TVioletLight<>;

} // namespace agmm