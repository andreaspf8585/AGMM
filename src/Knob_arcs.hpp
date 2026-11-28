#pragma once
#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

// Value arcs around every rotary knob of a module (flat track + flat arc).
//
// Usage: call agmm::addKnobArcs(this); as the LAST line of the ModuleWidget
// constructor, after all the addParam() calls.
//
// Params whose range crosses zero (e.g. -1..1) fill from the zero point and
// are drawn warm below it; all others fill from the left end.
// Switches, buttons and sliders get no arc.
namespace agmm
{
    struct KnobArcs : rack::widget::TransparentWidget
    {
        std::vector<rack::app::SvgKnob *> knobs;
        float gap = 4.f; // distance from the knob edge to the middle of the arc

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            const float a0 = -0.75f * (float)M_PI - (float)M_PI / 2.f;
            const float a1 = 0.75f * (float)M_PI - (float)M_PI / 2.f;

            for (rack::app::SvgKnob *knob : knobs)
            {
                if (!knob->visible)
                    continue;
                const rack::math::Vec c = knob->box.getCenter();
                const float R = knob->box.size.x * 0.5f + gap;

                // Track
                nvgBeginPath(vg);
                nvgArc(vg, c.x, c.y, R, a0, a1, NVG_CW);
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
                nvgStrokeWidth(vg, 2.f);
                nvgLineCap(vg, NVG_BUTT);
                nvgStroke(vg);

                // No value in the module browser: track only
                rack::engine::ParamQuantity *q = knob->getParamQuantity();
                if (!q)
                    continue;
                const float mn = q->getMinValue(), mx = q->getMaxValue();
                const bool bipolar = mn < 0.f && mx > 0.f;
                const float v = q->getScaledValue();
                const float from = bipolar ? -mn / (mx - mn) : 0.f;
                const float lo = std::min(from, v), hi = std::max(from, v);
                if (hi - lo < 0.004f)
                    continue;
                nvgBeginPath(vg);
                nvgArc(vg, c.x, c.y, R, a0 + (a1 - a0) * lo, a0 + (a1 - a0) * hi, NVG_CW);
                nvgStrokeColor(vg, (bipolar && v < from) ? warm() : accent());
                nvgStrokeWidth(vg, 2.f);
                nvgStroke(vg);
            }
        }
    };

    // Finds every rotary knob already added to `mw` and draws an arc around it.
    inline void addKnobArcs(rack::app::ModuleWidget *mw, float gap = 4.f)
    {
        KnobArcs *arcs = new KnobArcs;
        arcs->box.size = mw->box.size;
        arcs->gap = gap;
        for (rack::app::ParamWidget *p : mw->getParams())
        {
            if (rack::app::SvgKnob *knob = dynamic_cast<rack::app::SvgKnob *>(p))
                arcs->knobs.push_back(knob);
        }
        mw->addChild(arcs);
    }
} // namespace agmm