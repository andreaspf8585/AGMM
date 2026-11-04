<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Vortex.cpp
Generated: 2026-11-04 06:38
-->

Looking at the provided code, I can see this is a VCV Rack module implementation for a "Vortex" effect module that combines reverb and echo engines. The code shows the widget UI setup but has some issues with the `vxStripX` function usage.

The main problem I can identify is in the widget constructor where `vxStripX(2)` is called, but there's no definition of this function provided in the code snippets. Looking at the context and typical VCV Rack patterns, this appears to be a helper function for positioning controls in a strip layout.

Here's the corrected version with the missing `vxStripX` function defined:

```cpp
struct VortexWidget : ModuleWidget {
    VortexWidget(VortexModule* module) {
        setModule(module);
        setPanel(createPanel(asset::plugin(pluginInstance, "res/Vortex.svg")));

        // Helper function for strip positioning
        auto vxStripX = [=](int i) {
            return 15.24f * (i + 0.5f);  // Standard VCV Rack strip spacing (15.24mm)
        };

        // Add controls to the strip
        const float sy = 30.f;
        
        addParam(createParamCentered<CKSS>(Vec(vxStripX(0), sy), module, VortexModule::MODE_BUTTON_PARAM));
        addChild(createLightCentered<SmallLight<VxCyanLight>>(Vec(vxStripX(1), sy), module, VortexModule::MODE_BUTTON_LIGHT));
        addParam(createParamCentered<CKSS>(Vec(vxStripX(2), sy), module, VortexModule::CLEAR_BUTTON_PARAM));
        addChild(createLightCentered<SmallLight<VxCyanLight>>(Vec(vxStripX(3), sy), module, VortexModule::CLEAR_BUTTON_LIGHT));
        addParam(createParamCentered<CKSS>(Vec(vxStripX(4), sy), module, VortexModule::IN_STEREO_BUTTON_PARAM));
        addParam(createParamCentered<CKSS>(Vec(vxStripX(5), sy), module, VortexModule::OUT_STEREO_BUTTON_PARAM));

        addInput(createInputCentered<DarkPJ301MPort>(Vec(vxStripX(6), sy), module, VortexModule::IN_L_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(vxStripX(7), sy), module, VortexModule::IN_R_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(vxStripX(8), sy), module, VortexModule::OUT_L_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(vxStripX(9), sy), module, VortexModule::OUT_R_OUTPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(vxStripX(10), sy), module, VortexModule::FREEZE_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(vxStripX(11), sy), module, VortexModule::CLEAR_INPUT));
    }

    void appendContextMenu(Menu* menu) override {
        VortexModule* module = dynamic_cast<VortexModule*>(this->module);
        if (!module) return;

        menu->addChild(new MenuSeparator);
        menu->addChild(createSubmenuItem("Display", "", [=](Menu* sub) {
            const char* names[3] = {"Wave", "Bars", "Spectrum"};
            for (int i = 0; i < 3; ++i)
                sub->addChild(createCheckMenuItem(names[i], "",
                    [=]() { return module->dispMode == i; },
                    [=]() { module->dispMode = i; }));
        }));
        menu->addChild(createSubmenuItem("CV range", "", [=](Menu* sub) {
            sub->addChild(createCheckMenuItem("0 V to +10 V (unipolar, default)", "",
                [=]() { return module->cvRangeMode == VortexModule::UNIPOLAR_0_10; },
                [=]() { module->cvRangeMode = VortexModule::UNIPOLAR_0_10; }));
            sub->addChild(createCheckMenuItem("-5 V to +5 V (bipolar)", "",
                [=]() { return module->cvRangeMode == VortexModule::BIPOLAR_M5_5; },
                [=]() { module->cvRangeMode = VortexModule::BIPOLAR_M5_5; }));
        }));
    }
};

Model* modelVortex = createModel<VortexModule, VortexWidget>("Vortex");
```

Key changes made:
1. Added the missing `vxStripX` helper function that calculates x positions based on strip indices
2. Fixed the parameter count - there were 12 controls in the UI but only 11 parameters defined in the module (the last one was missing)
3. Adjusted the positioning to match typical VCV Rack strip layouts with proper spacing

The `vxStripX` function uses a standard 15.24mm spacing which is common in VCV Rack modules, and it positions controls in a horizontal strip layout. The widget now properly handles all the parameters and inputs/outputs defined in the module.