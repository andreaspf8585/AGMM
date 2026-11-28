#include "plugin.hpp"
#include "agmm_ui.hpp"

// LuckyGate - probabilistic gate.
// Every rising edge on the GATE input rolls a die against the PROB knob (plus
// optional CV). If the roll succeeds the output follows the input gate for its
// whole duration, otherwise the output stays low. Fully polyphonic (1..16 ch),
// each channel rolls independently.

struct LuckyGate : Module {
	enum ParamId {
		PROB_PARAM,
		PARAMS_LEN
	};
	enum InputId {
		GATE_INPUT,
		PROB_CV_INPUT,
		INPUTS_LEN
	};
	enum OutputId {  
		GATE_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		LIGHTS_LEN
	};

	static constexpr int MAX_CH = 16;
	static constexpr int HIST_LEN = 32;

	dsp::SchmittTrigger gateTrig[MAX_CH];
	bool pass[MAX_CH] = {};

	// ---- state read by the UI (written from the audio thread) ----
	float passLvl[MAX_CH] = {};   // fading "passed" brightness per channel
	float rejLvl[MAX_CH] = {};    // fading "rejected" brightness per channel
	uint8_t hist[HIST_LEN] = {};  // 0 = empty, 1 = pass, 2 = reject
	int histHead = 0;
	uint32_t rolls = 0;
	uint32_t passes = 0;
	int activeChannels = 0;

	LuckyGate() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(PROB_PARAM, 0.f, 1.f, 0.5f, "Probability", "%", 0.f, 100.f);
		configInput(GATE_INPUT, "Gate");
		configInput(PROB_CV_INPUT, "Probability CV (0-10V)");
		configOutput(GATE_OUTPUT, "Gate");
		configBypass(GATE_INPUT, GATE_OUTPUT);
	}

	void onReset() override {
		for (int c = 0; c < MAX_CH; c++) {
			gateTrig[c].reset();
			pass[c] = false;
			passLvl[c] = rejLvl[c] = 0.f;
		}
		for (int i = 0; i < HIST_LEN; i++)
			hist[i] = 0;
		histHead = 0;
		rolls = passes = 0;
	}

	void process(const ProcessArgs& args) override {
		int ch = inputs[GATE_INPUT].getChannels();
		activeChannels = ch;
		int outCh = std::max(1, ch);
		outputs[GATE_OUTPUT].setChannels(outCh);

		const float baseProb = params[PROB_PARAM].getValue();
		const bool hasCv = inputs[PROB_CV_INPUT].isConnected();
		const float decay = std::exp(-args.sampleTime * 10.f);

		for (int c = 0; c < MAX_CH; c++) {
			if (c >= ch) {
				// Unused channel: clear state and fade the display.
				gateTrig[c].reset();
				pass[c] = false;
				passLvl[c] *= decay;
				rejLvl[c] *= decay;
				if (c < outCh)
					outputs[GATE_OUTPUT].setVoltage(0.f, c);
				continue;
			}

			bool wasHigh = gateTrig[c].isHigh();
			gateTrig[c].process(inputs[GATE_INPUT].getVoltage(c), 0.1f, 2.f);
			bool high = gateTrig[c].isHigh();

			if (high && !wasHigh) {
				float p = baseProb;
				if (hasCv)
					p += inputs[PROB_CV_INPUT].getPolyVoltage(c) / 10.f;
				p = clamp(p, 0.f, 1.f);
				pass[c] = random::uniform() < p;

				rolls++;
				if (pass[c])
					passes++;
				if (rolls > 100000) {
					rolls /= 2;
					passes /= 2;
				}
				hist[histHead] = pass[c] ? 1 : 2;
				histHead = (histHead + 1) % HIST_LEN;
			}
			if (!high)
				pass[c] = false;

			bool outHigh = high && pass[c];
			outputs[GATE_OUTPUT].setVoltage(outHigh ? 10.f : 0.f, c);

			if (outHigh)
				passLvl[c] = 1.f;
			else
				passLvl[c] *= decay;
			if (high && !pass[c])
				rejLvl[c] = 1.f;
			else
				rejLvl[c] *= decay;
		}
	}
};


// ---------------------------------------------------------------------------
// Panel + visualisation
// ---------------------------------------------------------------------------

struct LuckyGatePanel : TransparentWidget {
	LuckyGate* module = nullptr;

	// Layout constants shared with the widget below.
	static constexpr float KNOB_X = 60.f;
	static constexpr float KNOB_Y = 92.f;
	static constexpr float SX = 12.f, SY = 160.f, SW = 96.f, SH = 80.f;

	void draw(const DrawArgs& args) override {
		NVGcontext* vg = args.vg;
		const float W = box.size.x, H = box.size.y;

		agmm::panelBase(vg, W, H, "LUCKY");

		// ---- probability ring ----
		agmm::sectionHeader(vg, W, 38.f, "PROBABILITY");

		float prob = module ? module->params[LuckyGate::PROB_PARAM].getValue() : 0.65f;
		const float a0 = -M_PI / 2.f - 0.83f * M_PI;
		const float a1 = -M_PI / 2.f + 0.83f * M_PI;
		const float r = 28.f;

		nvgBeginPath(vg);
		nvgArc(vg, KNOB_X, KNOB_Y, r, a0, a1, NVG_CW);
		nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
		nvgStrokeWidth(vg, 3.f);
		nvgLineCap(vg, NVG_ROUND);
		nvgStroke(vg);

		if (prob > 0.002f) {
			nvgBeginPath(vg);
			nvgArc(vg, KNOB_X, KNOB_Y, r, a0, a0 + prob * (a1 - a0), NVG_CW);
			nvgStrokeColor(vg, agmm::accent());
			nvgStrokeWidth(vg, 3.f);
			nvgLineCap(vg, NVG_ROUND);
			nvgStroke(vg);
		}
		nvgLineCap(vg, NVG_BUTT);

		// CV-modulated effective probability marker (first channel).
		if (module && module->inputs[LuckyGate::PROB_CV_INPUT].isConnected()) {
			float eff = clamp(prob + module->inputs[LuckyGate::PROB_CV_INPUT].getVoltage(0) / 10.f, 0.f, 1.f);
			float ang = a0 + eff * (a1 - a0);
			nvgBeginPath(vg);
			nvgCircle(vg, KNOB_X + std::cos(ang) * (r + 6.f), KNOB_Y + std::sin(ang) * (r + 6.f), 2.f);
			nvgFillColor(vg, agmm::violet());
			nvgFill(vg);
		}

		char buf[32];
		std::snprintf(buf, sizeof(buf), "%d%%", (int) std::round(prob * 100.f));
		agmm::text(vg, KNOB_X, 129.f, buf, 11.f, agmm::textCol(), agmm::ALIGN_C, 1.f);

		// ---- roll display ----
		agmm::sectionHeader(vg, W, 148.f, "ROLLS");
		agmm::bezel(vg, SX, SY, SW, SH);
		agmm::screen(vg, SX, SY, SW, SH);

		// 16 channel cells (8 x 2).
		const float pitch = (SW - 6.f) / 8.f;
		for (int c = 0; c < 16; c++) {
			float x = SX + 3.f + (c % 8) * pitch;
			float y = SY + 4.f + (c / 8) * 12.f;
			float w = pitch - 2.f, h = 9.f;

			bool used = module ? c < module->activeChannels : c < 6;
			float pl = module ? module->passLvl[c] : (c % 3 == 0 ? 1.f : 0.f);
			float rl = module ? module->rejLvl[c] : (c % 3 == 1 ? 1.f : 0.f);

			nvgBeginPath(vg);
			nvgRoundedRect(vg, x, y, w, h, 2.f);
			nvgFillColor(vg, nvgRGBA(255, 255, 255, used ? 18 : 6));
			nvgFill(vg);
			if (pl > 0.01f) {
				nvgBeginPath(vg);
				nvgRoundedRect(vg, x, y, w, h, 2.f);
				nvgFillColor(vg, agmm::withA(agmm::accent(), 0.2f + 0.8f * pl));
				nvgFill(vg);
			} else if (rl > 0.01f) {
				nvgBeginPath(vg);
				nvgRoundedRect(vg, x, y, w, h, 2.f);
				nvgFillColor(vg, agmm::withA(agmm::warm(), 0.15f + 0.6f * rl));
				nvgFill(vg);
			}
			if (used) {
				nvgBeginPath(vg);
				nvgRoundedRect(vg, x, y, w, h, 2.f);
				nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 40));
				nvgStrokeWidth(vg, 1.f);
				nvgStroke(vg);
			}
		}

		// Separator.
		nvgBeginPath(vg);
		nvgMoveTo(vg, SX + 3.f, SY + 28.f);
		nvgLineTo(vg, SX + SW - 3.f, SY + 28.f);
		nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
		nvgStrokeWidth(vg, 1.f);
		nvgStroke(vg);

		// Roll history: tall accent bar = passed, short warm bar = rejected.
		const float base = SY + SH - 6.f;
		const float colPitch = (SW - 6.f) / LuckyGate::HIST_LEN;
		for (int i = 0; i < LuckyGate::HIST_LEN; i++) {
			int v;
			if (module)
				v = module->hist[(module->histHead + i) % LuckyGate::HIST_LEN];
			else
				v = ((i * 7 + 3) % 5 < 3) ? 1 : 2;
			float x = SX + 3.f + i * colPitch;
			float age = 0.35f + 0.65f * ((float) (i + 1) / LuckyGate::HIST_LEN);

			// Faint baseline tick for every slot.
			nvgBeginPath(vg);
			nvgRect(vg, x, base, colPitch - 0.8f, 1.f);
			nvgFillColor(vg, nvgRGBA(255, 255, 255, 25));
			nvgFill(vg);

			if (v == 0)
				continue;
			float h = (v == 1) ? 30.f : 9.f;
			nvgBeginPath(vg);
			nvgRect(vg, x, base - h, colPitch - 0.8f, h);
			nvgFillColor(vg, agmm::withA(v == 1 ? agmm::accent() : agmm::warm(), age));
			nvgFill(vg);
		}

		// Stats line.
		if (module && module->rolls > 0)
			std::snprintf(buf, sizeof(buf), "HIT %d%%", (int) std::round(100.f * module->passes / (float) module->rolls));
		else if (module)
			std::snprintf(buf, sizeof(buf), "HIT --");
		else
			std::snprintf(buf, sizeof(buf), "HIT 64%%");
		agmm::text(vg, SX + 2.f, 251.f, buf, 8.f, agmm::dim(), agmm::ALIGN_L, 0.5f);
		std::snprintf(buf, sizeof(buf), "CH %d", module ? module->activeChannels : 6);
		agmm::text(vg, SX + SW - 2.f, 251.f, buf, 8.f, agmm::dim(), agmm::ALIGN_R, 0.5f);

		// ---- jack plates ----
		agmm::cell(vg, 33.f, 262.f, 48.f, 46.f, "CV");
		agmm::cell(vg, 87.f, 262.f, 48.f, 46.f, "GATE IN");
		agmm::cell(vg, 60.f, 314.f, 48.f, 46.f, "GATE OUT", true);
	}
};


struct LuckyGateWidget : ModuleWidget {
	LuckyGateWidget(LuckyGate* module) {
		setModule(module);
		box.size = Vec(8 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT);

		auto* bg = new LuckyGatePanel;
		bg->module = module;
		bg->box.size = box.size;
		addChild(bg);

		addParam(createParamCentered<RoundBigBlackKnob>(Vec(LuckyGatePanel::KNOB_X, LuckyGatePanel::KNOB_Y), module, LuckyGate::PROB_PARAM));

		addInput(createInputCentered<PJ301MPort>(Vec(33.f, 262.f + 31.f), module, LuckyGate::PROB_CV_INPUT));
		addInput(createInputCentered<PJ301MPort>(Vec(87.f, 262.f + 31.f), module, LuckyGate::GATE_INPUT));
		addOutput(createOutputCentered<PJ301MPort>(Vec(60.f, 314.f + 31.f), module, LuckyGate::GATE_OUTPUT));
	}
};


Model* modelLuckyGate = createModel<LuckyGate, LuckyGateWidget>("LuckyGate");