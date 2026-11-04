#include "plugin.hpp"

Plugin *pluginInstance;

void init(Plugin *p)
{
	pluginInstance = p;
	p->addModel(modelReverb);
	p->addModel(modelBlackHoleReverb);
	p->addModel(modelDarkHall);
	p->addModel(modelDarkHallGranular);
	p->addModel(modelInstantDrone);
	p->addModel(modelSickDelay);
	p->addModel(modelPsyDelay);
	p->addModel(modelPsyLFO);
	p->addModel(modelSpringPendulum);
	p->addModel(modelRandomWalk);
	p->addModel(modelBouncingBalls);
	p->addModel(modelPsyReverb);
	p->addModel(modelLSDrone);
	p->addModel(modelPharmakon);
	p->addModel(modelDice);
	p->addModel(modelDiceOnSteroids);
	p->addModel(modelRainStorm);
	p->addModel(modelStagger);
	p->addModel(modelPsySeq);
	p->addModel(modelHeatRev);
	p->addModel(modelAmbience);
	p->addModel(modelCellAutoGate);
	p->addModel(modelExpressArpeggiator);
	p->addModel(modelMixer6);
	p->addModel(modelMixer3);
	p->addModel(modelSwapper);
	p->addModel(modelFancyADSR);
	p->addModel(modelVCA2);
	p->addModel(modelVCAPoly);
	p->addModel(modelRange);
	p->addModel(modelSampler);
	p->addModel(modelWhirlpool);
	p->addModel(modelVortex);
	p->addModel(modelInfinityRev);
	p->addModel(modelStagger2);
	p->addModel(modelPsyReverb2);
	p->addModel(modelLifeform);
	p->addModel(modelSpikes);
	p->addModel(modelGates);
	p->addModel(modelBastion);
	p->addModel(modelMerger);
	p->addModel(modelSplitter);
	p->addModel(modelPhasmal);
	p->addModel(modelStasis);
	p->addModel(modelSwitchLines8to1);
	p->addModel(modelSwitchLines1to8);
	p->addModel(modelQuantara);
	p->addModel(modelMagicBox);
	p->addModel(modelOscillator);
	p->addModel(modelXBodyProblem);

	// Any other plugin initialization may go here.
	// As an alternative, consider lazy-loading assets and lookup tables when your module is created to reduce startup times of Rack.
}
