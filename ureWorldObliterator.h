#include "PureWorldObliterator.h"

void WorldObliteratorEngine::prepare(double sr) {
    sampleRate = sr;
    for (int i = 0; i < 2; ++i) {
        mainLP[i].reset();
        dcBlock[i].setup(20.0f, 0.707f, sr);     
        antiFizz[i].setup(5000.0f, 0.707f, sr);  
        crossLow[i].setup(120.0f, sr);           
        crossHigh[i].setup(120.0f, sr);
    }
}

void WorldObliteratorEngine::process(float** buffer, int numSamples) {
    float* left = buffer[0];
    float* right = buffer[1];

    float inGain = inputIntake * 4.0f;      
    float outGain = outputDoomsday * 4.0f;  
    float driveBase = 1.0f + (impactDistortion / 5.0f);
    float freq = 20.0f + (targetFreq * 1.8f); 
    float q = 0.1f + (resonance * 3.9f);    
    
    float compensation = autoGainActive ? (1.0f / std::sqrt(driveBase)) : 1.0f;

    for (int s = 0; s < numSamples; ++s) {
        float channels[2] = { left[s], right[s] };

        for (int c = 0; c < 2; ++c) {
            float x = channels[c];

            x *= inGain;
            float dry = x;

            float currentDrive = driveBase;
            float punchIn = x;
            if (punchActive) {
                punchIn *= 1.5f;
                currentDrive *= 0.5f;
            }

            float wet = applyAsymmetricDistortion(punchIn, currentDrive);

            mainLP[c].setup(freq, q, sampleRate);
            wet = mainLP[c].processLP(wet);
            
            wet = antiFizz[c].processLP(wet);
            wet = dcBlock[c].processHP(wet);

            channels[c] = (dry * (1.0f - mixBlend)) + (wet * mixBlend);
        }

        float l_low = crossLow[0].processLP(channels[0]);
        float r_low = crossLow[1].processLP(channels[1]);
        float l_high = crossHigh[0].processHP(channels[0]);
        float r_high = crossHigh[1].processHP(channels[1]);

        float monoLow = (l_low + r_low) * 0.5f;

        float outL = (monoLow + l_high) * outGain * compensation;
        float outR = (monoLow + r_high) * outGain * compensation;

        left[s] = std::max(-1.0f, std::min(1.0f, outL));
        right[s] = std::max(-1.0f, std::min(1.0f, outR));
    }
}
