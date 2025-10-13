#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <Arduino.h>

class ProfileStep {
  public:
    // Target temperature for this step
    double finalTempT; 
    double finalTempB;

    // Duration of this step in milliseconds, 
    // calculated by Init() based on this step's behavior and the previous step's final temperature
    unsigned long duration; 
    
    // given the final target temperature of the last step, start this step
    virtual void Init(double endTempT, double endTempB) = 0;

    // Reset any internal variables to prepare for a new profile run
    virtual void Start() = 0;

    // update the setpoints based on the elapsed time and this step's behavior
    virtual void Update(double* SetpointT, double* SetpointB) = 0;

    // Signal that the next step should start
    virtual bool IsComplete() = 0;

    // Draw itself on a graph sprite at position x,y with given step sizes (1 pixel = stepX in time, 1 pixel = stepY in temperature)
    virtual void DrawOnGraph(TFT_eSprite* graphSprite, uint16_t startX, uint16_t startY, double stepX, double stepY) = 0;
};
