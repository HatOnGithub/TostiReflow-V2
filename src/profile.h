#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <Arduino.h>
#include <LittleFS.h>
#include "config.h"

double* pInputTop;
double* pInputBottom;
double* pSetpointTop;
double* pSetpointBottom;

double start = 25;


class ProfileStep {
  public:
    static void begin(double* inputTop, double* inputBottom, double* setpointTop, double* setpointBottom) {
      pInputTop = inputTop;
      pInputBottom = inputBottom;
      pSetpointTop = setpointTop;
      pSetpointBottom = setpointBottom;
    }
    // Target temperature for this step
    double finalTempT, finalTempB;
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



class NameStep : public ProfileStep {
  public:
    NameStep(String* currentPhase, String phaseName) : currentPhase(currentPhase), phaseName(phaseName) {
      finalTempT = 0;
      finalTempB = 0;
      duration = 0;
    }
    void Init(double endTempT, double endTempB) override {
      // does not affect temperatures
      finalTempT = endTempT;
      finalTempB = endTempB;
    }
    void Start() override {
      *currentPhase = phaseName;
    }
    void Update(double* SetpointT, double* SetpointB) override {
      // does nothing
    }
    bool IsComplete() override {
      return true; // always complete
    }
    void DrawOnGraph(TFT_eSprite* graphSprite, uint16_t startX, uint16_t startY, double stepX, double stepY) override {
      // does nothing
    }
    String phaseName;
  private:
    String* currentPhase;
};

class BeepStep : public ProfileStep {
  public:
    BeepStep(unsigned long duration) {
      this->duration = duration;
      finalTempT = 0;
      finalTempB = 0;
    }
    void Init(double endTempT, double endTempB) override {
      // does not affect temperatures
      finalTempT = endTempT;
      finalTempB = endTempB;
    }
    void Start() override {
      startTime = millis();
      // Trigger beep here
      tone(BUZZER_PIN, 1000); // 1kHz tone
    }
    void Update(double* SetpointT, double* SetpointB) override {
      // does nothing
    }
    bool IsComplete() override {
      if (millis() - startTime >= duration) {
        noTone(BUZZER_PIN); // Stop the tone
        return true;
      }
      return false;
    }
    void DrawOnGraph(TFT_eSprite* graphSprite, uint16_t startX, uint16_t startY, double stepX, double stepY) override {
      // does nothing
    }
  private:
    unsigned long startTime; // time when this step started
};

// ------------- Profile steps for combined heating of top and bottom heater ------------
// Combined steps will always have the convection fan on due to the desire to equally heat both sides
// Linear ramp step
class LinearRateStep : public ProfileStep {
  public:
    LinearRateStep(double finalTemp, double rate);
    void Init(double endTempT, double endTempB) override;
    void Start() override;
    void Update(double* SetpointT, double* SetpointB) override;
    bool IsComplete() override;
    void DrawOnGraph(TFT_eSprite* graphSprite, uint16_t startX, uint16_t startY, double stepX, double stepY) override;
  private:
    double startTempT, startTempB; // starting temperatures for this step
    unsigned long startTime; // time when this step started
};

class LinearTimeStep : public ProfileStep {
  public:
    LinearTimeStep(double finalTemp, unsigned long duration);
    void Init(double endTempT, double endTempB) override;
    void Start() override;
    void Update(double* SetpointT, double* SetpointB) override;
    bool IsComplete() override;
    void DrawOnGraph(TFT_eSprite* graphSprite, uint16_t startX, uint16_t startY, double stepX, double stepY) override;
  private:
    double startTempT, startTempB; // starting temperatures for this step
    unsigned long startTime; // time when this step started
    double rate; // rate of temperature change in degrees per minute
};

// AFAP step -> may cause overshoot
class InstantStep : public ProfileStep {
  public:
    InstantStep(double finalTemp, unsigned long duration);
    void Init(double endTempT, double endTempB) override;
    void Start() override;
    void Update(double* SetpointT, double* SetpointB) override;
    bool IsComplete() override;
    void DrawOnGraph(TFT_eSprite* graphSprite, uint16_t startX, uint16_t startY, double stepX, double stepY) override;
};



struct Profile{
  public:
    Profile() : name(""), currentPhase("Idle"), stepCount(0), currentStep(0), running(false) {}
    ProfileStep* steps[MAX_PROFILE_STEPS];
    String name, currentPhase;
    uint8_t stepCount;
    uint8_t currentStep;
    bool running;

    bool LoadFromFile(fs::File file){
      if (!file) return false;

      // Read profile name
      currentPhase = "Idle";
      stepCount = 0;
      currentStep = 0;
      running = false;

      // Read steps
      while (file.available()) {
        String line = file.readStringUntil('\n');
        ProfileStep* step = ParseStep(line);
        if (step) {
          AddStep(step);
        }
      }

      return true;
    }

    ProfileStep* ParseStep(String line) {
      line.trim();
      if (line.length() == 0 || line.startsWith("#")) {
        return nullptr; // skip empty lines and comments
      }

      char type = line[0];
      // initialize variables for parsing
      String args[4];
      int argIndex = 0;
      int lastSpace = 0;
      while (argIndex < 4) {
        int spaceIndex = line.indexOf(' ', lastSpace + 1);
        if (spaceIndex == -1) {
          args[argIndex++] = line.substring(lastSpace + 1);
          break;
        } else {
          args[argIndex++] = line.substring(lastSpace + 1, spaceIndex);
          lastSpace = spaceIndex;
        }
      }

      double target = 0, rate = 0;
      double targetT = 0, targetB = 0, rateT = 0, rateB = 0;
      unsigned long duration = 0;

      switch (type)
      {
      case 'N':
        return new NameStep(&currentPhase, line.substring(2));
        break;
      case 'L':
        break;
      case 'I':
        break;
      case 'B':
        return new BeepStep(args[1].toInt());
        break;
      default:
        break;
      }

      return nullptr; // unknown step type

    }

    void AddStep(ProfileStep* step) {
      if (stepCount < MAX_PROFILE_STEPS) {
        steps[stepCount++] = step;
      }
    }
    void Init() {
      if (stepCount > 0) {
        steps[0]->Init(start, start);
      }

      for (uint8_t i = 1; i < stepCount; i++) {
        steps[i]->Init(steps[i - 1]->finalTempT, steps[i - 1]->finalTempB);
      }
      currentStep = 0;
      running = false;
    }
    void Start() {
      if (stepCount == 0) return;
      steps[0]->Start();
      currentStep = 0;
      running = true;
    }
    void Update() {
      if (running && stepCount > 0) {
        steps[currentStep]->Update(pSetpointTop, pSetpointBottom);
        if (steps[currentStep]->IsComplete()) {
          currentStep++;
          if (currentStep < stepCount) {
            steps[currentStep]->Start();
          } else {
            running = false; // profile complete
          }
        }
      }
    }
    void Reset() {
      for (uint8_t i = 0; i < stepCount; i++) {
        steps[i]->Start();
      }
      currentStep = 0;
      running = false;
      currentPhase = "Idle";
    }
    void Stop(){
      running = false;
      Reset();
    }
    unsigned long GetEstimatedDuration() {
      unsigned long total = 0;
      for (uint8_t i = 0; i < stepCount; i++) {
        total += steps[i]->duration;
      }
      return total;
    }
};
