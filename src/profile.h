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

    // Target temperature for this step
    double finalTemp;
    // Duration of this step in milliseconds, 
    // calculated by Init() based on this step's behavior and the previous step's final temperature
    unsigned long duration, progressTime, startTime;
    void TargetHeater(double** Input, double** Setpoint) {
      pInput = Input;
      pSetpoint = Setpoint;
    }
    
    // given the final target temperature of the last step, start this step
    virtual void Init(double lastStepEndTemp) = 0;
    // Reset any internal variables to prepare for a new profile run
    virtual void Start() = 0;
    // update the setpoints based on the elapsed time and this step's behavior
    virtual void Update() = 0;
    // Signal that the next step should start
    virtual bool IsComplete() = 0;
    // Draw itself on a graph sprite at position x,y with given step sizes (1 pixel = stepX in time, 1 pixel = stepY in temperature)
    virtual void DrawOnGraph(TFT_eSprite* graphSprite, uint16_t startX, uint16_t startY, double stepX, double stepY) = 0;
  protected:
    double **pInput, **pSetpoint;
    double Temperature() {
      return **pInput;
    }
    void Setpoint(double value) {
      **pSetpoint = value;
    }
};

class NameStep : public ProfileStep {
  public:
    NameStep(String* currentPhase, String phaseName) : currentPhase(currentPhase), phaseName(phaseName) {
      finalTemp = 0;
      duration = 0;
    }
    void Init(double lastStepEndTemp) override {
      // does not affect temperatures
      finalTemp = lastStepEndTemp;
    }
    void Start() override {
      *currentPhase = phaseName;
    }
    void Update() override {
      // does nothing
    }
    bool IsComplete() override {
      return true; // always complete
    }
    void DrawOnGraph(TFT_eSprite* graphSprite, uint16_t startX, uint16_t startY, double stepX, double stepY) override {
      // draw a vertical line to indicate phase change
      graphSprite->drawLine(startX, startY, startX, startY - GRAPH_HEIGHT, TFT_YELLOW);
    }
    String phaseName;
  private:
    String* currentPhase;
};

class BeepStep : public ProfileStep {
  public:
    BeepStep(unsigned long duration) {
      beepDuration = duration;
      finalTemp = 0;
    }
    void Init(double lastStepEndTemp) override {
      // does not affect temperatures
      finalTemp = lastStepEndTemp;
    }
    void Start() override {
      startTime = millis();
      // Trigger beep here
      tone(BUZZER_PIN, 1000, beepDuration); // 1kHz tone
    }
    void Update() override {
      // does nothing
    }
    bool IsComplete() override {
      return true; // immediately complete
    }
    void DrawOnGraph(TFT_eSprite* graphSprite, uint16_t startX, uint16_t startY, double stepX, double stepY) override {
      // draw a small square to indicate beep
      graphSprite->fillRect(startX - 1, startY - 2, 2, 4, TFT_ORANGE);
    }
  private:
    unsigned long beepDuration;
};

// ------------- Profile steps for combined heating of top and bottom heater ------------
// Combined steps will always have the convection fan on due to the desire to equally heat both sides
// Linear ramp step
class LinearRateStep : public ProfileStep {
  public:
    LinearRateStep(double finalTemp, double rate){
      this->rate = rate;
      this->finalTemp = finalTemp;
    }
    void Init(double lastStepEndTemp) override{
      // calculate duration based on rate and temperature difference
      startTemp = lastStepEndTemp;
      double tempDiff = finalTemp - lastStepEndTemp;
      duration = (unsigned long)((tempDiff / rate) * 1000); // rate is in degrees per second
    }
    void Start() override{
      startTime = millis();
    }
    void Update() override{
      unsigned long progressTime = millis() - startTime;

      double target = (rate * (progressTime / 1000.0)); // rate is in degrees per second

      Setpoint(startTemp + target);
      

    }
    bool IsComplete() override{
      return millis() - startTime >= duration;
    }
    void DrawOnGraph(TFT_eSprite* graphSprite, uint16_t startX, uint16_t startY, double stepX, double stepY) override{

    }
  private:
    double rate, startTemp;
    unsigned long startTime; // time when this step started
};

class LinearTimeStep : public ProfileStep {
  public:
    LinearTimeStep(double finalTemp, unsigned long duration){

    }
    void Init(double endTemp) override{

    }
    void Start() override{

    }
    void Update() override{

    }
    bool IsComplete() override{
      return millis() - startTime >= duration;
    }
    void DrawOnGraph(TFT_eSprite* graphSprite, uint16_t startX, uint16_t startY, double stepX, double stepY) override{

    }
  private:
    unsigned long startTime; // time when this step started
    double rate; // rate of temperature change in degrees per minute
};

// AFAP step -> may cause overshoot
class InstantStep : public ProfileStep {
  public:
    InstantStep(double finalTemp, unsigned long duration){

    }
    void Init(double endTemp) override{

    }
    void Start() override{

    }
    void Update() override{
      
    }
    bool IsComplete() override{
      return millis() - startTime >= duration;
    }
    void DrawOnGraph(TFT_eSprite* graphSprite, uint16_t startX, uint16_t startY, double stepX, double stepY) override{

    }
};



struct Profile{
  public:
    Profile() : name(""), currentPhase("Idle"), stepCount(0), currentStep(0), running(false) {}

    ProfileStep* stepsT[MAX_PROFILE_STEPS];
    ProfileStep* stepsB[MAX_PROFILE_STEPS];
    String name, currentPhase;
    uint8_t stepCount;
    uint8_t currentStep;
    uint8_t totalPhases = 0;
    bool running, twoSided = false;

    static void begin(double* inputTop, double* inputBottom, double* setpointTop, double* setpointBottom) {
      pInputTop = inputTop;
      pInputBottom = inputBottom;
      pSetpointTop = setpointTop;
      pSetpointBottom = setpointBottom;
    }
    
    bool LoadFromFile(fs::File file){
      if (!file) return false;

      // Read profile name
      currentPhase = "Idle";
      stepCount = 0;
      currentStep = 0;
      totalPhases = 0;
      running = false;

      int addTo = 0;

      // Read steps
      while (file.available()) {
        String line = file.readStringUntil('\n');

        if (line.startsWith("#") || line.length() == 0) 
          continue; // skip comments and empty lines

        if (line.startsWith("S")) 
          addTo = 0;
        else if (line.startsWith("PT")){
          addTo = 1; twoSided = true;
        }
        else if (line.startsWith("PB"))
          addTo = 2;

        switch (addTo) 
        {
        case 0: {
          ProfileStep* stepT = ParseStep(line);
          if (stepT && stepCount < MAX_PROFILE_STEPS) {
            stepsT[stepCount] = stepT;
            
            // For single-sided steps, bottom uses the same step as top, but to prevent double updates, we create a new instance
            ProfileStep* stepB = ParseStep(line);
            stepsB[stepCount] = stepB; // for combined steps, both top and bottom use the same step
            stepCount++;
          }
          break;
        }
        
        case 1: {
          ProfileStep* stepDT = ParseStep(line);
          if (stepDT && stepCount <= MAX_PROFILE_STEPS) {
            stepsB[stepCount - 1] = stepDT; // replace bottom step of last added top step
          }
          break;
        }
        case 2: {
          ProfileStep* stepDB = ParseStep(line);
          if (stepDB && stepCount <= MAX_PROFILE_STEPS) {
            stepsT[stepCount - 1] = stepDB; // replace top step of last added bottom step
          }
          break;
        }
        default:
          break;
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
      String args[3];
      int argIndex = 0;
      int lastSpace = 0;
      while (argIndex < 3) {
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
      unsigned long duration = 0;

      switch (type)
      {
      case 'L':
        target = args[1].toDouble();
        if (args[2].startsWith("R")) {
          rate = args[2].substring(1).toDouble();
          return new LinearRateStep(target, rate);
        } else {
          duration = args[2].substring(1).toInt() * 1000; // convert seconds to milliseconds
          return new LinearTimeStep(target, duration);
        }
        break;
      case 'I':
        target = args[1].toDouble();
        if (args[2].startsWith("INF")){
          duration = 0xFFFFFFFF; // effectively infinite
        } else {
          duration = args[2].substring(1).toInt() * 1000; // convert seconds to milliseconds
        }
        return new InstantStep(target, duration);
        break;
      case 'N':
        totalPhases++;
        return new NameStep(&currentPhase, line.substring(2));
        break;
      case 'B':
        return new BeepStep(args[1].toInt()); // duration in milliseconds
        break;
      default:

        break;
      }

      return nullptr; // unknown step type
    }

    void Init() {
      if (stepCount > 0) {
        stepsT[0]->Init(start);
      }

      for (uint8_t i = 1; i < stepCount; i++) {
        stepsT[i]->Init(stepsT[i - 1]->finalTemp);
      }
      currentStep = 0;
      running = false;
    }
    void Start() {
      if (stepCount == 0) return;
      stepsT[0]->Start();
      currentStep = 0;
      running = true;
    }
    void Update() {
      if (running && stepCount > 0) {
        stepsT[currentStep]->Update();
        if (stepsT[currentStep]->IsComplete()) {
          currentStep++;
          if (currentStep < stepCount) {
            stepsT[currentStep]->Start();
          } else {
            running = false; // profile complete
          }
        }
      }
    }
    void Reset() {
      for (uint8_t i = 0; i < stepCount; i++) {
        stepsT[i]->Start();
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
        total += stepsT[i]->duration;
      }
      return total;
    }
};
