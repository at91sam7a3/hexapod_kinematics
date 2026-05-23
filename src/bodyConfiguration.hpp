#pragma once
namespace bodyConfiguration
{
  struct HexapodFrame
  {
        double cLegPart;  // BODY * - C - * - B - * - A - END
        double bLegPart;
        double aLegPart;
        // phisical coordinates where legs attached on body, needed for rotation
        double centerYOffset; //From center to left/right middle servo
        double rearYOffset;
        double rearXOffset;

        static HexapodFrame getConfiguredFrame ()
        {
            HexapodFrame frame;
            frame.cLegPart = 53;
            frame.bLegPart = 81;
            frame.aLegPart = 120;
            frame.centerYOffset = 108;
            frame.rearYOffset = 76;
            frame.rearXOffset = 76;
            return frame;
        }
  };

  struct HexapodMovementConfiguration
  {
    double stepHeight;//80;//How far robot raise a leg on step

    static HexapodMovementConfiguration getDefaultSettings()
    {
        HexapodMovementConfiguration config;
        config.stepHeight = 20;
        return config;
    }
  };

  struct GaitParameters
  {
    double movementSmoothing = 0.2;
    double rotationSmoothing = 0.2;
    double stepHeight = 20;
    double maxStepLength = 25;
    double gaitFrequency = 0.08;
    double swingRatio = 0.5;

    static GaitParameters getDefault()
    {
        return GaitParameters{};
    }
  };

} // namespace bodyConfiguration
