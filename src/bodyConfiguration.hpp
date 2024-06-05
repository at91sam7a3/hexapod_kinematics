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
            frame.centerYOffset = 85;
            frame.rearYOffset = 72;
            frame.rearXOffset = 72;
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

} // namespace bodyConfiguration
