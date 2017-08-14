package us.ihmc.robotics.controllers;

import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;

public interface YoSE3PIDGainsInterface extends PIDSE3Gains
{
   @Override
   public abstract YoPID3DGains getPositionGains();

   @Override
   public abstract YoPID3DGains getOrientationGains();
}
