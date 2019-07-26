package us.ihmc.robotEnvironmentAwareness.fusion.controller;

import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;

public class StereoREASLAMAnchorPaneController
{
   private JavaFXMessager messager;

   @FXML private Button loadSavedData;
   @FXML private ToggleButton showRawData;
   @FXML private Button calculatePlanarRegions;
   @FXML private ToggleButton showPlanarRegions;
   @FXML private Button doSLAM;
   @FXML private ToggleButton showSLAMResult;

   public void initialize(SharedMemoryJavaFXMessager messager)
   {
      this.messager = messager;

      messager.bindBidirectional(LidarImageFusionAPI.ShowRawData, showRawData.selectedProperty(), false);
      messager.bindBidirectional(LidarImageFusionAPI.ShowPlanarRegions, showPlanarRegions.selectedProperty(), false);
      messager.bindBidirectional(LidarImageFusionAPI.ShowSLAM, showSLAMResult.selectedProperty(), false);
   }

   public void loadSavedData()
   {
      messager.submitMessage(LidarImageFusionAPI.LoadSavedData, true);
   }

   public void calculatePlanarRegions()
   {
      messager.submitMessage(LidarImageFusionAPI.CalculatePlanarRegions, true);
   }

   public void doSLAM()
   {
      messager.submitMessage(LidarImageFusionAPI.DoSLAM, true);
   }
}
