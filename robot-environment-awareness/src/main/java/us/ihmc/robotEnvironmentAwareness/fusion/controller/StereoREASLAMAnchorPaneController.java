package us.ihmc.robotEnvironmentAwareness.fusion.controller;

import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;

public class StereoREASLAMAnchorPaneController
{
   private JavaFXMessager messager;
   private REAUIMessager reaMessager;

   @FXML private Button loadSavedData;
   @FXML private ToggleButton showRawData;
   @FXML private Button calculatePlanarRegions;
   @FXML private ToggleButton showPlanarRegions;
   @FXML private Button doSLAM;
   @FXML private ToggleButton showSLAMResult;
   @FXML private TextField indexOfDataToExport;

   public void initialize(SharedMemoryJavaFXMessager messager, REAUIMessager reaMessager)
   {
      this.messager = messager;
      this.reaMessager = reaMessager;

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
      String text = indexOfDataToExport.getText();
      messager.submitMessage(LidarImageFusionAPI.DataIndexToCalculate, Integer.parseInt(text));
      messager.submitMessage(LidarImageFusionAPI.CalculatePlanarRegions, true);
   }

   public void doSLAM()
   {
      messager.submitMessage(LidarImageFusionAPI.DoSLAM, true);
   }
   
   public void exportPlanarRegions()
   {
      reaMessager.submitMessageInternal(REAModuleAPI.UIPlanarRegionDataExportRequest, true);
   }
}
