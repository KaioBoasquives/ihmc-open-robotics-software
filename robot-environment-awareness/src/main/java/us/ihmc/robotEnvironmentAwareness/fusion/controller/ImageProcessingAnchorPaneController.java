package us.ihmc.robotEnvironmentAwareness.fusion.controller;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.ToggleButton;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;

public class ImageProcessingAnchorPaneController
{
   private JavaFXMessager messager;
   @FXML
   private ToggleButton btnEnableStreaming;

   @FXML
   private Button btnTaskingSnapshot;

   @FXML
   private Button btnClearImageView;

   public void initialize(JavaFXMessager messager, REAUIMessager uiMessager)
   {
      this.messager = messager;
      this.reaMessager = uiMessager;
      btnTaskingSnapshot.setOnAction(new EventHandler<ActionEvent>()
      {
         @Override
         public void handle(ActionEvent event)
         {
            System.out.println("ImageProcessingAnchorPaneController true");
            messager.submitMessage(LidarImageFusionAPI.TakeSnapShot, true);
         }
      });

      messager.bindBidirectional(LidarImageFusionAPI.EnableStreaming, btnEnableStreaming.selectedProperty(), false);
   }

   public void clearImageView()
   {
      messager.submitMessage(LidarImageFusionAPI.ClearSnapShot, true);
   }

   private REAUIMessager reaMessager;

   public void load()
   {
      LogTools.info("load");

      long timestamp = 19870612L;
      File pointCloudDataFile = new File("C:\\Users\\inhol\\Desktop\\SavedData\\stereovision_pointcloud_1.txt");

      if (!pointCloudDataFile.canRead())
      {
         LogTools.warn("No file");
         return;
      }

      int maximumNumberOfPoints = 200000;
      double[] pointCloudBuffer = new double[maximumNumberOfPoints * 3];
      int[] colors = new int[maximumNumberOfPoints];

      BufferedReader bufferedReader = null;
      try
      {
         bufferedReader = new BufferedReader(new FileReader(pointCloudDataFile));
      }
      catch (FileNotFoundException e1)
      {
         e1.printStackTrace();
      }

      int lineIndex = 0;
      while (true)
      {
         String lineJustFetched = null;
         String[] idxyzcolorArray;
         try
         {
            lineJustFetched = bufferedReader.readLine();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         if (lineJustFetched == null)
         {
            break;
         }
         else
         {
            idxyzcolorArray = lineJustFetched.split("\t");
            int id = Integer.parseInt(idxyzcolorArray[0]);
            double x = Double.parseDouble(idxyzcolorArray[1]);
            double y = Double.parseDouble(idxyzcolorArray[2]);
            double z = Double.parseDouble(idxyzcolorArray[3]);
            int rgb = Integer.parseInt(idxyzcolorArray[4]);

            pointCloudBuffer[lineIndex * 3 + 0] = x;
            pointCloudBuffer[lineIndex * 3 + 1] = y;
            pointCloudBuffer[lineIndex * 3 + 2] = z;
            colors[lineIndex] = rgb;
            lineIndex++;
         }
      }

      float[] resizedPointCloud = new float[lineIndex * 3];
      int[] resizedColors = new int[lineIndex];
      for (int i = 0; i < resizedPointCloud.length; i++)
      {
         resizedPointCloud[i] = (float) pointCloudBuffer[i];
      }

      for (int i = 0; i < resizedColors.length; i++)
      {
         resizedColors[i] = colors[i];
      }

      StereoVisionPointCloudMessage stereoVisionMessage = MessageTools.createStereoVisionPointCloudMessage(timestamp, resizedPointCloud, resizedColors);
      reaMessager.submitMessageInternal(REAModuleAPI.StereoVisionPointCloudState, stereoVisionMessage);
   }

   public void showProjection()
   {
      LogTools.info("showProjection");
   }

   public void showFilteredProjection()
   {
      LogTools.info("showFilteredProjection");
   }

   public void showFilteredPoints()
   {
      LogTools.info("showFilteredPoints");
   }
}