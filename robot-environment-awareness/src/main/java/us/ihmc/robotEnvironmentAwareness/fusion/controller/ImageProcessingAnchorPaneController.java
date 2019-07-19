package us.ihmc.robotEnvironmentAwareness.fusion.controller;

import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.ToggleButton;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.filter.StereoPointCloudFilter;
import us.ihmc.robotEnvironmentAwareness.fusion.MultisenseInformation;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.PointCloudProjectionHelper;

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
      stereoPointCloudMessage = reaMessager.createInput(REAModuleAPI.StereoVisionPointCloudState);

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
      File pointCloudDataFile = new File("C:\\Users\\inhol\\Desktop\\SavedData\\stereovision_pointcloud_0.txt");

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

   private AtomicReference<StereoVisionPointCloudMessage> stereoPointCloudMessage;
   public static final int defaultImageWidth = 1024;
   public static final int defaultImageHeight = 544;

   public void showProjection()
   {
      LogTools.info("showProjection");
      BufferedImage projectedImage = new BufferedImage(defaultImageWidth, defaultImageHeight, BufferedImage.TYPE_INT_RGB);

      StereoVisionPointCloudMessage stereoVisionPointCloudMessage = stereoPointCloudMessage.get();
      Point3D[] pointCloudBuffer = MessageTools.unpackScanPoint3ds(stereoVisionPointCloudMessage);
      int[] colorBuffer = stereoVisionPointCloudMessage.getColors().toArray();

      for (int i = 0; i < pointCloudBuffer.length; i++)
      {
         Point3D point = pointCloudBuffer[i];
         if (point == null)
            break;
         int[] pixel = PointCloudProjectionHelper.projectMultisensePointCloudOnImage(point, MultisenseInformation.CART.getIntrinsicParameters(), new Point3D(),
                                                                                     new Quaternion());

         if (pixel[0] < 0 || pixel[0] >= defaultImageWidth || pixel[1] < 0 || pixel[1] >= defaultImageHeight)
            continue;

         projectedImage.setRGB(pixel[0], pixel[1], colorBuffer[i]);
      }

      messager.submitMessage(LidarImageFusionAPI.ImageResultState, projectedImage);
   }

   public void showFilteredProjection()
   {
      LogTools.info("showFilteredProjection");

      StereoVisionPointCloudMessage stereoVisionPointCloudMessage = stereoPointCloudMessage.get();
      Point3D[] pointCloudBuffer = MessageTools.unpackScanPoint3ds(stereoVisionPointCloudMessage);

      for (int i = 0; i < pointCloudBuffer.length; i++)
      {
         Point3D point = pointCloudBuffer[i];
         if (point == null)
            break;
      }

      StereoPointCloudFilter filter = new StereoPointCloudFilter(defaultImageWidth, defaultImageHeight, 12);
      filter.initialize();
      filter.updateInput(pointCloudBuffer);
      filter.calculate();
      Point3D[] filteredPoints = filter.getOutput();
      float[] filteredPointsBuffer = new float[filteredPoints.length * 3];
      int[] filteredColors = new int[filteredPoints.length];
      
      BufferedImage projectedImage = new BufferedImage(defaultImageWidth, defaultImageHeight, BufferedImage.TYPE_INT_RGB);
      for (int i = 0; i < filteredPoints.length; i++)
      {
         filteredPointsBuffer[i * 3 + 0] = (float) filteredPoints[i].getX();
         filteredPointsBuffer[i * 3 + 1] = (float) filteredPoints[i].getY();
         filteredPointsBuffer[i * 3 + 2] = (float) filteredPoints[i].getZ();
         filteredColors[i] = 0;
         
         Point3D point = pointCloudBuffer[i];
         if (point == null)
            break;
         int[] pixel = PointCloudProjectionHelper.projectMultisensePointCloudOnImage(point, MultisenseInformation.CART.getIntrinsicParameters(), new Point3D(),
                                                                                     new Quaternion());

         if (pixel[0] < 0 || pixel[0] >= defaultImageWidth || pixel[1] < 0 || pixel[1] >= defaultImageHeight)
            continue;

         projectedImage.setRGB(pixel[0], pixel[1], 0xFFFFFF);
      }
      messager.submitMessage(LidarImageFusionAPI.ImageResultState, projectedImage);
      
      StereoVisionPointCloudMessage stereoVisionMessage = MessageTools.createStereoVisionPointCloudMessage(0, filteredPointsBuffer, filteredColors);
      reaMessager.submitMessageInternal(REAModuleAPI.StereoVisionPointCloudState, stereoVisionMessage);
      
   }

   public void showFilteredPoints()
   {
      LogTools.info("showFilteredPoints");
      
   }
}