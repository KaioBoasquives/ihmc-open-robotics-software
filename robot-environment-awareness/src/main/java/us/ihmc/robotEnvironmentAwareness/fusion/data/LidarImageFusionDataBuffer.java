package us.ihmc.robotEnvironmentAwareness.fusion.data;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import boofcv.struct.calib.IntrinsicParameters;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.ImageSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.SegmentationRawDataFilteringParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.LidarImageFusionDataFactory;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.PointCloudProjectionHelper;

public class LidarImageFusionDataBuffer
{
   private final LidarImageFusionDataFactory lidarImageFusionDataFactory = new LidarImageFusionDataFactory();

   private final AtomicReference<StereoVisionPointCloudMessage> latestStereoVisionPointCloudMessage = new AtomicReference<>(null);
   private final AtomicReference<BufferedImage> latestBufferedImage = new AtomicReference<>(null);

   private final AtomicReference<Point3D> latestCameraPosition;
   private final AtomicReference<Quaternion> latestCameraOrientation;
   private final AtomicReference<IntrinsicParameters> latestCameraIntrinsicParameters;

   private final AtomicReference<Integer> bufferSize;

   private final AtomicReference<SegmentationRawDataFilteringParameters> latestSegmentationRawDataFilteringParameters;
   private final AtomicReference<ImageSegmentationParameters> latestImageSegmentationParameters;
   private final AtomicReference<LidarImageFusionData> newBuffer = new AtomicReference<>(null);

   public LidarImageFusionDataBuffer(Messager messager)
   {
      bufferSize = messager.createInput(LidarImageFusionAPI.StereoBufferSize, 50000);
      latestImageSegmentationParameters = messager.createInput(LidarImageFusionAPI.ImageSegmentationParameters, new ImageSegmentationParameters());
      latestSegmentationRawDataFilteringParameters = messager.createInput(LidarImageFusionAPI.SegmentationRawDataFilteringParameters,
                                                                          new SegmentationRawDataFilteringParameters());

      latestCameraPosition = messager.createInput(LidarImageFusionAPI.CameraPositionState, new Point3D());
      latestCameraOrientation = messager.createInput(LidarImageFusionAPI.CameraOrientationState, new Quaternion());
      latestCameraIntrinsicParameters = messager.createInput(LidarImageFusionAPI.CameraIntrinsicParametersState, new IntrinsicParameters());
   }

   public LidarImageFusionDataBuffer()
   {
      bufferSize = new AtomicReference<Integer>(100000);
      latestImageSegmentationParameters = new AtomicReference<ImageSegmentationParameters>(new ImageSegmentationParameters());
      latestSegmentationRawDataFilteringParameters = new AtomicReference<SegmentationRawDataFilteringParameters>(new SegmentationRawDataFilteringParameters());

      latestCameraPosition = new AtomicReference<Point3D>(new Point3D());
      latestCameraOrientation = new AtomicReference<Quaternion>(new Quaternion());
      latestCameraIntrinsicParameters = new AtomicReference<IntrinsicParameters>(PointCloudProjectionHelper.multisenseOnCartIntrinsicParameters);
   }

   public LidarImageFusionData pollNewBuffer()
   {
      return newBuffer.getAndSet(null);
   }

   public List<LidarImageFusionData> generateSavedDataBuffer(BufferedImage[] images, Point3D[][] pointClouds)
   {
      List<LidarImageFusionData> listOfFusionData = new ArrayList<LidarImageFusionData>();
      lidarImageFusionDataFactory.setImageSegmentationParameters(latestImageSegmentationParameters.get());
      lidarImageFusionDataFactory.setSegmentationRawDataFilteringParameters(latestSegmentationRawDataFilteringParameters.get());

      for (int i = 0; i < images.length; i++)
      {
         LidarImageFusionData fusionData = lidarImageFusionDataFactory.createLidarImageFusionData(pointClouds[i], null, images[i]);
         listOfFusionData.add(fusionData);
      }

      return listOfFusionData;
   }

   public void updateNewBuffer()
   {
      StereoVisionPointCloudMessage pointCloudMessage = latestStereoVisionPointCloudMessage.get();

      Point3D[] pointCloudBuffer = MessageTools.unpackScanPoint3ds(pointCloudMessage);
      int[] colorBuffer = pointCloudMessage.getColors().toArray();
      Random random = new Random();
      int numberOfPoints = pointCloudBuffer.length;

      while (numberOfPoints > bufferSize.get())
      {
         int indexToRemove = random.nextInt(numberOfPoints);
         int lastIndex = numberOfPoints - 1;

         pointCloudBuffer[indexToRemove] = pointCloudBuffer[lastIndex];
         colorBuffer[indexToRemove] = colorBuffer[lastIndex];

         numberOfPoints--;
      }

      Point3D[] pointCloud = new Point3D[numberOfPoints];
      int[] colors = new int[numberOfPoints];
      for (int i = 0; i < numberOfPoints; i++)
      {
         pointCloud[i] = pointCloudBuffer[i];
         colors[i] = colorBuffer[i];
      }

      //      lidarImageFusionDataFactory.setIntrinsicParameters(latestCameraIntrinsicParameters.get());
      lidarImageFusionDataFactory.setImageSegmentationParameters(latestImageSegmentationParameters.get());
      lidarImageFusionDataFactory.setSegmentationRawDataFilteringParameters(latestSegmentationRawDataFilteringParameters.get());
      //      lidarImageFusionDataFactory.setCameraPose(latestCameraPosition.get(), latestCameraOrientation.get());

      LidarImageFusionData data = lidarImageFusionDataFactory.createLidarImageFusionData(pointCloud, colors, latestBufferedImage.get());

      newBuffer.set(data);
   }

   public void updateLatestStereoVisionPointCloudMessage(StereoVisionPointCloudMessage message)
   {
      latestStereoVisionPointCloudMessage.set(message);
   }

   public void updateLatestBufferedImage(BufferedImage bufferedImage)
   {
      latestBufferedImage.set(bufferedImage);
   }
   
   public void setSegmentationRawDataFilteringParameters(SegmentationRawDataFilteringParameters parameters)
   {
      latestSegmentationRawDataFilteringParameters.set(parameters);
   }
   
   public void setImageSegmentationParaeters(ImageSegmentationParameters parameters)
   {
      latestImageSegmentationParameters.set(parameters);
   }
}
