package us.ihmc.robotEnvironmentAwareness.fusion;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.publisherTopicNameGenerator;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.subscriberTopicNameGenerator;

import java.awt.image.BufferedImage;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.robotEnvironmentAwareness.fusion.data.LidarImageFusionData;
import us.ihmc.robotEnvironmentAwareness.fusion.data.LidarImageFusionDataBuffer;
import us.ihmc.robotEnvironmentAwareness.fusion.data.StereoREAPlanarRegionFeatureUpdater;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.LidarImageFusionDataLoader;
import us.ihmc.robotEnvironmentAwareness.updaters.REAPlanarRegionPublicNetworkProvider;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2Node;

public class StereoREAModule implements Runnable
{
   private final Messager reaMessager;
   private final Messager messager;

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> isRunning = new AtomicReference<Boolean>(false);
   private final LidarImageFusionDataBuffer lidarImageFusionDataBuffer;
   private final StereoREAPlanarRegionFeatureUpdater planarRegionFeatureUpdater;

   private final REAPlanarRegionPublicNetworkProvider planarRegionNetworkProvider;

   private final AtomicReference<Integer> dataIndexToCalculate;

   public StereoREAModule(Ros2Node ros2Node, Messager reaMessager, SharedMemoryJavaFXMessager messager)
   {
      this.messager = messager;
      this.reaMessager = reaMessager;
      lidarImageFusionDataBuffer = new LidarImageFusionDataBuffer(messager);
      planarRegionFeatureUpdater = new StereoREAPlanarRegionFeatureUpdater(reaMessager, messager);

      enable = messager.createInput(LidarImageFusionAPI.EnableREA, false);

      planarRegionNetworkProvider = new REAPlanarRegionPublicNetworkProvider(reaMessager, planarRegionFeatureUpdater, ros2Node, publisherTopicNameGenerator,
                                                                             subscriberTopicNameGenerator);

      initializeREAPlanarRegionPublicNetworkProvider();

      messager.registerTopicListener(LidarImageFusionAPI.LoadSavedData, (content) -> loadSavedData());
      messager.registerTopicListener(LidarImageFusionAPI.CalculatePlanarRegions, (content) -> calculatePlanarRegions());
      messager.registerTopicListener(LidarImageFusionAPI.DoSLAM, (content) -> doSLAM());
      messager.registerTopicListener(LidarImageFusionAPI.ExportData, (content) -> exportData());
      dataIndexToCalculate = messager.createInput(LidarImageFusionAPI.DataIndexToCalculate, -1);
   }

   private void initializeREAPlanarRegionPublicNetworkProvider()
   {
      reaMessager.submitMessage(REAModuleAPI.LidarBufferEnable, false);
      reaMessager.submitMessage(REAModuleAPI.StereoVisionBufferEnable, false);
      reaMessager.submitMessage(REAModuleAPI.OcTreeClear, false);
      reaMessager.submitMessage(REAModuleAPI.LidarMinRange, Double.NEGATIVE_INFINITY);
      reaMessager.submitMessage(REAModuleAPI.LidarMaxRange, Double.POSITIVE_INFINITY);
      reaMessager.submitMessage(REAModuleAPI.OcTreeBoundingBoxParameters, new BoundingBoxParametersMessage());
   }

   public void registerCustomPlanarRegion(PlanarRegion planarRegion)
   {
      planarRegionFeatureUpdater.registerCustomPlanarRegion(planarRegion);
   }

   public void dispatchCustomPlanarRegion(PlanarRegionsListMessage message)
   {
      PlanarRegionsList customPlanarRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(message);
      customPlanarRegions.getPlanarRegionsAsList().forEach(planarRegionFeatureUpdater::registerCustomPlanarRegion);
   }

   public void updateLatestStereoVisionPointCloudMessage(StereoVisionPointCloudMessage message)
   {
      lidarImageFusionDataBuffer.updateLatestStereoVisionPointCloudMessage(message);
   }

   public void updateLatestBufferedImage(BufferedImage bufferedImage)
   {
      lidarImageFusionDataBuffer.updateLatestBufferedImage(bufferedImage);
   }

   @Override
   public void run()
   {
      if (!enable.get())
         return;

      singleRun();
   }

   public void singleRun()
   {
      isRunning.set(true);
      long runningStartTime = System.nanoTime();

      lidarImageFusionDataBuffer.updateNewBuffer();

      LidarImageFusionData newBuffer = lidarImageFusionDataBuffer.pollNewBuffer();
      messager.submitMessage(LidarImageFusionAPI.FusionDataState, newBuffer);

      planarRegionFeatureUpdater.updateLatestLidarImageFusionData(newBuffer);

      if (planarRegionFeatureUpdater.update())
      {
         reaMessager.submitMessage(REAModuleAPI.OcTreeEnable, true); // TODO: replace, or modify.
         reportPlanarRegionState();
      }

      double runningTime = Conversions.nanosecondsToSeconds(System.nanoTime() - runningStartTime);
      String computationTime = new DecimalFormat("##.###").format(runningTime) + "(sec)";
      messager.submitMessage(LidarImageFusionAPI.ComputationTime, computationTime);
      isRunning.set(false);
   }

   public void enable()
   {
      enable.set(true);
   }

   private void reportPlanarRegionState()
   {
      if (planarRegionFeatureUpdater.getPlanarRegionsList() != null)
      {
         PlanarRegionsList planarRegionsList = planarRegionFeatureUpdater.getPlanarRegionsList();
         PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList);
         reaMessager.submitMessage(REAModuleAPI.PlanarRegionsState, planarRegionsListMessage);

         planarRegionNetworkProvider.update(true);
         planarRegionNetworkProvider.publishCurrentState();
      }
   }

   private static final String filePath = "C:\\Users\\inhol\\Desktop\\SavedData\\SLAM\\complex blocks\\";
   private static final int numberOfData = 12;
   private static String[] imageFilePaths = {"image_0.jpg", "image_1.jpg", "image_2.jpg"};
   private static String[] pointCloudFilePaths = {"stereovision_pointcloud_0.txt", "stereovision_pointcloud_1.txt", "stereovision_pointcloud_2.txt"};

   private final List<LidarImageFusionData> listOfFusionData = new ArrayList<LidarImageFusionData>();
   private BufferedImage[] images;
   private Point3D[][] pointClouds;

   private PlanarRegionsList[] planarRegionsLists;

   private void loadSavedData()
   {
      System.out.println("loadSavedData " + numberOfData);
      imageFilePaths = new String[numberOfData];
      pointCloudFilePaths = new String[numberOfData];
      for (int i = 0; i < numberOfData; i++)
      {
         imageFilePaths[i] = "image_" + i + ".jpg";
         pointCloudFilePaths[i] = "stereovision_pointcloud_" + i + ".txt";
      }

      images = new BufferedImage[numberOfData];
      pointClouds = new Point3D[numberOfData][];

      for (int i = 0; i < numberOfData; i++)
      {
         images[i] = LidarImageFusionDataLoader.readImage(filePath + imageFilePaths[i]);
         pointClouds[i] = LidarImageFusionDataLoader.readPointCloud(filePath + pointCloudFilePaths[i]);
      }
   }

   private void calculatePlanarRegions()
   {
      System.out.println("calculatePlanarRegions");
      listOfFusionData.clear();
      listOfFusionData.addAll(lidarImageFusionDataBuffer.generateSavedDataBuffer(images, pointClouds));

      System.out.println("listOfFusionData.size() " + listOfFusionData.size());

      if (dataIndexToCalculate.get() == -1)
      {
         LogTools.info("Put proper index");
      }
      else if (dataIndexToCalculate.get() < images.length)
      {
         planarRegionFeatureUpdater.updateLatestLidarImageFusionData(listOfFusionData.get(dataIndexToCalculate.get()));

         if (planarRegionFeatureUpdater.update())
         {
            reaMessager.submitMessage(REAModuleAPI.OcTreeEnable, true); // TODO: replace, or modify.
            if (planarRegionFeatureUpdater.getPlanarRegionsList() != null)
            {
               PlanarRegionsList planarRegionsList = new PlanarRegionsList(planarRegionFeatureUpdater.getPlanarRegionsList());

               PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList);
               reaMessager.submitMessage(REAModuleAPI.PlanarRegionsState, planarRegionsListMessage);

               planarRegionNetworkProvider.update(true);
               planarRegionNetworkProvider.publishCurrentState();
            }
         }
      }
      else
      {
         LogTools.info("Put proper index");
      }
   }

   private void doSLAM()
   {
      System.out.println("doSLAM");

      planarRegionsLists = new PlanarRegionsList[images.length];

      for (int i = 0; i < listOfFusionData.size(); i++)
      {
         planarRegionFeatureUpdater.updateLatestLidarImageFusionData(listOfFusionData.get(i));

         if (planarRegionFeatureUpdater.update())
         {
            reaMessager.submitMessage(REAModuleAPI.OcTreeEnable, true); // TODO: replace, or modify.
            if (planarRegionFeatureUpdater.getPlanarRegionsList() != null)
            {
               PlanarRegionsList planarRegionsList = new PlanarRegionsList(planarRegionFeatureUpdater.getPlanarRegionsList());
               planarRegionsLists[i] = planarRegionsList;

               PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList);
               reaMessager.submitMessage(REAModuleAPI.PlanarRegionsState, planarRegionsListMessage);

               planarRegionNetworkProvider.update(true);
               planarRegionNetworkProvider.publishCurrentState();
            }
         }
      }
   }

   private void exportData()
   {
      reaMessager.submitMessage(REAModuleAPI.UIPlanarRegionDataExportRequest, true);
   }
}
