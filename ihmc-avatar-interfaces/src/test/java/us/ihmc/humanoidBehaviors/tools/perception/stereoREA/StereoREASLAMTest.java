package us.ihmc.humanoidBehaviors.tools.perception.stereoREA;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;

import org.junit.jupiter.api.Test;

import javafx.application.Platform;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.humanoidBehaviors.tools.perception.PlanarRegionSLAM;
import us.ihmc.humanoidBehaviors.tools.perception.PlanarRegionSLAMParameters;
import us.ihmc.humanoidBehaviors.tools.perception.PlanarRegionSLAMResult;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.robotEnvironmentAwareness.fusion.data.LidarImageFusionData;
import us.ihmc.robotEnvironmentAwareness.fusion.data.LidarImageFusionDataBuffer;
import us.ihmc.robotEnvironmentAwareness.fusion.data.StereoREAPlanarRegionFeatureUpdater;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.ImageSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.PlanarRegionPropagationParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.SegmentationRawDataFilteringParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.LidarImageFusionDataLoader;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class StereoREASLAMTest
{
   private static final String filePath = "C:\\Users\\inhol\\Desktop\\SavedData\\SLAM\\SET3\\";
   //   private static final String[] imageFilePaths = {"image_0.jpg", "image_1.jpg", "image_2.jpg"};
   //   private static final String[] pointCloudFilePaths = {"stereovision_pointcloud_0.txt", "stereovision_pointcloud_1.txt", "stereovision_pointcloud_2.txt"};
   private static final String[] imageFilePaths = {"image_0.jpg", "image_1.jpg"};
   private static final String[] pointCloudFilePaths = {"stereovision_pointcloud_0.txt", "stereovision_pointcloud_1.txt"};

   protected static final float SCAN_POINT_SIZE = 0.0075f;
   private static final int palleteSizeForMeshBuilder = 2048;

   private final LidarImageFusionDataBuffer lidarImageFusionDataBuffer = new LidarImageFusionDataBuffer();
   private final StereoREAPlanarRegionFeatureUpdater planarRegionFeatureUpdater = new StereoREAPlanarRegionFeatureUpdater();
   
   private ImageSegmentationParameters imageSegmentationParaeters = new ImageSegmentationParameters();
   private SegmentationRawDataFilteringParameters segmentationRawDataFilteringParameters = new SegmentationRawDataFilteringParameters();
   private PlanarRegionPropagationParameters planarRegionPropagationParameters = new PlanarRegionPropagationParameters();
   
   private PlanarRegionSLAMParameters planarRegionSLAMParameters = new PlanarRegionSLAMParameters();
   
   public static void main(String[] args)
   {
      StereoREASLAMTest test = new StereoREASLAMTest();
      test.loadAndVisualizeSavedRawData();
   }

   @Test
   public void loadAndVisualizeSavedRawData()
   {
      initialize();
      boolean visualize = true;
      int numberOfData = imageFilePaths.length;
      System.out.println("numberOfData " + numberOfData);

      BufferedImage[] images = new BufferedImage[numberOfData];
      Point3D[][] pointClouds = new Point3D[numberOfData][];

      for (int i = 0; i < numberOfData; i++)
      {
         images[i] = LidarImageFusionDataLoader.readImage(filePath + imageFilePaths[i]);
         pointClouds[i] = LidarImageFusionDataLoader.readPointCloud(filePath + pointCloudFilePaths[i]);
      }
      
      System.out.println("visualize");
      if (visualize)
      {
         visualizePointCloud(pointClouds[0]);
         ThreadTools.sleepForever();
      }
      System.out.println("done");
   }
   
   @Test
   public void stereoREA()
   {
      initialize();
      boolean visualize = true;
      int numberOfData = imageFilePaths.length;
      System.out.println("numberOfData " + numberOfData);

      List<LidarImageFusionData> listOfFusionData = new ArrayList<LidarImageFusionData>();
      BufferedImage[] images = new BufferedImage[numberOfData];
      Point3D[][] pointClouds = new Point3D[numberOfData][];
      PlanarRegionsList[] planarRegionsLists;
      int[] planarRegionsColors = new int[]{0x000000, 0x110000};

      long dataLoadingStartTime = System.nanoTime();
      for (int i = 0; i < numberOfData; i++)
      {
         images[i] = LidarImageFusionDataLoader.readImage(filePath + imageFilePaths[i]);
         pointClouds[i] = LidarImageFusionDataLoader.readPointCloud(filePath + pointCloudFilePaths[i]);
      }
      System.out.println("dataLoading Time " + Conversions.nanosecondsToSeconds(System.nanoTime() - dataLoadingStartTime));

      long generateSavedDataBufferStartTime = System.nanoTime();
      listOfFusionData.clear();
      listOfFusionData.addAll(lidarImageFusionDataBuffer.generateSavedDataBuffer(images, pointClouds));
      System.out.println("generateSavedDataBuffer Time " + Conversions.nanosecondsToSeconds(System.nanoTime() - generateSavedDataBufferStartTime));
      
      System.out.println("listOfFusionData.size() " + listOfFusionData.size());

      planarRegionsLists = new PlanarRegionsList[images.length];
      for (int i = 0; i < listOfFusionData.size(); i++)
      {
         planarRegionFeatureUpdater.updateLatestLidarImageFusionData(listOfFusionData.get(i));

         long startTime = System.nanoTime();

         if (planarRegionFeatureUpdater.update())
         {
            if (planarRegionFeatureUpdater.getPlanarRegionsList() != null)
            {
               PlanarRegionsList planarRegionsList = new PlanarRegionsList(planarRegionFeatureUpdater.getPlanarRegionsList());
               planarRegionsLists[i] = planarRegionsList;
               System.out.println(i + " PlanarRegionsList " + planarRegionsList.getNumberOfPlanarRegions());
            }
         }

         System.out.println(i + " calculation Time " + Conversions.nanosecondsToSeconds(System.nanoTime() - startTime));
      }

      System.out.println("visualize");
      if (visualize)
      {
         visualizePlanarRegions(planarRegionsLists, planarRegionsColors);
         ThreadTools.sleepForever();
      }
      System.out.println("done");
   }
   
   @Test
   public void slamResult()
   {
      initialize();
      boolean visualize = true;
      int numberOfData = imageFilePaths.length;
      System.out.println("numberOfData " + numberOfData);

      List<LidarImageFusionData> listOfFusionData = new ArrayList<LidarImageFusionData>();
      BufferedImage[] images = new BufferedImage[numberOfData];
      Point3D[][] pointClouds = new Point3D[numberOfData][];
      PlanarRegionsList[] planarRegionsLists;
      int[] planarRegionsColors = new int[]{0x000000, 0x110000};

      long dataLoadingStartTime = System.nanoTime();
      for (int i = 0; i < numberOfData; i++)
      {
         images[i] = LidarImageFusionDataLoader.readImage(filePath + imageFilePaths[i]);
         pointClouds[i] = LidarImageFusionDataLoader.readPointCloud(filePath + pointCloudFilePaths[i]);
      }
      System.out.println("dataLoading Time " + Conversions.nanosecondsToSeconds(System.nanoTime() - dataLoadingStartTime));

      long generateSavedDataBufferStartTime = System.nanoTime();
      listOfFusionData.clear();
      listOfFusionData.addAll(lidarImageFusionDataBuffer.generateSavedDataBuffer(images, pointClouds));
      System.out.println("generateSavedDataBuffer Time " + Conversions.nanosecondsToSeconds(System.nanoTime() - generateSavedDataBufferStartTime));
      
      System.out.println("listOfFusionData.size() " + listOfFusionData.size());

      planarRegionsLists = new PlanarRegionsList[images.length];
      for (int i = 0; i < listOfFusionData.size(); i++)
      {
         planarRegionFeatureUpdater.updateLatestLidarImageFusionData(listOfFusionData.get(i));

         long startTime = System.nanoTime();

         if (planarRegionFeatureUpdater.update())
         {
            if (planarRegionFeatureUpdater.getPlanarRegionsList() != null)
            {
               PlanarRegionsList planarRegionsList = new PlanarRegionsList(planarRegionFeatureUpdater.getPlanarRegionsList());
               planarRegionsLists[i] = planarRegionsList;
               System.out.println(i + " PlanarRegionsList " + planarRegionsList.getNumberOfPlanarRegions());
            }
         }

         System.out.println(i + " calculation Time " + Conversions.nanosecondsToSeconds(System.nanoTime() - startTime));
      }

      System.out.println("SLAM");
      PlanarRegionsList map = planarRegionsLists[0];
      PlanarRegionsList newData = planarRegionsLists[1];
      PlanarRegionSLAMResult slamResult = PlanarRegionSLAM.slam(map, newData, planarRegionSLAMParameters);
      
      RigidBodyTransform transformFromIncomingToMap = slamResult.getTransformFromIncomingToMap();
      LogTools.info("\nSlam result: transformFromIncomingToMap = \n" + transformFromIncomingToMap);

      map = slamResult.getMergedMap();
      
      if (visualize)
      {
         visualizePlanarRegions(map);
         ThreadTools.sleepForever();
      }
      System.out.println("done");
   }

   private void visualizePointCloud(Point3D[] pointCloud)
   {
      JavaFXApplicationCreator.createAJavaFXApplication();

      final CountDownLatch countDownLatch = new CountDownLatch(1);

      Platform.runLater(new Runnable()
      {
         @Override
         public void run()
         {
            View3DFactory view3dFactory = new View3DFactory(1200, 800);
            view3dFactory.addCameraController(0.05, 2000.0, true);
            view3dFactory.addWorldCoordinateSystem(0.3);
            view3dFactory.addDefaultLighting();

            JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(palleteSizeForMeshBuilder));
            meshBuilder.clear();

            for (Point3D point : pointCloud)
            {
               meshBuilder.addMesh(MeshDataGenerator.Tetrahedron(SCAN_POINT_SIZE), point, Color.rgb(255, 0, 0));
            }

            MeshView scanMeshView = new MeshView(meshBuilder.generateMesh());
            scanMeshView.setMaterial(meshBuilder.generateMaterial());
            view3dFactory.addNodeToView(scanMeshView);

            Stage stage = new Stage();
            stage.setTitle(getClass().getSimpleName());
            stage.setMaximized(false);
            stage.setScene(view3dFactory.getScene());

            stage.centerOnScreen();

            stage.show();

            countDownLatch.countDown();
         }
      });

      try
      {
         countDownLatch.await();
      }
      catch (InterruptedException e)
      {
      }
   }

   private void visualizePlanarRegions(PlanarRegionsList[] planarRegions, int[] colors)
   {
      for(int i=0;i<planarRegions.length;i++)
      {
         PlanarRegionsList planarRegionsList = planarRegions[i];
         for(int j=0;j<planarRegionsList.getNumberOfPlanarRegions();j++)
         {
            planarRegionsList.getPlanarRegion(j).setRegionId(colors[i]);
         }
      }
      visualizePlanarRegions(planarRegions);
   }
   
   private void visualizePlanarRegions(PlanarRegionsList... planarRegions)
   {
      JavaFXApplicationCreator.createAJavaFXApplication();

      ArrayList<PlanarRegionsGraphic> planarRegionGraphics = new ArrayList<PlanarRegionsGraphic>();

      for (PlanarRegionsList planarRegionsList : planarRegions)
      {
         PlanarRegionsGraphic regionsGraphic = new PlanarRegionsGraphic(false);
         regionsGraphic.generateMeshes(planarRegionsList);
         regionsGraphic.update();
         planarRegionGraphics.add(regionsGraphic);
      }

      final CountDownLatch countDownLatch = new CountDownLatch(1);

      Platform.runLater(new Runnable()
      {
         @Override
         public void run()
         {
            View3DFactory view3dFactory = new View3DFactory(1200, 800);
            view3dFactory.addCameraController(0.05, 2000.0, true);
            view3dFactory.addWorldCoordinateSystem(0.3);
            view3dFactory.addDefaultLighting();

            for (PlanarRegionsGraphic regionsGraphic : planarRegionGraphics)
            {
               view3dFactory.addNodeToView(regionsGraphic);
            }

            Stage stage = new Stage();
            stage.setTitle(getClass().getSimpleName());
            stage.setMaximized(false);
            stage.setScene(view3dFactory.getScene());

            stage.centerOnScreen();

            stage.show();

            countDownLatch.countDown();
         }
      });

      try
      {
         countDownLatch.await();
      }
      catch (InterruptedException e)
      {
      }
   }
   
   private void initialize()
   {
      setImageSegmentationParaeters();
      setSegmentationRawDataFilteringParameters();
      setPlanarRegionPropagationParameters();
      
      lidarImageFusionDataBuffer.setImageSegmentationParaeters(imageSegmentationParaeters);
      lidarImageFusionDataBuffer.setSegmentationRawDataFilteringParameters(segmentationRawDataFilteringParameters);
      
      planarRegionFeatureUpdater.setPlanarRegionPropagationParameters(planarRegionPropagationParameters);
      planarRegionFeatureUpdater.setSegmentationRawDataFilteringParameters(segmentationRawDataFilteringParameters);
      
      planarRegionSLAMParameters.setIterationsForMatching(3);
      planarRegionSLAMParameters.setMinimumNormalDotProduct(0.98);
      planarRegionSLAMParameters.setDampedLeastSquaresLambda(5);
      planarRegionSLAMParameters.setBoundingBoxHeight(0.03);
      planarRegionSLAMParameters.setMinimumRegionOverlapDistance(0.05);
   }

   private void setImageSegmentationParaeters()
   {
      imageSegmentationParaeters.setPixelSize(20);
      imageSegmentationParaeters.setIterate(6);
   }
   
   private void setSegmentationRawDataFilteringParameters()
   {
      segmentationRawDataFilteringParameters.setMinimumSparseThreshold(0.05);
      segmentationRawDataFilteringParameters.setMaximumSparsePropotionalRatio(4.0);
      
      segmentationRawDataFilteringParameters.setEnableFilterFlyingPoint(false);
      segmentationRawDataFilteringParameters.setEnableFilterCentrality(true);
      segmentationRawDataFilteringParameters.setEnableFilterEllipticity(false);
      
      segmentationRawDataFilteringParameters.setCentralityRadius(0.065);
      segmentationRawDataFilteringParameters.setCentralityThreshold(0.35);
   }

   private void setPlanarRegionPropagationParameters()
   {
      planarRegionPropagationParameters.setProximityThreshold(0.02);
      planarRegionPropagationParameters.setPlanarityThreshold(0.95);
   }
}
