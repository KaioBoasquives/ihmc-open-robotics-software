package us.ihmc.robotEnvironmentAwareness.fusion;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.GridPane;
import javafx.stage.Stage;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.fusion.controller.ImageProcessingAnchorPaneController;
import us.ihmc.robotEnvironmentAwareness.fusion.controller.ObjectDetectionAnchorPaneController;
import us.ihmc.robotEnvironmentAwareness.ui.controller.PointCloudAnchorPaneController;

public class LidarImageFusionProcessorUI
{
   private final SharedMemoryJavaFXMessager messager;

   private final BorderPane mainPane;

   private final Stage primaryStage;

   private final FusionSensorMeshViewer meshViewer;
   private final FusionSensorImageViewer imageViewer;
   private final FusionObjectDetector objectDetector;

   private static final String UI_CONFIGURATION_FILE_NAME = "./Configurations/defaultREAUIConfiguration.txt";

   @FXML
   private PointCloudAnchorPaneController pointCloudAnchorPaneController;
   
   @FXML
   private ImageProcessingAnchorPaneController imageProcessingAnchorPaneController;
   
   @FXML
   private ObjectDetectionAnchorPaneController objectDetectionAnchorPaneController;

   private LidarImageFusionProcessorUI(SharedMemoryJavaFXMessager messager, REAUIMessager reaMessager, Stage primaryStage) throws Exception
   {
      this.messager = messager;
      this.primaryStage = primaryStage;
      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));
      mainPane = loader.load();

      meshViewer = new FusionSensorMeshViewer(reaMessager);
      imageViewer = new FusionSensorImageViewer(messager);
      objectDetector = new FusionObjectDetector(messager);
      initializeControllers(reaMessager);

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);

      String imageLocation = "C:\\repos\\repository-group\\ihmc-open-robotics-software\\robot-environment-awareness\\src\\main\\java\\us\\ihmc\\robotEnvironmentAwareness\\ihmc.jpg";
      FileInputStream fis = new FileInputStream(imageLocation);

      ImageView imagePane = new ImageView();
      Image sampleImage = new Image(fis);
      imagePane.setImage(sampleImage);

      GridPane centerPane = new GridPane();
      centerPane.getChildren().add(imagePane);

      mainPane.setRight(centerPane);
      mainPane.setCenter(view3dFactory.getSubSceneWrappedInsidePane());

      view3dFactory.addNodeToView(meshViewer.getRoot());

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   public static LidarImageFusionProcessorUI creatIntraprocessUI(SharedMemoryJavaFXMessager messager, Stage primaryStage) throws Exception
   {
      Messager moduleMessager = KryoMessager.createIntraprocess(REAModuleAPI.API, NetworkPorts.REA_MODULE_UI_PORT,
                                                                REACommunicationProperties.getPrivateNetClassList());
      REAUIMessager reaMessager = new REAUIMessager(moduleMessager);
      reaMessager.startMessager();
      
      return new LidarImageFusionProcessorUI(messager, reaMessager, primaryStage);
   }

   public void show()
   {
      primaryStage.show();
      
      imageViewer.start();
      
      objectDetector.start();
   }

   public void stop()
   {
      try
      {
         messager.closeMessager();

         meshViewer.stop();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   private void initializeControllers(REAUIMessager reaMessager)
   {
      File configurationFile = new File(UI_CONFIGURATION_FILE_NAME);
      try
      {
         configurationFile.getParentFile().mkdirs();
         configurationFile.createNewFile();
      }
      catch (IOException e)
      {
         System.out.println(configurationFile.getAbsolutePath());
         e.printStackTrace();
      }

      pointCloudAnchorPaneController.setConfigurationFile(configurationFile);
      pointCloudAnchorPaneController.attachREAMessager(reaMessager);
      pointCloudAnchorPaneController.bindControls();
      
      imageProcessingAnchorPaneController.initialize(messager);
      objectDetectionAnchorPaneController.initialize(messager);
   }
}