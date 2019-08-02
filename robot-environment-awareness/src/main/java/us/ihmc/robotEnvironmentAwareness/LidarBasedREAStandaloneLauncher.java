package us.ihmc.robotEnvironmentAwareness;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.ui.LIDARBasedEnvironmentAwarenessUI;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;

public class LidarBasedREAStandaloneLauncher extends Application
{
   private static final String MODULE_CONFIGURATION_FILE_NAME = "./Configurations/defaultREAModuleConfiguration.txt";

   private LIDARBasedEnvironmentAwarenessUI ui;
   private LIDARBasedREAModule module;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      SharedMemoryJavaFXMessager messager = new SharedMemoryJavaFXMessager(LidarImageFusionAPI.API);
      messager.startMessager();
      
      ui = LIDARBasedEnvironmentAwarenessUI.creatIntraprocessUI(primaryStage, messager);
      module = LIDARBasedREAModule.createIntraprocessModule(MODULE_CONFIGURATION_FILE_NAME, messager);

      ui.show();
      module.start();
   }

   @Override
   public void stop() throws Exception
   {
      ui.stop();
      module.stop();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
