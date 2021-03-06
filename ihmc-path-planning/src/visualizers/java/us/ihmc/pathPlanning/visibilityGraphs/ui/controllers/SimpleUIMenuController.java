package us.ihmc.pathPlanning.visibilityGraphs.ui.controllers;

import java.io.File;

import javafx.fxml.FXML;
import javafx.scene.control.MenuItem;
import javafx.stage.DirectoryChooser;
import javafx.stage.Window;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class SimpleUIMenuController
{
   private static final boolean VERBOSE = true;

   private final DirectoryChooser directoryChooser = new DirectoryChooser();

   @FXML
   private MenuItem reloadMenuItem;

   private Messager messager;
   private Window ownerWindow;

   private File loadedFile = null;

   public SimpleUIMenuController()
   {
      directoryChooser.setInitialDirectory(new File("."));
   }

   public void attachMessager(Messager messager)
   {
      this.messager = messager;
   }

   public void setMainWindow(Window ownerWindow)
   {
      this.ownerWindow = ownerWindow;
      reloadMenuItem.setDisable(true);
   }

   @FXML
   public void loadPlanarRegion()
   {
      File result = directoryChooser.showDialog(ownerWindow);
      if (result == null)
         return;

      loadedFile = result;
      loadAndSubmitPlanarRegions();
   }

   @FXML
   public void reloadPlanarRegion()
   {
      if (loadedFile == null)
      {
         reloadMenuItem.setDisable(true);
         return;
      }
      loadAndSubmitPlanarRegions();
   }

   private void loadAndSubmitPlanarRegions()
   {
      PlanarRegionsList loadedPlanarRegions = PlanarRegionFileTools.importPlanarRegionData(loadedFile);
      directoryChooser.setInitialDirectory(loadedFile.getParentFile());

      if (loadedPlanarRegions != null)
      {
         if (VERBOSE)
            LogTools.info("Loaded planar regions, broadcasting data.", this);
         messager.submitMessage(UIVisibilityGraphsTopics.GlobalReset, true);
         messager.submitMessage(UIVisibilityGraphsTopics.PlanarRegionData, loadedPlanarRegions);
         messager.submitMessage(UIVisibilityGraphsTopics.StartPosition, new Point3D());
         messager.submitMessage(UIVisibilityGraphsTopics.GoalPosition, new Point3D());
         reloadMenuItem.setDisable(false);
      }
      else
      {
         if (VERBOSE)
            LogTools.info("Failed to load planar regions.", this);
         reloadMenuItem.setDisable(true);
         loadedFile = null;
      }
   }
}
