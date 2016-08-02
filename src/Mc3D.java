//java core packages
import java.awt.*;
import java.nio.file.Paths;
import java.util.ArrayList;

//java extension packages
import javax.swing.*;
import javax.vecmath.*;
import javax.media.j3d.*;

public class Mc3D extends JFrame
{
  private Java3DWorld java3DWorld;
  private ControlPanel controlPanel;

  public Mc3D()
  {
    super("Mc3D");

    //create components
    java3DWorld = new Java3DWorld();
    controlPanel = new ControlPanel(java3DWorld);
    java3DWorld.setControlPanel(controlPanel);

    //add components to JFrame
    getContentPane().add(java3DWorld, BorderLayout.CENTER);
    getContentPane().add(controlPanel, BorderLayout.EAST);

    setDefaultCloseOperation(EXIT_ON_CLOSE);
    pack();
    setVisible(true);
  }

  public static void main(String args[])
  {
    //start new application
    Mc3D application = new Mc3D();

    //create and add radar sector
    double minRange = 100000.0, maxRange = 2000000.0;
    double minAz = Math.toRadians(-10.0), maxAz = Math.toRadians(10.0);
    double minEl = Math.toRadians(10.0), maxEl = Math.toRadians(30.0);
    ShapeModel shapeModel1 = new ShapeModel(Color.red, "Radar Sector", true);
    GeodPoint radarLocation = new GeodPoint(Math.toRadians(0.0), Math.toRadians(0.0), 0.0);

    RadarSector radarSector = new RadarSector(minAz, maxAz, minEl, maxEl, minRange, maxRange, radarLocation, shapeModel1);
    application.java3DWorld.addShape3D(radarSector);

    //create and add an air corridor
    double minAlt = 200000.0, maxAlt = 300000.0, width = 10000.0;
    ShapeModel shapeModel2 = new ShapeModel(Color.red, "Air Corridor", true);

    LatLonPoint[] geodVertex1 = new LatLonPoint[4];
    geodVertex1[0] = new LatLonPoint(Math.toRadians(10.0), Math.toRadians(29.5));
    geodVertex1[1] = new LatLonPoint(Math.toRadians(15.0), Math.toRadians(29.5));
    geodVertex1[2] = new LatLonPoint(Math.toRadians(20.0), Math.toRadians(34.5));
    geodVertex1[3] = new LatLonPoint(Math.toRadians(25.0), Math.toRadians(24.5));

    AirCorridor airCorridor = new AirCorridor(geodVertex1, minAlt, maxAlt, width, shapeModel2);
    application.java3DWorld.addShape3D(airCorridor);

    //create and add a polygon region
    ShapeModel shapeModel3 = new ShapeModel(Color.red, "Polygon Region", true);

    LatLonPoint[] geodVertex2 = new LatLonPoint[5];
    geodVertex2[0] = new LatLonPoint(Math.toRadians(9.0), Math.toRadians(9.0));
    geodVertex2[1] = new LatLonPoint(Math.toRadians(10.0), Math.toRadians(9.0));
    geodVertex2[2] = new LatLonPoint(Math.toRadians(10.0), Math.toRadians(10.0));
    geodVertex2[3] = new LatLonPoint(Math.toRadians(9.0), Math.toRadians(10.0));
    geodVertex2[4] = new LatLonPoint(Math.toRadians(8.0), Math.toRadians(9.5));

    PolygonRegion polygonRegion = new PolygonRegion(geodVertex2, shapeModel3);
    application.java3DWorld.addShape3D(polygonRegion);

    //create and add a polyline region
    ShapeModel shapeModel4 = new ShapeModel(Color.red, "Polyline Region", true);

    LatLonPoint[] geodVertex3 = new LatLonPoint[4];
    geodVertex3[0] = new LatLonPoint(Math.toRadians(29.0), Math.toRadians(29.0));
    geodVertex3[1] = new LatLonPoint(Math.toRadians(30.0), Math.toRadians(29.0));
    geodVertex3[2] = new LatLonPoint(Math.toRadians(30.0), Math.toRadians(30.0));
    geodVertex3[3] = new LatLonPoint(Math.toRadians(29.0), Math.toRadians(30.0));

    PolylineRegion polylineRegion = new PolylineRegion(geodVertex3, shapeModel4);
    application.java3DWorld.addShape3D(polylineRegion);

    //read in flyout fan and create and add trajectory based on flyout fan
    double timeStep = 1.0;
    LatLonPoint launchPoint = new LatLonPoint(Math.toRadians(1.0), Math.toRadians(1.0));
    LatLonPoint impactPoint = new LatLonPoint(Math.toRadians(6.0), Math.toRadians(12.0));
    ShapeModel shapeModel5 = new ShapeModel(Color.red, "Traj From FlyoutFan", true);
    String path = Paths.get(".").toAbsolutePath().normalize().toString() + "/src/fans/sample.fan";
    FlyoutFan flyoutFan = new FlyoutFan(path);

    InterpolatedTBMTraj interpolatedTBMTraj = new InterpolatedTBMTraj(launchPoint, impactPoint, flyoutFan, timeStep, shapeModel5);
    application.java3DWorld.addShape3D(interpolatedTBMTraj);

    //create and add a trajectory using propagation
    double azimuth = Math.toRadians(45.0);
    double initVel = 5000.0;
    double initEl = Math.toRadians(45.0);
    ShapeModel shapeModel6 = new ShapeModel(Color.red, "Traj From Prop", true);

    PropagatedTBMTraj propagatedTBMTraj = new PropagatedTBMTraj(launchPoint, azimuth, initVel, initEl, timeStep, shapeModel6);
    application.java3DWorld.addShape3D(propagatedTBMTraj);

    //create and add a zone
    ShapeModel shapeModel7 = new ShapeModel(Color.red, "Zone", true);

    LatLonPoint[] geodVertex4 = new LatLonPoint[4];
    geodVertex4[0] = new LatLonPoint(Math.toRadians(-15.0), Math.toRadians(-15.0));
    geodVertex4[1] = new LatLonPoint(Math.toRadians(-5.0), Math.toRadians(-15.0));
    geodVertex4[2] = new LatLonPoint(Math.toRadians(-5.0), Math.toRadians(-5.0));
    geodVertex4[3] = new LatLonPoint(Math.toRadians(-15.0), Math.toRadians(-5.0));

    Zone zone = new Zone(geodVertex4, minAlt, maxAlt, shapeModel7);
    application.java3DWorld.addShape3D(zone);

    //create and add a flyout fan kinematic envelope
    ShapeModel shapeModel8 = new ShapeModel(Color.red, "Flyout Fan Kinematic Envelope", true);
    GeodPoint geodVertex5 = new GeodPoint(Math.toRadians(-10.0), Math.toRadians(10.0), 0.0);

    FlyoutFanKinematicEnvelope flyoutFanKinematicEnvelope = new FlyoutFanKinematicEnvelope(geodVertex5, shapeModel8);
    application.java3DWorld.addShape3D(flyoutFanKinematicEnvelope);

    //create and add a tbm threat tube
    int size = 10;
    double offset = -10.0;
    ShapeModel shapeModel9 = new ShapeModel(Color.red, "TBM Threat Tube", true);
    LatLonPoint[] geodVertex6 = new LatLonPoint[size];
    LatLonPoint[] geodVertex7 = new LatLonPoint[size];

    for (int i=0; i<size; i++)
    {
      double theta = (2.0 * i * Math.PI)/size;
      double lat = 5.0 + 1.0 * Math.sin(theta);
      double lon = 10.0 + 1.0 * Math.cos(theta);

      geodVertex6[i] = new LatLonPoint(Math.toRadians(lat), Math.toRadians(lon));
      geodVertex7[i] = new LatLonPoint(Math.toRadians(lat+offset), Math.toRadians(lon));
    }

    ThreatTube threatTube = new ThreatTube(geodVertex6, geodVertex7, flyoutFan, shapeModel9);
    application.java3DWorld.addShape3D(threatTube);

    //output so know code is finished processing
    System.out.println("finished");
  }
}