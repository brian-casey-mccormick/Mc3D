//java core packages
import java.awt.*;
import java.awt.event.*;

//java extension packages
import javax.swing.*;
import javax.vecmath.*;
import javax.media.j3d.*;

//java3D utility packages
import com.sun.j3d.utils.universe.*;
import com.sun.j3d.utils.behaviors.vp.*;
import com.sun.j3d.utils.picking.*;
import com.sun.j3d.utils.geometry.*;

public class Java3DWorld extends Canvas3D implements Constants
{
  private double initialCameraStepAngle;
  private SimpleUniverse simpleUniverse;
  private Earth earth;
  private OrbitBehavior orbitBehavior;
  private PickCanvas pickCanvas;
  private Transform3D transform3D;
  private Transform3D yawRightTransform3D;
  private Transform3D yawLeftTransform3D;
  private Transform3D pitchUpTransform3D;
  private Transform3D pitchDownTransform3D;
  private Transform3D rollRightTransform3D;
  private Transform3D rollLeftTransform3D;
  private Point3d initialEye;
  private Point3d initialLook;
  private Vector3d initialUp;
  private ControlPanel controlPanel;
  private RotateAboutCenter rotateAboutCenter;

  public Java3DWorld()
  {
    //call base class constructor
    super(SimpleUniverse.getPreferredConfiguration());

    //create simple universe
    simpleUniverse = new SimpleUniverse(this);

    //create scene
    earth = new Earth();
    BranchGroup scene = earth.createScene();

    //attach scene to simple universe
    addBranchGroup(scene);

    //set mouse/keyboard behavior and the view platform data
    setBehaviorAndViewData();
  }

  private void setBehaviorAndViewData()
  {
    //create and set orbit and zoom behavior
    orbitBehavior = new OrbitBehavior(this, OrbitBehavior.REVERSE_ALL | OrbitBehavior.PROPORTIONAL_ZOOM);
    BoundingSphere bounds = new BoundingSphere(new Point3d(), Double.MAX_VALUE);
    orbitBehavior.setSchedulingBounds(bounds);

    double zoomFactor = EARTH_MEAN_RADIUS/50.0;
    orbitBehavior.setTransFactors(zoomFactor, zoomFactor);
    simpleUniverse.getViewingPlatform().setViewPlatformBehavior(orbitBehavior);

    //set initial viewing platform location
    double z = 2*EARTH_MEAN_RADIUS/Math.tan(simpleUniverse.getViewer().getView().getFieldOfView()/2.0);

    //compute and set front and back clip distances and policies
    double frontClipDistance = 1000.0;
    double backClipDistance = z;

    simpleUniverse.getViewer().getView().setFrontClipDistance((float)frontClipDistance);
    simpleUniverse.getViewer().getView().setBackClipDistance((float)backClipDistance);

    //compute and set initial view platform position
    Transform3D initialViewPlatformTransform = new Transform3D();

    //set initial eye position to current
    initialEye = new Point3d();
    initialEye.z = z;

    //set initial look position = (0,0,0)
    initialLook = new Point3d();

    //set initial up direction = (0,1,0)
    initialUp = new Vector3d();
    initialUp.y = 1.0;

    //create initial view platform transform
    initialViewPlatformTransform.lookAt(initialEye, initialLook, initialUp);
    initialViewPlatformTransform.invert();
    simpleUniverse.getViewingPlatform().getViewPlatformTransform().setTransform(initialViewPlatformTransform);

    //create pick canvas
    pickCanvas = new PickCanvas(simpleUniverse.getCanvas(), simpleUniverse.getLocale());
    pickCanvas.setMode(PickCanvas.GEOMETRY);
    pickCanvas.setTolerance(4.0f);

    //create mouse listener
    addMouseListener(new MouseAdapter()
    {
      public void mousePressed(MouseEvent event)
      {
        pickCanvas.setShapeLocation(event);
        PickResult pickResult = pickCanvas.pickClosest();

        if (pickResult != null)
        {
          //get shape3D that was selected
          Shape3D shape3D = (Shape3D)pickResult.getObject();

          //display and modify appearance properties with dialog box
          ModifyPropertiesFrame modifyPropertiesFrame = new ModifyPropertiesFrame(shape3D);
        }
        else
        {
          Vector3d pos3dWorld = new Vector3d();
          boolean found = getPositionOnEarth(event.getX(), event.getY(), pos3dWorld);

          if (found == true)
          {
            //set rotation center
            Point3d pos3dWorldOnEarth = new Point3d(pos3dWorld.x, pos3dWorld.y, pos3dWorld.z);
            rotateAboutCenter.setNewRotationCenter(pos3dWorldOnEarth);
          }
        }
      }
    }
    );

    //create mouse motion listener
    addMouseMotionListener(new MouseMotionAdapter()
    {
      public void mouseMoved(MouseEvent event)
      {
        Vector3d pos3dWorld = new Vector3d();
        boolean found = getPositionOnEarth(event.getX(), event.getY(), pos3dWorld);

        if (found == true)
        {
          //compute updated coordinates and display
          Vector3d posECEF = CoordinateConversions.convert3dWorldVectorToECEF(pos3dWorld);
          GeodPoint geodPoint = CoordinateConversions.convertECEFPositionToGeodPoint(posECEF);
          controlPanel.updateLatLonTracker(geodPoint.lat, geodPoint.lon);
        }
        else
        {
          controlPanel.resetLatLonTracker();
        }
      }
    }
    );

    //allocate transform3D
    transform3D = new Transform3D();

    //create yaw right rotation matrix
    initialCameraStepAngle = Math.toRadians(-0.25);
    yawRightTransform3D = new Transform3D();
    yawRightTransform3D.rotY(-initialCameraStepAngle);

    //create yaw left rotation matrix
    yawLeftTransform3D = new Transform3D();
    yawLeftTransform3D.rotY(initialCameraStepAngle);

    //create pitch up rotation matrix
    pitchUpTransform3D = new Transform3D();
    pitchUpTransform3D.rotX(initialCameraStepAngle);

    //create pitch down rotation matrix
    pitchDownTransform3D = new Transform3D();
    pitchDownTransform3D.rotX(-initialCameraStepAngle);

    //create roll right rotation matrix
    rollRightTransform3D = new Transform3D();
    rollRightTransform3D.rotZ(-initialCameraStepAngle);

    //create roll left rotation matrix
    rollLeftTransform3D = new Transform3D();
    rollLeftTransform3D.rotZ(initialCameraStepAngle);

    //create mouse behaviors for rotating about a point on the earth
    rotateAboutCenter = new RotateAboutCenter(simpleUniverse.getViewingPlatform(), initialCameraStepAngle);
    rotateAboutCenter.setSchedulingBounds(bounds);
    BranchGroup rotationBehavior = new BranchGroup();
    rotationBehavior.addChild(rotateAboutCenter);
    addBranchGroup(rotationBehavior);
  }

  public void addShape3D(MyShape3D shape3D)
  {
    Geometry geometry = shape3D.getGeometry(0);

    if (geometry != null)
    {
      //set capabilities for picking
      pickCanvas.setCapabilities(shape3D, PickCanvas.INTERSECT_TEST);

      //create branch group and add shape3D to it
      BranchGroup bg = new BranchGroup();
      bg.addChild(shape3D);

      //compile branch group and add it to the simple universe
      bg.compile();
      addBranchGroup(bg);
    }
    else
    {
      JOptionPane.showMessageDialog(null, shape3D.getShapeModel().getLabel() + " not valid and cannot be drawn");
    }
  }

  public void removeShape3D(MyShape3D shape3D)
  {
    shape3D.removeGeometry(0);
  }

  private void addBranchGroup(BranchGroup bg)
  {
    simpleUniverse.addBranchGraph(bg);
  }

  public void resetCameraToInitialPosition()
  {
    //get current transform
    simpleUniverse.getViewingPlatform().getViewPlatformTransform().getTransform(transform3D);

    //reset
    transform3D.lookAt(initialEye, initialLook, initialUp);
    transform3D.invert();
    simpleUniverse.getViewingPlatform().getViewPlatformTransform().setTransform(transform3D);
  }

  public void yawRight()
  {
    //get current transform
    simpleUniverse.getViewingPlatform().getViewPlatformTransform().getTransform(transform3D);

    //reset
    transform3D.mul(yawRightTransform3D);
    simpleUniverse.getViewingPlatform().getViewPlatformTransform().setTransform(transform3D);
  }

  public void yawLeft()
  {
    //get current transform
    simpleUniverse.getViewingPlatform().getViewPlatformTransform().getTransform(transform3D);

    //reset
    transform3D.mul(yawLeftTransform3D);
    simpleUniverse.getViewingPlatform().getViewPlatformTransform().setTransform(transform3D);
  }

  public void pitchUp()
  {
    //get current transform
    simpleUniverse.getViewingPlatform().getViewPlatformTransform().getTransform(transform3D);

    //reset
    transform3D.mul(pitchUpTransform3D);
    simpleUniverse.getViewingPlatform().getViewPlatformTransform().setTransform(transform3D);
  }

  public void pitchDown()
  {
    //get current transform
    simpleUniverse.getViewingPlatform().getViewPlatformTransform().getTransform(transform3D);

    //reset
    transform3D.mul(pitchDownTransform3D);
    simpleUniverse.getViewingPlatform().getViewPlatformTransform().setTransform(transform3D);
  }

  public void rollRight()
  {
    //get current transform
    simpleUniverse.getViewingPlatform().getViewPlatformTransform().getTransform(transform3D);

    //reset
    transform3D.mul(rollRightTransform3D);
    simpleUniverse.getViewingPlatform().getViewPlatformTransform().setTransform(transform3D);
  }

  public void rollLeft()
  {
    //get current transform
    simpleUniverse.getViewingPlatform().getViewPlatformTransform().getTransform(transform3D);

    //reset
    transform3D.mul(rollLeftTransform3D);
    simpleUniverse.getViewingPlatform().getViewPlatformTransform().setTransform(transform3D);
  }

  public void resetCameraRotationAngles(double factor)
  {
    double newAngle = initialCameraStepAngle * factor;

    yawRightTransform3D.rotY(-newAngle);
    yawLeftTransform3D.rotY(newAngle);
    pitchUpTransform3D.rotX(newAngle);
    pitchDownTransform3D.rotX(-newAngle);
    rollRightTransform3D.rotZ(-newAngle);
    rollLeftTransform3D.rotZ(newAngle);
    rotateAboutCenter.setNewRotationAngle(newAngle);
  }

  public void setControlPanel(ControlPanel controlPanelIn)
  {
    controlPanel = controlPanelIn;
  }

  public Dimension getPreferredSize()
  {
    return new Dimension(550, 550);
  }

  public boolean getPositionOnEarth(int mouseX, int mouseY, Vector3d pos3dWorld)
  {
    //get the eye position relative to the image plate
    Point3d eyePos = new Point3d();
    getCenterEyeInImagePlate(eyePos);

    //get the mouse position relative to the image plate
    Point3d mousePos = new Point3d();
    getPixelLocationInImagePlate(mouseX, mouseY, mousePos);

    //get the image plate to 3d world transform
    getImagePlateToVworld(transform3D);

    //compute the eye and mouse 3d world locations
    transform3D.transform(eyePos);
    transform3D.transform(mousePos);

    //compute the ray vector from the eye to the mouse
    Vector3d rayVector = new Vector3d();
    rayVector.sub(mousePos, eyePos);

    //make sure point projects onto the earth
    Vector3d eyeVector = new Vector3d(eyePos.x, eyePos.y, eyePos.z);

    double A = rayVector.lengthSquared();
    double B = eyeVector.dot(rayVector);
    double C = eyeVector.lengthSquared() - EARTH_MEAN_RADIUS_SQUARED;
    double discriminant = B*B - A*C;

    if (discriminant >= 0.0)
    {
      double temp1 = -B/A;
      double temp2 = Math.sqrt(discriminant)/A;
      double intersectionTime = Math.min(temp1 + temp2, temp1 - temp2);

      //compute where the mouse cursor intersects with the earth
      pos3dWorld.x = eyePos.x + rayVector.x * intersectionTime;
      pos3dWorld.y = eyePos.y + rayVector.y * intersectionTime;
      pos3dWorld.z = eyePos.z + rayVector.z * intersectionTime;

      return true;
    }
    else
    {
      return false;
    }
  }
}