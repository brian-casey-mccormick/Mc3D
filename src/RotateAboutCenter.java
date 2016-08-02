//java core packages
import java.awt.*;
import java.awt.event.*;
import java.util.*;

//java extension packages
import javax.swing.*;
import javax.vecmath.*;
import javax.media.j3d.*;

//java3D utility packages
import com.sun.j3d.utils.universe.*;
import com.sun.j3d.utils.behaviors.vp.*;
import com.sun.j3d.utils.picking.*;

public class RotateAboutCenter extends Behavior implements Constants
{
  private double rotationAngle;
  private ViewingPlatform vp;
  private Transform3D transform3D;
  private Point3d rotationCenter;
  private WakeupCondition wakeupCondition;

  public RotateAboutCenter(ViewingPlatform vpIn, double rotationAngleIn)
  {
    rotationAngle = rotationAngleIn;
    rotationCenter = new Point3d();
    transform3D = new Transform3D();
    vp = vpIn;

    WakeupOnAWTEvent wakeupEvent = new WakeupOnAWTEvent(KeyEvent.KEY_PRESSED);
    WakeupCriterion[] wakeupCriteria = {wakeupEvent};
    wakeupCondition = new WakeupOr(wakeupCriteria);
  }

  public void initialize()
  {
    wakeupOn(wakeupCondition);
  }

  public void processStimulus(Enumeration detected)
  {
    while (detected.hasMoreElements())
    {
      WakeupCriterion wakeupCriterion = (WakeupCriterion)detected.nextElement();

      if (wakeupCriterion instanceof WakeupOnAWTEvent)
      {
        WakeupOnAWTEvent awtEvent = (WakeupOnAWTEvent)wakeupCriterion;
        AWTEvent[] events = awtEvent.getAWTEvent();
        moveViewPlatform(events);
      }
    }

    wakeupOn(wakeupCondition);
  }

  private void moveViewPlatform(AWTEvent[] awtEvents)
  {
    //make sure rotation center is set before allowing this operation
    double eps = 1e-10;

    if (rotationCenter.distance(new Point3d()) < eps)
    {
      return;
    }

    for (int i=0; i<awtEvents.length; i++)
    {
      if (awtEvents[i] instanceof KeyEvent)
      {
        KeyEvent keyEvent = (KeyEvent)awtEvents[i];

        if (keyEvent.getID() == KeyEvent.KEY_PRESSED)
        {
          switch (keyEvent.getKeyCode())
          {
            case KeyEvent.VK_LEFT:
              performHorizontalLeftRotation();
              break;

            case KeyEvent.VK_RIGHT:
              performHorizontalRightRotation();
              break;

            case KeyEvent.VK_UP:
              performVerticalUpRotation();
              break;

            case KeyEvent.VK_DOWN:
              performVerticalDownRotation();
              break;
          }
        }
      }
    }
  }

  private void performHorizontalLeftRotation()
  {
    //set rotationAngle to negative value
    rotationAngle *= -1.0;

    performHorizontalRotation();

    //return rotationAngle to positive value
    rotationAngle *= -1.0;
  }

  public void performVerticalUpRotation()
  {
    //set rotationAngle to negative value
    rotationAngle *= -1.0;

    performVerticalRotation();

    //return rotationAngle to positive value
    rotationAngle *= -1.0;
  }

  private void performHorizontalRightRotation()
  {
    performHorizontalRotation();
  }

  private void performVerticalDownRotation()
  {
    performVerticalRotation();
  }

  private void performHorizontalRotation()
  {
    vp.getViewPlatformTransform().getTransform(transform3D);

    //compute unit vector to center of rotation
    Vector3d u = new Vector3d(rotationCenter.x/EARTH_MEAN_RADIUS,
                              rotationCenter.y/EARTH_MEAN_RADIUS,
                              rotationCenter.z/EARTH_MEAN_RADIUS);

    //get current view platform transform
    Vector3d translate = new Vector3d();
    transform3D.get(translate);

    //save translate as eye position and rotate eye about u
    Point3d eye = new Point3d(translate.x, translate.y, translate.z);
    Point3d newEye = CoordinateConversions.rotatePointAboutArbitraryAxis(u, eye, rotationAngle);

    //reset view platform transform data
    transform3D.lookAt(newEye, rotationCenter, u);
    transform3D.invert();

    vp.getViewPlatformTransform().setTransform(transform3D);
  }

  private void performVerticalRotation()
  {
    vp.getViewPlatformTransform().getTransform(transform3D);

    //compute unit vector to center of rotation
    Vector3d u = new Vector3d(rotationCenter.x/EARTH_MEAN_RADIUS,
                              rotationCenter.y/EARTH_MEAN_RADIUS,
                              rotationCenter.z/EARTH_MEAN_RADIUS);

    //get current view platform transform
    Vector3d translate = new Vector3d();
    transform3D.get(translate);

    //translate eye to make it relative to rotation center
    Point3d relEyePoint = new Point3d(translate.x-rotationCenter.x,
                                      translate.y-rotationCenter.y,
                                      translate.z-rotationCenter.z);

    Vector3d relEyeVector = new Vector3d(relEyePoint.x, relEyePoint.y, relEyePoint.z);

    //compute v = relativeEyeVector x u
    Vector3d v = new Vector3d();
    v.cross(relEyeVector, u);
    v.normalize();

    //rotate eye about v
    Point3d newEye = CoordinateConversions.rotatePointAboutArbitraryAxis(v, relEyePoint, rotationAngle);

    //translate back
    newEye.x += rotationCenter.x;
    newEye.y += rotationCenter.y;
    newEye.z += rotationCenter.z;

    //reset view platform transform data
    transform3D.lookAt(newEye, rotationCenter, u);
    transform3D.invert();

    vp.getViewPlatformTransform().setTransform(transform3D);
  }

  public void setNewRotationCenter(Point3d rotationCenterIn)
  {
    rotationCenter.x = rotationCenterIn.x;
    rotationCenter.y = rotationCenterIn.y;
    rotationCenter.z = rotationCenterIn.z;
  }

  public void setNewRotationAngle(double rotationAngleIn)
  {
    rotationAngle = rotationAngleIn;
  }
}