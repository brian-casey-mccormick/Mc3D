//java extension packages
import javax.vecmath.*;
import javax.media.j3d.*;

public class PropagatedTBMTraj extends TBMTraj
{
  private double initVel;
  private double initEl;

  public PropagatedTBMTraj(LatLonPoint launchPoint, double azimuthIn, double initVelIn,
                           double initElIn, double timeStepIn, ShapeModel shapeModelIn)
  {
    super(launchPoint.lat, launchPoint.lon, timeStepIn, azimuthIn, shapeModelIn);
    initVel = initVelIn;
    initEl = initElIn;

    createPropagatedTBMTrajectory();
  }

  private void createPropagatedTBMTrajectory()
  {
    double altitude=0.0;

    //compute initial position of tbm in ecef
    GeodPoint launchPoint = new GeodPoint(initLat, initLon, altitude);
    Vector3d posECEF = CoordinateConversions.convertGeodPointToECEF(launchPoint);

    //compute initial velocity of tbm in ecef
    Vector3d velF = new Vector3d();
    velF.x = initVel * Math.sin(initEl);
    velF.y = 0.0;
    velF.z = initVel * Math.cos(initEl);

    Vector3d velENU = CoordinateConversions.convertFVelocityToENU(velF, rotMatrixFToENU);
    Vector3d velECEF = CoordinateConversions.convertENUVelocityToECEF(velENU, rotMatrixENUToECEF);

    //put these vectors in the state vector
    StateVector stateVector = new StateVector(posECEF, velECEF);

    //set point to current position
    Vector3d pos3dWorld = CoordinateConversions.convertECEFPositionTo3dWorld(stateVector.posECEF);
    Point3d initPoint = new Point3d(pos3dWorld.x, pos3dWorld.y, pos3dWorld.z);
    tbmTrajPointList.add(initPoint);
    numPoints++;

    //propagate until tbm trajectory returns to the surface of the earth
    while (altitude >= 0.0)
    {
      //propagate current state vector by time step
      Support.propagateUsingEuler(stateVector, timeStep);

      //set point to current position
      pos3dWorld = CoordinateConversions.convertECEFPositionTo3dWorld(stateVector.posECEF);
      Point3d trajPoint = new Point3d(pos3dWorld.x, pos3dWorld.y, pos3dWorld.z);
      tbmTrajPointList.add(trajPoint);
      numPoints++;

      //compute altitude
      altitude = CoordinateConversions.computeAltitude(stateVector.posECEF);
    }

    //create new tbm trajectory using common functionality
    createNewTBMTrajectory();
  }
}