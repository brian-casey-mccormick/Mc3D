//java extension packages
import javax.vecmath.*;
import javax.media.j3d.*;

public class InterpolatedTBMTraj extends TBMTraj
{
  private boolean validTraj;
  private double finalLat;
  private double finalLon;
  private FlyoutFan flyoutFan;

  public InterpolatedTBMTraj(LatLonPoint latLonLaunchPoint, LatLonPoint latLonImpactPoint,
                             FlyoutFan flyoutFanIn, double timeStepIn, ShapeModel shapeModelIn)
  {
    super(latLonLaunchPoint.lat, latLonLaunchPoint.lon, timeStepIn, shapeModelIn);

    if (flyoutFanIn.getNumTrajs() < 2)
    {
      validTraj = false;
      return;
    }

    finalLat = latLonImpactPoint.lat;
    finalLon = latLonImpactPoint.lon;

    //compute azimuth and set azimuth related data
    flyoutFan = flyoutFanIn;
    azimuth = Support.computeAzimuth(initLat, initLon, finalLat, finalLon);
    rotMatrixFToENU = new Matrix3d();
    validTraj = true;

    createInterpolatedTBMTrajectory();
  }

  private void createInterpolatedTBMTrajectory()
  {
    //interpolate knots of current trajectory based on threat's ground range
    double gr = Support.computeGroundRange(initLat, initLon, finalLat, finalLon);

    //get trajectory indices and alpha
    boolean rangeValid = false;
    double alpha = 0.0;

    //initialize trajectories
    TBMTrajCD t1 = flyoutFan.getTBMTrajCD(0);
    TBMTrajCD t2 = flyoutFan.getTBMTrajCD(1);

    for (int trajIndex=0; trajIndex<flyoutFan.getNumTrajs()-1; trajIndex++)
    {
      t1 = flyoutFan.getTBMTrajCD(trajIndex);
      t2 = flyoutFan.getTBMTrajCD(trajIndex+1);

      double gr1 = t1.getLastKnot().dr;
      double gr2 = t2.getLastKnot().dr;

      if (gr1 <= gr && gr < gr2)
      {
        alpha = (gr - gr1) / (gr2 - gr1);
        rangeValid = true;
        break;
      }
    }

    if (rangeValid == false)
    {
      validTraj = false;
      return;
    }

    //intepolate between the trajectories
    int numKnots = flyoutFan.getNumKnots();
    KnotECEF[] knotECEFInterpolated = new KnotECEF[numKnots];
    GeodPoint launchPoint = new GeodPoint(initLat, initLon, 0.0);
    Vector3d launchPointECEF = CoordinateConversions.convertGeodPointToECEF(launchPoint);

    for (int i=0; i<numKnots; i++)
    {
      //interpolate between corresponding knots of the trajectories
      KnotCD knotCDInterpolated = Support.interpolateCorrespondingKnotsBetweenTrajectories(alpha, t1.getKnot(i), t2.getKnot(i));

      //compute posECEF
      GeodPoint geodPos = CoordinateConversions.convertCDPositionToGeodPoint(knotCDInterpolated.dr,
                                                                             knotCDInterpolated.up,
                                                                             azimuth, launchPoint);

      Vector3d posECEF = CoordinateConversions.convertGeodPointToECEF(geodPos);

      //re-calculate rotation matrix enu to ecef for current knot position
      double sLat = Math.sin(geodPos.lat);
      double sLon = Math.sin(geodPos.lon);
      double cLat = Math.cos(geodPos.lat);
      double cLon = Math.cos(geodPos.lon);

      rotMatrixENUToECEF.m00 = -sLon;
      rotMatrixENUToECEF.m01 = -cLon * sLat;
      rotMatrixENUToECEF.m02 = cLon * cLat;
      rotMatrixENUToECEF.m10 = cLon;
      rotMatrixENUToECEF.m11 = -sLon * sLat;
      rotMatrixENUToECEF.m12 = sLon * cLat;
      rotMatrixENUToECEF.m20 = 0.0;
      rotMatrixENUToECEF.m21 = cLat;
      rotMatrixENUToECEF.m22 = sLat;

      //compute velECEF
      Vector3d velENU = CoordinateConversions.convertCDVelocityToENU(knotCDInterpolated.vel, knotCDInterpolated.fpa, azimuth);
      Vector3d velECEF = CoordinateConversions.convertENUVelocityToECEF(velENU, rotMatrixENUToECEF);

      //store data to knotECEF and add it to the list
      KnotECEF knotECEF = new KnotECEF(knotCDInterpolated.time, posECEF.x, posECEF.y, posECEF.z,
                                       velECEF.x, velECEF.y, velECEF.z);

      knotECEFInterpolated[i] = knotECEF;
    }

    //get start and stop times
    double initialTime = knotECEFInterpolated[0].time;
    double interpolationTime = knotECEFInterpolated[numKnots-1].time;

    //interpolate between knots from initial to final knot times
    for (double time=initialTime; time<interpolationTime; time+=timeStep)
    {
      Vector3d posECEF = Support.interpolatePositionAtTime(time, knotECEFInterpolated);
      Vector3d velECEF = Support.interpolateVelocityAtTime(time, knotECEFInterpolated);
      StateVector stateVector = new StateVector(posECEF, velECEF);
      tbmTrajStateVectorList.add(stateVector);

      Vector3d pos3dWorld = CoordinateConversions.convertECEFPositionTo3dWorld(posECEF);
      Point3d trajPoint = new Point3d(pos3dWorld.x, pos3dWorld.y, pos3dWorld.z);
      tbmTrajPointList.add(trajPoint);

      numPoints++;
    }

    //create new tbm trajectory using common functionality
    createNewTBMTrajectory();
  }

  public boolean GetValidTrajFlag()
  {
    return validTraj;
  }
}