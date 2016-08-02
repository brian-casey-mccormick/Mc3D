//java core packages
import java.awt.*;
import java.awt.image.BufferedImage;
import java.util.*;

//java extension packages
import javax.vecmath.*;
import javax.media.j3d.*;

public class Support implements Constants
{
  public static Vector3d computeAccelerationOfGravityAtPoint(Vector3d posECEF)
  {
    Vector3d accECEF = new Vector3d();

    //set n = (0, 0, 1), w = (0, 0, OMEGA), and u to normalized posECEF
    Vector3d n = new Vector3d(0.0, 0.0, 1.0);
    Vector3d w = new Vector3d(0.0, 0.0, OMEGA);
    Vector3d u = new Vector3d();
    u.normalize(posECEF);

    //compute acceleration terms
    double rMag = posECEF.length();
    double temp1 = -MU / (Math.pow(rMag, 2));
    double temp2 = 1.5 * J2 * Math.pow(EARTH_MEAN_RADIUS, 2) / (Math.pow(rMag, 2));
    double temp3 = 1 - 5 * Math.pow(posECEF.z, 2) / (Math.pow(rMag, 2));
    double temp4 = 2 * posECEF.z / rMag;

    //compute acceleration components
    accECEF.x = temp1 * (u.x + temp2 * (temp3 * u.x + temp4 * n.x));
    accECEF.y = temp1 * (u.y + temp2 * (temp3 * u.y + temp4 * n.y));
    accECEF.z = temp1 * (u.z + temp2 * (temp3 * u.z + temp4 * n.z));

    return accECEF;
  }

  public static void propagateUsingEuler(StateVector stateVector, double timeStep)
  {
    //create position dot and velocity dot vectors
    Vector3d posDotECEF = new Vector3d(stateVector.velECEF);
    Vector3d velDotECEF = computeAccelerationOfGravityAtPoint(stateVector.posECEF);

    //propagate using euler
    Vector3d newPosECEF = new Vector3d();
    newPosECEF.x = stateVector.posECEF.x + posDotECEF.x * timeStep;
    newPosECEF.y = stateVector.posECEF.y + posDotECEF.y * timeStep;
    newPosECEF.z = stateVector.posECEF.z + posDotECEF.z * timeStep;

    Vector3d newVelECEF = new Vector3d();
    newVelECEF.x = stateVector.velECEF.x + velDotECEF.x * timeStep;
    newVelECEF.y = stateVector.velECEF.y + velDotECEF.y * timeStep;
    newVelECEF.z = stateVector.velECEF.z + velDotECEF.z * timeStep;

    //set new position and velocity vectors and return
    stateVector.setPosECEF(newPosECEF);
    stateVector.setVelECEF(newVelECEF);
  }

  public static double computeGroundRange(double initLat, double initLon, double finalLat, double finalLon)
  {
    double diff1 = finalLon - initLon;
    double diff2 = finalLat - initLat;

    double cosTheta = Math.cos(finalLat)*Math.cos(initLat)*Math.cos(diff1) +
                      Math.sin(finalLat)*Math.sin(initLat);

    double theta = Math.acos(cosTheta);
    double range = EARTH_MEAN_RADIUS * theta;

    return range;
  }

  public static double computeAzimuth(double initLat, double initLon, double finalLat, double finalLon)
  {
    double azimuth = 0.0, eps = 1e-10;
    double diff1 = finalLon - initLon;
    double diff2 = finalLat - initLat;

    double cosTheta = Math.cos(finalLat)*Math.cos(initLat)*Math.cos(diff1) +
                      Math.sin(finalLat)*Math.sin(initLat);

    double theta = Math.acos(cosTheta);

    //if both points are on the same longitude, azimuth is zero or pi
    if (diff1 == 0.0)
    {
      if (diff2 > 0.0)
      {
        azimuth = 0.0;
      }
      else
      {
        azimuth = Math.PI;
      }
    }
    //else if the range angle and the longitude separation between the points
    //are both set to pi, set azimuth to zero
    else if (Math.abs(Math.abs(diff1) - Math.PI) <= eps && Math.abs(theta - Math.PI) <= eps)
    {
      azimuth = 0.0;
    }
    //else if first point is at either pole, set azimuth to zero or pi accordingly
    else if (Math.abs(Math.abs(initLat) - (Math.PI/2.0)) <= eps)
    {
      if (finalLat > 0.0)
      {
        azimuth = Math.PI;
      }
      else
      {
        azimuth = 0.0;
      }
    }
    else
    {
      double denom = Math.cos(initLat) * Math.sin(theta);

      if (denom != 0.0)
      {
        double temp = (Math.sin(finalLat) - Math.sin(initLat) * cosTheta) / denom;

        if (temp < -1.0)
        {
          temp = -1.0;
        }
        else if (temp > 1.0)
        {
          temp = 1.0;
        }

        azimuth = Math.acos(temp);

        if ((diff1 < 0.0 && diff1 > -Math.PI) || (diff1 > Math.PI))
        {
          azimuth = 2.0 * Math.PI - azimuth;
        }
      }
      else
      {
        azimuth = 0.0;
      }
    }

    return azimuth;
  }

  public static double hermitePosInterp(double t, double t0, double t1, double pos0,
                                        double pos1, double pos0dot, double pos1dot)
  {
    double interpValue;

    //compute convenenience variables
    double dt = t1-t0;
    double dy = pos1-pos0;
    double dyOverDt = dy / dt;
    double temp = pos1dot + pos0dot - 2*dyOverDt;

    //compute coefficients of interpolating polynomial
    double c0 = pos0;
    double c1 = pos0dot;
    double c2 = (dyOverDt - pos0dot - temp) / dt;
    double c3 = temp / (dt*dt);

    //compute interpolated value
    double relTime = t - t0;
    interpValue = c0 + (c1 + (c2 + c3 * relTime) * relTime) * relTime;

    return interpValue;
  }

  public static double hermiteVelInterp(double t, double t0, double t1, double vel0,
                                        double vel1, double vel0dot, double vel1dot)
  {
    double interpValue;

    //compute convenenience variables
    double dt = t1-t0;
    double dy = vel1-vel0;
    double dyOverDt = dy / dt;
    double temp = vel1dot + vel0dot - 2*dyOverDt;

    //compute coefficients of interpolating polynomial
    double c0 = vel0;
    double c1 = vel0dot;
    double c2 = (dyOverDt - vel0dot - temp) / dt;
    double c3 = temp / (dt*dt);

    //compute interpolated value
    double relTime = t - t0;
    interpValue = c1 + (2*c2 + 3*c3*relTime) * relTime;

    return interpValue;
  }

  public static Vector3d interpolatePositionAtTime(double time, KnotECEF[] knotECEFInterpolated)
  {
    int numKnots = knotECEFInterpolated.length;
    double x=0.0, y=0.0, z=0.0;
    Vector3d posECEF = new Vector3d();

    for (int i=0; i<numKnots-1; i++)
    {
      KnotECEF knotECEF1 = knotECEFInterpolated[i];
      KnotECEF knotECEF2 = knotECEFInterpolated[i+1];

      if (knotECEF1.time <= time && time < knotECEF2.time)
      {
        posECEF.x = hermitePosInterp(time, knotECEF1.time, knotECEF2.time, knotECEF1.x,
                                     knotECEF2.x, knotECEF1.xdot, knotECEF2.xdot);

        posECEF.y = hermitePosInterp(time, knotECEF1.time, knotECEF2.time, knotECEF1.y,
                                     knotECEF2.y, knotECEF1.ydot, knotECEF2.ydot);

        posECEF.z = hermitePosInterp(time, knotECEF1.time, knotECEF2.time, knotECEF1.z,
                                     knotECEF2.z, knotECEF1.zdot, knotECEF2.zdot);

        break;
      }
    }

    return posECEF;
  }

  public static Vector3d interpolateVelocityAtTime(double time, KnotECEF[] knotECEFInterpolated)
  {
    int numKnots = knotECEFInterpolated.length;
    double x=0.0, y=0.0, z=0.0;
    Vector3d velECEF = new Vector3d();

    for (int i=0; i<numKnots-1; i++)
    {
      KnotECEF knotECEF1 = knotECEFInterpolated[i];
      KnotECEF knotECEF2 = knotECEFInterpolated[i+1];

      if (knotECEF1.time <= time && time < knotECEF2.time)
      {
        velECEF.x = hermiteVelInterp(time, knotECEF1.time, knotECEF2.time, knotECEF1.x,
                                     knotECEF2.x, knotECEF1.xdot, knotECEF2.xdot);

        velECEF.y = hermiteVelInterp(time, knotECEF1.time, knotECEF2.time, knotECEF1.y,
                                     knotECEF2.y, knotECEF1.ydot, knotECEF2.ydot);

        velECEF.z = hermiteVelInterp(time, knotECEF1.time, knotECEF2.time, knotECEF1.z,
                                     knotECEF2.z, knotECEF1.zdot, knotECEF2.zdot);

        break;
      }
    }

    return velECEF;
  }

  public static KnotCD interpolateCorrespondingKnotsBetweenTrajectories(double alpha, KnotCD knotCD1, KnotCD knotCD2)
  {
    KnotCD knotCD = new KnotCD();

    double time1 = knotCD1.time;
    double time2 = knotCD2.time;

    double dr1 = knotCD1.dr;
    double dr2 = knotCD2.dr;

    double up1 = knotCD1.up;
    double up2 = knotCD2.up;

    double fpa1 = knotCD1.fpa;
    double fpa2 = knotCD2.fpa;

    double vel1 = knotCD1.vel;
    double vel2 = knotCD2.vel;

    knotCD.time = (1-alpha) * time1 + alpha * time2;
    knotCD.dr = (1-alpha) * dr1 + alpha * dr2;
    knotCD.up = (1-alpha) * up1 + alpha * up2;
    knotCD.fpa = (1-alpha) * fpa1 + alpha * fpa2;
    knotCD.vel = (1-alpha) * vel1 + alpha * vel2;

    return knotCD;
  }

  public static Point2d computeCenter(ArrayList point2dArrayList)
  {
    int size=point2dArrayList.size();
    Point2d center = new Point2d();

    if (size == 0)
    {
      return center;
    }

    double sumX=0.0;
    double sumY=0.0;

    for (int i=0; i<size; i++)
    {
      Point2d point2d = (Point2d)point2dArrayList.get(i);

      //sum all components of x and y
      sumX+=point2d.x;
      sumY+=point2d.y;
    }

    //compute centriod data
    center.x = sumX/size;
    center.y = sumY/size;

    return center;
  }

  public static LatLonPoint computeCenter(LatLonPoint[] latLonPointIn)
  {
    int length=latLonPointIn.length;
    LatLonPoint latLonCenter = new LatLonPoint();

    if (length == 0)
    {
      return latLonCenter;
    }

    double sumLat=0.0;
    double sumLon=0.0;

    for (int i=0; i<length; i++)
    {
      //accumulate lat and lon
      sumLat+=latLonPointIn[i].lat;
      sumLon+=latLonPointIn[i].lon;
    }

    //compute centriod data
    latLonCenter.lat = sumLat/length;
    latLonCenter.lon = sumLon/length;

    return latLonCenter;
  }

  public static double computeCrossProduct(Vector2d vec1, Vector2d vec2)
  {
    double crossProduct = vec1.x * vec2.y - vec2.x * vec1.y;

    return crossProduct;
  }

  public static Vector3d computeVectorNormalToEarthAtLocation(Vector3d posECEF)
  {
    Vector3d normal = new Vector3d();

    normal.x = posECEF.x / EARTH_MEAN_RADIUS;
    normal.y = posECEF.y / EARTH_MEAN_RADIUS;
    normal.z = posECEF.z / EARTH_MEAN_RADIUS;

    return normal;
  }

  public static void delay(int n)
  {
    try
    {
      Thread.sleep(n);
    }
    catch (Exception e)
    {
      System.out.println("error in delay function");
    }
  }

  public static boolean polygonVerticesInCWOrder(ArrayList polygon)
  {
    int size=polygon.size();
    double sum=0.0;

    Point2d p1 = new Point2d();
    Point2d p2 = new Point2d();
    Point2d p3 = new Point2d();

    for (int i=0; i<size; i++)
    {
      //set p1 -> prev vertex, p2 -> start vertex, p3 -> next vertex
      if (i==0)
      {
        p1 = (Point2d)polygon.get(size-1);
        p2 = (Point2d)polygon.get(0);
        p3 = (Point2d)polygon.get(1);
      }
      else if (i==size-1)
      {
        p1 = (Point2d)polygon.get(i-1);
        p2 = (Point2d)polygon.get(i);
        p3 = (Point2d)polygon.get(0);
      }
      else
      {
        p1 = (Point2d)polygon.get(i-1);
        p2 = (Point2d)polygon.get(i);
        p3 = (Point2d)polygon.get(i+1);
      }

      //compute cross product (p3-p1) x (p2-p1)
      Vector2d v1 = new Vector2d(p3.x - p1.x, p3.y - p1.y);
      Vector2d v2 = new Vector2d(p2.x - p1.x, p2.y - p1.y);
      double crossProduct = Support.computeCrossProduct(v1, v2);

      //compute angle between (p3 - p2) and (p1 - p2)
      v1.x = p3.x - p2.x;
      v1.y = p3.y - p2.y;

      v2.x = p1.x - p2.x;
      v2.y = p1.y - p2.y;

      double angle = v1.angle(v2);

      //sum angles under assumption that vertices are CW
      if (crossProduct > 0.0)
      {
        sum += angle;
      }
      else
      {
        sum += (TWO_PI - angle);
      }
    }

    //compute reqd sum if vertices are CW
    double eps = 0.001;
    double reqdSumIfVerticesAreCW = PI * (size-2);

    //if sum within tolerance, vertices are in CW order
    if (Math.abs(sum - reqdSumIfVerticesAreCW) < eps)
    {
      return true;
    }
    //else vertices are in CCW order
    else
    {
      return false;
    }
  }

  public static ArrayList computePerimeterGridForPolylineVertices(double alt, LatLonPoint[] latLonVertexIn)
  {
    ArrayList arrayListGrid = new ArrayList();

    //interpolate between vertices
    for (int i=0; i<latLonVertexIn.length-1; i++)
    {
      ArrayList geodPointArrayList = interpolateBetweenVertices(latLonVertexIn[i], latLonVertexIn[i+1], alt);
      arrayListGrid.addAll(geodPointArrayList);
    }

    return arrayListGrid;
  }

  public static ArrayList computePerimeterGridForPolygonVertices(double alt, LatLonPoint[] latLonVertexIn)
  {
    ArrayList arrayListGrid = new ArrayList();

    //interpolate between vertices
    for (int i=0; i<latLonVertexIn.length; i++)
    {
      ArrayList geodPointArrayList = new ArrayList();

      if (i==latLonVertexIn.length-1)
      {
        geodPointArrayList = interpolateBetweenVertices(latLonVertexIn[i], latLonVertexIn[0], alt);
      }
      else
      {
        geodPointArrayList = interpolateBetweenVertices(latLonVertexIn[i], latLonVertexIn[i+1], alt);
      }

      arrayListGrid.addAll(geodPointArrayList);
    }

    return arrayListGrid;
  }

  public static ArrayList interpolateBetweenVertices(LatLonPoint latLonVertex1, LatLonPoint latLonVertex2, double alt)
  {
    ArrayList geodPointArrayList = new ArrayList();

    int numSteps=1;
    double deltaLat = latLonVertex2.lat - latLonVertex1.lat;
    double deltaLon = latLonVertex2.lon - latLonVertex1.lon;

    //compute delta theta max
    double deltaThetaMax = 2.0*Math.acos(EARTH_MEAN_RADIUS/(EARTH_MEAN_RADIUS+alt));

    //compute delta theta between points
    double actualDeltaTheta = Math.sqrt(deltaLat*deltaLat + deltaLon*deltaLon);

    //compute the number of steps to take between the vertices
    while (actualDeltaTheta > deltaThetaMax)
    {
      numSteps++;
      deltaLat = (latLonVertex2.lat - latLonVertex1.lat)/numSteps;
      deltaLon = (latLonVertex2.lon - latLonVertex1.lon)/numSteps;
      actualDeltaTheta = Math.sqrt(deltaLat*deltaLat + deltaLon*deltaLon);
    }

    //set lat and lon of first point
    double lat = latLonVertex1.lat;
    double lon = latLonVertex1.lon;

    //interpolate new points
    for (int i=0; i<numSteps+1; i++)
    {
      GeodPoint geodPoint = new GeodPoint(lat, lon, alt);
      geodPointArrayList.add(geodPoint);

      //go to next point
      lat+=deltaLat;
      lon+=deltaLon;
    }

    return geodPointArrayList;
  }
}