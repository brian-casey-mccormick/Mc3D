//java core packages
import java.util.*;

//java extension packages
import javax.vecmath.*;
import javax.media.j3d.*;

//java3D utility packages
import com.sun.j3d.utils.geometry.*;

public class ThreatTube extends MyShape3D
{
  private int id;
  private static int nextId = 0;
  private int numPoints;
  private int numDivisions;
  private int[] connectivity;
  private double maxApogee;
  private double anglePerDivision;
  private double deltaTimeBetweenTrajPoints;
  private double minTimeDeltaForBinarySearch;
  private ArrayList point3dList;
  private FlyoutFan flyoutFan;
  private Point3d[] vertex;
  private LatLonPoint[] defendedAreaLatLonPoint;
  private LatLonPoint[] threatOriginLatLonPoint;
  private ArrayList tbmCentroidTrajStateVectorList;
  private ArrayList tbmTrajStateVectorArrayList;
  private Color3f[] color;
  private int[] colorIndices;

  public ThreatTube(LatLonPoint[] defendedAreaLatLonPointIn, LatLonPoint[] threatOriginLatLonPointIn,
                    FlyoutFan flyoutFanIn, ShapeModel shapeModelIn)
  {
    super(shapeModelIn);

    if (defendedAreaLatLonPointIn.length < 2 || threatOriginLatLonPointIn.length < 2)
    {
      return;
    }

    id = nextId++;

    flyoutFan = flyoutFanIn;
    point3dList = new ArrayList();

    anglePerDivision = Math.toRadians(10.0);
    numDivisions = 37;
    deltaTimeBetweenTrajPoints = 2.0;
    minTimeDeltaForBinarySearch = 0.1;
    maxApogee = -1e10;

    //store defended area data
    int length1 = defendedAreaLatLonPointIn.length;
    defendedAreaLatLonPoint = new LatLonPoint[length1];

    for (int i=0; i<length1; i++)
    {
      defendedAreaLatLonPoint[i] = new LatLonPoint(defendedAreaLatLonPointIn[i]);
    }

    //store threat origin data
    int length2 = threatOriginLatLonPointIn.length;
    threatOriginLatLonPoint = new LatLonPoint[length2];

    for (int i=0; i<length2; i++)
    {
      threatOriginLatLonPoint[i] = new LatLonPoint(threatOriginLatLonPointIn[i]);
    }

    createNewThreatTube();
  }

  private void createNewThreatTube()
  {
    //compute centroids and create a trajectory between them
    computeCenterTrajectoryData();

    //generate and store trajectories for each LP/IP combination
    interpolateTrajectoriesBetweenAllCombinations();

    //make sure can generate threat tube
    if (tbmTrajStateVectorArrayList.size() < 2)
    {
      return;
    }

    //generate the threat tube vertices
    boolean successful = generateAndSetThreatTubeVertices();

    if (successful == false)
    {
      return;
    }

    //generate connectivity matrix and set color
    computeConnectivityMatrixAndSetColor();

    //set geometry data
    GeometryInfo gi = new GeometryInfo(GeometryInfo.QUAD_ARRAY);
    gi.setCoordinates(vertex);
    gi.setCoordinateIndices(connectivity);
    gi.setColors(color);
    gi.setColorIndices(colorIndices);

    //create normals
    NormalGenerator ng = new NormalGenerator();
    ng.setCreaseAngle((float)Math.toRadians(0.0));
    ng.generateNormals(gi);

    //set geometry
    setGeometry(gi.getGeometryArray());
  }

  private void computeCenterTrajectoryData()
  {
    LatLonPoint impactLatLonCenter = Support.computeCenter(defendedAreaLatLonPoint);
    LatLonPoint launchLatLonCenter = Support.computeCenter(threatOriginLatLonPoint);

    InterpolatedTBMTraj centerTraj = new InterpolatedTBMTraj(launchLatLonCenter, impactLatLonCenter, flyoutFan,
                                                             deltaTimeBetweenTrajPoints, getShapeModel());

    tbmCentroidTrajStateVectorList = centerTraj.getTBMTrajStateVectorList();
  }

  private void interpolateTrajectoriesBetweenAllCombinations()
  {
    InterpolatedTBMTraj interpolatedTBMTraj;
    tbmTrajStateVectorArrayList = new ArrayList();

    for (int i=0; i < defendedAreaLatLonPoint.length; i++)
    {
      for (int j=0; j < threatOriginLatLonPoint.length; j++)
      {
        //compute trajectory from launch to impact for given flyout fan
        interpolatedTBMTraj = new InterpolatedTBMTraj(threatOriginLatLonPoint[j], defendedAreaLatLonPoint[i],
                                                      flyoutFan, deltaTimeBetweenTrajPoints, getShapeModel());

        //store each valid trajectory in trajPoint3DArrayList
        if (interpolatedTBMTraj.GetValidTrajFlag())
        {
          double altitude = interpolatedTBMTraj.getApogee();
          maxApogee = Math.max(maxApogee, altitude);
          tbmTrajStateVectorArrayList.add(interpolatedTBMTraj.getTBMTrajStateVectorList());
        }
      }
    }
  }

  private boolean generateAndSetThreatTubeVertices()
  {
    if (tbmCentroidTrajStateVectorList.size() < 3)
    {
      return false;
    }

    //compute vector normal to trajectory and set it to the 'up' vector
    StateVector state1 = (StateVector)tbmCentroidTrajStateVectorList.get(0);
    Vector3d pos1 = new Vector3d(state1.posECEF);

    StateVector state2 = (StateVector)tbmCentroidTrajStateVectorList.get(1);
    Vector3d pos2 = new Vector3d(state2.posECEF);

    StateVector state3 = (StateVector)tbmCentroidTrajStateVectorList.get(2);
    Vector3d pos3 = new Vector3d(state3.posECEF);

    Vector3d up = new Vector3d();
    Vector3d temp1 = new Vector3d(pos1.x - pos2.x, pos1.y - pos2.y, pos1.z - pos2.z);
    Vector3d temp2 = new Vector3d(pos3.x - pos2.x, pos3.y - pos2.x, pos3.z - pos2.z);
    up.cross(temp1, temp2);

    //step over all the points in the center trajectory
    for (int i=0; i < tbmCentroidTrajStateVectorList.size(); i++)
    {
      //get center posECEF
      StateVector stateVector = (StateVector)tbmCentroidTrajStateVectorList.get(i);
      Vector3d centerPosECEF = new Vector3d(stateVector.posECEF);

      //get center velECEF and normalize it
      Vector3d normalizedCenterVelECEF = new Vector3d(stateVector.velECEF);
      normalizedCenterVelECEF.normalize();

      //compute rotation component of transformation matrix
      Vector3d a = new Vector3d();
      Vector3d b = new Vector3d();

      a.cross(up, normalizedCenterVelECEF);
      b.cross(normalizedCenterVelECEF, a);

      //compute translation component of transformation matrix
      Vector3d negCenterPosECEF = new Vector3d(-centerPosECEF.x, -centerPosECEF.y, -centerPosECEF.z);
      double dx = negCenterPosECEF.dot(normalizedCenterVelECEF);
      double dy = negCenterPosECEF.dot(a);
      double dz = negCenterPosECEF.dot(b);

      //set transformation matrix and inverted transformation matrix
      Vector3d temp = new Vector3d(normalizedCenterVelECEF);

      Matrix4d transformMatrix = new Matrix4d(temp.x, temp.y, temp.z, dx,
                                               a.x,    a.y,    a.z,   dy,
                                               b.x,    b.y,    b.z,   dz,
                                                0,      0,      0,    1);

      //set the transform matrix to a transform3D to use the java3d api to invert it
      Transform3D tempTransform3D = new Transform3D(transformMatrix);
      tempTransform3D.invert();

      //get the inverted transform matrix from tempTransform3D
      Matrix4d invertedTransformMatrix = new Matrix4d();
      tempTransform3D.get(invertedTransformMatrix);

      //declare point2dList
      ArrayList point2dList = new ArrayList();

      //create a profile point for each of the tbm trajectories
      for (int j=0; j < tbmTrajStateVectorArrayList.size(); j++)
      {
        //compute the point where the other trajectories intersect the plane normal to
        //the velocity vector of centroid point with the plane's 'center' at the point
        ArrayList tbmTrajStateVectorList = (ArrayList)tbmTrajStateVectorArrayList.get(j);

        Vector3d intersectionPosECEF = computeLocationTBMTrajIntersectsPlane(tbmTrajStateVectorList,
                                                                             centerPosECEF,
                                                                             normalizedCenterVelECEF);

        //set point3d as intersection point and convert it to planar coordinates
        Point3d point3d = new Point3d(intersectionPosECEF.x, intersectionPosECEF.y, intersectionPosECEF.z);
        Point2d point2d = multiplyMatrixTimesPoint(transformMatrix, point3d);

        //add point2d to the list
        point2dList.add(point2d);
      }

      //compute convex hull of data
      ArrayList point2dResultList = ConvexHull.computeConvexHull(point2dList);

      if (point2dResultList.isEmpty() == false)
      {
        //linearly interpolate new points around convex hull data in constant, equal angle format
        int initialSize = point3dList.size();
        ArrayList interpolated2dPointsOnConvexHull = interpolate2dPointsOnConvexHullData(point2dResultList);

        if (interpolated2dPointsOnConvexHull.isEmpty() == false)
        {
          //create a final point3d for each of the interpolated profile convex hull points
          for (int k=0; k < interpolated2dPointsOnConvexHull.size(); k++)
          {
            //get the interpolated 2d point and convert it back to 3d
            Point2d point2d = (Point2d)interpolated2dPointsOnConvexHull.get(k);
            Point3d point3d = multiplyMatrixTimesPoint(invertedTransformMatrix, point2d);

            //convert point3d to final coordinate system
            Vector3d posECEF = new Vector3d(point3d.x, point3d.y, point3d.z);
            Vector3d pos3dWorld = CoordinateConversions.convertECEFPositionTo3dWorld(posECEF);

            //put point3d on list
            Point3d point3dFinal = new Point3d(pos3dWorld.x, pos3dWorld.y, pos3dWorld.z);
            point3dList.add(point3dFinal);

            //repeat first point to assist in generating connectivity matrix
            if (k == interpolated2dPointsOnConvexHull.size()-1)
            {
              Point3d extraPoint3d = new Point3d((Point3d)point3dList.get(initialSize));
              point3dList.add(extraPoint3d);
            }
          }
        }
        else
        {
          return false;
        }
      }
      else
      {
        return false;
      }
    }

    //set points to data format required for plotting
    numPoints = point3dList.size();
    vertex = new Point3d[numPoints];

    for (int l=0; l < vertex.length; l++)
    {
      vertex[l] = new Point3d((Point3d)point3dList.get(l));
    }

    return true;
  }

  private ArrayList interpolate2dPointsOnConvexHullData(ArrayList point2dResultList)
  {
    double alpha=0.0;
    double outsideDiff;
    double polarAngle1;
    double polarAngle2;
    double minPolarAngle = 1e10;
    double maxPolarAngle = -1e10;
    double polarAngle[] = new double[point2dResultList.size()];
    Point2d p1;
    Point2d p2;
    Point2d minPoint = new Point2d();
    Point2d maxPoint = new Point2d();
    ArrayList interpolated2dPointsOnConvexHull = new ArrayList();

    //compute centroid of all the convex hull points
    Point2d centroid = Support.computeCenter(point2dResultList);

    //pre-compute polar angles and maximum and minimum points
    for (int i=0; i < point2dResultList.size(); i++)
    {
      //compute polar angle
      Point2d tempPoint = (Point2d)point2dResultList.get(i);
      polarAngle[i] = Math.atan2(tempPoint.y - centroid.y, tempPoint.x - centroid.x);

      //set min and max polar angle data
      if (polarAngle[i] < minPolarAngle)
      {
        minPoint = tempPoint;
        minPolarAngle = polarAngle[i];
      }

      if (polarAngle[i] > maxPolarAngle)
      {
        maxPoint = tempPoint;
        maxPolarAngle = polarAngle[i];
      }
    }

    //compute distance between min and max polar angle
    outsideDiff = 2.0 * Math.PI - (maxPolarAngle - minPolarAngle);

    //linearly interpolate points at equally spaced angles
    for (int i=0; i < numDivisions-1; i++)
    {
      boolean found = false;
      double tempPolarAngle = -Math.PI + i * anglePerDivision;
      Point2d interpolated2dPoint = new Point2d();

      if (tempPolarAngle > maxPolarAngle)
      {
        alpha = (tempPolarAngle - maxPolarAngle) / outsideDiff;
        interpolated2dPoint.x = (1 - alpha) * maxPoint.x + alpha * minPoint.x;
        interpolated2dPoint.y = (1 - alpha) * maxPoint.y + alpha * minPoint.y;
        found = true;
      }
      else if (tempPolarAngle < minPolarAngle)
      {
        alpha = (minPolarAngle - tempPolarAngle) / outsideDiff;
        interpolated2dPoint.x = (1 - alpha) * minPoint.x + alpha * maxPoint.x;
        interpolated2dPoint.y = (1 - alpha) * minPoint.y + alpha * maxPoint.y;
        found = true;
      }
      else
      {
        //search for bracketing points on profile point result list
        for (int j=0; j<point2dResultList.size(); j++)
        {
          //perform wrap around search for polar angles
          if (j==point2dResultList.size()-1)
          {
            polarAngle1 = polarAngle[j];
            polarAngle2 = polarAngle[0];
          }
          else
          {
            polarAngle1 = polarAngle[j];
            polarAngle2 = polarAngle[j+1];
          }

          //make sure bi-linearly interpolating between correct points
          if (Math.abs(polarAngle1 - polarAngle2) < Math.PI)
          {
            if (polarAngle1 > polarAngle2)
            {
              if (polarAngle1 > tempPolarAngle && tempPolarAngle > polarAngle2)
              {
                alpha = (tempPolarAngle - polarAngle2) / (polarAngle1 - polarAngle2);

                if (j==point2dResultList.size()-1)
                {
                  p1 = (Point2d)point2dResultList.get(j);
                  p2 = (Point2d)point2dResultList.get(0);
                }
                else
                {
                  p1 = (Point2d)point2dResultList.get(j);
                  p2 = (Point2d)point2dResultList.get(j+1);
                }

                interpolated2dPoint.x = (1 - alpha) * p2.x + alpha * p1.x;
                interpolated2dPoint.y = (1 - alpha) * p2.y + alpha * p1.y;
                found = true;
                break;
              }
            }
            else
            {
              if (polarAngle2 > tempPolarAngle && tempPolarAngle > polarAngle1)
              {
                alpha = (tempPolarAngle - polarAngle1) / (polarAngle2 - polarAngle1);

                if (j==point2dResultList.size()-1)
                {
                  p1 = (Point2d)point2dResultList.get(j);
                  p2 = (Point2d)point2dResultList.get(0);
                }
                else
                {
                  p1 = (Point2d)point2dResultList.get(j);
                  p2 = (Point2d)point2dResultList.get(j+1);
                }

                interpolated2dPoint.x = (1 - alpha) * p1.x + alpha * p2.x;
                interpolated2dPoint.y = (1 - alpha) * p1.y + alpha * p2.y;
                found = true;
                break;
              }
            }
          }
        }
      }

      //add profile point
      if (found == true)
      {
        interpolated2dPointsOnConvexHull.add(interpolated2dPoint);
      }
      else
      {
        interpolated2dPointsOnConvexHull.clear();
        return interpolated2dPointsOnConvexHull;
      }
    }

    return interpolated2dPointsOnConvexHull;
  }

  private void computeConnectivityMatrixAndSetColor()
  {
    //set colors
    int numColors = 1;
    color = new Color3f[numColors];
    color[0] = new Color3f(getShapeModel().getColor());

    //compute size of connectivity matrix
    int numPlanes = tbmCentroidTrajStateVectorList.size();
    int numFaces = (numDivisions-1)*(numPlanes-1);
    int sizeOfConnectivity = 4*numFaces;
    connectivity = new int[sizeOfConnectivity];
    colorIndices = new int[sizeOfConnectivity];
    int trajPointCnt=0, tmp=0;

    for (int cnt=0; cnt < numFaces; cnt++)
    {
      int diff = cnt - tmp;

      if (diff == 0 || diff == (numDivisions-1))
      {
        connectivity[4*cnt] = trajPointCnt;
        connectivity[4*cnt+1] = trajPointCnt + 1;
        connectivity[4*cnt+2] = trajPointCnt + numPlanes + 1;
        connectivity[4*cnt+3] = trajPointCnt + numPlanes;

        tmp = cnt;
        trajPointCnt++;
      }
      else
      {
        connectivity[4*cnt] = connectivity[4*cnt-4] + numPlanes;
        connectivity[4*cnt+1] = connectivity[4*cnt-3] + numPlanes;
        connectivity[4*cnt+2] = connectivity[4*cnt-2] + numPlanes;
        connectivity[4*cnt+3] = connectivity[4*cnt-1] + numPlanes;
      }

      colorIndices[4*cnt] = 0;
      colorIndices[4*cnt+1] = 0;
      colorIndices[4*cnt+2] = 0;
      colorIndices[4*cnt+3] = 0;
    }
  }

  private Point2d multiplyMatrixTimesPoint(Matrix4d transformMatrix, Point3d point3d)
  {
    Point2d point2d = new Point2d();

    point2d.x = transformMatrix.m10 * point3d.x +
                transformMatrix.m11 * point3d.y +
                transformMatrix.m12 * point3d.z +
                transformMatrix.m13;

    point2d.y = transformMatrix.m20 * point3d.x +
                transformMatrix.m21 * point3d.y +
                transformMatrix.m22 * point3d.z +
                transformMatrix.m23;

    return point2d;
  }

  private Point3d multiplyMatrixTimesPoint(Matrix4d invertedTransformMatrix, Point2d point2d)
  {
    Point3d point3d = new Point3d();

    point3d.x = invertedTransformMatrix.m01 * point2d.x +
                invertedTransformMatrix.m02 * point2d.y +
                invertedTransformMatrix.m03;

    point3d.y = invertedTransformMatrix.m11 * point2d.x +
                invertedTransformMatrix.m12 * point2d.y +
                invertedTransformMatrix.m13;

    point3d.z = invertedTransformMatrix.m21 * point2d.x +
                invertedTransformMatrix.m22 * point2d.y +
                invertedTransformMatrix.m23;

    return point3d;
  }

  public double getMaxApogeeOfAllInterpolatedTrajectories()
  {
    return maxApogee;
  }

  private Vector3d computeLocationTBMTrajIntersectsPlane(ArrayList tbmTrajStateVectorList,
                                                         Vector3d centerPosECEF,
                                                         Vector3d normalizedVelECEF)
  {
    boolean found = false;
    int i, previousSignOfDotProduct = 0;
    int currentSignOfDotProduct = 0;

    //get initial sign of dot product
    Vector3d intersectionECEF = new Vector3d();
    StateVector stateVector = (StateVector)tbmTrajStateVectorList.get(0);

    Vector3d diffVector = new Vector3d();
    diffVector.x = stateVector.posECEF.x - centerPosECEF.x;
    diffVector.y = stateVector.posECEF.y - centerPosECEF.y;
    diffVector.z = stateVector.posECEF.z - centerPosECEF.z;

    double dotProduct = normalizedVelECEF.dot(diffVector);

    //set sign of previous dot product
    if (dotProduct >= 0.0)
    {
      previousSignOfDotProduct = 1;
    }
    else
    {
      previousSignOfDotProduct = -1;
    }

    //find where dot product changes sign along trajectory
    for (i=1; i < tbmTrajStateVectorList.size(); i++)
    {
      stateVector = (StateVector)tbmTrajStateVectorList.get(i);

      diffVector.x = stateVector.posECEF.x - centerPosECEF.x;
      diffVector.y = stateVector.posECEF.y - centerPosECEF.y;
      diffVector.z = stateVector.posECEF.z - centerPosECEF.z;

      dotProduct = normalizedVelECEF.dot(diffVector);

      if (dotProduct >= 0.0)
      {
        currentSignOfDotProduct = 1;
      }
      else
      {
        currentSignOfDotProduct = -1;
      }

      //break out when find points that bracket plane
      if (previousSignOfDotProduct != currentSignOfDotProduct)
      {
        found = true;
        break;
      }
    }

    //if tbm trajectory never crosses plane, compute approximate intersection
    if (found == true)
    {
      //delete trajectory points up to first point that brackets plane
      for (int j=0; j<i-1; j++)
      {
        tbmTrajStateVectorList.remove(j);
      }
    }
    else
    {
      int size = tbmTrajStateVectorList.size();
      StateVector stateVector1 = (StateVector)tbmTrajStateVectorList.get(0);
      StateVector stateVector2 = (StateVector)tbmTrajStateVectorList.get(size-1);

      double distanceSqrd1 = Math.pow(centerPosECEF.x - stateVector1.posECEF.x, 2) +
                             Math.pow(centerPosECEF.y - stateVector1.posECEF.y, 2) +
                             Math.pow(centerPosECEF.z - stateVector1.posECEF.z, 2);

      double distanceSqrd2 = Math.pow(centerPosECEF.x - stateVector2.posECEF.x, 2) +
                             Math.pow(centerPosECEF.y - stateVector2.posECEF.y, 2) +
                             Math.pow(centerPosECEF.z - stateVector2.posECEF.z, 2);

      if (distanceSqrd1 < distanceSqrd2)
      {
        stateVector.setStateVector(stateVector1);
      }
      else
      {
        stateVector.setStateVector(stateVector2);
      }

      //find intersection
      Vector3d differenceVector = new Vector3d(centerPosECEF.x - stateVector.posECEF.x,
                                               centerPosECEF.y - stateVector.posECEF.y,
                                               centerPosECEF.z - stateVector.posECEF.z);

      double t = normalizedVelECEF.dot(differenceVector)/normalizedVelECEF.dot(stateVector.velECEF);

      intersectionECEF.x = stateVector.posECEF.x + stateVector.velECEF.x * t;
      intersectionECEF.y = stateVector.posECEF.y + stateVector.velECEF.y * t;
      intersectionECEF.z = stateVector.posECEF.z + stateVector.velECEF.z * t;

      return intersectionECEF;
    }

    //step along trajectory until just pass the plane
    StateVector stateVector1 = (StateVector)tbmTrajStateVectorList.get(0);
    StateVector stateVector2 = (StateVector)tbmTrajStateVectorList.get(1);

    double x0 = stateVector1.posECEF.x;
    double x0dot = stateVector1.velECEF.x;

    double x1 = stateVector2.posECEF.x;
    double x1dot = stateVector2.velECEF.x;

    double y0 = stateVector1.posECEF.y;
    double y0dot = stateVector1.velECEF.y;

    double y1 = stateVector2.posECEF.y;
    double y1dot = stateVector2.velECEF.y;

    double z0 = stateVector1.posECEF.z;
    double z0dot = stateVector1.velECEF.z;

    double z1 = stateVector2.posECEF.z;
    double z1dot = stateVector2.velECEF.z;

    double time = deltaTimeBetweenTrajPoints/2.0;
    double timeInterval = deltaTimeBetweenTrajPoints;

    while (timeInterval > minTimeDeltaForBinarySearch)
    {
      intersectionECEF.x = Support.hermitePosInterp(time, 0.0, deltaTimeBetweenTrajPoints, x0, x1, x0dot, x1dot);
      intersectionECEF.y = Support.hermitePosInterp(time, 0.0, deltaTimeBetweenTrajPoints, y0, y1, y0dot, y1dot);
      intersectionECEF.z = Support.hermitePosInterp(time, 0.0, deltaTimeBetweenTrajPoints, z0, z1, z0dot, z1dot);

      diffVector.x = intersectionECEF.x - centerPosECEF.x;
      diffVector.y = intersectionECEF.y - centerPosECEF.y;
      diffVector.z = intersectionECEF.z - centerPosECEF.z;

      dotProduct = normalizedVelECEF.dot(diffVector);

      if (dotProduct >= 0.0)
      {
        currentSignOfDotProduct = 1;
      }
      else
      {
        currentSignOfDotProduct = -1;
      }

      //divide time interval and reset time
      timeInterval/=2.0;

      if (previousSignOfDotProduct != currentSignOfDotProduct)
      {
        time-=(timeInterval/2.0);
      }
      else
      {
        time+=(timeInterval/2.0);
      }
    }

    return intersectionECEF;
  }
}