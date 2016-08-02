//java core packages
import java.util.ArrayList;

//java extension packages
import javax.vecmath.*;
import javax.media.j3d.*;

public class TBMTraj extends MyShape3D
{
  private int id;
  private static int nextId = 0;
  protected int numPoints;
  protected double initLat;
  protected double initLon;
  protected double azimuth;
  protected double timeStep;
  protected Matrix3d rotMatrixFToENU;
  protected Matrix3d rotMatrixENUToECEF;
  protected ArrayList tbmTrajPointList;
  protected ArrayList tbmTrajStateVectorList;

  protected TBMTraj(double initLatIn, double initLonIn, double timeStepIn, ShapeModel shapeModelIn)
  {
    super(shapeModelIn);
    id = nextId++;

    //set trajectory data
    initLat = initLatIn;
    initLon = initLonIn;

    //set time step
    timeStep = timeStepIn;

    //compute enu to ecef matrix
    double sLat = Math.sin(initLat);
    double sLon = Math.sin(initLon);
    double cLat = Math.cos(initLat);
    double cLon = Math.cos(initLon);

    rotMatrixENUToECEF = new Matrix3d(-sLon, -cLon * sLat, cLon * cLat,
                                       cLon, -sLon * sLat, sLon * cLat,
                                        0.0,      cLat,       sLat     );

    tbmTrajPointList = new ArrayList();
    tbmTrajStateVectorList = new ArrayList();

    //initial numPoints to zero
    numPoints = 0;
  }

  protected TBMTraj(double initLatIn, double initLonIn, double timeStepIn, double azimuthIn, ShapeModel shapeModelIn)
  {
    this(initLatIn, initLonIn, timeStepIn, shapeModelIn);

    //compute azimuth related data
    azimuth = azimuthIn;

    //compute f to enu matrix
    double cos = Math.cos(azimuth);
    double sin = Math.sin(azimuth);

    rotMatrixFToENU = new Matrix3d(0.0,  cos, sin,
                                   0.0, -sin, cos,
                                   1.0,  0.0, 0.0);
  }

  protected void createNewTBMTrajectory()
  {
    //set the trajectory points from previously data
    Point3d[] trajPoints = new Point3d[numPoints];
    Color3f[] colors = new Color3f[numPoints];

    for (int i=0; i<numPoints; i++)
    {
      trajPoints[i] = new Point3d((Point3d)tbmTrajPointList.get(i));
      colors[i] = new Color3f(getShapeModel().getColor());
    }

    //set data to have only one strip consisting of all the points
    int stripCounts[] = new int[1];
    stripCounts[0] = numPoints;

    //create trajectory line strip array
    LineStripArray lineStripArray = new LineStripArray(numPoints, LineStripArray.COORDINATES | LineStripArray.COLOR_3, stripCounts);
    lineStripArray.setCoordinates(0, trajPoints);
    lineStripArray.setColors(0, colors);

    //set geometry
    setGeometry(lineStripArray);
  }

  public double getDeltaTimeBetweenTrajPoints()
  {
    return timeStep;
  }

  public ArrayList getTBMTrajStateVectorList()
  {
    ArrayList tbmTrajStateVectorListOut = new ArrayList();

    for (int i=0; i<tbmTrajStateVectorList.size(); i++)
    {
      StateVector stateVectorTemp = (StateVector)tbmTrajStateVectorList.get(i);
      StateVector stateVector = new StateVector(stateVectorTemp);
      tbmTrajStateVectorListOut.add(i, stateVector);
    }

    return tbmTrajStateVectorListOut;
  }

  public double getApogee()
  {
    double apogee=-1e10;

    for (int i=0; i<tbmTrajStateVectorList.size(); i++)
    {
      StateVector stateVector = (StateVector)tbmTrajStateVectorList.get(i);
      double tempAltitude = CoordinateConversions.computeAltitude(stateVector.posECEF);
      apogee = Math.max(apogee, tempAltitude);
    }

    return apogee;
  }
}