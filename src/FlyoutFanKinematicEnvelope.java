//java core packages
import java.util.*;

//java extension packages
import javax.vecmath.*;
import javax.media.j3d.*;

//java3D utility packages
import com.sun.j3d.utils.geometry.*;

public class FlyoutFanKinematicEnvelope extends MyShape3D
{
  private int id;
  private static int nextId = 0;
  private double maxAlt;
  private double maxSR;
  private GeodPoint baseLocation;
  private Matrix3d rotMatrixENUToECEF;
  private ArrayList profileList;
  private int numAzimuthDivisions;
  private double deltaAz;

  public FlyoutFanKinematicEnvelope(GeodPoint baseLocationIn, ShapeModel shapeModelIn)
  {
    this(70000.0, 100000.0, baseLocationIn, shapeModelIn);
  }

  public FlyoutFanKinematicEnvelope(double maxAltIn, double maxSRIn, GeodPoint baseLocationIn, ShapeModel shapeModelIn)
  {
    super(shapeModelIn);
    id = nextId++;

    //set input data
    maxAlt = maxAltIn;
    maxSR = maxSRIn;
    baseLocation = new GeodPoint(baseLocationIn);

    //set delta azimuth and number of azimuth divisions
    deltaAz = Math.toRadians(18.0);
    numAzimuthDivisions = 20;
    int numProfilePoints = 26;

    //set profile
    double increment = maxSR/(numProfilePoints-1);
    profileList = new ArrayList();

    for (int cnt=0; cnt<numProfilePoints; cnt++)
    {
      double temp1 = increment*cnt;
      double temp2 = maxAlt*maxAlt - Math.pow((maxAlt*temp1/maxSR), 2);

      ProfilePoint profilePoint = new ProfilePoint();
      profilePoint.x = temp1;
      profilePoint.z = Math.sqrt(temp2);

      profileList.add(profilePoint);
    }

    //compute enu to ecef matrix
    double sLat = Math.sin(baseLocation.lat);
    double sLon = Math.sin(baseLocation.lon);
    double cLat = Math.cos(baseLocation.lat);
    double cLon = Math.cos(baseLocation.lon);

    rotMatrixENUToECEF = new Matrix3d(-sLon, -cLon * sLat, cLon * cLat,
                                       cLon, -sLon * sLat, sLon * cLat,
                                        0.0,      cLat,       sLat     );

    computeFlyoutFanKinematicEnvelope();
  }

  void computeFlyoutFanKinematicEnvelope()
  {
    //convert base location to ecef
    Vector3d posSiteECEF = CoordinateConversions.convertGeodPointToECEF(baseLocation);

    //allocate total number of vertices
    int numVertices = numAzimuthDivisions*profileList.size() +
                      (profileList.size()-1)*(numAzimuthDivisions+1) +
                      2*numAzimuthDivisions;

    int index=0;
    Point3d[] vertex = new Point3d[numVertices];

    //compute the vertex positions in enu coordinates
    for (int cnt1=0; cnt1 < numAzimuthDivisions; cnt1++)
    {
      double currAz = cnt1 * deltaAz;
      double c = Math.cos(currAz);
      double s = Math.sin(currAz);

      for (int cnt2=0; cnt2 < profileList.size(); cnt2++)
      {
        ProfilePoint profilePoint = (ProfilePoint)profileList.get(cnt2);

        vertex[index] = new Point3d();
        vertex[index].x = s * profilePoint.x;
        vertex[index].y = c * profilePoint.x;
        vertex[index].z = profilePoint.z;
        index++;
      }
    }

    for (int cnt3=1; cnt3 < profileList.size(); cnt3++)
    {
      ProfilePoint profilePoint = (ProfilePoint)profileList.get(cnt3);

      for (int cnt4=0; cnt4 < numAzimuthDivisions+1; cnt4++)
      {
        double currAz = cnt4 * deltaAz;
        double c = Math.cos(currAz);
        double s = Math.sin(currAz);

        vertex[index] = new Point3d();
        vertex[index].x = s * profilePoint.x;
        vertex[index].y = c * profilePoint.x;
        vertex[index].z = profilePoint.z;
        index++;
      }
    }

    for (int cnt5=0; cnt5 < numAzimuthDivisions; cnt5++)
    {
      ProfilePoint profilePoint = (ProfilePoint)profileList.get(profileList.size()-1);

      double currAz = cnt5 * deltaAz;
      double c = Math.cos(currAz);
      double s = Math.sin(currAz);

      vertex[index] = new Point3d();
      vertex[index].x = s * profilePoint.x;
      vertex[index].y = c * profilePoint.x;
      vertex[index].z = profilePoint.z;
      index++;

      vertex[index] = new Point3d();
      vertex[index].x = 0.0;
      vertex[index].y = 0.0;
      vertex[index].z = 0.0;
      index++;
    }

    //convert enu points to 3d world points
    for (int cnt6=0; cnt6 < numVertices; cnt6++)
    {
      //store point over to a temporary vector
      Vector3d posENU = new Vector3d();
      posENU.x = vertex[cnt6].x;
      posENU.y = vertex[cnt6].y;
      posENU.z = vertex[cnt6].z;

      //transform enu vector to ecef
      Vector3d posECEF = CoordinateConversions.convertENUPositionToECEF(posENU, posSiteECEF, rotMatrixENUToECEF);

      //transform ecef to 3d world
      Vector3d pos3dWorld = CoordinateConversions.convertECEFPositionTo3dWorld(posECEF);

      //store it to vertex
      vertex[cnt6].x = pos3dWorld.x;
      vertex[cnt6].y = pos3dWorld.y;
      vertex[cnt6].z = pos3dWorld.z;
    }

    //set strip length data
    int cnt7, cnt8, cnt9;
    int[] stripLength = new int[2*numAzimuthDivisions + profileList.size()-1];

    for (cnt7=0; cnt7<numAzimuthDivisions; cnt7++)
    {
      stripLength[cnt7] = profileList.size();
    }

    for (cnt8=0; cnt8<profileList.size()-1; cnt8++)
    {
      stripLength[cnt7+cnt8] = numAzimuthDivisions+1;
    }

    for (cnt9=0; cnt9<numAzimuthDivisions; cnt9++)
    {
      stripLength[cnt7+cnt8+cnt9] = 2;
    }

    //create line strip array and set coordinates for it
    LineStripArray lineStripArray = new LineStripArray(numVertices, LineArray.COORDINATES, stripLength);
    lineStripArray.setCoordinates(0, vertex);

    //set geometry
    setGeometry(lineStripArray);
  }
}