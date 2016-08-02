//java extension packages
import javax.vecmath.*;
import javax.media.j3d.*;

//java3D utility packages
import com.sun.j3d.utils.geometry.*;

public class RadarSector extends MyShape3D
{
  private int id;
  private static int nextId = 0;
  private boolean renderSideFaces;
  private int numAzimuthDivisions;
  private int numElevationDivisions;
  private double epsilon;
  private double deltaAz;
  private double deltaEl;
  private double minAz;
  private double maxAz;
  private double minEl;
  private double maxEl;
  private double minRange;
  private double maxRange;
  private GeodPoint radarLocation;
  private Matrix3d rotMatrixENUToECEF;

  public RadarSector(double minAzIn, double maxAzIn, double minElIn,
                     double maxElIn, double minRangeIn, double maxRangeIn,
                     GeodPoint radarLocationIn, ShapeModel shapeModelIn)
  {
    super(shapeModelIn);
    id = nextId++;

    //set delta's
    epsilon = Math.toRadians(0.0001);
    deltaAz = Math.toRadians(0.5);
    deltaEl = Math.toRadians(0.5);

    //reset default delta's as appropriate
    if ((maxAzIn - minAzIn) < deltaAz)
    {
      deltaAz = Math.abs(maxAzIn - minAzIn);
    }

    if ((maxElIn - minElIn) < deltaEl)
    {
      deltaEl = Math.abs(maxElIn - minElIn);
    }

    //compute number of divisions and handle possible round-off error
    numAzimuthDivisions = (int)((maxAzIn - minAzIn + epsilon)/deltaAz + 1.0);
    numElevationDivisions = (int)((maxElIn - minElIn + epsilon)/deltaEl + 1.0);

    //set flag indicating if side faces are rendered
    if ((maxAzIn - minAzIn + epsilon) < 2.0 * Math.PI)
    {
      renderSideFaces = true;
    }
    else
    {
      renderSideFaces = false;
    }

    //set data
    deltaAz = deltaAz;
    deltaEl = deltaEl;

    minAz = minAzIn;
    maxAz = maxAzIn;

    minEl = minElIn;
    maxEl = maxElIn;

    minRange = minRangeIn;
    maxRange = maxRangeIn;

    radarLocation = new GeodPoint(radarLocationIn);

    //compute enu to ecef matrix
    double sLat = Math.sin(radarLocation.lat);
    double sLon = Math.sin(radarLocation.lon);
    double cLat = Math.cos(radarLocation.lat);
    double cLon = Math.cos(radarLocation.lon);

    rotMatrixENUToECEF = new Matrix3d(-sLon, -cLon * sLat, cLon * cLat,
                                       cLon, -sLon * sLat, sLon * cLat,
                                        0.0,      cLat,       sLat     );

    createNewRadarSector();
  }

  private void createNewRadarSector()
  {
    //create profile
    ProfilePoint[] profile = new ProfilePoint[2*numElevationDivisions];

    for (int cnt = 0; cnt < numElevationDivisions; cnt++)
    {
      double elevation = minEl + cnt * deltaEl;
      double c = Math.cos(elevation);
      double s = Math.sin(elevation);

      profile[2*cnt] = new ProfilePoint();
      profile[2*cnt].x = c * minRange;
      profile[2*cnt].z = s * minRange;

      profile[2*cnt+1] = new ProfilePoint();
      profile[2*cnt+1].x = c * maxRange;
      profile[2*cnt+1].z = s * maxRange;
    }

    //allocate total number of vertices
    int numVertices = profile.length * numAzimuthDivisions;
    Point3d[] vertex = new Point3d[numVertices];

    //compute the vertex positions in enu coordinates
    for (int cnt1=0; cnt1 < numAzimuthDivisions; cnt1++)
    {
      double currAz = minAz + cnt1 * deltaAz;
      double c = Math.cos(currAz);
      double s = Math.sin(currAz);

      for (int cnt2=0; cnt2 < profile.length; cnt2++)
      {
        int index = cnt2 + profile.length * cnt1;

        vertex[index] = new Point3d();
        vertex[index].x = s * profile[cnt2].x;
        vertex[index].y = c * profile[cnt2].x;
        vertex[index].z = profile[cnt2].z;
      }
    }

    Vector3d posSiteECEF = CoordinateConversions.convertGeodPointToECEF(radarLocation);

    for (int cnt3 = 0; cnt3 < numVertices; cnt3++)
    {
      //store point over to a temporary vector
      Vector3d posENU = new Vector3d();
      posENU.x = vertex[cnt3].x;
      posENU.y = vertex[cnt3].y;
      posENU.z = vertex[cnt3].z;

      //transform enu vector to ecef
      Vector3d posECEF = CoordinateConversions.convertENUPositionToECEF(posENU, posSiteECEF, rotMatrixENUToECEF);

      //transform ecef vector to 3d world
      Vector3d pos3dWorld = CoordinateConversions.convertECEFPositionTo3dWorld(posECEF);

      //store it to vertex
      vertex[cnt3].x = pos3dWorld.x;
      vertex[cnt3].y = pos3dWorld.y;
      vertex[cnt3].z = pos3dWorld.z;
    }

    //compute number of faces
    int numFaces=0;

    if (renderSideFaces == true)
    {
      numFaces = 2*(numAzimuthDivisions - 1) * (numElevationDivisions - 1) +
                 2*(numAzimuthDivisions - 1) + 2*(numElevationDivisions - 1);
    }
    else
    {
      numFaces = 2*(numAzimuthDivisions - 1) * (numElevationDivisions - 1) +
                 2*(numAzimuthDivisions - 1);
    }

    //allocate connectivity matrices
    int sizeOfConnectivity = 4 * numFaces;
    int[] connectivity = new int[sizeOfConnectivity];
    int[] colorIndices = new int[sizeOfConnectivity];

    //set colors
    int numColors = 3;
    Color3f[] color = new Color3f[numColors];
    color[0] = new Color3f(1.0f, 0.0f, 0.0f);
    color[1] = new Color3f(0.0f, 1.0f, 0.0f);
    color[2] = new Color3f(0.0f, 0.0f, 1.0f);

    //set top/bottom quads of connectivity matrix
    int numLoops1 = numAzimuthDivisions-1;

    for (int cnt4 = 0; cnt4 < numLoops1; cnt4++)
    {
      if (cnt4==0)
      {
        connectivity[0] = 0;
        connectivity[1] = 1;
        connectivity[2] = 2 * numElevationDivisions + 1;
        connectivity[3] = 2 * numElevationDivisions;

        connectivity[4] = connectivity[3] + 2*(numElevationDivisions-1);
        connectivity[5] = connectivity[2] + 2*(numElevationDivisions-1);
        connectivity[6] = connectivity[1] + 2*(numElevationDivisions-1);
        connectivity[7] = connectivity[0] + 2*(numElevationDivisions-1);
      }
      else
      {
        connectivity[8*cnt4] = connectivity[8*cnt4-8] + 2*numElevationDivisions;
        connectivity[8*cnt4+1] = connectivity[8*cnt4-7] + 2*numElevationDivisions;
        connectivity[8*cnt4+2] = connectivity[8*cnt4-6] + 2*numElevationDivisions;
        connectivity[8*cnt4+3] = connectivity[8*cnt4-5] + 2*numElevationDivisions;

        connectivity[8*cnt4+4] = connectivity[8*cnt4-4] + 2*numElevationDivisions;
        connectivity[8*cnt4+5] = connectivity[8*cnt4-3] + 2*numElevationDivisions;
        connectivity[8*cnt4+6] = connectivity[8*cnt4-2] + 2*numElevationDivisions;
        connectivity[8*cnt4+7] = connectivity[8*cnt4-1] + 2*numElevationDivisions;
      }

      colorIndices[8*cnt4] = 0;
      colorIndices[8*cnt4+1] = 0;
      colorIndices[8*cnt4+2] = 0;
      colorIndices[8*cnt4+3] = 0;
      colorIndices[8*cnt4+4] = 0;
      colorIndices[8*cnt4+5] = 0;
      colorIndices[8*cnt4+6] = 0;
      colorIndices[8*cnt4+7] = 0;
    }

    //set front/back quads of connectivity matrix
    int azimuthCnt=0, tmp1=numLoops1;
    int numLoops2 = (numAzimuthDivisions-1) * (numElevationDivisions-1);

    for (int cnt5 = numLoops1; cnt5 < numLoops1 + numLoops2; cnt5++)
    {
      int diff = cnt5 - tmp1;

      if (diff == 0 || diff == (numElevationDivisions-1))
      {
        connectivity[8*cnt5] = azimuthCnt * profile.length;
        connectivity[8*cnt5+1] = azimuthCnt * profile.length + 2*numElevationDivisions;
        connectivity[8*cnt5+2] = azimuthCnt * profile.length + 2*numElevationDivisions + 2;
        connectivity[8*cnt5+3] = azimuthCnt * profile.length + 2;

        connectivity[8*cnt5+4] = connectivity[8*cnt5] + 1;
        connectivity[8*cnt5+5] = connectivity[8*cnt5+3] + 1;
        connectivity[8*cnt5+6] = connectivity[8*cnt5+2] + 1;
        connectivity[8*cnt5+7] = connectivity[8*cnt5+1] + 1;

        tmp1 = cnt5;
        azimuthCnt++;
      }
      else
      {
        connectivity[8*cnt5] = connectivity[8*cnt5-8] + 2;
        connectivity[8*cnt5+1] = connectivity[8*cnt5-7] + 2;
        connectivity[8*cnt5+2] = connectivity[8*cnt5-6] + 2;
        connectivity[8*cnt5+3] = connectivity[8*cnt5-5] + 2;

        connectivity[8*cnt5+4] = connectivity[8*cnt5-4] + 2;
        connectivity[8*cnt5+5] = connectivity[8*cnt5-3] + 2;
        connectivity[8*cnt5+6] = connectivity[8*cnt5-2] + 2;
        connectivity[8*cnt5+7] = connectivity[8*cnt5-1] + 2;
      }

      colorIndices[8*cnt5] = 1;
      colorIndices[8*cnt5+1] = 1;
      colorIndices[8*cnt5+2] = 1;
      colorIndices[8*cnt5+3] = 1;
      colorIndices[8*cnt5+4] = 1;
      colorIndices[8*cnt5+5] = 1;
      colorIndices[8*cnt5+6] = 1;
      colorIndices[8*cnt5+7] = 1;
    }

    if (renderSideFaces == true)
    {
      //set side quads of connectivity matrix
      int numLoops3 = numElevationDivisions-1;

      for (int cnt6 = numLoops1 + numLoops2; cnt6 < numLoops1 + numLoops2 + numLoops3; cnt6++)
      {
        if (cnt6 == (numLoops1 + numLoops2))
        {
          connectivity[8*cnt6] = 0;
          connectivity[8*cnt6+1] = 2;
          connectivity[8*cnt6+2] = 3;
          connectivity[8*cnt6+3] = 1;

          connectivity[8*cnt6+4] = connectivity[8*cnt6] + 2*numElevationDivisions*(numAzimuthDivisions-1);
          connectivity[8*cnt6+5] = connectivity[8*cnt6+3] + 2*numElevationDivisions*(numAzimuthDivisions-1);
          connectivity[8*cnt6+6] = connectivity[8*cnt6+2] + 2*numElevationDivisions*(numAzimuthDivisions-1);
          connectivity[8*cnt6+7] = connectivity[8*cnt6+1] + 2*numElevationDivisions*(numAzimuthDivisions-1);
        }
        else
        {
          connectivity[8*cnt6] = connectivity[8*cnt6-8] + 2;
          connectivity[8*cnt6+1] = connectivity[8*cnt6-7] + 2;
          connectivity[8*cnt6+2] = connectivity[8*cnt6-6] + 2;
          connectivity[8*cnt6+3] = connectivity[8*cnt6-5] + 2;

          connectivity[8*cnt6+4] = connectivity[8*cnt6-4] + 2;
          connectivity[8*cnt6+5] = connectivity[8*cnt6-3] + 2;
          connectivity[8*cnt6+6] = connectivity[8*cnt6-2] + 2;
          connectivity[8*cnt6+7] = connectivity[8*cnt6-1] + 2;
        }

        colorIndices[8*cnt6] = 2;
        colorIndices[8*cnt6+1] = 2;
        colorIndices[8*cnt6+2] = 2;
        colorIndices[8*cnt6+3] = 2;
        colorIndices[8*cnt6+4] = 2;
        colorIndices[8*cnt6+5] = 2;
        colorIndices[8*cnt6+6] = 2;
        colorIndices[8*cnt6+7] = 2;
      }
    }

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
}