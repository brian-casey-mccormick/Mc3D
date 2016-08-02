//java core packages
import java.util.*;

//java extension packages
import javax.vecmath.*;
import javax.media.j3d.*;

//java3D utility packages
import com.sun.j3d.utils.geometry.*;

public class AirCorridor extends MyShape3D
{
  private int id;
  private static int nextId = 0;
  private GeodPoint[] geodVertex;

  public AirCorridor(LatLonPoint[] centerIn, double minAlt, double maxAlt, double width, ShapeModel shapeModelIn)
  {
    super(shapeModelIn);

    if (centerIn.length < 3)
    {
      return;
    }

    id = nextId++;

    //compute and store vertices
    geodVertex = new GeodPoint[4*centerIn.length];

    //transform data to series of geodetic points
    computeGeodVertexData(centerIn, minAlt, maxAlt, width);

    //create air corridor
    createNewAirCorridor();
  }

  private void computeGeodVertexData(LatLonPoint[] centerIn, double minAlt, double maxAlt, double width)
  {
    int cnt=0;
    int size=centerIn.length;
    double range = width/2.0;
    ArrayList latLonPointList = new ArrayList();

    //compute the offset points for first center point
    double az = Support.computeAzimuth(centerIn[0].lat, centerIn[0].lon, centerIn[1].lat, centerIn[1].lon);

    LatLonPoint latLonPoint1 = CoordinateConversions.translateLatLonPoint(centerIn[0], range, az + PI_OVER_TWO);
    latLonPointList.add(latLonPoint1);

    LatLonPoint latLonPoint2 = CoordinateConversions.translateLatLonPoint(centerIn[0], range, az - PI_OVER_TWO);
    latLonPointList.add(latLonPoint2);

    //compute offset points for interior points
    for (int i=0; i<size-2; i++)
    {
      double az1 = Support.computeAzimuth(centerIn[i].lat, centerIn[i].lon, centerIn[i+1].lat, centerIn[i+1].lon);
      double az2 = Support.computeAzimuth(centerIn[i+1].lat, centerIn[i+1].lon, centerIn[i+2].lat, centerIn[i+2].lon);
      double averageAz = (az1+az2)/2.0;

      LatLonPoint latLonPoint3 = CoordinateConversions.translateLatLonPoint(centerIn[i+1], range, averageAz + PI_OVER_TWO);
      latLonPointList.add(latLonPoint3);

      LatLonPoint latLonPoint4 = CoordinateConversions.translateLatLonPoint(centerIn[i+1], range, averageAz - PI_OVER_TWO);
      latLonPointList.add(latLonPoint4);
    }

    //compute offset points for last center point
    az = Support.computeAzimuth(centerIn[size-2].lat, centerIn[size-2].lon, centerIn[size-1].lat, centerIn[size-1].lon);

    LatLonPoint latLonPoint5 = CoordinateConversions.translateLatLonPoint(centerIn[size-1], range, az + PI_OVER_TWO);
    latLonPointList.add(latLonPoint5);

    LatLonPoint latLonPoint6 = CoordinateConversions.translateLatLonPoint(centerIn[size-1], range, az - PI_OVER_TWO);
    latLonPointList.add(latLonPoint6);

    //store points in required format
    for (int i=0; i<latLonPointList.size()/2; i++)
    {
      LatLonPoint latLonPoint7 = (LatLonPoint)latLonPointList.get(2*i);
      geodVertex[cnt++] = new GeodPoint(latLonPoint7.lat, latLonPoint7.lon, minAlt);
    }

    for (int i=0; i<latLonPointList.size()/2; i++)
    {
      LatLonPoint latLonPoint8 = (LatLonPoint)latLonPointList.get(2*i+1);
      geodVertex[cnt++] = new GeodPoint(latLonPoint8.lat, latLonPoint8.lon, minAlt);
    }

    for (int i=0; i<latLonPointList.size()/2; i++)
    {
      LatLonPoint latLonPoint9 = (LatLonPoint)latLonPointList.get(2*i);
      geodVertex[cnt++] = new GeodPoint(latLonPoint9.lat, latLonPoint9.lon, maxAlt);
    }

    for (int i=0; i<latLonPointList.size()/2; i++)
    {
      LatLonPoint latLonPoint10 = (LatLonPoint)latLonPointList.get(2*i+1);
      geodVertex[cnt++] = new GeodPoint(latLonPoint10.lat, latLonPoint10.lon, maxAlt);
    }
  }

  private void createNewAirCorridor()
  {
    Point3d vertex[] = new Point3d[geodVertex.length];

    for (int cnt=0; cnt < geodVertex.length; cnt++)
    {
      //transform geodetic points to ecef
      Vector3d posECEF = CoordinateConversions.convertGeodPointToECEF(geodVertex[cnt]);

      //transform ecef to the 3d world
      Vector3d pos3dWorld = CoordinateConversions.convertECEFPositionTo3dWorld(posECEF);

      //store to vertex point
      vertex[cnt] = new Point3d(pos3dWorld.x, pos3dWorld.y, pos3dWorld.z);
    }

    //allocate connectivity and color index matrices
    int numFaces = vertex.length - 2;
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
    int delta1 = vertex.length/4;
    int delta2 = vertex.length/2;
    int numLoops1 = vertex.length/4 - 1;

    for (int cnt1 = 0; cnt1 < numLoops1; cnt1++)
    {
      if (cnt1==0)
      {
        connectivity[0] = 1;
        connectivity[1] = 0;
        connectivity[2] = delta1;
        connectivity[3] = delta1 + 1;

        connectivity[4] = connectivity[3] + delta2;
        connectivity[5] = connectivity[2] + delta2;
        connectivity[6] = connectivity[1] + delta2;
        connectivity[7] = connectivity[0] + delta2;
      }
      else
      {
        connectivity[8*cnt1] = connectivity[8*cnt1-8] + 1;
        connectivity[8*cnt1+1] = connectivity[8*cnt1-7] + 1;
        connectivity[8*cnt1+2] = connectivity[8*cnt1-6] + 1;
        connectivity[8*cnt1+3] = connectivity[8*cnt1-5] + 1;

        connectivity[8*cnt1+4] = connectivity[8*cnt1-4] + 1;
        connectivity[8*cnt1+5] = connectivity[8*cnt1-3] + 1;
        connectivity[8*cnt1+6] = connectivity[8*cnt1-2] + 1;
        connectivity[8*cnt1+7] = connectivity[8*cnt1-1] + 1;
      }

      int index = cnt1%numColors;
      colorIndices[8*cnt1] = index;
      colorIndices[8*cnt1+1] = index;
      colorIndices[8*cnt1+2] = index;
      colorIndices[8*cnt1+3] = index;
      colorIndices[8*cnt1+4] = index;
      colorIndices[8*cnt1+5] = index;
      colorIndices[8*cnt1+6] = index;
      colorIndices[8*cnt1+7] = index;
    }

    //set front/back quads of connectivity matrix
    int numLoops2 = vertex.length/4 - 1;

    for (int cnt2 = numLoops1; cnt2 < numLoops1 + numLoops2; cnt2++)
    {
      if (cnt2 == numLoops1)
      {
        connectivity[8*cnt2] = delta2 + 1;
        connectivity[8*cnt2+1] = delta2;
        connectivity[8*cnt2+2] = 0;
        connectivity[8*cnt2+3] = 1;

        connectivity[8*cnt2+4] = connectivity[8*cnt2] + delta1;
        connectivity[8*cnt2+5] = connectivity[8*cnt2+3] + delta1;
        connectivity[8*cnt2+6] = connectivity[8*cnt2+2] + delta1;
        connectivity[8*cnt2+7] = connectivity[8*cnt2+1] + delta1;
      }
      else
      {
        connectivity[8*cnt2] = connectivity[8*cnt2-8] + 1;
        connectivity[8*cnt2+1] = connectivity[8*cnt2-7] + 1;
        connectivity[8*cnt2+2] = connectivity[8*cnt2-6] + 1;
        connectivity[8*cnt2+3] = connectivity[8*cnt2-5] + 1;

        connectivity[8*cnt2+4] = connectivity[8*cnt2-4] + 1;
        connectivity[8*cnt2+5] = connectivity[8*cnt2-3] + 1;
        connectivity[8*cnt2+6] = connectivity[8*cnt2-2] + 1;
        connectivity[8*cnt2+7] = connectivity[8*cnt2-1] + 1;
      }

      int index = cnt2%numColors;
      colorIndices[8*cnt2] = index;
      colorIndices[8*cnt2+1] = index;
      colorIndices[8*cnt2+2] = index;
      colorIndices[8*cnt2+3] = index;
      colorIndices[8*cnt2+4] = index;
      colorIndices[8*cnt2+5] = index;
      colorIndices[8*cnt2+6] = index;
      colorIndices[8*cnt2+7] = index;
    }

    //set side quads of connectivity matrix
    int totalLoops = numLoops1 + numLoops2;

    connectivity[8*totalLoops] = delta2;
    connectivity[8*totalLoops+1] = delta1 + delta2;
    connectivity[8*totalLoops+2] = delta1;
    connectivity[8*totalLoops+3] = 0;

    connectivity[8*totalLoops+4] = connectivity[8*totalLoops] + delta1 - 1;
    connectivity[8*totalLoops+5] = connectivity[8*totalLoops+3] + delta1 - 1;
    connectivity[8*totalLoops+6] = connectivity[8*totalLoops+2] + delta1 - 1;
    connectivity[8*totalLoops+7] = connectivity[8*totalLoops+1] + delta1 - 1;

    int index = totalLoops%numColors;
    colorIndices[8*totalLoops] = index;
    colorIndices[8*totalLoops+1] = index;
    colorIndices[8*totalLoops+2] = index;
    colorIndices[8*totalLoops+3] = index;
    colorIndices[8*totalLoops+4] = index;
    colorIndices[8*totalLoops+5] = index;
    colorIndices[8*totalLoops+6] = index;
    colorIndices[8*totalLoops+7] = index;

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