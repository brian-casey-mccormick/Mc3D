//java core packages
import java.util.*;

//java extension packages
import javax.vecmath.*;
import javax.media.j3d.*;

//java3D utility packages
import com.sun.j3d.utils.geometry.*;

public class Zone extends MyShape3D
{
  private int id;
  private static int nextId = 0;
  private GeodPoint[] geodVertex;

  public Zone(LatLonPoint[] initialLatLonVertexIn, double minAlt, double maxAlt, ShapeModel shapeModelIn)
  {
    super(shapeModelIn);

    if (initialLatLonVertexIn.length < 3)
    {
      return;
    }

    id = nextId++;

    //convert input polygon to a point2d array list
    ArrayList initialLatLonArrayList = new ArrayList();

    for (int i=0; i<initialLatLonVertexIn.length; i++)
    {
      LatLonPoint latLonPoint = initialLatLonVertexIn[i];
      Point2d latLonPoint2d = new Point2d(latLonPoint.lon, latLonPoint.lat);
      initialLatLonArrayList.add(latLonPoint2d);
    }

    //determine ordering of input polygon
    boolean verticesInCWOrder = Support.polygonVerticesInCWOrder(initialLatLonArrayList);

    //compute and store vertices
    int cnt = 0;
    geodVertex = new GeodPoint[2*initialLatLonVertexIn.length];

    //store goedetic vertices in CW order
    if (verticesInCWOrder)
    {
      for (int i=0; i < initialLatLonVertexIn.length; i++)
      {
        LatLonPoint latLonPoint = initialLatLonVertexIn[i];
        geodVertex[cnt++] = new GeodPoint(latLonPoint.lat, latLonPoint.lon, minAlt);
      }

      for (int i=0; i < initialLatLonVertexIn.length; i++)
      {
        LatLonPoint latLonPoint = initialLatLonVertexIn[i];
        geodVertex[cnt++] = new GeodPoint(latLonPoint.lat, latLonPoint.lon, maxAlt);
      }
    }
    else
    {
      for (int i=initialLatLonVertexIn.length-1; i >= 0; i--)
      {
        LatLonPoint latLonPoint = initialLatLonVertexIn[i];
        geodVertex[cnt++] = new GeodPoint(latLonPoint.lat, latLonPoint.lon, minAlt);
      }

      for (int i=initialLatLonVertexIn.length-1; i >= 0; i--)
      {
        LatLonPoint latLonPoint = initialLatLonVertexIn[i];
        geodVertex[cnt++] = new GeodPoint(latLonPoint.lat, latLonPoint.lon, maxAlt);
      }
    }

    createNewZone();
  }

  private void createNewZone()
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

    //allocate connectivity matrices
    int numFaces = 2 * vertex.length - 4;
    int sizeOfConnectivity = 3 * numFaces;
    int[] connectivity = new int[sizeOfConnectivity];
    int[] colorIndices = new int[sizeOfConnectivity];

    //set colors
    int numColors = 2;
    Color3f[] color = new Color3f[numColors];
    color[0] = new Color3f(1.0f, 0.0f, 0.0f);
    color[1] = new Color3f(0.0f, 1.0f, 0.0f);

    //set connectivity matrix for sides of object
    int delta1 = vertex.length/2;
    int numLoops1 = vertex.length/2;

    for (int cnt1 = 0; cnt1 < numLoops1; cnt1++)
    {
      if (cnt1==0)
      {
        connectivity[0] = 0;
        connectivity[1] = delta1;
        connectivity[2] = 1;

        connectivity[3] = 1;
        connectivity[4] = delta1;
        connectivity[5] = delta1 + 1;
      }
      else if (cnt1==numLoops1-1)
      {
        connectivity[6*cnt1] = connectivity[6*cnt1-6] + 1;
        connectivity[6*cnt1+1] = connectivity[6*cnt1-5] + 1;
        connectivity[6*cnt1+2] = 0;

        connectivity[6*cnt1+3] = 0;
        connectivity[6*cnt1+4] = connectivity[6*cnt1-2] + 1;
        connectivity[6*cnt1+5] = delta1;
      }
      else
      {
        connectivity[6*cnt1] = connectivity[6*cnt1-6] + 1;
        connectivity[6*cnt1+1] = connectivity[6*cnt1-5] + 1;
        connectivity[6*cnt1+2] = connectivity[6*cnt1-4] + 1;

        connectivity[6*cnt1+3] = connectivity[6*cnt1-3] + 1;
        connectivity[6*cnt1+4] = connectivity[6*cnt1-2] + 1;
        connectivity[6*cnt1+5] = connectivity[6*cnt1-1] + 1;
      }

      colorIndices[6*cnt1] = 0;
      colorIndices[6*cnt1+1] = 0;
      colorIndices[6*cnt1+2] = 0;
      colorIndices[6*cnt1+3] = 0;
      colorIndices[6*cnt1+4] = 0;
      colorIndices[6*cnt1+5] = 0;
    }

    //set connectivity matrix for top
    int numLoops2 = vertex.length/2 - 2;

    for (int cnt2 = numLoops1; cnt2 < numLoops1 + numLoops2; cnt2++)
    {
      if (cnt2==numLoops1)
      {
        connectivity[6*cnt2] = 0;
        connectivity[6*cnt2+1] = 1;
        connectivity[6*cnt2+2] = 2;

        connectivity[6*cnt2+3] = delta1;
        connectivity[6*cnt2+4] = delta1 + 2;
        connectivity[6*cnt2+5] = delta1 + 1;
      }
      else
      {
        connectivity[6*cnt2] = 0;
        connectivity[6*cnt2+1] = connectivity[6*cnt2-5] + 1;
        connectivity[6*cnt2+2] = connectivity[6*cnt2-4] + 1;

        connectivity[6*cnt2+3] = delta1;
        connectivity[6*cnt2+4] = connectivity[6*cnt2-2] + 1;
        connectivity[6*cnt2+5] = connectivity[6*cnt2-1] + 1;
      }

      colorIndices[6*cnt2] = 1;
      colorIndices[6*cnt2+1] = 1;
      colorIndices[6*cnt2+2] = 1;
      colorIndices[6*cnt2+3] = 1;
      colorIndices[6*cnt2+4] = 1;
      colorIndices[6*cnt2+5] = 1;
    }

    //set geometry data
    GeometryInfo gi = new GeometryInfo(GeometryInfo.TRIANGLE_ARRAY);
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