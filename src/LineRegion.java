//java core packages
import java.util.*;

//java extension packages
import javax.vecmath.*;
import javax.media.j3d.*;

public class LineRegion extends MyShape3D
{
  private int id;
  private static int nextId = 0;
  protected ArrayList geodVertexList;

  protected LineRegion(ShapeModel shapeModelIn)
  {
    super(shapeModelIn);
    id = nextId++;
  }

  protected void createNewLineRegion()
  {
    //set the trajectory points from previously propagated data
    int numPoints = geodVertexList.size();
    Point3d[] trajPoints = new Point3d[numPoints];
    Color3f[] colors = new Color3f[numPoints];

    for (int i=0; i<numPoints; i++)
    {
      GeodPoint geodPoint = (GeodPoint)geodVertexList.get(i);
      Vector3d posECEF = CoordinateConversions.convertGeodPointToECEF(geodPoint);
      Vector3d pos3dWorld = CoordinateConversions.convertECEFPositionTo3dWorld(posECEF);

      trajPoints[i] = new Point3d(pos3dWorld.x, pos3dWorld.y, pos3dWorld.z);
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

  public ArrayList getFinalUniqueGeodVertexList()
  {
    ArrayList geodVertexListOut = new ArrayList();

    for (int i=0; i<geodVertexList.size()-1; i++)
    {
      GeodPoint geodVertexPointTemp = (GeodPoint)geodVertexList.get(i);
      GeodPoint geodVertexPoint = new GeodPoint(geodVertexPointTemp);
      geodVertexListOut.add(i, geodVertexPoint);
    }

    return geodVertexListOut;
  }
}