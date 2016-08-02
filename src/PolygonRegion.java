public class PolygonRegion extends LineRegion
{
  public PolygonRegion(LatLonPoint[] initialLatLonVertexIn, ShapeModel shapeModelIn)
  {
    super(shapeModelIn);

    if (initialLatLonVertexIn.length < 3)
    {
      return;
    }

    //create new polygon region
    double alt = 10.0;
    geodVertexList = Support.computePerimeterGridForPolygonVertices(alt, initialLatLonVertexIn);

    //call base class to compute new line region
    createNewLineRegion();
  }
}