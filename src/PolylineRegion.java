public class PolylineRegion extends LineRegion
{
  public PolylineRegion(LatLonPoint[] initialLatLonVertexIn, ShapeModel shapeModelIn)
  {
    super(shapeModelIn);

    if (initialLatLonVertexIn.length < 2)
    {
      return;
    }

    //create new polygon region
    double alt = 10.0;
    geodVertexList = Support.computePerimeterGridForPolylineVertices(alt, initialLatLonVertexIn);

    //call base class to compute new line region
    createNewLineRegion();
  }
}