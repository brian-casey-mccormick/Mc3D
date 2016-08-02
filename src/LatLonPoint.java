public class LatLonPoint
{
  public double lat;
  public double lon;

  public LatLonPoint()
  {

  }

  public LatLonPoint(double latIn, double lonIn)
  {
    lat = latIn;
    lon = lonIn;
  }

  public LatLonPoint(LatLonPoint latLonPoint)
  {
    lat = latLonPoint.lat;
    lon = latLonPoint.lon;
  }
}