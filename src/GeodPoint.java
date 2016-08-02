public class GeodPoint
{
  public double lat;
  public double lon;
  public double alt;

  public GeodPoint()
  {

  }

  public GeodPoint(double latIn, double lonIn, double altIn)
  {
    lat = latIn;
    lon = lonIn;
    alt = altIn;
  }

  public GeodPoint(GeodPoint geodPoint)
  {
    lat = geodPoint.lat;
    lon = geodPoint.lon;
    alt = geodPoint.alt;
  }
}