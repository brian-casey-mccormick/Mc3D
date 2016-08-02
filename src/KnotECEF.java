public class KnotECEF
{
  public double time;
  public double x;
  public double y;
  public double z;
  public double xdot;
  public double ydot;
  public double zdot;

  public KnotECEF()
  {

  }

  public KnotECEF(double timeIn, double xIn, double yIn, double zIn,
                  double xdotIn, double ydotIn, double zdotIn)
  {
    time = timeIn;
    x = xIn;
    y = yIn;
    z = zIn;
    xdot = xdotIn;
    ydot = ydotIn;
    zdot = zdotIn;
  }
}