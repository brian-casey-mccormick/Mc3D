public class KnotCD
{
  public double time;
  public double dr;
  public double up;
  public double fpa;
  public double vel;

  public KnotCD()
  {

  }

  public KnotCD(KnotCD knotCD)
  {
    time = knotCD.time;
    dr = knotCD.dr;
    up = knotCD.up;
    fpa = knotCD.fpa;
    vel = knotCD.vel;
  }

  public KnotCD(double timeIn, double drIn, double upIn, double velIn, double fpaIn)
  {
    time = timeIn;
    dr = drIn;
    up = upIn;
    vel = velIn;
    fpa = fpaIn;
  }
}