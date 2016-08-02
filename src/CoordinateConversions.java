//java extension packages
import javax.vecmath.*;

public class CoordinateConversions implements Constants
{
  private static double thetaFrom3DWorldXAxisToPrimeMeridian = Math.toRadians(-90.0);
  private static double sin = Math.sin(thetaFrom3DWorldXAxisToPrimeMeridian);
  private static double cos = Math.cos(thetaFrom3DWorldXAxisToPrimeMeridian);

  private static final Matrix3d rotMatrixECEFTo3dWorld = new Matrix3d(cos, -sin, 0.0,
                                                                      0.0,  0.0, 1.0,
                                                                     -sin, -cos, 0.0);

  private static final Matrix3d rotMatrix3dWorldToECEF = new Matrix3d( cos, 0.0, -sin,
                                                                      -sin, 0.0, -cos,
                                                                       0.0, 1.0,  0.0);

  public static Vector3d convertGeodPointToECEF(GeodPoint geodPoint)
  {
    Vector3d posECEF = new Vector3d();

    double tempX = (EARTH_MEAN_RADIUS + geodPoint.alt) * Math.cos(geodPoint.lat);
    double tempZ = (EARTH_MEAN_RADIUS + geodPoint.alt) * Math.sin(geodPoint.lat);

    posECEF.x = tempX * Math.cos(geodPoint.lon);
    posECEF.y = tempX * Math.sin(geodPoint.lon);
    posECEF.z = tempZ;

    return posECEF;
  }

  public static GeodPoint convertECEFPositionToGeodPoint(Vector3d posECEF)
  {
    GeodPoint geodPoint = new GeodPoint();

    double radiusProjectedInXY2 = Math.pow(posECEF.x, 2) + Math.pow(posECEF.y, 2);
    double radiusProjectedInXY = Math.sqrt(radiusProjectedInXY2);
    double radius = Math.sqrt(radiusProjectedInXY2 + Math.pow(posECEF.z, 2));

    geodPoint.lat = Math.atan(posECEF.z / radiusProjectedInXY);
    geodPoint.lon = Math.atan2(posECEF.y, posECEF.x);
    geodPoint.alt = radius - EARTH_MEAN_RADIUS;

    return geodPoint;
  }

  public static double computeAltitude(Vector3d posECEF)
  {
    double radiusProjectedInXY2 = Math.pow(posECEF.x, 2) + Math.pow(posECEF.y, 2);
    double radius = Math.sqrt(radiusProjectedInXY2 + Math.pow(posECEF.z, 2));
    double alt = radius - EARTH_MEAN_RADIUS;

    return alt;
  }

  public static Vector3d convertECEFPositionTo3dWorld(Vector3d posECEF)
  {
    Vector3d pos3dWorld = new Vector3d();

    pos3dWorld.x = rotMatrixECEFTo3dWorld.m00 * posECEF.x +
                   rotMatrixECEFTo3dWorld.m01 * posECEF.y +
                   rotMatrixECEFTo3dWorld.m02 * posECEF.z;

    pos3dWorld.y = rotMatrixECEFTo3dWorld.m10 * posECEF.x +
                   rotMatrixECEFTo3dWorld.m11 * posECEF.y +
                   rotMatrixECEFTo3dWorld.m12 * posECEF.z;

    pos3dWorld.z = rotMatrixECEFTo3dWorld.m20 * posECEF.x +
                   rotMatrixECEFTo3dWorld.m21 * posECEF.y +
                   rotMatrixECEFTo3dWorld.m22 * posECEF.z;

    return pos3dWorld;
  }

  public static Vector3d convert3dWorldVectorToECEF(Vector3d pos3dWorld)
  {
    Vector3d posECEF = new Vector3d();

    posECEF.x = rotMatrix3dWorldToECEF.m00 * pos3dWorld.x +
                rotMatrix3dWorldToECEF.m01 * pos3dWorld.y +
                rotMatrix3dWorldToECEF.m02 * pos3dWorld.z;

    posECEF.y = rotMatrix3dWorldToECEF.m10 * pos3dWorld.x +
                rotMatrix3dWorldToECEF.m11 * pos3dWorld.y +
                rotMatrix3dWorldToECEF.m12 * pos3dWorld.z;

    posECEF.z = rotMatrix3dWorldToECEF.m20 * pos3dWorld.x +
                rotMatrix3dWorldToECEF.m21 * pos3dWorld.y +
                rotMatrix3dWorldToECEF.m22 * pos3dWorld.z;

    return posECEF;
  }

  public static Vector3d convertENUPositionToECEF(Vector3d posENU, Vector3d sitePosECEF, Matrix3d rotMatrixENUToECEF)
  {
    Vector3d posECEF = new Vector3d();

    posECEF.x = rotMatrixENUToECEF.m00 * posENU.x +
                rotMatrixENUToECEF.m01 * posENU.y +
                rotMatrixENUToECEF.m02 * posENU.z +
                sitePosECEF.x;

    posECEF.y = rotMatrixENUToECEF.m10 * posENU.x +
                rotMatrixENUToECEF.m11 * posENU.y +
                rotMatrixENUToECEF.m12 * posENU.z +
                sitePosECEF.y;

    posECEF.z = rotMatrixENUToECEF.m20 * posENU.x +
                rotMatrixENUToECEF.m21 * posENU.y +
                rotMatrixENUToECEF.m22 * posENU.z +
                sitePosECEF.z;

    return posECEF;
  }

  public static Vector3d convertENUVelocityToECEF(Vector3d velENU, Matrix3d rotMatrixENUToECEF)
  {
    Vector3d velECEF = new Vector3d();

    velECEF.x = rotMatrixENUToECEF.m00 * velENU.x +
                rotMatrixENUToECEF.m01 * velENU.y +
                rotMatrixENUToECEF.m02 * velENU.z;

    velECEF.y = rotMatrixENUToECEF.m10 * velENU.x +
                rotMatrixENUToECEF.m11 * velENU.y +
                rotMatrixENUToECEF.m12 * velENU.z;

    velECEF.z = rotMatrixENUToECEF.m20 * velENU.x +
                rotMatrixENUToECEF.m21 * velENU.y +
                rotMatrixENUToECEF.m22 * velENU.z;

    return velECEF;
  }

  public static Vector3d convertFPositionToENU(Vector3d posF, Matrix3d rotMatrixFToENU)
  {
    Vector3d posENU = new Vector3d();

    posENU.x = rotMatrixFToENU.m00 * posF.x +
               rotMatrixFToENU.m01 * posF.y +
               rotMatrixFToENU.m02 * posF.z;

    posENU.y = rotMatrixFToENU.m10 * posF.x +
               rotMatrixFToENU.m11 * posF.y +
               rotMatrixFToENU.m12 * posF.z;

    posENU.z = rotMatrixFToENU.m20 * posF.x +
               rotMatrixFToENU.m21 * posF.y +
               rotMatrixFToENU.m22 * posF.z;

    return posENU;
  }

  public static Vector3d convertFVelocityToENU(Vector3d velF, Matrix3d rotMatrixFToENU)
  {
    Vector3d velENU = new Vector3d();

    velENU.x = rotMatrixFToENU.m00 * velF.x +
               rotMatrixFToENU.m01 * velF.y +
               rotMatrixFToENU.m02 * velF.z;

    velENU.y = rotMatrixFToENU.m10 * velF.x +
               rotMatrixFToENU.m11 * velF.y +
               rotMatrixFToENU.m12 * velF.z;

    velENU.z = rotMatrixFToENU.m20 * velF.x +
               rotMatrixFToENU.m21 * velF.y +
               rotMatrixFToENU.m22 * velF.z;

    return velENU;
  }

  public static GeodPoint convertCDPositionToGeodPoint(double dr, double up, double azimuth, GeodPoint launchPoint)
  {
    //convert knotCD to altitude
    GeodPoint geodPoint = new GeodPoint();

    //calculate the earth central angles for down range
    double alpha = dr / EARTH_MEAN_RADIUS;
    double sinAlpha = Math.sin(alpha);
    double cosAlpha = Math.cos(alpha);

    //compute sine and cosine of launch point
    double sinLatLP = Math.sin(launchPoint.lat);
    double cosLatLP = Math.cos(launchPoint.lat);

    //compute latitude, longitude, and set altitude to up
    double sinLat = sinLatLP * cosAlpha + cosLatLP * sinAlpha * Math.cos(azimuth);

    geodPoint.lat = Math.asin(sinLat);
    geodPoint.lon = launchPoint.lon + Math.atan2(cosLatLP * sinAlpha * Math.sin(azimuth), (cosAlpha - sinLat * sinLatLP));
    geodPoint.alt = up;

    return geodPoint;
  }

  public static Vector3d convertCDVelocityToENU(double velocity, double fpa, double azimuth)
  {
    Vector3d velENU = new Vector3d();

    velENU.x = velocity * Math.cos(fpa) * Math.sin(azimuth);
    velENU.y = velocity * Math.cos(fpa) * Math.cos(azimuth);
    velENU.z = velocity * Math.sin(fpa);

    return velENU;
  }

  public static Vector3d rotateVectorAboutXAxis(Vector3d vectorIn, double theta)
  {
    Vector3d vectorOut = new Vector3d();
    double c = Math.cos(theta);
    double s = Math.sin(theta);

    vectorOut.x = vectorIn.x;
    vectorOut.y = c * vectorIn.y - s * vectorIn.z;
    vectorOut.z = s * vectorIn.y + c * vectorIn.z;

    return vectorOut;
  }

  public static Vector3d rotateVectorAboutYAxis(Vector3d vectorIn, double theta)
  {
    Vector3d vectorOut = new Vector3d();
    double c = Math.cos(theta);
    double s = Math.sin(theta);

    vectorOut.x = c * vectorIn.x + s * vectorIn.z;
    vectorOut.y = vectorIn.y;
    vectorOut.z = -s * vectorIn.x + c * vectorIn.z;

    return vectorOut;
  }

  public static Vector3d rotateVectorAboutZAxis(Vector3d vectorIn, double theta)
  {
    Vector3d vectorOut = new Vector3d();
    double c = Math.cos(theta);
    double s = Math.sin(theta);

    vectorOut.x = c * vectorIn.x - s * vectorIn.y;
    vectorOut.y = s * vectorIn.x - c * vectorIn.y;
    vectorOut.z = vectorIn.z;

    return vectorOut;
  }

  public static Point3d rotatePointAboutArbitraryAxis(Vector3d u, Point3d eye, double theta)
  {
    double c = Math.cos(theta);
    double s = Math.sin(theta);

    Matrix3d rotation = new Matrix3d();
    rotation.m00 = c + (1 - c) * u.x * u.x;
    rotation.m10 = (1 - c) * u.x * u.y + s * u.z;
    rotation.m20 = (1 - c) * u.x * u.z - s * u.y;
    rotation.m01 = (1 - c) * u.y * u.x - s * u.z;
    rotation.m11 = c + (1 - c) * u.y * u.y;
    rotation.m21 = (1 - c) * u.y * u.z + s * u.x;
    rotation.m02 = (1 - c) * u.z * u.x + s * u.y;
    rotation.m12 = (1 - c) * u.z * u.y - s * u.x;
    rotation.m22 = c + (1 - c) * u.z * u.z;

    Point3d newEye = new Point3d();
    newEye.x = rotation.m00*eye.x + rotation.m01*eye.y + rotation.m02*eye.z;
    newEye.y = rotation.m10*eye.x + rotation.m11*eye.y + rotation.m12*eye.z;
    newEye.z = rotation.m20*eye.x + rotation.m21*eye.y + rotation.m22*eye.z;

    return newEye;
  }

  public static LatLonPoint translateLatLonPoint(LatLonPoint startPoint, double dr, double azimuth)
  {
    //convert knotCD to altitude
    LatLonPoint stopPoint = new LatLonPoint();

    //calculate the earth central angles for down range
    double alpha = dr / EARTH_MEAN_RADIUS;
    double sinAlpha = Math.sin(alpha);
    double cosAlpha = Math.cos(alpha);

    //compute sine and cosine of launch point
    double sinLatLP = Math.sin(startPoint.lat);
    double cosLatLP = Math.cos(startPoint.lat);

    //compute sine and cosine of azimuth
    double sinAzimuth = Math.sin(azimuth);
    double cosAzimuth = Math.cos(azimuth);

    //compute latitude, longitude, and set altitude to up
    double sinLat = sinLatLP * cosAlpha + cosLatLP * sinAlpha * cosAzimuth;
    double temp1 = cosLatLP * sinAlpha * sinAzimuth;
    double temp2 = cosAlpha - sinLat * sinLatLP;

    stopPoint.lat = Math.asin(sinLat);
    stopPoint.lon = startPoint.lon + Math.atan2(temp1, temp2);

    return stopPoint;
  }
}