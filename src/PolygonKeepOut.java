import java.awt.*;
import java.awt.event.*;
import java.lang.Math;
import javax.vecmath.*;
import java.util.ArrayList;

public class PolygonKeepOut implements Constants
{
  private static boolean verticesInCWOrder;
  private static double keepOutDistance;
  private static ArrayList originalPolygonAsPoints;
  private static ArrayList convexPolygon;
  private static ArrayList convexPolygonAsPoints;
  private static ArrayList intermediateKeepOut;
  private static ArrayList finalKeepOut;

  private static void computeOffsetVertices()
  {
    intermediateKeepOut = new ArrayList();

    //loop over all pairs of vertices and compute keepout vertices for each pair
    LatLonPoint p1 = new LatLonPoint();
    LatLonPoint p2 = new LatLonPoint();

    for (int i=0; i<convexPolygon.size(); i++)
    {
      //set p1 -> current vertex, p2 -> next vertex
      if (i == convexPolygon.size()-1)
      {
        p1 = (LatLonPoint)convexPolygon.get(i);
        p2 = (LatLonPoint)convexPolygon.get(0);
      }
      else
      {
        p1 = (LatLonPoint)convexPolygon.get(i);
        p2 = (LatLonPoint)convexPolygon.get(i+1);
      }

      //compute azimuth
      double azimuth = Support.computeAzimuth(p1.lat, p1.lon, p2.lat, p2.lon);

      if (verticesInCWOrder)
      {
        azimuth-=PI_OVER_TWO;
      }
      else
      {
        azimuth+=PI_OVER_TWO;
      }

      LatLonPoint p3 = CoordinateConversions.translateLatLonPoint(p1, keepOutDistance, azimuth);
      LatLonPoint p4 = CoordinateConversions.translateLatLonPoint(p2, keepOutDistance, azimuth);

      //add points to keepout polygon
      intermediateKeepOut.add(p3);
      intermediateKeepOut.add(p4);
    }
  }

  public static void roundOutIntersections()
  {
    int cnt=0;
    int size = convexPolygon.size();
    finalKeepOut = new ArrayList();

    LatLonPoint p0 = new LatLonPoint();
    LatLonPoint p1 = new LatLonPoint();
    LatLonPoint p2 = new LatLonPoint();

    for (int i=0; i<size; i++)
    {
      //get point on the convex polygon
      p0 = (LatLonPoint)convexPolygon.get(i);

      //get offset points from the convex polygon point
      if (i==0)
      {
        p1 = (LatLonPoint)intermediateKeepOut.get(2*size-1);
        p2 = (LatLonPoint)intermediateKeepOut.get(0);
      }
      else
      {
        p1 = (LatLonPoint)intermediateKeepOut.get(2*i-1);
        p2 = (LatLonPoint)intermediateKeepOut.get(2*i);
      }

      double initAz = Support.computeAzimuth(p0.lat, p0.lon, p1.lat, p1.lon);
      double finalAz = Support.computeAzimuth(p0.lat, p0.lon, p2.lat, p2.lon);

      //round out intersections
      if (verticesInCWOrder)
      {
        //adjust azimuth for wrap around effect
        if (finalAz < initAz)
        {
          initAz-=TWO_PI;
        }

        for (double az=initAz; az<=finalAz; az+=ONE_DEGREE_IN_RADIANS)
        {
          LatLonPoint p3 = CoordinateConversions.translateLatLonPoint(p0, keepOutDistance, az);
          finalKeepOut.add(p3);
        }
      }
      else
      {
        //adjust azimuth for wrap around effect
        if (initAz < finalAz)
        {
          finalAz-=TWO_PI;
        }

        for (double az=initAz; az>=finalAz; az-=ONE_DEGREE_IN_RADIANS)
        {
          LatLonPoint p3 = CoordinateConversions.translateLatLonPoint(p0, keepOutDistance, az);
          finalKeepOut.add(p3);
        }
      }
    }
  }

  public static ArrayList computeOffsetPolygon(ArrayList polygonIn, double offset)
  {
    keepOutDistance = offset;
    originalPolygonAsPoints = new ArrayList();

    if (polygonIn.size() < 3)
    {
      return originalPolygonAsPoints;
    }

    //store original polygon as points
    for (int i=0; i<polygonIn.size(); i++)
    {
      Point2d point2d = new Point2d();
      point2d.x = ((LatLonPoint)polygonIn.get(i)).lon;
      point2d.y = ((LatLonPoint)polygonIn.get(i)).lat;
      originalPolygonAsPoints.add(point2d);
    }

    //compute convex hull of input polygon
    convexPolygonAsPoints = ConvexHull.computeConvexHull(originalPolygonAsPoints);

    //determine order of vertices
    verticesInCWOrder = Support.polygonVerticesInCWOrder(convexPolygonAsPoints);

    //store convex hull data as latLon
    convexPolygon = new ArrayList();

    for (int i=0; i<convexPolygonAsPoints.size(); i++)
    {
      LatLonPoint latLonPoint = new LatLonPoint();
      latLonPoint.lon = ((Point2d)convexPolygonAsPoints.get(i)).x;
      latLonPoint.lat = ((Point2d)convexPolygonAsPoints.get(i)).y;
      convexPolygon.add(latLonPoint);
    }

    //generate keepout polygon
    computeOffsetVertices();
    roundOutIntersections();

    return finalKeepOut;
  }
}