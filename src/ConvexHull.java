//java core packages
import java.util.ArrayList;

//java extension packages
import javax.vecmath.*;
import javax.swing.*;

public class ConvexHull
{
  private static ArrayList point2dList;
  private static ArrayList polarAngle;
  private static ArrayList distance;
  private static ArrayList point2dResultList;

  private static double computeCrossProduct(Vector2d vec1, Vector2d vec2)
  {
    double crossProduct = vec1.x * vec2.y - vec2.x * vec1.y;

    return crossProduct;
  }

  private static void sortData()
  {
    //set minimum y point default to first point on list
    Point2d minimumYPoint = (Point2d)point2dList.get(0);

    //determine p0, the most negative y-coordinate point that is furthest to the left
    for (int i=1; i<point2dList.size(); i++)
    {
      Point2d point2d = (Point2d)point2dList.get(i);

      if ((point2d.y < minimumYPoint.y) ||
          (point2d.y == minimumYPoint.y && point2d.x < minimumYPoint.x))
      {
        minimumYPoint = (Point2d)point2dList.get(i);
      }
    }

    //move p0 to first element location
    point2dList.remove(minimumYPoint);
    point2dList.add(0, minimumYPoint);

    //determine p1, p2, ..., plast, sorted counter-clockwise by polar angle
    double x1 = minimumYPoint.x;
    double y1 = minimumYPoint.y;

    //pre-compute polar angles and distances
    for (int i=0; i<point2dList.size(); i++)
    {
      Point2d point2d = (Point2d)point2dList.get(i);
      double x2 = point2d.x;
      double y2 = point2d.y;

      double angle = Math.atan2((y2-y1),(x2-x1));
      double dist = (x2-x1)*(x2-x1)+(y2-y1)*(y2-y1);

      polarAngle.add(i, Double.toString(angle));
      distance.add(i, Double.toString(dist));
    }

    //sort by polar angles using insertion sort
    performQuickSort(1, point2dList.size()-1);

    //get rid of any identical points
    for (int i=0; i<point2dList.size()-1; i++)
    {
      Point2d currentPoint2d = (Point2d)point2dList.get(i);

      while (currentPoint2d.equals((Point2d)point2dList.get(i+1)))
      {
        polarAngle.remove(i+1);
        distance.remove(i+1);
        point2dList.remove(i+1);

        //break out if will exceed array bounds on next check
        if (i+1 == point2dList.size())
        {
          break;
        }
      }
    }

    //get rid of inner points of those with the same polar angle
    double eps = 1e-10;
    boolean foundPrevZeroPolarAngle = false;

    for (int i=0; i<point2dList.size(); i++)
    {
      int currentMaxIndex = i;
      double angle1, currentMaxDistance;

      if (i==0)
      {
        angle1 = 0.0;
        currentMaxDistance = 0.0;
      }
      else
      {
        angle1 = (double)Double.parseDouble((String)polarAngle.get(i));
        currentMaxDistance = (double)Double.parseDouble((String)distance.get(i));
      }

      for (int j=i+1; j<point2dList.size(); j++)
      {
        double angle2 = (double)Double.parseDouble((String)polarAngle.get(j));

        if (angle2 - angle1 < eps)
        {
          double dist = (double)Double.parseDouble((String)distance.get(j));

          if (i==0 && foundPrevZeroPolarAngle == false)
          {
            currentMaxIndex = j;
            currentMaxDistance = dist;
            foundPrevZeroPolarAngle = true;
            continue;
          }

          //if new max, remove old max and reset new to current
          if (dist > currentMaxDistance)
          {
            polarAngle.remove(currentMaxIndex);
            distance.remove(currentMaxIndex);
            point2dList.remove(currentMaxIndex);
            j--;

            currentMaxIndex = j;
            currentMaxDistance = (double)Double.parseDouble((String)distance.get(j));
          }
          //else less than new max and must be removed
          else
          {
            polarAngle.remove(j);
            distance.remove(j);
            point2dList.remove(j);
            j--;
          }
        }
        else
        {
          break;
        }
      }
    }
  }

  private static void performQuickSort(int p, int r)
  {
    if (p < r)
    {
      int q = partition(p, r);
      performQuickSort(p, q-1);
      performQuickSort(q+1, r);
    }
  }

  private static int partition(int p, int r)
  {
    int i = p-1;
    double angle = (double)Double.parseDouble((String)polarAngle.get(r));

    for (int j=p; j<r; j++)
    {
      if ((double)Double.parseDouble((String)polarAngle.get(j)) <= angle)
      {
        i++;

        //exchange angle and distance
        double tempAngle = (double)Double.parseDouble((String)polarAngle.get(i));
        double tempDistance = (double)Double.parseDouble((String)distance.get(i));

        polarAngle.set(i, (String)polarAngle.get(j));
        distance.set(i, (String)distance.get(j));

        polarAngle.set(j, Double.toString(tempAngle));
        distance.set(j, Double.toString(tempDistance));

        //exhange point2d
        Point2d tempPoint2d_i = (Point2d)point2dList.get(i);
        Point2d tempPoint2d_j = (Point2d)point2dList.get(j);

        double tempX = tempPoint2d_i.x;
        double tempY = tempPoint2d_i.y;

        tempPoint2d_i.x = tempPoint2d_j.x;
        tempPoint2d_i.y = tempPoint2d_j.y;

        tempPoint2d_j.x = tempX;
        tempPoint2d_j.y = tempY;
      }
    }

    //exchange angle and distance
    double tempAngle = (double)Double.parseDouble((String)polarAngle.get(i+1));
    double tempDistance = (double)Double.parseDouble((String)distance.get(i+1));

    polarAngle.set(i+1, (String)polarAngle.get(r));
    distance.set(i+1, (String)distance.get(r));

    polarAngle.set(r, Double.toString(tempAngle));
    distance.set(r, Double.toString(tempDistance));

    //exhange point2d
    Point2d tempPoint2d_iplus1 = (Point2d)point2dList.get(i+1);
    Point2d tempPoint2d_r = (Point2d)point2dList.get(r);

    double tempX = tempPoint2d_iplus1.x;
    double tempY = tempPoint2d_iplus1.y;

    tempPoint2d_iplus1.x = tempPoint2d_r.x;
    tempPoint2d_iplus1.y = tempPoint2d_r.y;

    tempPoint2d_r.x = tempX;
    tempPoint2d_r.y = tempY;

    return i+1;
  }

  public static ArrayList computeConvexHull(ArrayList point2dListIn)
  {
    if (point2dListIn.size() < 3)
    {
      return point2dResultList;
    }

    //create new data containers
    polarAngle = new ArrayList();
    distance = new ArrayList();
    point2dResultList = new ArrayList();

    //save input data to local data
    point2dList = new ArrayList();

    for (int i=0; i<point2dListIn.size(); i++)
    {
      Point2d point2d = new Point2d((Point2d)point2dListIn.get(i));
      point2dList.add(i, point2d);
    }

    //sort the data
    sortData();

    //push p0, p1, and p2 onto point2dResultList to get started
    point2dResultList.add((Point2d)point2dList.get(0));
    point2dResultList.add((Point2d)point2dList.get(1));
    point2dResultList.add((Point2d)point2dList.get(2));

    //loop over the rest of the points
    for (int i=3; i<point2dList.size(); i++)
    {
      //determine if angle from next to top, the top, and pi make a right turn
      Point2d nextToTop = (Point2d)point2dResultList.get(point2dResultList.size()-2);
      Point2d top = (Point2d)point2dResultList.get(point2dResultList.size()-1);
      Point2d pi = (Point2d)point2dList.get(i);

      Vector2d v1 = new Vector2d(pi.x - nextToTop.x, pi.y - nextToTop.y);
      Vector2d v2 = new Vector2d(top.x - nextToTop.x, top.y - nextToTop.y);

      double crossProduct = computeCrossProduct(v1, v2);

      //if crossProduct greater than zero, points make a right turn and top point must be removed
      while (crossProduct > 0.0)
      {
        //remove top point from result list
        point2dResultList.remove(top);

        //determine cross product again for same sequence of points
        nextToTop = (Point2d)point2dResultList.get(point2dResultList.size()-2);
        top = (Point2d)point2dResultList.get(point2dResultList.size()-1);

        v1.x = pi.x - nextToTop.x;
        v1.y = pi.y - nextToTop.y;

        v2.x = top.x - nextToTop.x;
        v2.y = top.y - nextToTop.y;

        crossProduct = computeCrossProduct(v1, v2);
      }

      //put pi onto the point2dResultList
      point2dResultList.add(pi);
    }

    return point2dResultList;
  }
}