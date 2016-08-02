//java core packages
import java.util.*;
import java.io.*;

//java extension packages
import javax.swing.*;

public class FlyoutFan
{
  private int id;
  private static int nextId = 0;
  private double minRange;
  private double maxRange;
  private ArrayList tbmTrajCDList;
  private int numKnots;
  private int numTrajs;

  public FlyoutFan(String fileName)
  {
    id = nextId++;

    //create trajCDList and set default min and max range
    tbmTrajCDList = new ArrayList();
    minRange = 1e20;
    maxRange = -1e20;

    //read flyout fan
    readFlyoutFan(fileName);
  }

  public FlyoutFan(FlyoutFan flyoutFan)
  {
    id = nextId++;
    minRange = flyoutFan.minRange;
    maxRange = flyoutFan.maxRange;

    tbmTrajCDList = new ArrayList();

    numKnots = flyoutFan.numKnots;
    numTrajs = flyoutFan.numTrajs;

    for (int cnt=0; cnt<numTrajs; cnt++)
    {
      TBMTrajCD tbmTrajCDTemp = flyoutFan.getTBMTrajCD(cnt);
      TBMTrajCD tbmTrajCD = new TBMTrajCD(tbmTrajCDTemp);
      tbmTrajCDList.add(cnt, tbmTrajCD);
    }
  }

  public int getNumKnots()
  {
    return numKnots;
  }

  public int getNumTrajs()
  {
    return numTrajs;
  }

  public TBMTrajCD getTBMTrajCD(int index)
  {
    return (TBMTrajCD)tbmTrajCDList.get(index);
  }

  private void readFlyoutFan(String fileName)
  {
    //open file for reading
    try
    {
      FileReader fileReader = new FileReader(fileName);
      BufferedReader reader = new BufferedReader(fileReader);
      String string;

      //read flyout fan id
      string = reader.readLine();
      StringTokenizer stringTokenizer1 = new StringTokenizer(string);
      stringTokenizer1.nextToken(); //skip the word ID
      stringTokenizer1.nextToken(); //skip the equals sign
      id = Integer.parseInt(stringTokenizer1.nextToken());

      //read number of trajectories
      string = reader.readLine();
      StringTokenizer stringTokenizer2 = new StringTokenizer(string);
      stringTokenizer2.nextToken(); //skip the word numTrajs
      stringTokenizer2.nextToken(); //skip the equals sign
      numTrajs = Integer.parseInt(stringTokenizer2.nextToken());

      //read number of knots
      string = reader.readLine();
      StringTokenizer stringTokenizer3 = new StringTokenizer(string);
      stringTokenizer3.nextToken(); //skip the word numKnots
      stringTokenizer3.nextToken(); //skip the equals sign
      numKnots = Integer.parseInt(stringTokenizer3.nextToken());

      //loop over the trajectories
      for (int cnt1=0; cnt1<numTrajs; cnt1++)
      {
        //read the current trajectory's loft angle and create trajCD
        string = reader.readLine();
        string = reader.readLine();
        StringTokenizer stringTokenizer4 = new StringTokenizer(string);
        stringTokenizer4.nextToken(); //skip the word loft
        stringTokenizer4.nextToken(); //skip the equals sign
        double loftIn = Double.parseDouble(stringTokenizer4.nextToken());
        double loft = Math.toRadians(loftIn);
        TBMTrajCD tbmTrajCD = new TBMTrajCD(loft);

        //loop over all the knots in each trajectory
        for (int cnt2=0; cnt2<numKnots; cnt2++)
        {
          //read the knot data
          string = reader.readLine();
          StringTokenizer stringTokenizer5 = new StringTokenizer(string);
          double time = Double.parseDouble(stringTokenizer5.nextToken());

          double drIn = Double.parseDouble(stringTokenizer5.nextToken());
          double dr = 1000.0 * drIn;

          double upIn = Double.parseDouble(stringTokenizer5.nextToken());
          double up = 1000.0 * upIn;

          double velIn = Double.parseDouble(stringTokenizer5.nextToken());
          double vel = 1000.0 * velIn;

          double fpaIn = Double.parseDouble(stringTokenizer5.nextToken());
          double fpa = Math.toRadians(fpaIn);

          //create new knot and store it in the tbmTrajCD
          KnotCD knotCD = new KnotCD(time, dr, up, vel, fpa);
          tbmTrajCD.setKnot(cnt2, knotCD);

          //keep running count of the minimum and maximum range
          minRange = Math.min(minRange, dr);
          maxRange = Math.max(maxRange, dr);
        }

        //add the trajCD to the trajCD list
        tbmTrajCDList.add(cnt1, tbmTrajCD);
      }

      //close file
      fileReader.close();
    }
    catch (IOException ioException)
    {
      JOptionPane.showMessageDialog(null, "FILE ERROR", "FILE ERROR", JOptionPane.ERROR_MESSAGE);
    }
  }
}