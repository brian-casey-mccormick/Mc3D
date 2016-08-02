//java core packages
import java.util.ArrayList;

public class TBMTrajCD
{
  private static int nextId = 0;
  private double loft;
  private ArrayList knotCDList;

  public TBMTrajCD(double loftIn)
  {
    loft = loftIn;
    knotCDList = new ArrayList();
  }

  public TBMTrajCD(TBMTrajCD tbmTrajCD)
  {
    loft = tbmTrajCD.loft;
    knotCDList = new ArrayList();

    for (int i=0; i<tbmTrajCD.knotCDList.size(); i++)
    {
      KnotCD knotCDTemp = new KnotCD(tbmTrajCD.getKnot(i));
      knotCDList.add(i, knotCDTemp);
    }
  }

  public double getLoft()
  {
    return loft;
  }

  public void setKnot(int index, KnotCD knotCDIn)
  {
    knotCDList.add(index, knotCDIn);
  }

  public KnotCD getKnot(int index)
  {
    KnotCD knotCDOut = new KnotCD((KnotCD)knotCDList.get(index));

    return knotCDOut;
  }

  public KnotCD getLastKnot()
  {
    int size = knotCDList.size();
    KnotCD knotCDOut = new KnotCD((KnotCD)knotCDList.get(size-1));

    return knotCDOut;
  }
}