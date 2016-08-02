//java core packages
import java.awt.*;
import java.awt.event.*;
import java.text.DecimalFormat;

//java extension packages
import javax.swing.*;
import javax.swing.border.*;
import javax.swing.event.*;

public class ControlPanel extends JPanel
{
  private Java3DWorld java3DWorld;
  private JSlider slider;
  private JButton yawRightButton;
  private JButton yawLeftButton;
  private JButton pitchUpButton;
  private JButton pitchDownButton;
  private JButton rollRightButton;
  private JButton rollLeftButton;
  private JTextField latLonTracker;
  private DecimalFormat decimalFormat;

  public ControlPanel(Java3DWorld java3DWorldIn)
  {
    //store java3DWorld and create decimal format
    java3DWorld = java3DWorldIn;
    decimalFormat = new DecimalFormat("0.0");

    //create panels and add them to the control panel
    JPanel panel1 = new JPanel();
    panel1.setLayout(new GridLayout(2, 1));
    add(panel1);

    JPanel panel2 = new JPanel();
    panel2.setLayout(new GridLayout(7, 1));
    add(panel2);

    JPanel panel3 = new JPanel();
    panel3.setLayout(new FlowLayout(FlowLayout.LEFT, 20, 5));
    add(panel3);

    JPanel panel4 = new JPanel();
    panel4.setLayout(new GridLayout(2, 1));
    add(panel4);

    //create reset button and add it to panel1
    JButton resetButton = new JButton("Reset");
    panel1.add(resetButton);

    resetButton.addActionListener(new ActionListener()
    {
      public void actionPerformed(ActionEvent event)
      {
        java3DWorld.resetCameraToInitialPosition();
      }
    }
    );

    //create camera control buttons and add them to panel2
    CustomMouseAdapter mouseAdapter = new CustomMouseAdapter();

    yawRightButton = new JButton("Yaw Right");
    yawRightButton.addMouseListener(mouseAdapter);
    panel2.add(yawRightButton);

    yawLeftButton = new JButton("Yaw Left");
    yawLeftButton.addMouseListener(mouseAdapter);
    panel2.add(yawLeftButton);

    pitchUpButton = new JButton("Pitch Up");
    pitchUpButton.addMouseListener(mouseAdapter);
    panel2.add(pitchUpButton);

    pitchDownButton = new JButton("Pitch Down");
    pitchDownButton.addMouseListener(mouseAdapter);
    panel2.add(pitchDownButton);

    rollRightButton = new JButton("Roll Right");
    rollRightButton.addMouseListener(mouseAdapter);
    panel2.add(rollRightButton);

    rollLeftButton = new JButton("Roll Left");
    rollLeftButton.addMouseListener(mouseAdapter);
    panel2.add(rollLeftButton);

    //create rotation factor slider and add it to panel3
    TitledBorder titledBorder = new TitledBorder("Rotation Factor");
    titledBorder.setTitleJustification(TitledBorder.CENTER);
    panel3.setBorder(titledBorder);

    slider = new JSlider(SwingConstants.VERTICAL, 1, 10, 1);
    panel3.add(slider);

    ChangeListener slideListener = new ChangeListener()
    {
      public void stateChanged(ChangeEvent event)
      {
        java3DWorld.resetCameraRotationAngles(slider.getValue());
      }
    };

    slider.addChangeListener(slideListener);
    slider.setMajorTickSpacing(1);
    slider.setPaintTicks(true);
    slider.setSnapToTicks(true);
    slider.setPaintLabels(true);

    //add (lat,lon) tracker to panel4
    latLonTracker = new JTextField("      NaN, NaN", 10);
    panel4.add(latLonTracker);
  }

  public void resetLatLonTracker()
  {
    latLonTracker.setText("      NaN , NaN");
  }

  public void updateLatLonTracker(double lat, double lon)
  {
    latLonTracker.setText("      " + decimalFormat.format(Math.toDegrees(lat)) + ", " + decimalFormat.format(Math.toDegrees(lon)));
  }

  //return preferred dimensions of container
  public Dimension getPreferredSize()
  {
    return new Dimension(100, 550);
  }

  //return minimum size of container
  public Dimension getMinimumSize()
  {
    return getPreferredSize();
  }

  //create thread classes
  private class YawRightThread extends Thread
  {
    private boolean holdFlag;

    YawRightThread()
    {
      holdFlag = true;
    }

    public void turnOffHoldFlag()
    {
      holdFlag = false;
    }

    public void run()
    {
      while (holdFlag == true)
      {
        java3DWorld.yawRight();
        Support.delay(100);
      }
    }
  }

  private class YawLeftThread extends Thread
  {
    private boolean holdFlag;

    YawLeftThread()
    {
      holdFlag = true;
    }

    public void turnOffHoldFlag()
    {
      holdFlag = false;
    }

    public void run()
    {
      while (holdFlag == true)
      {
        java3DWorld.yawLeft();
        Support.delay(100);
      }
    }
  }

  private class PitchUpThread extends Thread
  {
    private boolean holdFlag;

    PitchUpThread()
    {
      holdFlag = true;
    }

    public void turnOffHoldFlag()
    {
      holdFlag = false;
    }

    public void run()
    {
      while (holdFlag == true)
      {
        java3DWorld.pitchUp();
        Support.delay(100);
      }
    }
  }

  private class PitchDownThread extends Thread
  {
    private boolean holdFlag;

    PitchDownThread()
    {
      holdFlag = true;
    }

    public void turnOffHoldFlag()
    {
      holdFlag = false;
    }

    public void run()
    {
      while (holdFlag == true)
      {
        java3DWorld.pitchDown();
        Support.delay(100);
      }
    }
  }

  private class RollRightThread extends Thread
  {
    private boolean holdFlag;

    RollRightThread()
    {
      holdFlag = true;
    }

    public void turnOffHoldFlag()
    {
      holdFlag = false;
    }

    public void run()
    {
      while (holdFlag == true)
      {
        java3DWorld.rollRight();
        Support.delay(100);
      }
    }
  }

  private class RollLeftThread extends Thread
  {
    private boolean holdFlag;

    RollLeftThread()
    {
      holdFlag = true;
    }

    public void turnOffHoldFlag()
    {
      holdFlag = false;
    }

    public void run()
    {
      while (holdFlag == true)
      {
        java3DWorld.rollLeft();
        Support.delay(100);
      }
    }
  }

  //create mouse adapter class
  private class CustomMouseAdapter extends MouseAdapter
  {
    private YawRightThread yawRightThread;
    private YawLeftThread yawLeftThread;
    private PitchUpThread pitchUpThread;
    private PitchDownThread pitchDownThread;
    private RollRightThread rollRightThread;
    private RollLeftThread rollLeftThread;

    public void mousePressed(MouseEvent e)
    {
      if (e.getSource() == yawRightButton)
      {
        yawRightThread = new YawRightThread();
        yawRightThread.start();
      }
      else if (e.getSource() == yawLeftButton)
      {
        yawLeftThread = new YawLeftThread();
        yawLeftThread.start();
      }
      else if (e.getSource() == pitchUpButton)
      {
        pitchUpThread = new PitchUpThread();
        pitchUpThread.start();
      }
      else if (e.getSource() == pitchDownButton)
      {
        pitchDownThread = new PitchDownThread();
        pitchDownThread.start();
      }
      else if (e.getSource() == rollRightButton)
      {
        rollRightThread = new RollRightThread();
        rollRightThread.start();
      }
      else if (e.getSource() == rollLeftButton)
      {
        rollLeftThread = new RollLeftThread();
        rollLeftThread.start();
      }
    }

    public void mouseReleased(MouseEvent e)
    {
      if (e.getSource() == yawRightButton)
      {
        yawRightThread.turnOffHoldFlag();
      }
      else if (e.getSource() == yawLeftButton)
      {
        yawLeftThread.turnOffHoldFlag();
      }
      else if (e.getSource() == pitchUpButton)
      {
        pitchUpThread.turnOffHoldFlag();
      }
      else if (e.getSource() == pitchDownButton)
      {
        pitchDownThread.turnOffHoldFlag();
      }
      else if (e.getSource() == rollRightButton)
      {
        rollRightThread.turnOffHoldFlag();
      }
      else if (e.getSource() == rollLeftButton)
      {
        rollLeftThread.turnOffHoldFlag();
      }
    }
  }
}