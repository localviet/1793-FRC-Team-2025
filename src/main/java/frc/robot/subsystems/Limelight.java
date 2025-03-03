package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Use limelightheplers.java on the link in discord to get more useful links


public class Limelight extends SubsystemBase {

    // .getTable("limelight") is limelight name. Once the limelight indentified through IP address, give it the name and use the 
    // SAME NAME as the limelight name you give in the software

    private static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight"); 

    public double getDoubleEntry(String entry) {
        return limelight.getEntry(entry).getDouble(0);
    }

    public double[] getAtrrayEntry(String entry){

        //make sure that arrays limelight is returning isnt more than the double array value
        return limelight.getEntry(entry).getDoubleArray(new double [6]);
    }


    //returns id of apriltag

    public double getID() {
        //tid is id of limelight, gets the tid from the networktable of the limelight
        return getDoubleEntry("tid");
    }

    //returns area of apriltag

    public double getTargetArea() {
        return getDoubleEntry("ta");
    }

    //ensures apriltag is a valid target

    public boolean hasValidTargets(){
        return getDoubleEntry("tv") == 1.0;
    }

    //horizontal offset
    public double getXOffset(){
        return getDoubleEntry("tx");
    }

    //vertical offset
    public double getYOffset(){
        return getDoubleEntry("ty");
    }


    //gets bot pose relative to target at (0,0). Can change to botpose_fieldspace to get it relative to WPI coordinate system.""
    public double[] getBotPose(){
        return getAtrrayEntry("botpose_targetspace");
    }


   
}


