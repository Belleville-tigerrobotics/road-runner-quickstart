package org.firstinspires.ftc.teamcode;


/*
    This is a class that can hold variables that can be persistent between opmodes.  An example would be to store
    the heading from auton and then use it in the drive.
    Another example might be to store the elevator height this way so that we always know what it is and can track it
    properly.
    We could store system states here as well.
 */
public class HoldingVariables {
    //lastHeading, accessed by get and setHeading. Starts at 0.
    private static int lastHeading = 0;

    //Setter method
    public static void setHeading(int aHeading){
        lastHeading=aHeading;
    }

    //Getter method
    public static int getHeading(){
        return lastHeading;
    }
}
