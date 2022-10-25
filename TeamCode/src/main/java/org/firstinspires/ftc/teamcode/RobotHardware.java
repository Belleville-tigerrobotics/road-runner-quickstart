/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or.. In OnBot Java, add a new file named RobotHardware.java, drawing from this Sample; select Not an OpMode.
 * Also add a new OpMode, drawing from the Sample ConceptExternalHardwareClass.java; select TeleOp.
 *
 */

public class RobotHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public BNO055IMU imu;

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightRearDrive = null;
    private DcMotor elevator = null;
    private Servo gripper = null;
    //    private Servo   rightHand = null;
    private Servo shuttle = null;

    //now setup the sensors
    ColorSensor sensorColorLeft;
    ColorSensor sensorColorRight;
    DistanceSensor sensorDistanceLeft;
    DistanceSensor sensorDistanceRight;


    // Define speed and power limits
    public double elevatorPower = 0.8;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double MID_SERVO = 0.5;
    public static final double HAND_SPEED = 0.02;  // sets rate to move servo
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftRearDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftRearDrive");
        rightRearDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightRearDrive");
        elevator = myOpMode.hardwareMap.get(DcMotor.class, "elevator");

        //initialize the IMU

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // initialize the distance sensors
        sensorDistanceLeft = myOpMode.hardwareMap.get(DistanceSensor.class, "rangeleft");
        sensorDistanceRight = myOpMode.hardwareMap.get(DistanceSensor.class, "rangeright");

        sensorColorLeft = myOpMode.hardwareMap.get(ColorSensor.class, "rangeleft");
        sensorColorRight = myOpMode.hardwareMap.get(ColorSensor.class, "rangeright");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
//        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        //      rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);


        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setTargetPosition(-100);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(.8);


        // Define and initialize ALL installed servos.
        gripper = myOpMode.hardwareMap.get(Servo.class, "gripper");
        shuttle = myOpMode.hardwareMap.get(Servo.class, "shuttle");
        gripper.setPosition(0);
        shuttle.setPosition(MID_SERVO);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param Drive     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param Turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
//    public void driveRobot(double Drive, double Turn) {
    // Combine drive and turn for blended motion.
//        double left  = Drive + Turn;
//        double right = Drive - Turn;

    // Scale the values so neither exceed +/- 1.0
//        double max = Math.max(Math.abs(left), Math.abs(right));
//        if (max > 1.0)
//        {
//            left /= max;
//            right /= max;
//        }

    // Use existing function to drive both wheels.
//        setDrivePower(left, right);
//    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param leftWheel  Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightWheel Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     */
    public void setDrivePower(double leftFrontWheel, double rightFrontWheel, double leftRearWheel, double rightRearWheel) {
        // Output the values to the motor drives.
        leftFrontDrive.setPower(leftFrontWheel);
        rightFrontDrive.setPower(rightFrontWheel);
        leftRearDrive.setPower(leftRearWheel);
        rightRearDrive.setPower(rightRearWheel);
    }

    public void setDrivePower2(double rightFrontWheel, double leftFrontWheel, double rightRearWheel, double leftRearWheel) {
        // Output the values to the motor drives.
        leftFrontDrive.setPower(leftFrontWheel);
        rightFrontDrive.setPower(rightFrontWheel);
        leftRearDrive.setPower(leftRearWheel);
        rightRearDrive.setPower(rightRearWheel);
    }


    public int getElevatorHeight() {
        return -elevator.getCurrentPosition();
    }

    public void setElevatorPosition(int newposition) {
        int currentposition = getElevatorHeight();
        elevator.setTargetPosition(newposition);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(.4);
    }

    public void setElevatorPower(double elevatorPower) {
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setPower(elevatorPower);
    }

    public void setShuttlePower(double shuttlePower) {
        //       shuttle.setDirection();
        shuttle.setPosition(shuttlePower);
    }

    public void gripperpickup() {
        gripper.setPosition(1);
    }

    public void gripperdrop() {
        gripper.setPosition(0);
    }

    public double getGripperPosition() {
        return gripper.getPosition();
    }

    public void imustart() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public double imugetAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public double getDistanceLeft() {
        return sensorDistanceLeft.getDistance(DistanceUnit.CM);
    }

    public double getDistanceRight() {
        return sensorDistanceRight.getDistance(DistanceUnit.CM);
    }

    public double getleftcolor() {
        return sensorColorLeft.argb();
    }

    public double getrightcolor() {
        return sensorColorRight.argb();
    }

}



