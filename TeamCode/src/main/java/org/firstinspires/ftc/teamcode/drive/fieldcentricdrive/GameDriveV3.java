package org.firstinspires.ftc.teamcode.drive.fieldcentricdrive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.RobotAccessoriesPowerPlay;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * Borrowed from TeleOpFieldCentric and to be enhanced with 2022 robot stuff using RobotAccessriesPowerPlay
 *
 *
 *
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@TeleOp(group = "advanced")
public class GameDriveV3 extends LinearOpMode {

    // Here's our global variables
    private int currentElevatorHeight = 0;
    private int elevatorPosition0 = 350;
    private int elevatorPosition1 = 1650;//1729
    private int elevatorPosition2 = 2820;//2890
    private int elevatorPosition3 = 4070;
    private boolean elevatorMoving = false;
    private double elevatorSpeed = 0;
    private double currentShuttlePosition = 0.5;
    private int newShuttlePosition;
    private boolean stackInOkPosition = false;
    private int stackIndex = 0;
    private int theloopcounter = 0;
    double speedMultiplier = .5;
    double colorvalues[] = {0F, 0F, 0F};
    double shuttlePositionForAutoPickup = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotAccessoriesPowerPlay robot = new RobotAccessoriesPowerPlay(this);
        robot.init();
        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y *0.7,
                    -gamepad1.left_stick_x *0.7
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x *0.7
                    )
            );

            // Update everything. Odometry. Etc.
            drive.update();

            // Now here's the operational code from the drive v1 program
            elevatorSpeed = gamepad2.left_trigger - gamepad2.right_trigger;
            //need to do some stuff here so that we don't exceen the elevator range and trip the overcurrent on the controller
            currentElevatorHeight = Math.abs(robot.getElevatorHeight());
            if ((Math.abs(elevatorSpeed)) > .2) {  //then we have some speed request happening
                if ((currentElevatorHeight > 4200 && elevatorSpeed < 0) || (currentElevatorHeight < 100 && elevatorSpeed > 0))  //|| means OR
                { //we've exceeded the range
                    elevatorMoving = false;
                    robot.setElevatorPower(0);
                } else { //now we do some elevator moving stuff
                    if (currentElevatorHeight < 400) { // slow it down below 400
                        robot.setElevatorPower(elevatorSpeed * 0.6);
                    } else {
                        robot.setElevatorPower(elevatorSpeed);
                    }
                    elevatorMoving = true;
                }
            } else if (elevatorMoving) { // no more elevator speed, but only if we were already moving
                elevatorMoving = false;
                robot.setElevatorPower(0);
            }

            if (gamepad2.dpad_left) {
                robot.setElevatorPosition(-elevatorPosition1);
            } else {
                if (gamepad2.dpad_up) {
                    robot.setElevatorPosition(-elevatorPosition2);
                } else {
                    if (gamepad2.dpad_right) {
                        robot.setElevatorPosition(-elevatorPosition3);
                    } else {
                        if (gamepad2.dpad_down) {
                            robot.setElevatorPosition(-elevatorPosition0);
                        }
                    }
                }
            }

            if (gamepad2.left_bumper) {
                robot.gripperpickup();
            }
            ;
            if (gamepad2.right_bumper) {
                robot.gripperdrop();
            }




            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                robot.setShuttlePower(.5);
            } else {
                if (gamepad1.left_bumper) {
                    robot.setShuttlePower(0);
                }
                if (gamepad1.right_bumper) {
                    robot.setShuttlePower(1);
                }
            }

            //Special pickup function--press y when elevator is lower than level 2

            if (currentElevatorHeight < 1650 && gamepad2.y) {
                telemetry.addData("Beginning Pickup Sequence", "");
                telemetry.update();
//                robot.setDrivePower(0, 0, 0, 0); //stop drive motors so we don't run away
                drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

                robot.gripperdrop(); //open the gripper
                robot.setShuttlePower(shuttlePositionForAutoPickup);  // slide slightly forward   Origianally was 0.9, now extended fully to 1.0
                sleep(1500); //brief pause to let the gripper and shuttle get to where they need to be.  Might be able to shorten this sleep by a bit to make it all faster.
                //now lower the elevator to 50
                robot.setElevatorPosition(-95);
                telemetry.addData("Pickup Sequence:Moving Elevator", "");
                telemetry.update();
//                while (robot.getElevatorHeight() <-85 && gamepad2.y )  { }  //do nothing until the elevator gets there or we let go of y
                sleep(700); //wait for it to get there
                //now pick up the cone

                robot.gripperpickup();  //pickup the cone
                sleep(800);   //wait for the pickup to finish
                robot.setElevatorPosition(-elevatorPosition0);  //raise the elevator to driving position
            }

                if (gamepad2.a) { //this preps the elevator for 5 cone stack pickup
                    robot.setShuttlePower(0);
                    robot.setElevatorPosition(-850);
                }

                if (currentElevatorHeight < 1550 && gamepad2.x) {
 //                   robot.setDrivePower(0, 0, 0, 0); //stop drive motors so we don't run away
                    drive.setWeightedDrivePower(new Pose2d(0,0,0));

                            //
                    stackInOkPosition = false; //set our flag before we test
                    telemetry.addData("Aligning with tower of cones", "");
                    telemetry.update();
//                    robot.setDrivePower(0, 0, 0, 0); //stop drive motors so we don't run away
                    drive.setWeightedDrivePower(new Pose2d(0,0,0));

                    robot.gripperdrop(); //open the gripper
                    robot.setElevatorPosition(-750); //raise it up high enough to clear the top of the stack
//put some code here to read the sensors and make sure the stack is where we want it...move if not
                    //start with the forward distance, then use the sides to make sure we're centeredish
                    //if these things line up, then set the OK flag
                    if (robot.getDistanceFront() > 11 && robot.getDistanceFront() < 14.5) {
                        //       if ( robot.getDistanceLeft() >6 && robot.getDistanceLeft() <15 ) {
                        stackInOkPosition = true;
                    } else {
                        stackInOkPosition = false;
                    }
                    //  }
                    //now slowly lower the elevator until the limit switch hits or we get back to 0
                    //set the stackIndex value to which itm we're at=-or maybe just use where we're at and go from here
                    //lots of work to do here...need to htink about this
//need the cone distance between 13
                    //left-right between 9 and 15

                    telemetry.addData("Beginning Pickup Sequence", "");
                    telemetry.update();
                    if (stackInOkPosition) {
                        robot.gripperdrop(); //open the gripper
                        robot.setShuttlePower(0);
                        robot.dropToFindCone();


                        int currentElevator = robot.getElevatorHeight();
                        robot.setShuttlePower(.5);
                        robot.setElevatorPosition(-(currentElevator + 550));
                        //creep forward just a smidge
//                        robot.setDrivePower(.18, .18, .18, .18);
                        drive.setWeightedDrivePower(new Pose2d(0,0,0));

                        sleep(200);
//                        robot.setDrivePower(0, 0, 0, 0);
                        drive.setWeightedDrivePower(new Pose2d(0,0,0));

                        sleep(450);
                        robot.setShuttlePower(.91);
                        sleep(700);
                        robot.setElevatorPosition(-(currentElevator + 250)); //was 335
                        sleep(370);
                        robot.gripperpickup();
                        sleep(500);
                        robot.setElevatorPosition(-(currentElevator + 850));
                    }

                    //              robot.setShuttlePower(.9);  // slide slightly forward
                    //              sleep(1500); //brief pause to let the gripper and shuttle get to where they need to be
                    //now lower the elevator to 50
                    //              robot.setElevatorPosition(-95);
                    //       telemetry.addData("Pickup Sequence:Moving Elevator","");
                    //       telemetry.update();
//                while (robot.getElevatorHeight() <-85 && gamepad2.y )  { }  //do nothing until the elevator gets there or we let go of y
                    //         sleep (700); //wait for it to get there
                    //now pick up the cone

                    //             robot.gripperpickup();
                    //           sleep (800);
                    //           robot.setElevatorPosition(-elevatorPosition0);

                }
//need to fix controller b...a doesn't work, bumpers don't work

                if (gamepad1.b && gamepad2.b) {
                    robot.panicReset1();
                    sleep(500);
                    robot.panicReset2();
                }

            if (robot.getDistanceFront() >11 && robot.getDistanceFront() <14.5 ) {
                //       if ( robot.getDistanceLeft() >6 && robot.getDistanceLeft() <15 ) {
                stackInOkPosition = true;
            } else { stackInOkPosition = false ; }




            // Print pose to telemetry
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());

            telemetry.addData("left Distance", robot.getDistanceLeft());
            telemetry.addData("right Distance", robot.getDistanceRight());
            telemetry.addData("front Distance", robot.getDistanceFront());
            if (!stackInOkPosition) {
                telemetry.addData("NOT LINED UP FOR STACK","!!!! Front Needs to be ");
                telemetry.addData("NOT LINED UP FOR STACK","!! between 11 & 14.5");
                telemetry.addData("NOT LINED UP FOR STACK","!!!!");
            }


            telemetry.update();
            }
        }
    }
//}