/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Controls:
 * Controller A:  Drive, shuttle (bumpers)
 *
 * Controller B:  Lift, , Gripper (bumpers)
 *
 *
**/


@TeleOp(name="14670GameDriveV1", group="Linear Opmode")
//@Disabled
public class GameDriveV1 extends LinearOpMode {

    RobotHardware   robot       = new RobotHardware(this);

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
//These are the elevator heights.  Note, these are absolute values...the actual encoder is reversed so these are inverted when implemented
    private int currentElevatorHeight = 0;
    private int elevatorPosition0 = 350;
    private int elevatorPosition1 = 1729;
    private int elevatorPosition2 = 2890;
    private int elevatorPosition3 = 4100;
  //  private boolean manualElevatorActive = false;
    private boolean elevatorMoving = false;
    private double elevatorSpeed =0 ;
    private double currentShuttlePosition = 0.5 ;
    private int newShuttlePosition;

//variables for field-centric drive
    double driveTurn;
    //double driveVertical;
    //double driveHorizontal;

    double gamepadXCoordinate;
    double gamepadYCoordinate;
    double gamepadHypot;
    double gamepadDegree;
    double robotDegree;
    double movementDegree;
    double gamepadXControl;
    double gamepadYControl;
    double speedMultiplier=.6;

//rgb values for color sensor
    double colorvalues[] = {0F, 0F, 0F};



    @Override
    public void runOpMode() {

        robot.init();


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();//start the clock

        robot.imustart();//setup the IMU



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double maximumDriveSpeed = 1 ;
            double max;

            elevatorSpeed = gamepad2.left_trigger - gamepad2.right_trigger;
            //need to do some stuff here so that we don't exceen the elevator range and trip the overcurrent on the controller
            currentElevatorHeight = Math.abs(robot.getElevatorHeight());
                if ((Math.abs(elevatorSpeed)) > .2) {  //then we have some speed request happening
                    if ((currentElevatorHeight > 4200 && elevatorSpeed <0) || (currentElevatorHeight < 100 && elevatorSpeed >0))  //|| means OR
                    { //we've exceeded the range
                        elevatorMoving = false;
                        robot.setElevatorPower(0);
                    } else{ //now we do some elevator moving stuff
                        if (currentElevatorHeight <400) { // slow it down below 400
                            robot.setElevatorPower(elevatorSpeed* 0.6);
                        } else {
                        robot.setElevatorPower(elevatorSpeed);}
                        elevatorMoving = true;
                    }
                } else if (elevatorMoving) { // no more elevator speed, but only if we were already moving
                    elevatorMoving = false;
                    robot.setElevatorPower(0);
                }


            if (gamepad1.left_bumper && gamepad1.right_bumper) {robot.setShuttlePower(.5);} else {
                if (gamepad1.left_bumper) {
                    robot.setShuttlePower(0);
                }
                if (gamepad1.right_bumper) {
                    robot.setShuttlePower(1);
                }
            }

            //Special pickup function--press y when elevator is lower than leve

            if ( currentElevatorHeight <550 && gamepad2.y ){
                telemetry.addData("Beginning Pickup Sequence","");
                telemetry.update();
                robot.setDrivePower(0,0,0,0); //stop drive motors so we don't run away
                robot.gripperdrop(); //open the gripper
                robot.setShuttlePower(1);  // slide slightly forward
                sleep(1500); //brief pause to let the gripper and shuttle get to where they need to be
                //now lower the elevator to 50
                robot.setElevatorPosition(-90);
                telemetry.addData("Pickup Sequence:Moving Elevator","");
                telemetry.update();
//                while (robot.getElevatorHeight() <-85 && gamepad2.y )  { }  //do nothing until the elevator gets there or we let go of y
                sleep (700); //wait for it to get there
                //now pick up the cone

                    robot.gripperpickup();
                    sleep (800);
                    robot.setElevatorPosition(-elevatorPosition0);

            }




//            newShuttlePosition = (gamepad1.left_bumper ? 1:0) - (gamepad1.right_bumper ? 2:0); //some funny math here to make it so left=1 right = 2 both=3, neither=0
//            switch (newShuttlePosition) {
//                case 1:
//                    robot.setShuttlePower(0); //{left button}
//                    break;
//                case 2:
//                    robot.setShuttlePower(1); //right button
//                    break;
//                case 3:
//                    robot.setShuttlePower(.5); //both buttons
//                    break;
//            }

            if ( gamepad2.left_bumper) { robot.gripperpickup() ; };
            if ( gamepad2.right_bumper) { robot.gripperdrop() ; };

//Now it's time to see if we need to set the elevators
            if (gamepad2.a) {
                    if (gamepad2.dpad_left) { elevatorPosition1 = Math.abs(robot.getElevatorHeight()); }  else {
                        if (gamepad2.dpad_up) { elevatorPosition2 = Math.abs(robot.getElevatorHeight()); }  else {
                            if (gamepad2.dpad_right) { elevatorPosition3 = Math.abs(robot.getElevatorHeight()); } else {
                                if (gamepad2.dpad_down) { elevatorPosition0 = Math.abs(robot.getElevatorHeight()); } }}}

            } else { //if not gamepad1.a pressed

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
            }
// Reduce the maximum drive speed if the elevator is up high
//            currentElevatorHeight = Math.abs(robot.getElevatorHeight()); //removing this..we just set it above a few lines back should be still good
            if (currentElevatorHeight < elevatorPosition1 ) { maximumDriveSpeed = .8 ; } else {
                if (currentElevatorHeight < elevatorPosition2) { maximumDriveSpeed = .6; } else {
                    if (currentElevatorHeight < elevatorPosition3) {maximumDriveSpeed = .2; } else {
                        maximumDriveSpeed = .3; }
                }
            }
/*
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.//         double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
//            double axial   = gamepad1.right_stick_x;
            double axial    = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
 //           double yaw      = -gamepad1.left_stick_y;
            double yaw      = gamepad1.right_stick_x;
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.


            // Send calculated power to wheels, and reduce speed as necessary due to elevator height
            robot.setDrivePower(leftFrontPower*maximumDriveSpeed,
                    rightFrontPower*maximumDriveSpeed,
                    leftBackPower*maximumDriveSpeed,
                    rightBackPower*maximumDriveSpeed);
*/


            //field centric drive code goes here


            driveTurn = -gamepad1.right_stick_x;
            //driveVertical = -gamepad1.right_stick_y;
            //driveHorizontal = gamepad1.right_stick_x;

            gamepadXCoordinate = gamepad1.left_stick_x; //this simply gives our x value relative to the driver
            gamepadYCoordinate = -gamepad1.left_stick_y; //this simply gives our y vaue relative to the driver
            gamepadHypot = Range.clip(Math.hypot(gamepadXCoordinate, gamepadYCoordinate), 0, 1);
            //finds just how much power to give the robot based on how much x and y given by gamepad
            //range.clip helps us keep our power within positive 1
            // also helps set maximum possible value of 1/sqrt(2) for x and y controls if at a 45 degree angle (which yields greatest possible value for y+x)
            gamepadDegree = Math.atan2(gamepadYCoordinate, gamepadXCoordinate) *57;
            //the inverse tangent of opposite/adjacent gives us our gamepad degree
            robotDegree = robot.imugetAngle();
            movementDegree = gamepadDegree - robotDegree;

            //adjust the angle we need to move at by finding needed movement degree based on gamepad and robot angles
            gamepadXControl = Math.cos(Math.toRadians(movementDegree)) * gamepadHypot;
            //by finding the adjacent side, we can get our needed x value to power our motors
            gamepadYControl = Math.sin(Math.toRadians(movementDegree)) * gamepadHypot;
            //by finding the opposite side, we can get our needed y value to power our motors

            /**
             * again, make sure you've changed the motor names and variables to fit your team
             */

            //by mulitplying the gamepadYControl and gamepadXControl by their respective absolute values, we can guarantee that our motor powers will not exceed 1 without any driveTurn
            //since we've maxed out our hypot at 1, the greatest possible value of x+y is (1/sqrt(2)) + (1/sqrt(2)) = sqrt(2)
            //since (1/sqrt(2))^2 = 1/2 = .5, we know that we will not exceed a power of 1 (with no turn), giving us more precision for our driving
            robot.setDrivePower2(
                ((gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) + driveTurn)*speedMultiplier),
                ((gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) - driveTurn)*speedMultiplier),
                    ((gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) + driveTurn)*speedMultiplier),
                ((gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) - driveTurn)*speedMultiplier)
            );
//note, needed to switch middle two?




            // Show the elapsed game time and wheel power.
 //           telemetry.addData("Press and hold A to set elevator heights", "height: " + currentElevatorHeight );
       //     telemetry.addData("ElevatorPosition", "Run Time: " + runtime.toString());
            telemetry.addData("Controller A bumpers for shuttle", " "  );
            telemetry.addData("ControllerB Bumpers for gripper, triggers for up down, dpad for set positions", " "  );
            telemetry.addData("Servo Position", robot.getGripperPosition());

            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
//            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Movement degree"  , movementDegree);
            telemetry.addData("gamepad degree"  , gamepadDegree);
            telemetry.addData("robot degree"  , robotDegree);
            telemetry.addData("x/y", "%4.2f, %4.2f", gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData("left Distance", robot.getDistanceLeft());
            telemetry.addData("right Distance", robot.getDistanceRight());
            telemetry.addData("left Color", robot.getleftcolor());
            telemetry.addData("right Color", robot.getrightcolor());


            telemetry.update();
        }
    }}
