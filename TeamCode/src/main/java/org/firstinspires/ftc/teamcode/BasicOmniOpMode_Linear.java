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

/**
 * Controls:
 * Controller A:  Drive, shuttle (bumpers)
 *
 * Controller B:  Lift, , Gripper (bumpers)
 *
 *
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//Elevator moves manually with triggers (left up, right down (additive))
//can set elevator positions using A plus the dpad
    // dpad also can move to 3 positions and bottom

    //Shuttle moves using bumpers

    // gripper controlled with controller2 bumpers

@TeleOp(name="Basic: v1.1 Omni Linear OpMode", group="Linear Opmode")
//@Disabled
public class BasicOmniOpMode_Linear extends LinearOpMode {

    RobotHardware   robot       = new RobotHardware(this);

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
//These are the elevator heights.  Note, these are absolute values...the actual encoder is reversed so these are inverted when implemented
    private int currentElevatorHeight = 0;
    private int elevatorPosition0 = 250;
    private int elevatorPosition1 = 1729;
    private int elevatorPosition2 = 2890;
    private int elevatorPosition3 = 4100;
  //  private boolean manualElevatorActive = false;
    private boolean elevatorMoving = false;
    private double elevatorSpeed =0 ;
    private double currentShuttlePosition = 0.5 ;
    private int newShuttlePosition;

//    private int speedreducer = .8;

    @Override
    public void runOpMode() {

        robot.init();


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double maximumDriveSpeed = 1 ;
            double max;

            elevatorSpeed = gamepad2.left_trigger - gamepad2.right_trigger;
            //need to do some stuff here so that we don't exceen the elevator range and trip the overcurrent on the controller
            currentElevatorHeight = Math.abs(robot.getElevatorHeight());
                if ((Math.abs(elevatorSpeed)) > .2) {  //then we have some speed request happening
                    if ((currentElevatorHeight > 4200) || (currentElevatorHeight < 1))  //|| means OR
                    { //we've exceeded the range
                        elevatorMoving = false;
                        robot.setElevatorPower(0);
                    } else{ //now we do some elevator moving stuff
                        robot.setElevatorPower(elevatorSpeed);
                        elevatorMoving = true;
                    }
                } else if (elevatorMoving) { // no more elevator speed, but only if we were already moving
                    elevatorMoving = false;
                    robot.setElevatorPower(0);
                }

//some old code here...delete if the above code works as planned
            // Set Elevator power
 //           robot.setElevatorPower( (gamepad1.dpad_up ? .5 : 0) - (gamepad1.dpad_down ? .5 : 0));
 //           if (gamepad1.left_trigger +gamepad1.right_trigger > .1)   {
 //               robot.setElevatorPower(gamepad1.left_trigger - gamepad1.right_trigger);
 //               manualElevatorActive = true;
 //           } else if ( manualElevatorActive ) {
//                robot.setElevatorPower(0) ;
 //               manualElevatorActive = false;
  //          }


            newShuttlePosition = (gamepad1.left_bumper ? 1:0) - (gamepad1.right_bumper ? 2:0); //some funny math here to make it so left=1 right = 2 both=3, neither=0
            switch (newShuttlePosition) {
                case 1:
                    robot.setShuttlePower(0); //{left button}
                    break;
                case 2:
                    robot.setShuttlePower(1); //right button
                    break;
                case 3:
                    robot.setShuttlePower(.5); //both buttons
                    break;
            }

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

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels, and reduce speed as necessary due to elevator height
            robot.setDrivePower(leftFrontPower*maximumDriveSpeed,
                    rightFrontPower*maximumDriveSpeed,
                    leftBackPower*maximumDriveSpeed,
                    rightBackPower*maximumDriveSpeed);

           // Show the elapsed game time and wheel power.
            telemetry.addData("Press and hold A to set elevator heights", "height: " + currentElevatorHeight );
       //     telemetry.addData("ElevatorPosition", "Run Time: " + runtime.toString());
            telemetry.addData("Controller A bumpers for shuttle", " "  );
            telemetry.addData("ControllerB Bumpers for gripper, triggers for up down, dpad for set positions", " "  );
            telemetry.addData("Servo Position", robot.getGripperPosition());

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }}
