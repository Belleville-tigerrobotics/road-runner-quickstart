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
//    private DcMotor leftFrontDrive = null;
//    private DcMotor leftBackDrive = null;
//    private DcMotor rightFrontDrive = null;
//    private DcMotor rightBackDrive = null;
    private int currentElevatorHeight = 0;
    private int elevatorPosition0 = 0;
    private int elevatorPosition1 = -100;
    private int elevatorPosition2 = -800;
    private int elevatorPosition3 = -1500;
    private boolean manualElevatorActive = false;

    @Override
    public void runOpMode() {

        robot.init();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
//        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
//        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);//       leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double maximumDriveSpeed = 1 ;
            double max;

            // Set Elevator power
 //           robot.setElevatorPower( (gamepad1.dpad_up ? .5 : 0) - (gamepad1.dpad_down ? .5 : 0));
            if (gamepad1.left_trigger +gamepad1.right_trigger > .1)   {
                robot.setElevatorPower(gamepad1.left_trigger - gamepad1.right_trigger);
                manualElevatorActive = true;
            } else if ( manualElevatorActive ) {
                robot.setElevatorPower(0) ;
                manualElevatorActive = false;
            }
            robot.setShuttlePower( (gamepad1.left_bumper ? 1:0) - (gamepad1.right_bumper ? 1:0));

            if ( gamepad1.x) { robot.gripperpickup() ; };
            if ( gamepad1.y) { robot.gripperdrop() ; };

//Now it's time to see if we need to set the elevators
            if (gamepad1.a) {
                    if (gamepad1.dpad_left) { elevatorPosition1 = robot.getElevatorHeight(); }  else {
                        if (gamepad1.dpad_up) { elevatorPosition2 = robot.getElevatorHeight(); }  else {
                            if (gamepad1.dpad_right) { elevatorPosition3 = robot.getElevatorHeight(); } else {
                                if (gamepad1.dpad_down) { elevatorPosition0 = robot.getElevatorHeight(); } }}}

            }

            if (gamepad1.dpad_left) { robot.setElevatorPosition (elevatorPosition1); }  else {
                if (gamepad1.dpad_up) { robot.setElevatorPosition (elevatorPosition2);}  else {
                    if (gamepad1.dpad_right) { robot.setElevatorPosition (elevatorPosition3);}  else {
                        if (gamepad1.dpad_down) { robot.setElevatorPosition (elevatorPosition0); } }
                }
            }

// Reduce the maximum drive speed if the elevator is up high
            currentElevatorHeight = robot.getElevatorHeight();
            if (currentElevatorHeight < elevatorPosition1 ) { maximumDriveSpeed = 1 ; } else {
                if (currentElevatorHeight < elevatorPosition2) { maximumDriveSpeed = .75; } else {
                    if (currentElevatorHeight < elevatorPosition3) {maximumDriveSpeed = .4; } else {
                        maximumDriveSpeed = .3; }
                }
            }

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
   //         double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double axial   = gamepad1.right_stick_x;
            double lateral =  -gamepad1.left_stick_x;
   //         double yaw     =  gamepad1.right_stick_x;
            double yaw      = -gamepad1.left_stick_y;
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

            if (max > maximumDriveSpeed) {
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

            // Send calculated power to wheels
            robot.setDrivePower(leftFrontPower,rightFrontPower,leftBackPower,rightBackPower);

           // Show the elapsed game time and wheel power.
            telemetry.addData("Press and hold A to set elevator heights", "height: " + currentElevatorHeight );
       //     telemetry.addData("ElevatorPosition", "Run Time: " + runtime.toString());
            telemetry.addData("bumpers for shuttle, trigger for up/dn", " "  );
            telemetry.addData("x,y for gripper", " "  );
            telemetry.addData("Servo Position", robot.getGripperPosition());

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }}