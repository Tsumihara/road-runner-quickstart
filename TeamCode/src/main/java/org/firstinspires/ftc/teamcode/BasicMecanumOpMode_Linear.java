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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
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
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Mecanum Linear OpMode", group="Linear OpMode")
public class BasicMecanumOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime slideCoolDown = new ElapsedTime();
    private ElapsedTime armCoolDown = new ElapsedTime();
    private ElapsedTime extenderCoolDown = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor linearSlide = null;
    private DcMotor armMotor = null;
    private Servo   claw = null;
    private Servo   bucket = null;
    private CRServo armExtender = null;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double CLAW_SPEED      =  0.02 ;  // sets rate to move servo
    public static final double BUCKET_SPEED    =  0.02 ;  // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    double clawPos = 0;
    double extendPower;
    double bucketPos;
    double armLengthPos;
    public static double ArmController(int currentPos, int destinationPos)
    {
        return 0.05 + 0.00326764 * (Math.pow(1.01442,(destinationPos - currentPos)));
    }

    @Override
    public void runOpMode() {
            // Initialize the hardware variables. Note that the strings used here must correspond
            // to the names assigned during the robot configuration step on the DS or RC devices.
            leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        int targetPosition = 0;
        int armTargetPosition = 0;
        //

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); // FORWARD directions technically not necessary
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        linearSlide.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.FORWARD);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Define and initialize ALL installed servos.
        claw = hardwareMap.get(Servo.class, "claw");
        armExtender = hardwareMap.get(CRServo.class, "armExtender");
        bucket = hardwareMap.get(Servo.class, "bucket");
        bucket.setDirection(Servo.Direction.FORWARD);
        armExtender.setDirection(CRServo.Direction.FORWARD);
        boolean armDown = false;
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = gamepad1.right_stick_y;
            double lateral = -gamepad1.right_stick_x;
            double yaw = -gamepad1.left_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;
            if (gamepad1.circle)
            {
                if (extendPower == 1)
                {
                    extendPower = -1;
                }
                else if (extendPower == -1)
                {
                    extendPower = 0;
                }
                else extendPower = 1;

            }



            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
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


            clawPos = gamepad1.right_trigger > 0 ? .33 : 0.15;
            bucketPos = gamepad1.left_trigger > 0 ? 0.2 : 0.5;

            /*if (gamepad1.dpad_left && extenderCoolDown.milliseconds() > 200)
            {
                extendPos = Math.abs(extendPos - 1)  ;
            extenderCoolDown.reset();}
            if (gamepad1.cross) {
                clawPos += .025;
            }
            */
            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            linearSlide.setTargetPosition(targetPosition);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setTargetPosition(armTargetPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (gamepad1.dpad_up && slideCoolDown.milliseconds() > 200)
                {
                    if (linearSlide.getCurrentPosition() < 50) {
                        targetPosition = 4150;
                        slideCoolDown.reset();

                    }
                    if (linearSlide.getCurrentPosition() > 50) {
                        targetPosition = 0;
                        slideCoolDown.reset();
                    }
                }
                if (gamepad1.cross) {
                    clawPos += .025;
                }
                if (gamepad1.dpad_down && armCoolDown.milliseconds() > 200)
                {
                    if (armMotor.getCurrentPosition() > -140) {
                        armDown = false;
                        armMotor.setPower(1.0);
                        armTargetPosition = -450;
                        armCoolDown.reset();
                    }
                    if (armMotor.getCurrentPosition() < -400) {
                        armDown = true;
                        armMotor.setPower(1);
                        armTargetPosition = -35;
                        armCoolDown.reset();
                    }
                }

                /*
                if ((-135 < armMotor.getCurrentPosition() && armMotor.getCurrentPosition() < -95) && armDown)
                {
                    armMotor.setPower(0.09);
                    armTargetPosition = -20;
                    armDown = false;
                } */

                if (armDown)
                {
                    armMotor.setPower(ArmController(armMotor.getCurrentPosition(), -15));
                }
                if (armMotor.getCurrentPosition() == armTargetPosition)
                {
                    armDown = false;
                }
                if (linearSlide.getCurrentPosition() <= 4200) {
                    linearSlide.setPower(1.0);
                } else {
                    linearSlide.setPower(0);
                }

                claw.setPosition(clawPos);
                bucket.setPosition(bucketPos);
                armExtender.setPower(extendPower);

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
                telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
                telemetry.addData("Targeted LinearSlide Pos:", targetPosition);
                telemetry.addData("Arm", armTargetPosition);
                telemetry.addData("Armpower", armMotor.getPower());
                telemetry.addData("Claw", "%4.2f", claw.getPosition());
                telemetry.addData("Bucket", "%4.2f", bucketPos);
                telemetry.addData("CurrentSlidePos", linearSlide.getCurrentPosition());
                telemetry.addData("slidePower", linearSlide.getPower());
                telemetry.addData("Current arm Position (ticks)", armMotor.getCurrentPosition());
                telemetry.addData("Arm CoolDown", armCoolDown.toString());
                telemetry.addData("Slide CoolDown", slideCoolDown.toString());
                telemetry.update();
            }
        }}
