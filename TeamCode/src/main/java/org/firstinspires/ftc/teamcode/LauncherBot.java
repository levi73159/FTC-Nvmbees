/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must eetain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels", and two servos
 * which feed that launcher.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */

@TeleOp(name = "LauncherBot")
//@Disabled
public class LauncherBot extends OpMode {
    final double FEED_TIME_SECONDS = 0.2; //The feeder servos run this long when a shot is requested.
    final double SETUP_TIME_SECOND = 0.3;
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 0.6;
    
    final double SPIN_SPEED = 1;
    final double SPIN_TIME_SECONDS = 0.5;

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    final double LAUNCHER_TARGET_VELOCITY = 1300;
    final double LAUNCHER_MIN_VELOCITY = 1240;

    final double LAUNCHER_FAR_TARGET_VELOCITY = LAUNCHER_TARGET_VELOCITY + 300;
    final double LAUNCHER_FAR_MIN_VELOCITY = LAUNCHER_MIN_VELOCITY + 300;

    final double LAUNCHER_OFF_VELOCITY = 500;

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor intake = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private CRServo leftIntake = null;
    private CRServo rightIntake = null;
    private Servo restrictorArm = null;


    private final double FULL_EXTEND = 0.9;



    ElapsedTime feederTimer = new ElapsedTime();
    ElapsedTime spinTime = new ElapsedTime();
    ElapsedTime setupTime = new ElapsedTime();
    ElapsedTime buttonTime = new ElapsedTime();

    private final double BUTTON_TIMEOUT = 0.3;

    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
        SETUP,
    }

    private LaunchState launchState;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftPower;
    double rightPower;

    private boolean isOpen = false;
    
    double axis = 1.0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        rightFrontDrive = super.hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = super.hardwareMap.get(DcMotor.class, "leftBackDrive");
        leftFrontDrive = super.hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightBackDrive = super.hardwareMap.get(DcMotor.class, "rightBackDrive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        restrictorArm = hardwareMap.get(Servo.class, "restrictorArm");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftIntake = hardwareMap.get(CRServo.class, "leftIntake");
        rightIntake = hardwareMap.get(CRServo.class, "rightIntake");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */

        // 0.8 fully ingage motor
        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        restrictorArm.setPosition(0.0);

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        leftFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    private boolean upPress = false;
    private boolean downPress = false;
    @Override
    public void loop() {
        /*
         * Here we call a function called arcadeDrive. The arcadeDrive function takes the input from
         * the joysticks, and applies power to the left and right drive motor to move the robot
         * as requested by the driver. "arcade" refers to the control style we're using here.
         * Much like a classic arcade game, when you move the left joystick forward both motors
         * work to drive the robot forward, and when you move the right joystick left and right
         * both motors work to rotate the robot. Combinations of these inputs can be used to create
         * more complex maneuvers.
         */
        mecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        /*
         * Here we give the user control of the speed of the launcher motor without automatically
         * queuing a shot.
         */
        if (gamepad1.y) {
            launch(true, true);
        }

        if (gamepad1.b) {
            launcher.setVelocity(-LAUNCHER_MIN_VELOCITY);
        }
        
        if (gamepad1.left_bumper) {
            intake.setPower(-1.0);
            leftIntake.setPower(-1.0);
            rightIntake.setPower(-1.0);
        } else if (gamepad1.left_trigger > 0.4) {
            intake.setPower(1.0);
            leftIntake.setPower(1.0);
            rightIntake.setPower(1.0);
        }

        if (gamepad1.dpadUpWasPressed()) {
            if (leftFeeder.getPower() >= 1.0) {
                leftFeeder.setPower(0.0);
                rightFeeder.setPower(0.0);
            } else {
                leftFeeder.setPower(1.0);
                rightFeeder.setPower(1.0);
            }
        }

        if (gamepad1.yWasPressed())

        if (gamepad1.dpadDownWasPressed()) {
            if (leftFeeder.getPower() <= -1.0) {
                leftFeeder.setPower(0.0);
                rightFeeder.setPower(0.0);
            } else {
                leftFeeder.setPower(-1.0);
                rightFeeder.setPower(-1.0);
            }
        }

        if (gamepad1.dpad_left) {
            leftFeeder.setPower(0);
            rightFeeder.setPower(0);
        }

        if (gamepad1.a) {
            intake.setPower(0.0);
            leftIntake.setPower(0.0);
            rightIntake.setPower(0.0);
        }

        if (gamepad1.xWasPressed()) {
            if (launchState == LaunchState.IDLE) {
                launchState = LaunchState.SETUP;
            }
        }

        /*
         * Now we call our "Launch" function.
         */
        launch(gamepad1.rightBumperWasPressed(), false);

        /*
         * Show the state and motor powers
         */
        telemetry.addData("State", launchState);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("motorSpeed", launcher.getVelocity());
        telemetry.addData("direction", axis > 0 ? "front" : "back");
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    
    void spin() {
        spinTime.reset();
        while (true) {
            if (spinTime.seconds() > SPIN_TIME_SECONDS) {
                break;
            }
            arcadeDrive(-0.01, SPIN_SPEED);
        }
        axis = -axis;
    }

    void arcadeDrive(double forward, double rotate) {
        if (forward < 0) {
            forward /= 1.2;
        }
        leftPower = (forward * axis) + (rotate / 2);
        rightPower = (forward * axis) - (rotate / 2);

        /*
         * Send calculated power to wheels
         */
        rightBackDrive.setPower(leftPower);
        leftBackDrive.setPower(rightPower);
    }

    void mecanumDrive(double forward, double strafe, double rotate) {
        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1.0);

        double frontLeft = (-forward + strafe + rotate) / denominator;
        double frontRight = (-forward - strafe - rotate) / denominator;
        double backLeft = (-forward - strafe + rotate) / denominator;
        double backRight = (-forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(frontLeft);
        rightFrontDrive.setPower(frontRight);
        leftBackDrive.setPower(backLeft);
        rightBackDrive.setPower(backRight);
    }


    private boolean _launchFar = false;
    void launch(boolean shotRequested, boolean far) {
        double targetVelocity = _launchFar ? LAUNCHER_FAR_TARGET_VELOCITY : LAUNCHER_TARGET_VELOCITY;
        double minVelocity = _launchFar ? LAUNCHER_FAR_MIN_VELOCITY : LAUNCHER_MIN_VELOCITY;
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                    _launchFar = far;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(targetVelocity);
                if (launcher.getVelocity() > minVelocity) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftFeeder.setPower(-1);
                rightFeeder.setPower(-1);
                intake.setPower(1.0);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.SETUP;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                    intake.setPower(STOP_SPEED);
                }
                break;
            case SETUP:
                launcher.setVelocity(STOP_SPEED);
                while (launcher.getVelocity() > LAUNCHER_OFF_VELOCITY) {
                    ;
                }
                feederTimer.reset();
                intake.setPower(-0.75);
                while (true) {
                    if (feederTimer.seconds() > 0.2)
                        break;
                }
                intake.setPower(0.0);
                launchState = LaunchState.IDLE;
        }
    }
}