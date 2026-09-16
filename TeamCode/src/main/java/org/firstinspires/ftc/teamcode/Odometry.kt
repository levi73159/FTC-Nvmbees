package org.firstinspires.ftc.teamcode
import com.acmerobotics.roadrunner.InstantFunction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.PI

@Autonomous(name="Odometry Auto")
class Odometry : LinearOpMode() {

    private enum class LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
        SETUP,

    }

    private enum class TeamColor { RED, BLUE }

    private var _launchState: LaunchState = LaunchState.IDLE
    private lateinit var _intake: DcMotor
    private lateinit var _launcher: DcMotorEx
    private lateinit var _leftFeeder: CRServo
    private lateinit var _rightFeeder: CRServo
    private lateinit var _leftIntake: CRServo
    private lateinit var _rightIntake: CRServo

    private var _teamcolor = TeamColor.RED

    private val LAUNCHER_TARGET_VELOCITY: Double = 1300.0+25.0
    private val LAUNCHER_MIN_VELOCITY: Double = 1240.0+25.0

    private val LAUNCHER_FAR_TARGET_VELOCITY: Double = LAUNCHER_TARGET_VELOCITY + 300 + 150
    private val LAUNCHER_FAR_MIN_VELOCITY: Double = LAUNCHER_MIN_VELOCITY + 300 + 150

    private val LAUNCHER_OFF_VELOCITY: Double = 500.0
    private val FEED_TIME_SECONDS: Double = 0.2 //The feeder servos run this long when a shot is requested.
    private val STOP_SPEED: Double = 0.0 //We send this power to the servos when we want them to stop.

    private var _feederTimer = ElapsedTime()

    private var _waitTimer = ElapsedTime()

    private fun toSided(pose2d: Pose2d): Pose2d {
        val fliped = pose2d.position.y * if (_teamcolor == TeamColor.RED) 1.0 else -1.0
        val otherHeading =  if (_teamcolor == TeamColor.RED) pose2d.heading.toDouble() else Rotation2d.fromDouble(2 * PI) - pose2d.heading
        return Pose2d(Vector2d(pose2d.position.x, fliped), otherHeading)
    }

    private fun toSidedNoRotate(pose2d: Pose2d): Pose2d {
        val fliped = pose2d.position.y * if (_teamcolor == TeamColor.RED) 1.0 else -1.0
        return Pose2d(Vector2d(pose2d.position.x, fliped), pose2d.heading)
    }

    private fun toSided(x: Double): Double {
        return if (_teamcolor == TeamColor.RED) x else x * -1.0
    }

    private fun launchAmount(i: Int, special: Boolean = false) {
        _amountLaunch = 0;
        while (_amountLaunch < i) {
            launch(true, true, special)
        }
    }

    override fun runOpMode() {

        // begin at (61.6, 18.2)
        // red tower at (-53.5, 54.5)
        // heading = 131 at tower red
        // heading = 90? at start

        // 1st ball red side: (-10, 24) head 270
        _launcher = hardwareMap.get(DcMotorEx::class.java, "launcher")
        _leftFeeder = hardwareMap.get(CRServo::class.java, "leftFeeder")
        _rightFeeder = hardwareMap.get(CRServo::class.java, "rightFeeder")
        _intake = hardwareMap.get(DcMotor::class.java, "intake")
        _leftIntake = hardwareMap.get(CRServo::class.java, "leftIntake")
        _rightIntake = hardwareMap.get(CRServo::class.java, "rightIntake")

        _launcher.mode = DcMotor.RunMode.RUN_USING_ENCODER
        _launcher.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        _leftFeeder.power = 0.0
        _rightFeeder.power = 0.0
        _launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
            PIDFCoefficients(300.0, 0.0, 0.0, 10.0)
        );
        _leftFeeder.direction = DcMotorSimple.Direction.FORWARD
        _rightFeeder.direction = DcMotorSimple.Direction.REVERSE

        _leftIntake.direction = DcMotorSimple.Direction.REVERSE
        _rightIntake.direction = DcMotorSimple.Direction.FORWARD

        var ballOrder = 0
        var multiGrab = true
        var startTower = true
        var waitSeconds = 0.0
        // create RR drive obj
        while (opModeInInit()) {
            if (isStopRequested) return
            if (super.gamepad1.xWasPressed()) {
                _teamcolor = TeamColor.BLUE
            }

            if (super.gamepad1.bWasPressed()) {
                _teamcolor = TeamColor.RED
            }

            if (super.gamepad1.yWasPressed()) {
                ballOrder++;
                if (ballOrder >= 3) {
                    ballOrder = 0;
                }
            }

            if (super.gamepad1.rightBumperWasPressed()) {
                multiGrab = !multiGrab
            }

            if (super.gamepad1.leftBumperWasPressed()) {
                startTower = !startTower
            }

            if (!startTower) {
                if (super.gamepad1.dpadUpWasPressed()) {
                    waitSeconds++
                }
                if (super.gamepad1.dpadDownWasPressed()) {
                    waitSeconds--
                }
            }

            super.telemetry.clearAll();
            super.telemetry.addLine("Ball 1: Closest to tower, Ball 2: Middle, Ball 3, farthest from tower")
            super.telemetry.addLine("---------------------------------")
            super.telemetry.addLine("Press RB to toggle be able to grab from two lines (Multi Grab)")
            super.telemetry.addLine("Press Y to change ball")
            super.telemetry.addLine("Press X for blue and B for red")
            super.telemetry.addLine("Press A to pick up other ball")
            super.telemetry.addLine("Press LB to toggle starting location")
            super.telemetry.addLine("Up dpad increase wait time, Down dpad decresse wait time")
            super.telemetry.addLine("---------------------------------")
            super.telemetry.addData("TeamColor", _teamcolor)
            super.telemetry.addData("Ball", ballOrder + 1)
            super.telemetry.addData("Multi Grab", multiGrab)
            super.telemetry.addData("Start At Tower?", startTower)
            if (!startTower) {
                super.telemetry.addData("Wait time", waitSeconds)
            }
            super.telemetry.update()
        }

        val beginPosRed = Pose2d(Vector2d(61.6, 18.2), Math.toRadians(180.0))
        val beginPosBlue = toSided(beginPosRed)

        val beginTowerRed = Pose2d(Vector2d(-53.5, 54.5), Math.toRadians(135.0))
        val beginTowerBlue = toSided(beginTowerRed)

        val redPos = Pose2d(Vector2d(-53.5+28.0, 54.5-28.0), Math.toRadians(135.0+toSided(10.0)))
        val bluePos = toSided(redPos)

        val dif = 23.1
        val heading = Math.toRadians(-270.0)
        val ball = Vector2d(-11.2, 24.0)

        val ballPos = Vector2d(ball.x + (dif * ballOrder), ball.y)
        val otherPos = Vector2d(ball.x + (dif * if (ballOrder == 2) 0 else ballOrder + 1), ball.y)

        val ball1 = toSided(Pose2d(ballPos, heading))
        val ball2 = toSided(Pose2d(otherPos, heading))

        val startPos = when (startTower) {
            true -> if (_teamcolor == TeamColor.RED) beginTowerRed else beginTowerBlue
            false -> if (_teamcolor == TeamColor.RED) beginPosRed else beginPosBlue
        }
        val drive = MecanumDrive(super.hardwareMap, startPos)

        val pos = if (_teamcolor == TeamColor.RED) redPos else bluePos
        val toTower = drive.actionBuilder(startPos)
            .splineToSplineHeading(
                pos,
                tangent = Math.toRadians(toSided(175.0))
            )
            .stopAndAdd(InstantFunction{
                pushBallIn()
                launchAmount(3, true)
            })
            .build()

        val toTowerAtTower = drive.actionBuilder(startPos)
            .strafeToSplineHeading(
                pos.position,
                pos.heading,
            )
            .stopAndAdd(InstantFunction{
                pushBallIn()
                launchAmount(3, true)
            })
            .build()

        val ballPath = drive.actionBuilder(pos)
            .strafeToSplineHeading(ball1.position, ball1.heading)
            .stopAndAdd(InstantFunction{
                setIntake(1.0)
            })
            .strafeTo(Vector2d(ball1.position.x, ball1.position.y + toSided(30.0)))
            .stopAndAdd(InstantFunction{
                setIntake(0.0)
            })
            .build()

        val otherBall = drive.actionBuilder(pos)
            .strafeToSplineHeading(ball2.position, ball1.heading)
            .stopAndAdd(InstantFunction{
                setIntake(1.0)
            })
            .strafeTo(Vector2d(ball2.position.x, ball1.position.y + toSided(30.0)))
            .stopAndAdd(InstantFunction{
                setIntake(0.0)
            })
            .build()

        val backToTower = drive.actionBuilder(Pose2d(ball1.position + Vector2d(0.0, toSided(30.0)), ball1.heading))
            .setReversed(true)
            .splineToLinearHeading(
                pos,
                PI,
            )
            .stopAndAdd(InstantFunction{
                launchAmount(1)
            })
            .build()

        val backToTower2 = drive.actionBuilder(Pose2d(ball2.position + Vector2d(0.0, toSided(30.0)), ball2.heading))
            .setReversed(true)
            .splineToLinearHeading(
                pos,
                PI,
            )
            .stopAndAdd(InstantFunction{
                launchAmount(1)
            })
            .build()

        val moveAway = drive.actionBuilder(pos).strafeTo(Vector2d(pos.position.x - 20.0, pos.position.y - toSided(3.0))).build()

        _waitTimer.reset()
        while (_waitTimer.seconds() < waitSeconds) {
            if (super.isStopRequested) return
        }

        if (multiGrab)
            runBlocking(SequentialAction(if (startTower) toTowerAtTower else toTower, ballPath, backToTower, otherBall, backToTower2, moveAway))
        else
            runBlocking(SequentialAction(if (startTower) toTowerAtTower else toTower, ballPath, backToTower, moveAway))
    }

    fun setIntake(speed: Double) {
        _intake.power = speed
        _leftIntake.power = speed
        _rightIntake.power = speed
    }

    fun pushBallIn() {
        _feederTimer.reset();
        _intake.power = -0.75;
        while (true) {
            if (_feederTimer.seconds() > 0.3)
                break;
        }
        _intake.power = 0.0;
        _feederTimer.reset();
        while (true) {
            if (_feederTimer.seconds() > 0.2) {
                break;
            }
        }
        _feederTimer.reset();
    }

    private var _launchFar = false
    private var _amountLaunch = 0
    fun launch(shotRequested: Boolean, far: Boolean = false, special: Boolean=false) {
        var targetVelocity: Double =
            if (_launchFar) LAUNCHER_FAR_TARGET_VELOCITY else LAUNCHER_TARGET_VELOCITY
        var minVelocity: Double =
            if (_launchFar) LAUNCHER_FAR_MIN_VELOCITY else LAUNCHER_MIN_VELOCITY

        if (special && _amountLaunch == 0) {
            targetVelocity += 35
            minVelocity += 35
        }
        when (_launchState) {
            LaunchState.IDLE -> if (shotRequested) {
                _launchState = LaunchState.SPIN_UP
                _launchFar = far
            }

            LaunchState.SPIN_UP -> {
                _launcher.velocity = targetVelocity
                if (_launcher.velocity > minVelocity) {
                    _launchState = LaunchState.LAUNCH
                }
            }

            LaunchState.LAUNCH -> {
                _leftFeeder.power = -1.0
                _rightFeeder.power = -1.0
                _intake.power = 1.0
                _feederTimer.reset()
                _launchState = LaunchState.LAUNCHING
            }

            LaunchState.LAUNCHING -> if (_feederTimer.seconds() > FEED_TIME_SECONDS) {
                _launchState = LaunchState.SETUP
                _leftFeeder.power = STOP_SPEED
                _rightFeeder.power = STOP_SPEED
                _intake.power = STOP_SPEED
            }

            LaunchState.SETUP -> {
                _launcher.velocity = STOP_SPEED
                while (_launcher.velocity > LAUNCHER_OFF_VELOCITY) {
                }
                _feederTimer.reset()
                _intake.power = -0.75
                while (true) {
                    if (_feederTimer.seconds() > 0.3) break
                }
                _intake.power = 0.0
                _feederTimer.reset()
                while (true) {
                    if (_feederTimer.seconds() > 0.4) break
                }
                _launchState = LaunchState.IDLE
                _amountLaunch++
            }
        }
    }
}