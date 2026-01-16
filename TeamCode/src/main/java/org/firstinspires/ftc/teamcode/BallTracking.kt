package org.firstinspires.ftc.teamcode

import android.graphics.Color
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.hardware.dfrobot.HuskyLens
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor
import org.firstinspires.ftc.vision.opencv.ColorRange

@Autonomous(name="Ball Tracker")
class BallTracking : LinearOpMode() {
    private enum class LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
        SETUP,

    }

    private var _launchState: LaunchState = LaunchState.IDLE
    private lateinit var _intake: DcMotor
    private lateinit var _launcher: DcMotorEx
    private lateinit var _leftFeeder: CRServo
    private lateinit var _rightFeeder: CRServo
    private lateinit var _leftIntake: CRServo
    private lateinit var _rightIntake: CRServo
    private lateinit var _cam: HuskyLens
    private lateinit var _portal: VisionPortal

    private lateinit var _purpleFinder: Locator
    private lateinit var _greenFinder: Locator

    private val LAUNCHER_TARGET_VELOCITY: Double = 1300.0
    private val LAUNCHER_MIN_VELOCITY: Double = 1240.0

    private val LAUNCHER_FAR_TARGET_VELOCITY: Double = LAUNCHER_TARGET_VELOCITY + 300
    private val LAUNCHER_FAR_MIN_VELOCITY: Double = LAUNCHER_MIN_VELOCITY + 300

    private val LAUNCHER_OFF_VELOCITY: Double = 500.0
    private val FEED_TIME_SECONDS: Double = 0.2 //The feeder servos run this long when a shot is requested.
    private val STOP_SPEED: Double = 0.0 //We send this power to the servos when we want them to stop.

    private var _feederTimer = ElapsedTime()

    fun initLocator(targetColorRange: ColorRange): Locator {
        return LocatorBuilder()
            .setTargetColorRange(targetColorRange)
            .setContourMode(ContourMode.EXTERNAL_ONLY)
            .setDrawContours(true)
            .setBoxFitColor(0)
            .setCircleFitColor(Color.GREEN)
            .setBlurSize(5)
            .build()!!

    }

    override fun runOpMode() {

        // begin at (61.6, 18.2)
        // red tower at (-53.5, 54.5)
        // heading = 131 at tower red
        // heading = 90? at start
        val beginPos = Pose2d(Vector2d(61.6, 18.2), Math.toRadians(180.0))
        val redPos = Pose2d(Vector2d(-53.5+10.0, 54.5-10.0), Math.toRadians(131.0))

        _launcher = super.hardwareMap.get(DcMotorEx::class.java, "launcher")
        _leftFeeder = super.hardwareMap.get(CRServo::class.java, "leftFeeder")
        _rightFeeder = super.hardwareMap.get(CRServo::class.java, "rightFeeder")
        _intake = super.hardwareMap.get(DcMotor::class.java, "intake")
        _leftIntake = super.hardwareMap.get(CRServo::class.java, "leftIntake")
        _rightIntake = super.hardwareMap.get(CRServo::class.java, "rightIntake")
        _cam = super.hardwareMap.get(HuskyLens::class.java, "cam")

        _greenFinder = initLocator(ColorRange.ARTIFACT_GREEN)
        _purpleFinder = initLocator(ColorRange.ARTIFACT_PURPLE)

        _portal = VisionPortal.Builder()
            .addProcessors(_greenFinder, _purpleFinder)
            .setCamera(
                super.hardwareMap.get(WebcamName::class.java, "cam")
            )
            .build()!!

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

        // create RR drive obj
        val drive = MecanumDrive(super.hardwareMap, beginPos)

        waitForStart();

//        while (opModeIsActive()) {
//            LocatorUtil.filterByCriteria(
//                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
//                500.0,
//                65_000.0,
//            )
//        }
    }

    private var _launchFar = false
    private var _amountLaunch = 0
    fun launch(shotRequested: Boolean, far: Boolean = false) {
        val targetVelocity: Double =
            if (_launchFar) LAUNCHER_FAR_TARGET_VELOCITY else LAUNCHER_TARGET_VELOCITY
        val minVelocity: Double =
            if (_launchFar) LAUNCHER_FAR_MIN_VELOCITY else LAUNCHER_MIN_VELOCITY
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
                    if (_feederTimer.seconds() > 0.2) break
                }
                _intake.power = 0.0
                _launchState = LaunchState.IDLE
                _amountLaunch++
            }
        }
    }
}