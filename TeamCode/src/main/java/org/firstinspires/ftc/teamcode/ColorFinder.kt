package org.firstinspires.ftc.teamcode

import android.graphics.Color
import android.util.Size
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor.Blob
import org.firstinspires.ftc.vision.opencv.ColorRange
import org.firstinspires.ftc.vision.opencv.ImageRegion
import kotlin.math.abs
import kotlin.math.max

typealias Locator = ColorBlobLocatorProcessor
typealias LocatorUtil = ColorBlobLocatorProcessor.Util
typealias LocatorBuilder = ColorBlobLocatorProcessor.Builder
typealias ContourMode = ColorBlobLocatorProcessor.ContourMode

@Autonomous(name = "Color Finder")
class ColorFinder : OpMode() {
    enum class Direction {
        LEFT,
        RIGHT,
        CENTER,
    }

    enum class TeamColor {
        RED,
        BLUE,
    }

    enum class RobotState {
        LookingForOrder,
        FindingTarget,
        Launch,
        GoBack,
        Load,
    }

    val moveTime = 1.0;
    val LAUNCHER_TARGET_VELOCITY: Double = 1200.0
    val LAUNCHER_MIN_VELOCITY: Double = 1176.0

    val LAUNCHER_OFF_VELOCITY: Double = 500.0
    private val FULL_EXTEND = 0.8
     val FEED_TIME_SECONDS: Double =
        0.2 //The feeder servos run this long when a shot is requested.
     val SETUP_TIME_SECOND: Double = 0.6
     val STOP_SPEED: Double = 0.0 //We send this power to the servos when we want them to stop.
     val FULL_SPEED: Double = 0.6

     val SPIN_SPEED: Double = 1.0
     val SPIN_TIME_SECONDS: Double = 0.5

    private enum class LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
        SETUP,
    }

    private enum class Location {
        Tower,
        LoadZone,
    }

    private var _location = Location.Tower
    private var launchState: LaunchState = LaunchState.IDLE

    private val region = ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75)!!
    private val resolution = Size(640, 480)

    private lateinit var _purpleFinder: Locator
    private lateinit var _greenFinder: Locator
    private lateinit var _aprilFinder: AprilTagProcessor
    private lateinit var _portal: VisionPortal

    private lateinit var _leftFrontDrive: DcMotor
    private lateinit var _rightFrontDrive: DcMotor
    private lateinit var _leftBackDrive: DcMotor
    private lateinit var _rightBackDrive: DcMotor

    private lateinit var _intake: DcMotor
    private lateinit var _launcher: DcMotorEx
    private lateinit var _leftFeeder: CRServo
    private lateinit var _rightFeeder: CRServo
    private lateinit var _restrictorArm: Servo

    private var _ballsLaunch = 0
    private var _launchDelay = 0.5
    private val stayOpenTime = 2.0
    private val stayCloseTIme = 2.6


    private val ballFinder: Locator? get() =
        if (_findGreen)
            _greenFinder
        else
            _purpleFinder

    private var _color = TeamColor.RED

    private var _direction: Direction = Direction.CENTER
    private var _findGreen = false

    private var _roboState = RobotState.FindingTarget
    private var _ballNumber = 0

    // order 0 = order not found
    // order 1 = green purple purple
    // order 2 = purple green purple
    // order 3 = purple purple green
    private var _order = 2 // order in detirm by which position is the first ball

    // these two variables are use to make find how long the item can go out frame before it stop seraching
    private var _timeSearching = ElapsedTime()
    private val maxTimeSearch = 0.5
    private var _beenOutFrame = true // whether it have been out of frame

    private var _leftFrontPower: Double = 0.0
    private var _rightFrontPower: Double = 0.0
    private var _leftBackPower: Double = 0.0
    private var _rightBackPower: Double = 0.0

    private var _atCenterTime = ElapsedTime()
    private val considerCenterTime = 0.2

    private var _driveTime = ElapsedTime()
    private val _maxDriveTime = 0.5

    private var _centered: Boolean = false

    var feederTimer: ElapsedTime = ElapsedTime()
    var spinTime: ElapsedTime = ElapsedTime()
    var launchTime: ElapsedTime = ElapsedTime()

    fun initLocator(targetColorRange: ColorRange): Locator {
        return LocatorBuilder()
            .setTargetColorRange(targetColorRange)
            .setContourMode(ContourMode.EXTERNAL_ONLY)
            .setRoi(region)
            .setDrawContours(true)
            .setBoxFitColor(0)
            .setCircleFitColor(Color.GREEN)
            .setBlurSize(5)
            .build()!!

    }

    override fun init() {
        super.telemetry.addLine("Hello, world... starting")

        _greenFinder = initLocator(ColorRange.ARTIFACT_GREEN)
        _purpleFinder = initLocator(ColorRange.ARTIFACT_PURPLE)
        _aprilFinder = AprilTagProcessor.easyCreateWithDefaults()!!
        _aprilFinder.setDecimation(2F)


        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        _leftFrontDrive = super.hardwareMap.get(DcMotor::class.java, "leftFrontDrive")
        _rightFrontDrive = super.hardwareMap.get(DcMotor::class.java, "rightFrontDrive")
        _leftBackDrive = super.hardwareMap.get(DcMotor::class.java, "leftBackDrive")
        _rightBackDrive = super.hardwareMap.get(DcMotor::class.java, "rightBackDrive")
        _launcher = super.hardwareMap.get(DcMotorEx::class.java, "launcher")
        _leftFeeder = super.hardwareMap.get(CRServo::class.java, "leftFeeder")
        _rightFeeder = super.hardwareMap.get(CRServo::class.java, "rightFeeder")
        _intake = super.hardwareMap.get(DcMotor::class.java, "intake")
        _restrictorArm = hardwareMap.get(Servo::class.java, "restrictorArm")

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */

        // 0.8 fully ingage motor
        _leftFrontDrive.direction = DcMotorSimple.Direction.FORWARD
        _rightFrontDrive.direction = DcMotorSimple.Direction.REVERSE
        _leftBackDrive.direction = DcMotorSimple.Direction.FORWARD
        _rightBackDrive.direction = DcMotorSimple.Direction.FORWARD

        _launcher.mode = DcMotor.RunMode.RUN_USING_ENCODER

        _leftFrontDrive.zeroPowerBehavior = ZeroPowerBehavior.BRAKE
        _rightFrontDrive.zeroPowerBehavior = ZeroPowerBehavior.BRAKE
        _leftBackDrive.zeroPowerBehavior = ZeroPowerBehavior.BRAKE
        _rightBackDrive.zeroPowerBehavior = ZeroPowerBehavior.BRAKE
        _launcher.zeroPowerBehavior = ZeroPowerBehavior.BRAKE
        _intake.zeroPowerBehavior = ZeroPowerBehavior.BRAKE


        _leftFeeder.power = 0.0
        _rightFeeder.power = 0.0
        _intake.power = 0.0

        _launcher.setPIDFCoefficients(
            DcMotor.RunMode.RUN_USING_ENCODER,
            PIDFCoefficients(300.0, 0.0, 0.0, 10.0)
        )

        _leftFeeder.direction = DcMotorSimple.Direction.FORWARD;
        _rightFeeder.direction = DcMotorSimple.Direction.REVERSE;

        _portal =
            VisionPortal.Builder()
                .addProcessors(_greenFinder, _purpleFinder)
                .addProcessor(_aprilFinder)
                .setCameraResolution(resolution)
                .setCamera(
                    super.hardwareMap.get(WebcamName::class.java, "Webcam 1")
                )
                .build()!!

        // _portal.setProcessorEnabled(_greenFinder, false)
        // _portal.setProcessorEnabled(_purpleFinder, false)

        super.telemetry.msTransmissionInterval = 100
        super.telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE)
    }

    var protype = false;
    override fun init_loop() {
        super.telemetry.addData("Color", _color::name)
        super.telemetry.addData("Pos", _location::name)
        if (super.gamepad1.x) {
            _color = TeamColor.BLUE
        } else if (super.gamepad1.b) {
            _color = TeamColor.RED
        }

        if (super.gamepad1.a) {
            _location = if (_location == Location.Tower) Location.LoadZone else Location.Tower
        }
        if (super.gamepad1.y) {
            protype = !protype;
        }
        super.telemetry.addData("Is Pro",protype);
        _roboState = if (protype) RobotState.Load else if (_location == Location.Tower) RobotState.GoBack else RobotState.FindingTarget
        super.telemetry.update()
        super.telemetry.clearAll()
    }

    override fun loop() {
        when (_roboState) {
            RobotState.LookingForOrder -> throw Exception("Not good")
/*
            RobotState.SearchingForBall -> findBall()
            RobotState.IntakeBall -> driveAndIntake()
*/
            RobotState.FindingTarget -> findTarget()
            RobotState.Launch -> launchBalls()
            RobotState.GoBack -> goBack()
            RobotState.Load -> load()
        }

        when (_order) {
            1 -> super.telemetry.addData("Order", "Green Purple Purple")
            2 -> super.telemetry.addData("Order", "Purple Green Purple")
            3 -> super.telemetry.addData("Order", "Purple Purple Green")
            else -> super.telemetry.addData("Order", "Unknown")
        }

        super.telemetry.addData("Balls collected", _ballNumber)
        super.telemetry.update()
    }

    private fun driveAndIntake() {
        _intake.power = 1.0
        mecanumDrive(0.6, 0.0, 0.0)
    }

    var oldBlobContourArea: Int = -1;
    fun findBall() {
        super.telemetry.addLine("Searching for ball")
        if (ballFinder == null) {
            super.telemetry.addLine("Warning! Ball processor not enabled!")
            return
        }
        val blobs = ballFinder!!.blobs

        LocatorUtil.filterByCriteria(
            ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
            500.0,
            65_000.0,
            blobs
        )

        super.telemetry.addLine("Ctr:(X,Y) Area Dens Asspect Arc Rect")

        var biggestBlob: Blob? = null
        for (b in blobs) {
            if (biggestBlob == null || biggestBlob.contourArea < b.contourArea) {
                biggestBlob = b
            }
            oldBlobContourArea = biggestBlob.contourArea;
            val boxFit = b.boxFit
            val center = boxFit.center
            super.telemetry.addLine(
                "(%3d,%3d) %5d %4.2f  %5.2f %3d %5.3f ".format(
                    center.x.toInt(),
                    center.y.toInt(),
                    b.contourArea,
                    b.density,
                    b.aspectRatio,
                    b.arcLength.toInt(),
                    b.circularity
                )
            )
        }

        if (biggestBlob != null) {
            _beenOutFrame = false
            val center = biggestBlob.boxFit!!.center!!
            _direction = when {
                center.x <= 146 -> Direction.RIGHT
                center.x >= 506 -> Direction.LEFT
                else -> Direction.CENTER
            }
        } else {
            if (!_beenOutFrame) {
                _timeSearching.reset()
            }
            _beenOutFrame = true
        }

        val rotate = when (_direction) {
            Direction.LEFT -> -0.45
            Direction.RIGHT -> 0.45
            Direction.CENTER -> 0.0
        }

        if (oldBlobContourArea > 20_000 && _beenOutFrame && _timeSearching.seconds() < maxTimeSearch && _direction == Direction.CENTER) {
//            _roboState = RobotState.IntakeBall;
            return;
        } else {
            mecanumDrive(if (_direction == Direction.CENTER && biggestBlob != null) 0.4 else 0.0, 0.0, rotate)
        }

        if (_beenOutFrame && _timeSearching.seconds() >= maxTimeSearch)
            _direction = Direction.CENTER

        super.telemetry.addData("Position", _direction)
    }

    fun ballCollected() {
        _ballNumber ++
        if (_ballNumber > 2) {
            _roboState = RobotState.FindingTarget
            _ballNumber = 0
            _direction = Direction.CENTER
        } else {
            updateColorSensors()
        }
    }

    fun updateColorSensors() {
        _findGreen = _ballNumber == _order-1
    }

/*
    fun findApril() {
        super.telemetry.addLine("Searching for april")
        val aprilTag = _aprilFinder.detections.find {
            when (it.id) {
                21,22,23 -> true
                else -> false
            }
        }

        if (aprilTag == null) return

        _order = aprilTag.id - 20
        _roboState = RobotState.SearchingForBall
        updateColorSensors()
    }
*/

    fun mecanumDrive(forward: Double, strafe: Double, rotate: Double) {
        /* the denominator is the largest motor power (absolute value) or 1
                 * This ensures all the powers maintain the same ratio,
                 * but only if at least one is out of the range [-1, 1]
                 */

        val denominator = max(abs(forward) + abs(strafe) + abs(rotate), 1.0)

        _leftFrontPower = (-forward + strafe + rotate) / denominator
        _rightFrontPower = (-forward - strafe - rotate) / denominator
        _leftBackPower = (-forward - strafe + rotate) / denominator
        _rightBackPower = (-forward + strafe - rotate) / denominator

        _leftFrontDrive.power = _leftFrontPower
        _rightFrontDrive.power = _rightFrontPower
        _leftBackDrive.power = _leftBackPower
        _rightBackDrive.power = _rightBackPower
    }

    fun findTarget() {
        super.telemetry.addLine("Searching for target")
        val aprilTag = _aprilFinder.detections.find {
            val id = when (_color) {
                TeamColor.RED -> 24
                TeamColor.BLUE -> 20
            }
            it.id == id
        }

        var speed: Double
        if (aprilTag != null) {
            _beenOutFrame = false
            val center = aprilTag.ftcPose
            super.telemetry.addData("pos", "(%f, %f, %f)", center.x, center.y, center.z)

            _direction = when {
                center.x <= -2 -> Direction.RIGHT
                center.x >= 2 -> Direction.LEFT
                else -> Direction.CENTER
            }

            speed = when {
                abs(center.x) > 1.25 -> 0.18
                abs(center.x) <= 1.25 -> 0.1
                else -> 0.1
            }
        } else {
            super.telemetry.addData("pos", "NOT FOUND")
            if (!_beenOutFrame) {
                _timeSearching.reset()
            }
            _beenOutFrame = true
            speed = 0.1
        }

        val rotate = if (_beenOutFrame) {
            speed
        } else {when (_direction) {
            Direction.LEFT -> -speed
            Direction.RIGHT -> speed
            Direction.CENTER -> 0.0
        }}

        mecanumDrive(0.0, 0.0, rotate)

        if (_direction == Direction.CENTER && !_beenOutFrame) {
            if (!_centered)
                _atCenterTime.reset()
            _centered = true
            if (_atCenterTime.seconds() > considerCenterTime) {
                _driveTime.reset();
                while (true) {
                    mecanumDrive(1.0, 0.0, 0.00)
                    if (_driveTime.seconds() > _maxDriveTime) break
                }
            }
        } else {
            _centered = false
        }

        if (_beenOutFrame && _timeSearching.seconds() >= maxTimeSearch)
            _direction = Direction.CENTER
    }
    fun launch(shotRequested: Boolean) {
        when (launchState) {
            LaunchState.IDLE -> if (shotRequested) {
                launchState = LaunchState.SPIN_UP
            }

            LaunchState.SPIN_UP -> {
                _launcher.setVelocity(LAUNCHER_TARGET_VELOCITY)
                if (_launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH
                }
            }

            LaunchState.LAUNCH -> {
                _leftFeeder.setPower(-1.0)
                _rightFeeder.setPower(-1.0)
                _intake.setPower(1.0)
                feederTimer.reset()
                launchState = LaunchState.LAUNCHING
            }

            LaunchState.LAUNCHING -> if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                launchState = LaunchState.SETUP
                _leftFeeder.setPower(0.0)
                _rightFeeder.setPower(0.0)
                _intake.setPower(0.0)
            }

            LaunchState.SETUP -> {
                _launcher.setVelocity(0.0)
                while (_launcher.getVelocity() > LAUNCHER_OFF_VELOCITY) {
                }
                _restrictorArm.setPosition(0.0)
                feederTimer.reset()
                while (true) {
                    if (feederTimer.seconds() > SETUP_TIME_SECOND) break
                }
                _restrictorArm.setPosition(FULL_EXTEND)
                launchState = LaunchState.IDLE
                _ballsLaunch ++;
            }
        }
    }

    fun launchBalls() {
        while (_ballsLaunch < 3) {
            launch(true)
        }
        _roboState = RobotState.FindingTarget;
    }

    fun goBack() {
        spinTime.reset()
        while (spinTime.seconds() < _maxDriveTime) {
            mecanumDrive(0.1, 0.0, 0.0)
        }
        mecanumDrive(0.0, 0.0, 0.0)

        _roboState = RobotState.Launch
        spinTime.reset();
    }

    var firstBall = true
    var start = false
    fun load() {
        if (!start) {
            spinTime.reset()
            start = true
        }
        if (firstBall) {
            _restrictorArm.position = 0.0
            if (spinTime.seconds() > stayOpenTime) {
                firstBall = false
                _restrictorArm.position = 1.0
                spinTime.reset();
            }
        } else {
            if (spinTime.seconds() > stayCloseTIme) {
                firstBall = true
                _restrictorArm.position = 1.0
                spinTime.reset()
                _roboState = RobotState.FindingTarget
            }
        }
    }
}
