package org.firstinspires.ftc.teamcode

import android.util.Size
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.firstinspires.ftc.vision.opencv.ImageRegion
import kotlin.math.*

@TeleOp(name = "April tag")
class AprilTagDecode : OpMode() {
    enum class Direction {
        LEFT,
        RIGHT,
        CENTER,
    }

    private lateinit var _portal: VisionPortal
    private lateinit var _aprilProcessor: AprilTagProcessor
    private lateinit var _servo: Servo

    private var _direction: ColorFinder.Direction = ColorFinder.Direction.CENTER;

    // these two variables are use to make find how long the item can go out frame before it stop seraching
    private var _timeSearching = ElapsedTime();
    private val maxTimeSearch = 1.5;
    private var _beenOutFrame = true; // whether it have been out of frame

    override fun init() {
        _aprilProcessor = AprilTagProcessor.easyCreateWithDefaults()!!
        _portal = VisionPortal.easyCreateWithDefaults(
            super.hardwareMap.get(WebcamName::class.java, "Webcam 1"),
            _aprilProcessor,
        )

        _servo = super.hardwareMap.get(Servo::class.java, "Servo")
        _servo.position = 0.5
    }

    override fun loop() {
        val tags = _aprilProcessor.detections!!
        val blueTag = tags.find { it.id == 20 } // blue target

        if (blueTag != null) {
            _beenOutFrame = false;
            val center = blueTag.ftcPose
            super.telemetry.addData("pos", "(%f, %f, %f)", center.x, center.y, center.z)

            _direction = when {
                center.x <= -2.12 -> ColorFinder.Direction.LEFT
                center.x >= 2.12 -> ColorFinder.Direction.RIGHT
                else -> ColorFinder.Direction.CENTER
            }
        } else {
            super.telemetry.addData("pos", "NOT FOUND")
            if (!_beenOutFrame) {
                _timeSearching.reset()
            }
            _beenOutFrame = true
        }

        _servo.position += when (_direction) {
            ColorFinder.Direction.LEFT -> -0.1 / 360.0
            ColorFinder.Direction.RIGHT -> 0.1 / 360.0
            ColorFinder.Direction.CENTER -> 0.0
        }

        _servo.position = max(0.0, min(1.0, _servo.position));

        if (_beenOutFrame && _timeSearching.seconds() >= maxTimeSearch)
            _direction = ColorFinder.Direction.CENTER
    }
}