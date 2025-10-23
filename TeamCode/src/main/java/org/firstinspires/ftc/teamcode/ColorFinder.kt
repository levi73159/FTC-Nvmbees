package org.firstinspires.ftc.teamcode

import android.graphics.Color
import android.util.Size
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor
import org.firstinspires.ftc.vision.opencv.ColorRange
import org.firstinspires.ftc.vision.opencv.ColorSpace
import org.firstinspires.ftc.vision.opencv.ImageRegion
import org.opencv.core.Scalar

typealias Locator = ColorBlobLocatorProcessor
typealias LocatorUtil = ColorBlobLocatorProcessor.Util;
typealias LocatorBuilder = ColorBlobLocatorProcessor.Builder
typealias ContourMode = ColorBlobLocatorProcessor.ContourMode

@TeleOp(name="Color Finder")
class ColorFinder : OpMode() {
    private val region = ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75)!!
    private val resolution = Size(320, 240)
    private val targetColorRange = ColorRange(ColorSpace.RGB, Scalar(0.0, 0.0, 0.0), Scalar(5.0, 5.0, 5.0))

    private lateinit var _colorLocator: Locator
    private lateinit var _portal: VisionPortal

    override fun init() {
        _colorLocator = LocatorBuilder()
            .setTargetColorRange(targetColorRange)
            .setContourMode(ContourMode.EXTERNAL_ONLY)
            .setRoi(region)
            .setDrawContours(true )
            .setBoxFitColor(Color.GREEN)
            .setBlurSize(5)
            .build()!!

        _portal = VisionPortal.Builder()
            .addProcessor(_colorLocator)
            .setCameraResolution(resolution)
            .setCamera(super.hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .build()!!

        super.telemetry.msTransmissionInterval = 100;
        super.telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
    }

    override fun loop() {
        val blobs = _colorLocator.blobs;

        LocatorUtil.filterByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
            500.0, 20_000.0, blobs);

        super.telemetry.addLine("Ctr:(X,Y) Area Dens Asspect Arc Rect")

        for (b in blobs) {
            val boxFit = b.boxFit;
            val center = boxFit.center;
            super.telemetry.addLine("(%3d,%3d) %5d %4.2f  %5.2f %3d %5.3f ".format(
                center.x.toInt(), center.y.toInt(),
                b.contourArea, b.density, b.aspectRatio, b.arcLength.toInt(), b.circularity))
        }

        super.telemetry.update();
    }
}