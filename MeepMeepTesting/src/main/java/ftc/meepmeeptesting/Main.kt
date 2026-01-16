package ftc.meepmeeptesting

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import java.util.Vector
import kotlin.math.PI


enum class TeamColor { RED, BLUE, }

fun toSided(color: TeamColor, pose2d: Pose2d): Pose2d {
    val fliped = pose2d.position.y * if (color == TeamColor.RED) 1.0 else -1.0
    val otherHeading =  if (color == TeamColor.RED) pose2d.heading.toDouble() else Rotation2d.fromDouble(2 * PI) - pose2d.heading
    return Pose2d(Vector2d(pose2d.position.x, fliped), otherHeading)
}

fun toSided(color: TeamColor, x: Double): Double {
    return if (color == TeamColor.RED) x else x * -1.0
}

fun toSidedDeg(color: TeamColor, x: Double): Double {
    return if (color == TeamColor.RED) x else 360 - x;
}

fun main(args: Array<String>) {
    System.setProperty("sun.java2d.opengl", "true");
    val meepMeep = MeepMeep(800)

    val beginPosRed = Pose2d(Vector2d(61.6, 18.2), Math.toRadians(180.0))
    val beginPosBlue = Pose2d(Vector2d(61.6, -18.2), Math.toRadians(180.0))
    val redPos = Pose2d(Vector2d(-53.5+10.0, 54.5-10.0), Math.toRadians(131.0))
    val bluePos = Pose2d(Vector2d(-53.5+10.0, -54.5+10.0), Math.toRadians(360.0 - 131.0))


    val _teamcolor = TeamColor.BLUE

    val dif = 11.2
    val heading = Math.toRadians(-270.0)
    val bal1 = Vector2d(-11.2, 24.0)

    val ball1 = toSided(_teamcolor, Pose2d(Vector2d(-11.2, 24.0), Math.toRadians(-270.0)))

    val pos = if (_teamcolor == TeamColor.RED) redPos else bluePos
    val beginPos = if (_teamcolor == TeamColor.RED) beginPosRed else beginPosBlue
    val myBot =
        DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
            .setStartPose(beginPos)
            .build()

    val drive = myBot.drive

    val toTower = drive.actionBuilder(beginPos)
        .splineToLinearHeading(
            pos,
            tangent = Math.toRadians(toSided(_teamcolor, 120.0))
        )
        .build()

    val ballPath = drive.actionBuilder(pos)
        .lineToY(pos.position.y - (20.0 * if (_teamcolor == TeamColor.RED) 1 else -1))
        .strafeToSplineHeading(ball1.position, ball1.heading)
        .build()
    val intakePath = drive.actionBuilder(ball1)
        .lineToY(ball1.position.y + toSided(_teamcolor, 30.0))
        .build()
    val towerPos = drive.actionBuilder(Pose2d(ball1.position + Vector2d(0.0, toSided(_teamcolor, 30.0)), ball1.heading))
        .strafeToSplineHeading(
            pos.position,
            pos.heading,
        )
        .build()

    val moveAway = drive.actionBuilder(pos).strafeTo(Vector2d(pos.position.x - 5.0, pos.position.y - toSided(_teamcolor, 20.0))).build()

    myBot.runAction(SequentialAction(toTower, ballPath, intakePath, towerPos, moveAway))

    // meepMeep.setBackground(Background.FIELD_DECODE_OFFICIAL)
    //     .setDarkMode(true)
    //     .setBackgroundAlpha(0.95f)
    //     .addEntity(myBot)
    //     .start()

    meepMeep.setBackground(Background.FIELD_DECODE_OFFICIAL)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(myBot)
        .start()
}