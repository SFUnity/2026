// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constantsGlobal;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public final class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(690.876);
  public static final double fieldWidth = Units.inchesToMeters(317);
  public static final double startingLineX =
      Units.inchesToMeters(299.438); // Measured from the inside of starting line

  public static final Pose2d processorScore =
      new Pose2d(Units.inchesToMeters(235.726), .84 / 2, Rotation2d.fromDegrees(-90));

  public static class Barge {
    public static final Translation2d farCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
    public static final Translation2d middleCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
    public static final Translation2d closeCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

    // Measured from floor to bottom of cage
    public static final double deepHeight = Units.inchesToMeters(3.125);
    public static final double shallowHeight = Units.inchesToMeters(30.125);
  }

  public static class CoralStation {
    public static final Pose2d leftCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(291.176),
            Rotation2d.fromDegrees(90 - 144.011));
    public static final Pose2d rightCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(25.824),
            Rotation2d.fromDegrees(144.011 - 90));
  }

  public static class StagingPositions {
    // Measured from the center of the ice cream
    public static final Pose2d leftIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());
    public static final Pose2d middleIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5), new Rotation2d());
    public static final Pose2d rightIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
  }

  private static final LoggedTunableNumber extraXOffsetAlgae =
      new LoggedTunableNumber("Distances/extraXAlgae", Units.inchesToMeters(17));
  private static final LoggedTunableNumber extraXOffsetBranch =
      new LoggedTunableNumber("Distances/extraXBranch", 0.475);
  public static final LoggedTunableNumber extraYOffset =
      new LoggedTunableNumber("Distances/extraY", 0);
  public static final LoggedTunableNumber elevatorSafeExtensionDistanceMeters =
      new LoggedTunableNumber("Distances/elevatorExtension", 2);
  public static final LoggedTunableNumber intakeDistanceMeters =
      new LoggedTunableNumber("Distances/intake", .5);

  public static final Translation2d reefCenter =
      new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
  private static final double xOffset = Units.inchesToMeters(30.738);
  private static final DoubleSupplier yOffset =
      () -> Units.inchesToMeters(6.469) + extraYOffset.get();

  public static enum Branch {
    A(
        () ->
            new Pose2d(reefCenter, Rotation2d.fromDegrees(180 + (60 * 0)))
                .transformBy(
                    new Transform2d(
                        xOffset + extraXOffsetBranch.get(),
                        -yOffset.getAsDouble(),
                        new Rotation2d(Math.PI)))),
    B(
        () ->
            new Pose2d(reefCenter, Rotation2d.fromDegrees(180 + (60 * 0)))
                .transformBy(
                    new Transform2d(
                        xOffset + extraXOffsetBranch.get(),
                        yOffset.getAsDouble(),
                        new Rotation2d(Math.PI)))),
    C(
        () ->
            new Pose2d(reefCenter, Rotation2d.fromDegrees(180 + (60 * 1)))
                .transformBy(
                    new Transform2d(
                        xOffset + extraXOffsetBranch.get(),
                        -yOffset.getAsDouble(),
                        new Rotation2d(Math.PI)))),
    D(
        () ->
            new Pose2d(reefCenter, Rotation2d.fromDegrees(180 + (60 * 1)))
                .transformBy(
                    new Transform2d(
                        xOffset + extraXOffsetBranch.get(),
                        yOffset.getAsDouble(),
                        new Rotation2d(Math.PI)))),
    E(
        () ->
            new Pose2d(reefCenter, Rotation2d.fromDegrees(180 + (60 * 2)))
                .transformBy(
                    new Transform2d(
                        xOffset + extraXOffsetBranch.get(),
                        -yOffset.getAsDouble(),
                        new Rotation2d(Math.PI)))),
    F(
        () ->
            new Pose2d(reefCenter, Rotation2d.fromDegrees(180 + (60 * 2)))
                .transformBy(
                    new Transform2d(
                        xOffset + extraXOffsetBranch.get(),
                        yOffset.getAsDouble(),
                        new Rotation2d(Math.PI)))),
    G(
        () ->
            new Pose2d(reefCenter, Rotation2d.fromDegrees(180 + (60 * 3)))
                .transformBy(
                    new Transform2d(
                        xOffset + extraXOffsetBranch.get(),
                        -yOffset.getAsDouble(),
                        new Rotation2d(Math.PI)))),
    H(
        () ->
            new Pose2d(reefCenter, Rotation2d.fromDegrees(180 + (60 * 3)))
                .transformBy(
                    new Transform2d(
                        xOffset + extraXOffsetBranch.get(),
                        yOffset.getAsDouble(),
                        new Rotation2d(Math.PI)))),
    I(
        () ->
            new Pose2d(reefCenter, Rotation2d.fromDegrees(180 + (60 * 4)))
                .transformBy(
                    new Transform2d(
                        xOffset + extraXOffsetBranch.get(),
                        -yOffset.getAsDouble(),
                        new Rotation2d(Math.PI)))),
    J(
        () ->
            new Pose2d(reefCenter, Rotation2d.fromDegrees(180 + (60 * 4)))
                .transformBy(
                    new Transform2d(
                        xOffset + extraXOffsetBranch.get(),
                        yOffset.getAsDouble(),
                        new Rotation2d(Math.PI)))),
    K(
        () ->
            new Pose2d(reefCenter, Rotation2d.fromDegrees(180 + (60 * 5)))
                .transformBy(
                    new Transform2d(
                        xOffset + extraXOffsetBranch.get(),
                        -yOffset.getAsDouble(),
                        new Rotation2d(Math.PI)))),
    L(
        () ->
            new Pose2d(reefCenter, Rotation2d.fromDegrees(180 + (60 * 5)))
                .transformBy(
                    new Transform2d(
                        xOffset + extraXOffsetBranch.get(),
                        yOffset.getAsDouble(),
                        new Rotation2d(Math.PI))));

    Branch(Supplier<Pose2d> pose) {
      this.pose = pose;
    }

    public Pose2d getPose() {
      return pose.get();
    }

    public final Supplier<Pose2d> pose;
  }

  public static enum Face {
    One(
        () ->
            new Pose2d(reefCenter, Rotation2d.fromDegrees(180))
                .transformBy(
                    new Transform2d(xOffset + extraXOffsetAlgae.get(), 0, new Rotation2d(Math.PI))),
        Branch.A,
        Branch.B,
        true),
    Two(
        () ->
            new Pose2d(reefCenter, Rotation2d.fromDegrees(180 + (60 * 1)))
                .transformBy(
                    new Transform2d(xOffset + extraXOffsetAlgae.get(), 0, new Rotation2d(Math.PI))),
        Branch.C,
        Branch.D,
        false),
    Three(
        () ->
            new Pose2d(reefCenter, Rotation2d.fromDegrees(180 + (60 * 2)))
                .transformBy(
                    new Transform2d(xOffset + extraXOffsetAlgae.get(), 0, new Rotation2d(Math.PI))),
        Branch.E,
        Branch.F,
        true),
    Four(
        () ->
            new Pose2d(reefCenter, Rotation2d.fromDegrees(180 + (60 * 3)))
                .transformBy(
                    new Transform2d(xOffset + extraXOffsetAlgae.get(), 0, new Rotation2d(Math.PI))),
        Branch.G,
        Branch.H,
        false),
    Five(
        () ->
            new Pose2d(reefCenter, Rotation2d.fromDegrees(180 + (60 * 4)))
                .transformBy(
                    new Transform2d(xOffset + extraXOffsetAlgae.get(), 0, new Rotation2d(Math.PI))),
        Branch.I,
        Branch.J,
        true),
    Six(
        () ->
            new Pose2d(reefCenter, Rotation2d.fromDegrees(180 + (60 * 5)))
                .transformBy(
                    new Transform2d(xOffset + extraXOffsetAlgae.get(), 0, new Rotation2d(Math.PI))),
        Branch.K,
        Branch.L,
        false),
    ;

    Face(Supplier<Pose2d> pose, Branch lefBranch, Branch rightBranch, boolean highAlgae) {
      this.pose = pose;
      this.leftBranch = lefBranch;
      this.rightBranch = rightBranch;
      this.highAlgae = highAlgae;
    }

    public Pose2d getPose() {
      return pose.get();
    }

    public final Supplier<Pose2d> pose;
    public final Branch leftBranch;
    public final Branch rightBranch;
    public final boolean highAlgae;
  }
}
