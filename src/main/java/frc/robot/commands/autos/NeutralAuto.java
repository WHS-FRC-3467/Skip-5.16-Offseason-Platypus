package frc.robot.commands.autos;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.commands.ResilientTrajectoryFollower;
import frc.robot.commands.autos.utils.AutoCommands;
import frc.robot.commands.autos.utils.AutoContext;
import frc.robot.commands.autos.utils.AutoOption;
import frc.robot.commands.autos.utils.AutoUtil;
import frc.robot.generated.ChoreoTraj;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

/** Native Choreo routine for the neutral-zone multi-piece autonomous variants. */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public final class NeutralAuto {
    private static final Alert TRAJECTORIES_MISSING =
            new Alert("Neutral Auto Trajectories Missing, Auto(s) Unavailable", AlertType.kError);

    /**
     * Builds the selected neutral auto variant.
     *
     * @param shouldMirror Whether to mirror the route for the opposite starting side
     */
    public static Optional<AutoOption> create(AutoContext ctx, boolean shouldMirror) {
        List<String> names =
                List.of(ChoreoTraj.NeutralSafe1.name(), ChoreoTraj.NeutralSafe2.name());
        List<Trajectory<SwerveSample>> trajectories =
                AutoUtil.loadTrajectories(names, shouldMirror).orElse(null);
        if (trajectories == null) {
            TRAJECTORIES_MISSING.set(true);
            return Optional.empty();
        }

        return Optional.of(
                AutoUtil.trajectoryOption(
                        trajectories,
                        () -> {
                            AutoRoutine routine =
                                    ctx.autoFactory()
                                            .newRoutine(
                                                    "NeutralSafe"
                                                            + (shouldMirror ? "Right" : "Left"));
                            AutoTrajectory first = routine.trajectory(trajectories.get(0));
                            Map<String, Command> eventBindings = AutoUtil.createEventBindings(ctx);
                            ResilientTrajectoryFollower firstFollow =
                                    ctx.drive()
                                            .followTrajectoryResilient(
                                                    trajectories.get(0), eventBindings);
                            ResilientTrajectoryFollower secondFollow =
                                    ctx.drive()
                                            .followTrajectoryResilient(
                                                    trajectories.get(1), eventBindings);

                            routine.active()
                                    .onTrue(
                                            Commands.sequence(
                                                    Commands.runOnce(
                                                            ctx.drive()
                                                                    ::resetTrajectoryControllers),
                                                    first.resetOdometry(),
                                                    Commands.defer(
                                                            () ->
                                                                    Commands.waitSeconds(
                                                                            AutoCommands
                                                                                    .getAutoDelay()),
                                                            Set.of()),
                                                    firstFollow));

                            routine.observe(firstFollow.done())
                                    .onTrue(AutoCommands.shootThenFollow(ctx, 4.0, secondFollow));

                            routine.observe(secondFollow.done())
                                    .onTrue(AutoCommands.shootThenFollow(ctx, 3.0, secondFollow));

                            return routine;
                        }));
    }
}
