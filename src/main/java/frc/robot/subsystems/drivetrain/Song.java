package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import frc.robot.constants.Constants;
import frc.robot.util.Elastic;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.IntStream;

public class Song {
    public final int[] trackWeights;
    private final String filename;

    public static final Song[] startupSongs = new Song[] {
            new Song("mario.chrp", new int[] {5, 2, 1}),
    };

    public static final Song[] disableSongs = new Song[] {
            new Song("mario_death.chrp", new int[] {5, 2, 1}),
    };

    public static void playRandom(Swerve swerve, Song[] songs) {
        Song song = songs[swerve.random.nextInt(Song.startupSongs.length)];

        song.play(swerve);
    }

    public Song(String filename, int[] trackWeights) {
        this.trackWeights = trackWeights;
        this.filename = filename;
    }

    public String getPath() {
        return "sound/" + filename;
    }

    public void play(Swerve swerve) {
        swerve.orchestra.stop();

        List<ParentDevice> motors = new ArrayList<>();

        for (SwerveModule<TalonFX, TalonFX, CANcoder> module : swerve.getModules()) {
            module.getDriveMotor().getConfigurator().apply(Constants.SwerveConstants.AUDIO_CONFIGS, 0.05);
            module.getSteerMotor().getConfigurator().apply(Constants.SwerveConstants.AUDIO_CONFIGS, 0.05);

            motors.add(module.getDriveMotor());
            motors.add(module.getSteerMotor());
        }

        swerve.orchestra.clearInstruments();

        IntStream.range(0, trackWeights.length).forEach(
                weightIndex -> IntStream.range(0, trackWeights[weightIndex]).forEach(
                        i -> swerve.orchestra.addInstrument(motors.remove(0), weightIndex)
                )
        );

        StatusCode status = swerve.orchestra.loadMusic(getPath());

        Elastic.Notification.NotificationLevel notificationLevel;
        if (status.isWarning()) {
            notificationLevel = Elastic.Notification.NotificationLevel.WARNING;
        } else if (status.isError()) {
            notificationLevel = Elastic.Notification.NotificationLevel.ERROR;
        } else {
            notificationLevel = Elastic.Notification.NotificationLevel.INFO;
        }

        Elastic.sendNotification(
                new Elastic.Notification(
                        notificationLevel,
                        "Orchestra status",
                        status.getName() + ": " + status.getDescription()
                )
        );

        swerve.orchestra.play();
    }
}
