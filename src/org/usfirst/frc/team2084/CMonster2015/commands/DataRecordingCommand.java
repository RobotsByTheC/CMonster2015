/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.commands;

import java.io.File;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.AsynchronousFileChannel;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

import edu.wpi.first.wpilibj.command.Command;

/**
 * @author ben
 */
public abstract class DataRecordingCommand extends Command {

    public static final String DATA_PATH = "/home/lvuser/recorded-data";

    private static final DateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss-SS");

    private final String dataName;
    private final File dataDirectory;
    private AsynchronousFileChannel writer;

    public DataRecordingCommand(String dataName) {
        this.dataName = dataName;
        dataDirectory = new File(dataName);
        if (dataDirectory.exists()) {
            if (!dataDirectory.isDirectory()) {
                throw new IllegalArgumentException("Data name cannot be the name of an existing file in "
                        + DATA_PATH);
            }
        } else {
            if (!dataDirectory.mkdirs()) {
                throw new IllegalStateException("Could not create data directory.");
            }
        }
    }

    /**
     * 
     */
    @Override
    protected void initialize() {
        try {
            String fileName = dateFormat.format(new Date()) + ".csv";
            writer = AsynchronousFileChannel.open(Paths.get(DATA_PATH, dataName, fileName),
                    StandardOpenOption.CREATE_NEW, StandardOpenOption.WRITE);
        } catch (IOException e) {
            System.err.println("Could not open data file for writing or file already exists.");
        }
    }

    protected void writeData(String... data) {
        if (writer != null) {
            StringBuilder line = new StringBuilder(data[0]);
            for (int i = 1; i < data.length; i++) {
                line.append(',');
                line.append(data[i]);
            }
            line.append('\n');

            ByteBuffer buffer = ByteBuffer.wrap(line.toString().getBytes());
            writer.write(buffer, 0);
        }
    }

    /**
     * @return
     */
    @Override
    protected boolean isFinished() {
        return false;
    }

    /**
     * 
     */
    @Override
    protected void end() {
        if (writer != null) {
            try {
                writer.force(true);
                writer.close();
            } catch (IOException e) {
                System.err.println("Could not close data file.");
            }
            writer = null;
        }
    }

    @Override
    protected void interrupted() {
        end();
    }

}
