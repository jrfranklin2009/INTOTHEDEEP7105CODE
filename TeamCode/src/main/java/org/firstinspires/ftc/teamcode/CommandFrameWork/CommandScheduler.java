package org.firstinspires.ftc.teamcode.CommandFrameWork;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
 // schedules commands
public class CommandScheduler {
	protected HardwareMap hwMap;
	protected ArrayList<Subsystem> subsystems = new ArrayList<>();
	protected ArrayList<Command> activeCommands = new ArrayList<>();

	public CommandScheduler(HardwareMap hardwareMap, Subsystem... initSubsystems) {
		hwMap = hardwareMap;
		Collections.addAll(subsystems, initSubsystems);
	}

	public void initAuto() {
		for (Subsystem subsystem : subsystems)
			subsystem.initAuto(hwMap);
	}

	public void initTeleop() {
		for (Subsystem subsystem : subsystems)
			subsystem.initTeleop(hwMap);
	}

	public void shutdown() {
		for (Subsystem subsystem : subsystems)
			subsystem.shutdown();
	}

	public void runAuto() {
		for (Subsystem subsystem : subsystems) {
			subsystem.startTimer();
			subsystem.periodicAuto();
			subsystem.stopTimer();
		}

		Iterator<Command> commands = activeCommands.iterator();

		ArrayList<Command> nextCommands = new ArrayList<>();
		while (commands.hasNext()) {
			Command command = commands.next();
			command.periodic();

			if (command.completed()) {
				command.shutdown();
				if (command.getNext() != null)
					nextCommands.add(command.getNext());

				commands.remove();
			}
		}

		for (Command nextCommand : nextCommands)
			forceCommand(nextCommand);
	}


//	 public void runTele() {
//		 for (Subsystem subsystem : subsystems) {
//			 subsystem.startTimer();
//			 subsystem.periodicTele();
//			 subsystem.stopTimer();
//		 }
//
//		 Iterator<Command> commands = activeCommands.iterator();
//
//		 ArrayList<Command> nextCommands = new ArrayList<>();
//		 while (commands.hasNext()) {
//			 Command command = commands.next();
//			 command.periodic();
//
//			 if (command.completed()) {
//				 command.shutdown();
//				 if (command.getNext() != null)
//					 nextCommands.add(command.getNext());
//
//				 commands.remove();
//			 }
//		 }
//
//		 for (Command nextCommand : nextCommands)
//			 forceCommand(nextCommand);
//	 }

	public void forceCommand(Command command) {
		ArrayList<Subsystem> nextCommandDependencies = command.getDependencies();

		Iterator<Command> currentCommands = activeCommands.iterator();

		while (currentCommands.hasNext()) {
			Command currentCommand = currentCommands.next();
			for (Subsystem subsystem : currentCommand.getDependencies())
				if (nextCommandDependencies.contains(subsystem)) {
					currentCommand.shutdown();
					currentCommands.remove();
					break;
				}
		}

		activeCommands.add(command);
		command.init();
	}

	public boolean isEmpty() {
		return activeCommands.isEmpty();
	}
}
