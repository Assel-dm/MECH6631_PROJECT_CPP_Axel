# Variant patch for MECH6631 project

Files to replace in the common project:

- `program.cpp`
- `StrategyEngine.h`
- `StrategyEngine.cpp`

New file to add to every project folder:

- `ProgramVariant.h`

For the final submission folders, use the corresponding `ProgramVariant.h` from:

- `variants/robot_A_offence/ProgramVariant.h`
- `variants/robot_A_defence/ProgramVariant.h`
- `variants/robot_B_offence/ProgramVariant.h`
- `variants/robot_B_defence/ProgramVariant.h`
- `variants/robot_A_offence_challenge_level/ProgramVariant.h`
- `variants/robot_B_defence_challenge_level/ProgramVariant.h`

Known-profile folders skip ID dance and identify our robot by color profile:

- robot A = GR = green front / red rear
- robot B = OB = orange front / blue rear

Challenge folders use `PROFILE_AUTO` and enable ID dance, so the robot is identified by motion signature.

In Visual Studio, add `ProgramVariant.h` to the project if it does not appear automatically. Header filters do not affect build, but the file must be physically present in the project folder.
