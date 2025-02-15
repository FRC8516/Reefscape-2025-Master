# REEFSCAPE v2025.3

FRC 8516 Wired Up

## Description

Drive Train: swerve drivetrain that uses Swerve Drive Specialties MK4 L3 Billet.
- drivetrain composed of four MK4 L3 Billet Modules, each configured with two Karken X60, CTRE Can Encoder Through Bore Encoder as the absolute turning encoder.

Actuators: Falcon 500, using Pigeon 2.
-Motion Magic for routines for intake and shooter positions.

## Prerequisites
* WPI 2025.3.1
* Phoenix 6 v25.3.1 - Adds features that are required for swerve
* Phoenix 6 v25.3.1 -- Includes APIs for firmware features ^Changed to Phoenix 6 api
* Pathplanner v25.2.3

## Configuration

It is possible that this project will not work for your robot right out of the box. Various things like the CAN IDs, PIDF gains, chassis configuration, etc. must be determined for your own robot!

These values can be adjusted in the `Constants.java` file.
Motor configuration is in the 'CalibrationSettings.java' file.

## Updated During Season
