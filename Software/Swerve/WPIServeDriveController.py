import math
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState, SwerveDrive4Kinematics
from wpimath.geometry import Rotation2d, Translation2d

class SwerveDriveController:
    """
    Controller class for a four-wheel swerve drive system.
    Handles joystick inputs and controls the swerve drive motors.
    """
    
    def __init__(self, front_left_module, front_right_module, back_left_module, back_right_module, 
                 gyro=None, max_speed=0.5, max_angular_speed=0.5 * math.pi):
        """
        Initialize the swerve drive controller.
        
        Args:
            front_left_module: The front left swerve module
            front_right_module: The front right swerve module
            back_left_module: The back left swerve module
            back_right_module: The back right swerve module
            gyro: Optional gyroscope for field-relative driving
            max_speed: Maximum speed in meters per second
            max_angular_speed: Maximum angular speed in radians per second
        """
        # Module locations relative to the robot center (in meters)
        # TODO: Get these values from the robot's CAD model or physical measurements
        self.front_left_location = Translation2d(0.381, 0.381) # Placeholders from documentation
        self.front_right_location = Translation2d(0.381, -0.381)
        self.back_left_location = Translation2d(-0.381, 0.381)
        self.back_right_location = Translation2d(-0.381, -0.381)
        
        # Create kinematics object
        self.kinematics = SwerveDrive4Kinematics(
            self.front_left_location, self.front_right_location, 
            self.back_left_location, self.back_right_location
        )
        
        # Store the swerve modules
        self.front_left_module = front_left_module
        self.front_right_module = front_right_module
        self.back_left_module = back_left_module
        self.back_right_module = back_right_module
        
        # Store the gyro for field-relative driving
        self.gyro = gyro
        
        # Configuration
        self.max_speed = max_speed  # Max speed in m/s
        self.max_angular_speed = max_angular_speed  # Max angular speed in rad/s
        
        # Field-relative driving flag
        self.field_relative = True if gyro else False
    
    def handle_input(self, joystick1_mag, joystick1_angle, joystick2_mag, joystick2_angle):
        """
        Handle joystick input in polar form to drive the robot.
        
        Args:
            joystick1_mag: Magnitude of the first joystick [0.0..1.0]
            joystick1_angle: Angle of the first joystick in radians
            joystick2_mag: Magnitude of the second joystick [0.0..1.0]
            joystick2_angle: Angle of the second joystick in radians
        """
        # Apply deadband to magnitudes to avoid small unwanted movements
        joystick1_mag = self._apply_deadband(joystick1_mag)
        joystick2_mag = self._apply_deadband(joystick2_mag)
        
        if joystick1_mag == 0 and joystick2_mag == 0:
            # If both joysticks are within the deadband, stop the robot
            self.stop()
            return
        
        # Convert joystick1 polar coordinates to robot-relative speeds
        # Forward velocity is along X-axis, lateral velocity is along Y-axis
        x_speed = joystick1_mag * math.cos(joystick1_angle) * self.max_speed
        y_speed = joystick1_mag * math.sin(joystick1_angle) * self.max_speed
        
        # Joystick2 controls rotation, typically using X-axis component
        rot_speed = joystick2_mag * math.cos(joystick2_angle) * self.max_angular_speed
        
        # Drive the robot with the calculated speeds
        self.drive(x_speed, y_speed, rot_speed)
    
    def drive(self, x_speed, y_speed, rot_speed, field_relative=None):
        """
        Drive the robot using the specified speeds.
        
        Args:
            x_speed: Speed along the X axis in m/s
            y_speed: Speed along the Y axis in m/s
            rot_speed: Rotational speed in rad/s
            field_relative: Whether to use field-relative driving (overrides instance setting)
        """
        # Determine whether to use field-relative control
        use_field_relative = self.field_relative if field_relative is None else field_relative
        
        # Create chassis speeds object
        if use_field_relative and self.gyro:
            # For field-relative driving, we need the gyro angle
            gyro_angle = Rotation2d.fromDegrees(self.gyro.getAngle())
            chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                x_speed, y_speed, rot_speed, gyro_angle
            )
        else:
            # For robot-relative driving
            chassis_speeds = ChassisSpeeds(x_speed, y_speed, rot_speed)
        
        # Calculate module states
        module_states = self.kinematics.toSwerveModuleStates(chassis_speeds)
        
        # Normalize wheel speeds if any exceed the maximum speed
        SwerveDrive4Kinematics.desaturateWheelSpeeds(module_states, self.max_speed)
        
        # Set the states to each module
        self._set_module_states(module_states)
    
    def _set_module_states(self, module_states):
        """
        Set the states for each swerve module.
        
        Args:
            module_states: Array of SwerveModuleState objects
        """
        front_left_state, front_right_state, back_left_state, back_right_state = module_states
        
        # Optimize each module state to minimize wheel rotation
        
        # Front Left
        front_left_optimized = SwerveModuleState.optimize(
            front_left_state, 
            Rotation2d(self.front_left_module.get_angle())
        )
        self.front_left_module.set_desired_state(front_left_optimized)
        
        # Front Right
        front_right_optimized = SwerveModuleState.optimize(
            front_right_state, 
            Rotation2d(self.front_right_module.get_angle())
        )
        self.front_right_module.set_desired_state(front_right_optimized)
        
        # Back Left
        back_left_optimized = SwerveModuleState.optimize(
            back_left_state, 
            Rotation2d(self.back_left_module.get_angle())
        )
        self.back_left_module.set_desired_state(back_left_optimized)
        
        # Back Right
        back_right_optimized = SwerveModuleState.optimize(
            back_right_state, 
            Rotation2d(self.back_right_module.get_angle())
        )
        self.back_right_module.set_desired_state(back_right_optimized)

        # TODO: Send the speeds to the microcontrollers
        # self.back_right_module.speedMetersPerSecond
        # self.back_right_module.angle.getRadians() / (2 * math.PI)
    
    def _apply_deadband(self, value, deadband=0.1):
        """
        Apply a deadband to a value to eliminate small joystick movements.
        
        Args:
            value: The value to apply the deadband to
            deadband: The deadband to apply
            
        Returns:
            The value with the deadband applied
        """
        if abs(value) < deadband:
            return 0.0
        
        # Scale the value to maintain the full range
        return (value - math.copysign(deadband, value)) / (1.0 - deadband)
    
    def stop(self):
        """Stop all modules by setting speeds to zero."""
        self.drive(0, 0, 0)
    
    def set_field_relative(self, field_relative):
        """
        Set whether to use field-relative driving.
        
        Args:
            field_relative: Whether to use field-relative driving
        """
        if field_relative and not self.gyro:
            print("Warning: Cannot enable field-relative mode without a gyro")
            return
        
        self.field_relative = field_relative
    
    def reset_gyro(self):
        """Reset the gyro to zero."""
        if self.gyro:
            self.gyro.reset()
        else:
            print("Warning: Cannot reset gyro because no gyro is connected")
