class PIDController:
    def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0, output_limits=(-1.0, 1.0)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_limits = output_limits

        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None
        
    def clamp(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))

    def update(self, error, current_time, pause_time=0.0):
        if self.last_time is None:
            self.last_time = current_time.nanoseconds / 1e9 + pause_time
            return 0.0

        dt = current_time.nanoseconds/1e9 + pause_time - self.last_time
        if dt <= 0.0:
            return 0.0

        self.integral += error * dt
        derivative = (error - self.last_error) / dt

        output = (self.Kp * error +
                  self.Ki * self.integral +
                  self.Kd * derivative)

        output = self.clamp(output, self.output_limits[0], self.output_limits[1])

        self.last_error = error
        self.last_time = current_time.nanoseconds/1e9 + pause_time

        return output
