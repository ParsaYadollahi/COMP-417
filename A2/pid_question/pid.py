class PIDController:
    def __init__(self, target_pos):
        self.target_pos = target_pos
        self.Kp = 0.0
        self.Ki = 0.0
        self.Kd = 0.0
        self.bias = 0.0
        self.error = 0.0
        self.integral = 0.0
        self.old_position = 0.0
        return

    def reset(self):
        self.Kp = 0.0
        self.Ki = 0.0
        self.Kd = 0.0
        self.bias = 0.0
        self.integral = 0.0
        self.error = 0.0
        return

# TODO: Complete your PID control within this function. At the moment, it holds
#      only the bias. Your final solution must use the error between the
#      target_pos and the ball position, plus the PID gains. You cannot
#      use the bias in your final answer.
    def get_fan_rpm(self, vertical_ball_position):
        output = self.bias

        ratio = 1.0/60.0

        self.error = self.target_pos - vertical_ball_position
        self.integral = self.integral+self.error*ratio

        p = self.Kp * self.error
        i = self.Ki * self.integral
        d = self.Kd * self.old_position-vertical_ball_position / ratio

        output = p+i+d
        self.old_position = vertical_ball_position

        return output
