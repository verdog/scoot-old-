import math
import time
from util import *

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket


class Scoot(BaseAgent):

    def initialize_agent(self):
        #This runs once before the bot starts up
        self.controller_state = SimpleControllerState()

        self.me = obj()
        self.opponent = obj()
        self.ball = obj()

        self.time_elapsed = 0
        self.time_delta = 0
        self.time_start = time.time()

        self.plan = "atba"

    def preprocess(self, packet: GameTickPacket):
        for i in range(packet.num_cars):
            newcar = packet.game_cars[i]
            newcar.location = packet.game_cars[i].physics.location
            newcar.rotation = packet.game_cars[i].physics.rotation
            newcar.velocity = packet.game_cars[i].physics.velocity
            newcar.angular_velocity = packet.game_cars[i].physics.angular_velocity
            newcar.goals = packet.game_cars[i].score_info.goals
            newcar.own_goals = packet.game_cars[i].score_info.own_goals

            if packet.game_cars[i].team == 1:
                newcar.team_modifier = 1
            else:
                newcar.team_modifier = -1

            if i == self.index:
                self.me = newcar
            else:
                self.opponent = newcar

        self.ball.location = packet.game_ball.physics.location
        self.ball.rotation = packet.game_ball.physics.rotation
        self.ball.velocity = packet.game_ball.physics.velocity
        self.ball.angular_velocity = packet.game_ball.physics.angular_velocity

        self.time_delta = time.time() - self.time_start
        self.time_elapsed += self.time_delta
        self.time_start = time.time()

        self.score_diff = self.me.goals + self.opponent.own_goals - self.opponent.goals - self.me.own_goals

    def think(self):
        # choose what the current plan is

        if self.ball.location.x == 0 and self.ball.location.y == 0:
            self.plan = "attack"
            return

        if self.score_diff <= 0:
            self.plan = "attack"
        else:
            self.plan = "defend"

    def restart_clock(self):
        self.time_start = time.time()

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        self.preprocess(packet)
        self.think()

        if self.plan == "atba":
            atba_tick(self)
        elif self.plan == "attack":
            push_towards_goal_tick(self)
        elif self.plan == "defend":
            defend_tick(self)

        return self.controller_state

class Vector2:
    def __init__(self, x=0, y=0):
        self.x = float(x)
        self.y = float(y)

    def __add__(self, val):
        return Vector2(self.x + val.x, self.y + val.y)

    def __sub__(self, val):
        return Vector2(self.x - val.x, self.y - val.y)

    def correction_to(self, ideal):
        # The in-game axes are left handed, so use -x
        current_in_radians = math.atan2(self.y, -self.x)
        ideal_in_radians = math.atan2(ideal.y, -ideal.x)

        correction = ideal_in_radians - current_in_radians

        # Make sure we go the 'short way'
        if abs(correction) > math.pi:
            if correction < 0:
                correction += 2 * math.pi
            else:
                correction -= 2 * math.pi

        return correction

    def normalized(self):
        mag = self.magnitude()
        return Vector2(self.x/mag, self.y/mag)

    def magnitude(self):
        return math.sqrt(self.x**2 + self.y**2)

def get_car_facing_vector(car):
    pitch = float(car.me.rotation.pitch)
    yaw = float(car.me.rotation.yaw)

    facing_x = math.cos(pitch) * math.cos(yaw)
    facing_y = math.cos(pitch) * math.sin(yaw)

    return Vector2(facing_x, facing_y)

def atba_tick(agent):
    # print("atba!")
    steer_trigger = math.pi/50
    handbrake_trigger = 2 * math.pi/5
    target_location = to_local(agent.ball, agent.me)

    angle = math.atan2(target_location.data[1], target_location.data[0])

    handbrake = 0
    if math.fabs(angle) > handbrake_trigger:
        handbrake = 1

    turn = 0
    if math.fabs(angle) > steer_trigger:
        if angle > 0:
            # Positive radians in the unit circle is a turn to the left.
            turn = 1.0  # Negative value for a turn to the left.
        else:
            turn = -1.0

    if turn == 0 and handbrake == 0:
        agent.controller_state.boost = 1

    agent.controller_state.throttle = 1.0
    agent.controller_state.steer = turn
    agent.controller_state.handbrake = handbrake

    return agent.controller_state

def push_towards_goal_tick(agent):
    # print("attack?")

    ball_y_diff = (agent.ball.location.y - agent.me.location.y) * agent.me.team_modifier
    if (ball_y_diff <= -500):
        return atba_tick(agent)
    else:
        return get_behind_ball_tick(agent)

def get_behind_ball_tick(agent, attack = True):
    # print("get behind ball!")
    steer_trigger = math.pi/50
    handbrake_trigger = 2 * math.pi/5
    ball_location = Vector2(agent.ball.location.x, agent.ball.location.y)

    goal_to_ball = ball_location - Vector2(0, 5120 * -1 * agent.me.team_modifier)
    goal_to_ball_dir = goal_to_ball.normalized()

    target_location = ball_location + Vector2(goal_to_ball_dir.x * 500, goal_to_ball_dir.y * 500)

    car_location = Vector2(agent.me.location.x, agent.me.location.y)
    car_direction = get_car_facing_vector(agent)
    car_to_ball = target_location - car_location

    steer_correction_radians = car_direction.correction_to(car_to_ball)

    handbrake = 0
    if math.fabs(steer_correction_radians) > handbrake_trigger:
        handbrake = 1

    turn = 0
    if math.fabs(steer_correction_radians) > steer_trigger:
        if steer_correction_radians > 0:
            # Positive radians in the unit circle is a turn to the left.
            turn = -1.0  # Negative value for a turn to the left.
        else:
            turn = 1.0

    if turn == 0 and handbrake == 0:
        agent.controller_state.boost = 1

    agent.controller_state.throttle = 1.0
    agent.controller_state.steer = turn
    agent.controller_state.handbrake = handbrake

    return agent.controller_state

def defend_tick(agent):
    # print("defend.")

    ball_location = Vector2(agent.ball.location.x, agent.ball.location.y)
    ball_distance_from_my_goal = math.fabs(ball_location.y - 5120 * agent.me.team_modifier)

    if ball_distance_from_my_goal < 3 * 5120 / 4:
        # try to push it away
        # print("offensive defense.")
        return push_towards_goal_tick(agent)
    else:
        return go_to_own_goal(agent)
        
def go_to_own_goal(agent):
    # print("go to own goal")
    # print("get behind ball!")
    steer_trigger = math.pi/10
    handbrake_trigger = 2 * math.pi/5

    target_location = Vector2(0, 5120 * agent.me.team_modifier)

    car_location = Vector2(agent.me.location.x, agent.me.location.y)
    car_direction = get_car_facing_vector(agent)
    car_to_ball = target_location - car_location

    steer_correction_radians = car_direction.correction_to(car_to_ball)

    handbrake = 0
    if math.fabs(steer_correction_radians) > handbrake_trigger:
        handbrake = 1

    turn = 0
    if math.fabs(steer_correction_radians) > steer_trigger:
        if steer_correction_radians > 0:
            # Positive radians in the unit circle is a turn to the left.
            turn = -1.0  # Negative value for a turn to the left.
        else:
            turn = 1.0

    if turn == 0 and handbrake == 0:
        agent.controller_state.boost = 1

    if (target_location - car_location).magnitude() <= 800:
        agent.controller_state.throttle = 0
        agent.controller_state.boost = 0
        agent.controller_state.steer = turn
        agent.controller_state.handbrake = handbrake
    else:
        agent.controller_state.boost = 0
        agent.controller_state.throttle = 1.0
        agent.controller_state.steer = turn
        agent.controller_state.handbrake = handbrake

    return agent.controller_state