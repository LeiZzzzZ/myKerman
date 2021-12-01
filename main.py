import krpc
import time
import math
import threading
import sys


class MyKerman:
    def __init__(self, name):
        self.name = name
        self.conn = krpc.connect(name=self.name)

    def auto_landing(self, tolerance_coef: float = 1.1):
        """
        Automatically lands the active vessel with a tolerance coef.

        :param tolerance_coef: tolerance of systemic deviation
        """

        print_time(self.name + ' AutoLanding ready. Coefficient of tolerance set to {:.4f}'.format(tolerance_coef))
        vessel = self.conn.space_center.active_vessel
        ctrl = vessel.control
        ap = vessel.auto_pilot
        ap.reference_frame = vessel.orbit.body.reference_frame

        ctrl.sas = False
        ctrl.rcs = False
        ctrl.throttle = 0

        srf_altitude = self.conn.add_stream(getattr, vessel.flight(), 'surface_altitude')
        ver_speed = self.conn.add_stream(getattr, vessel.flight(vessel.orbit.body.reference_frame), 'vertical_speed')
        hor_speed = self.conn.add_stream(getattr, vessel.flight(vessel.orbit.body.reference_frame), 'horizontal_speed')
        vel = self.conn.add_stream(getattr, vessel.flight(vessel.orbit.body.reference_frame), 'velocity')
        g = self.conn.add_stream(getattr, vessel.orbit.body, 'surface_gravity')
        alpha = self.conn.add_stream(getattr, vessel.flight(), 'angle_of_attack')
        beta = self.conn.add_stream(getattr, vessel.flight(), 'sideslip_angle')
        drag = self.conn.add_stream(getattr, vessel.flight(), 'drag')

        tgt_h_1 = 20000
        # tgt_h_2 = 5000
        # hor_mod = 1

        ap.engage()

        while True:
            ap.target_direction = (-vel()[0], -vel()[1], -vel()[2])
            if srf_altitude() < tgt_h_1 * tolerance_coef and ver_speed() < -1:
                break
            time.sleep(1)

        print_time(
            self.name + ' Initiate auto landing at height of {:.2f} with vertical speed of {:.2f} and horizontal speed of {:.2f}'.format(
                srf_altitude(), ver_speed(), hor_speed()))

        while True:
            if hor_speed() < 10:
                ctrl.throttle = 0
                break
            ap.target_direction = (-vel()[0], -vel()[1], -vel()[2])
            time.sleep(0.1)
            throttle = (math.sqrt(hor_speed()) - 3) / math.sqrt(hor_speed())
            if throttle > 1:
                throttle = 1
            elif throttle < 0:
                throttle = 0
            ctrl.throttle = throttle

        print_time(
            self.name + ' Horizontal decelerating finished at height of {:.2f} with vertical speed of {:.2f} and horizontal speed of {:.2f}'.format(
                srf_altitude(), ver_speed(), hor_speed()))

        # Find a proper height to start vertical deceleration
        while True:
            vessel_height = com_adj(vessel)
            tgt_h_2 = (ver_speed() ** 2) * tolerance_coef / (
                        2 * vessel.available_thrust / vessel.mass - 2 * g()) + vessel_height
            ap.target_direction = (-vel()[0], -vel()[1], -vel()[2])
            if srf_altitude() < tgt_h_2:
                break
            time.sleep(0.2)

        print_time(
            self.name + ' Vertical decelerating at height of {:.2f} with vertical speed of {:.2f} and horizontal speed of {:.2f}'.format(
                srf_altitude(), ver_speed(), hor_speed()))

        while True:
            liq_fuel = vessel.resources.amount('LiquidFuel')
            vessel_height = com_adj(vessel)
            hor_mod = math.cos(math.radians(alpha())) * math.cos(math.radians(beta()))
            ap.target_direction = (-vel()[0], -vel()[1], -vel()[2])

            if srf_altitude() < vessel_height + 300 * tolerance_coef:
                ctrl.gear = True
                if srf_altitude() < vessel_height + 0.2 / tolerance_coef:
                    ctrl.throttle = 0
                    break

            if ver_speed() > 0:
                throttle = 0
            elif liq_fuel < 10:
                throttle = 0
                if srf_altitude() > vessel_height + 5 * tolerance_coef:
                    print_time(self.name + ' Insufficient fuel')
                    break
            else:
                # simple physics
                throttle = ((vessel.mass / hor_mod) * (
                            ver_speed() ** 2 / (2 * (srf_altitude() - vessel_height)) + g()) - math.sqrt(
                    drag()[0] ** 2 + drag()[1] ** 2 + drag()[2] ** 2)) / vessel.available_thrust

            if throttle < 0:
                throttle = 0
            elif throttle > 1:
                throttle = 1
            ctrl.throttle = throttle
            time.sleep(0.01)

        ap.disengage()
        print_time(self.name + ' Auto landing completed')
        return

    def dal(self, num: int = 1):
        """
        DECOUPLE & AUTO LANDING: Fire a number of decouplers and automatically lands all decoupled inactive vessels. Current active vessel not included. Auto pilot disengaged. You can start another thread to land this active vessel.

        :param num: the number of decouplers to fire (searched in a reversed order on the parts tree)
        """
        vessel = self.conn.space_center.active_vessel
        vessel.auto_pilot.disengage()
        dec = vessel.parts.decouplers
        dec = dec[-num:]
        new_vessel = []
        try:
            for i in dec:
                new_vessel.append(i.decouple())
            for i in new_vessel:
                thread = threading.Thread(target=self.auto_landing, args=(i, 'decoupled_' + i.name, 1.1))
                thread.start()
        except Exception as e:
            print(e.args)
        return


def print_time(string: str):
    """
    Print str to both console and ./missionLog.txt with current time.
    """
    log_file = open('./missionLog.txt', 'a+', encoding='utf-8')
    log_console = sys.stdout
    sys.stdout = log_file
    print(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())) + ' ' + string)
    sys.stdout = log_console
    print(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())) + ' ' + string)
    return


def print_structure():
    """
    Print the structure of active vessel.
    """
    log_file = open('./missionLog.txt', 'a+', encoding='utf-8')
    log_console = sys.stdout
    conn = krpc.connect(name='_temp_')
    vessel = conn.space_center.active_vessel
    root = vessel.parts.root
    stack = [(root, 0)]
    experiment_parts = []
    print_time('Mission begins')
    sys.stdout = log_file
    print('Structure of ' + vessel.name + ':')
    while stack:
        part, depth = stack.pop()
        if part.experiment:
            experiment_parts.append(part)
        print(' '*depth, part.title)
        for child in part.children:
            stack.append((child, depth+1))
    if not experiment_parts:
        print('No experiment on this vessel')
    else:
        print('Experiment parts on this vessel:')
        for i in experiment_parts:
            print(i.name)
    conn.close()
    sys.stdout = log_console
    return


def launch(sec: int):
    """
    Launch active vessel after seconds.
    """
    conn = krpc.connect(name='Launch')
    vessel = conn.space_center.active_vessel
    ctrl = vessel.control
    if not sec:
        sec = 1
    time.sleep(sec)
    print_time('Launch Countdown')
    time.sleep(1)
    ctrl.sas = False
    ctrl.rcs = False
    ctrl.throttle = 1.0
    print('3...')
    time.sleep(1)
    print('2...')
    time.sleep(1)
    print('1...')
    time.sleep(1)
    print_time('Launch!')
    vessel.control.activate_next_stage()
    conn.close()
    return


def jettison_fairing(vessel):
    """
    Jettison all fairings.
    """
    # jF = vessel.parts.fairings
    # for i in jF:
    #     i.fairing.jettison()
    # printTime(' ' + vessel.name + ' fairings jettisoned')
    # Something is wrong with part.fairing.jettisoned, it always returns True if reverted to launch

    for fairing in vessel.parts.fairings:
        for module in fairing.part.modules:
            if 'Fairing' in module.name:
                # Change this line if not a Chinese version KSP
                # module.trigger_event('Deploy')
                module.trigger_event('抛整流罩')
    return


def com_adj(vessel):
    """
    Calculates the center of mass adjustment of the given vessel. Returns a float of distance between the CoM of the vessel and the first engine in part tree.
    """
    eng = vessel.parts.engines
    box = eng[0].part.bounding_box(vessel.reference_frame)
    dist = abs(box[0][1])
    return dist


if __name__ == '__main__':
    mk = MyKerman('AutoLanding')
    mk.auto_landing()
