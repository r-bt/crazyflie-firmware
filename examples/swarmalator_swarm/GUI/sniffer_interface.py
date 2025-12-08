import threading
import time
from typing import List
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

from colorama import Fore
import cflib.crtp
import zmq
from common import MAX_COPTERS


class Position:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return "Position(x={}, y={}, z={})".format(self.x, self.y, self.z)


class Copter:
    VOLTAGE_MIN = 3.0
    VOLTAGE_MAX = 4.2

    def __init__(self):
        self.state = 255
        self.voltage = 0.0
        self.counter = 0

        self.position = Position()
        self.phase = 0

    def get_voltage(self):
        return self._decompress_voltage(self.voltage)

    @staticmethod
    def _decompress_voltage(voltage):
        return (voltage / 255.0) * (
            Copter.VOLTAGE_MAX - Copter.VOLTAGE_MIN
        ) + Copter.VOLTAGE_MIN


class SnifferInterface:
    REPORT_FREQUENCY = 20  # Hz
    # Request these at 20 Hz (period = 50 ms). Note: increasing this increases
    # radio/log bandwidth. If you run many copters you may hit throughput limits.
    LOG_FREQUENCY = 20  # Hz
    LOG_ACTIONS_FREQUENCY = 20  # Hz

    def __init__(
        self, uri, report_socket: zmq.Socket = None, command_socket: zmq.Socket = None
    ):
        self.uri = uri
        self.copters: List[Copter] = None

        self.cf = Crazyflie(rw_cache="./cache")

        self.cf.fully_connected.add_callback(self._connected)
        self.cf.disconnected.add_callback(self._disconnected)
        self.cf.connection_failed.add_callback(self._connection_failed)
        self.cf.connection_lost.add_callback(self._connection_lost)

        self.connection_successful = None

        self.cf.open_link(uri)

        self._initialize_copters()

        self.report_socket = report_socket
        self.command_socket = command_socket

        self.desired_cfs = 0

        self._log_confs = []

        self._is_experiment_running = False
        self._is_target_set = False

    def _initialize_copters(self) -> List[Copter]:
        copters = []
        for i in range(MAX_COPTERS):
            copters.append(Copter())

        self.copters = copters

    def _connected(self, link_uri):
        print(Fore.GREEN + "Connected to {}".format(link_uri), Fore.RESET)
        self.connection_successful = True
        self._setup_logging()

    def _disconnected(self, link_uri):
        print(Fore.RED + "Disconnected from {}".format(link_uri))
        self.connection_successful = False

    def _connection_failed(self, link_uri, msg):
        print(
            Fore.RED + "Connection to {} failed: {}".format(link_uri, msg), Fore.RESET
        )
        self.connection_successful = False

    def _connection_lost(self, link_uri, msg):
        print(Fore.RED + "Connection to {} lost: {}".format(link_uri, msg))
        self.connection_successful = False

    def _setup_logging(self):
        for i in range(MAX_COPTERS):
            # Use integer milliseconds for the LogConfig period
            period_ms = int(1000 / self.LOG_FREQUENCY)
            log_conf = LogConfig(name="{}-Conf".format(i), period_in_ms=period_ms)

            log_conf.add_variable("id_{}.state".format(i + 1), "uint8_t")
            log_conf.add_variable("id_{}.voltage".format(i + 1), "uint8_t")
            log_conf.add_variable("id_{}.counter".format(i + 1), "uint8_t")
            log_conf.add_variable("id_{}.x".format(i + 1), "float")
            log_conf.add_variable("id_{}.y".format(i + 1), "float")
            log_conf.add_variable("id_{}.z".format(i + 1), "float")
            log_conf.add_variable("id_{}.phase".format(i + 1), "float")

            self.cf.log.add_config(log_conf)
            log_conf.data_received_cb.add_callback(
                lambda *args, i=i: self.log_data(i, *args)
            )
            log_conf.start()

            self._log_confs.append(log_conf)

    def log_data(self, copter, _1, data, _2):
        cop = self.copters[copter]

        try:
            cop.state = data["id_{}.state".format(copter + 1)]
            cop.voltage = data["id_{}.voltage".format(copter + 1)]
            cop.counter = data["id_{}.counter".format(copter + 1)]
            cop.position.x = data["id_{}.x".format(copter + 1)]
            cop.position.y = data["id_{}.y".format(copter + 1)]
            cop.position.z = data["id_{}.z".format(copter + 1)]
            cop.phase = data["id_{}.phase".format(copter + 1)]
        except KeyError as e:
            print(data)
            print(
                Fore.RED
                + "KeyError in log_data for copter {}: {}".format(copter, e)
                + Fore.RESET
            )

    def log_actions(self, timestamp, data, logconf):
        self.desired_cfs = int(data["ds.desired"])

    def send_report(self):
        if self.report_socket is None or self.connection_successful is None:
            return

        report = []
        if not self.connection_successful:
            report.append("connection_failed")
        else:
            for i, cop in enumerate(self.copters):
                data = {
                    "id": i + 1,
                    "state": cop.state,
                    "battery": cop.get_voltage(),
                    "counter": cop.counter,
                    "position": {
                        "x": cop.position.x,
                        "y": cop.position.y,
                        "z": cop.position.z,
                    },
                    "phase": cop.phase,
                }
                report.append(data)

            actions_data = {"id": "action", "desired": self.desired_cfs}

            report.append(actions_data)

        try:
            self.report_socket.send_json(report, zmq.NOBLOCK)
        except Exception as e:
            print(Fore.RED + "Error sending report: {}".format(e), Fore.RESET)

    def monitor(self):
        self.send_report()

        self.check_for_commands()

        time.sleep(1 / self.REPORT_FREQUENCY)

    def check_for_commands(self):
        # Use a poller to efficiently check if a message is available
        poller = zmq.Poller()
        poller.register(self.command_socket, zmq.POLLIN)
        
        # Check if there's a message waiting (non-blocking poll with 0 timeout)
        socks = dict(poller.poll(0))
        
        if self.command_socket in socks and socks[self.command_socket] == zmq.POLLIN:
            try:
                report = self.command_socket.recv_json(flags=zmq.NOBLOCK)
                
                command = report.get("command", None)

                # print(Fore.YELLOW + "Received command: {}".format(command), Fore.RESET)
                
                if command == "toggleIsExperimentRunning":
                    self.toggleIsExperimentRunning()
                elif command == "toggleIsTargetSet":
                    active = report.get("active", False)
                    self.toggleIsTargetSet(active)
                elif command == "updateTargetPosition":
                    x = report["position"]["x"]
                    y = report["position"]["y"]
                    z = report["position"]["z"]

                    self.updateTargetPosition(x, y, z)
                elif command == 'updateTargetAlpha':
                    alpha = report['alpha']
                    self.cf.param.set_value("app.targetAlpha", alpha)
                elif command == "updateSwarmalatorParameters":
                    params = report["parameters"]
                    agentId = report["droneId"]

                    self.updateSwarmalatorParameters(agentId, params)
                    print(Fore.CYAN + "Updated swarmalator parameters for drone {}".format(agentId), Fore.RESET)

            except Exception as e:
                print(Fore.RED + "Error receiving command: {}".format(e), Fore.RESET)

    def disconnect(self):
        self.cf.close_link()

    def more(self):
        self.cf.param.set_value("app.more", 1)

    def less(self):
        self.cf.param.set_value("app.less", 1)

    def toggleIsExperimentRunning(self):
        self._is_experiment_running = not self._is_experiment_running
        self.cf.param.set_value("app.isExperimentRunning", self._is_experiment_running)

    def toggleIsTargetSet(self, active: bool):
        self.cf.param.set_value("app.isTargetSet", active)

    def updateTargetPosition(self, x, y, z):
        self.cf.param.set_value("app.targetX", x)
        self.cf.param.set_value("app.targetY", y)
        self.cf.param.set_value("app.targetZ", z)

    def updateSwarmalatorParameters(self, agentId, params):
        if 'K' in params:
            self.cf.param.set_value("app.{}_K".format(agentId), params['K'])
        if 'J' in params:
            self.cf.param.set_value("app.{}_J".format(agentId), params['J'])
        if 'phase' in params:
            self.cf.param.set_value("app.{}_phase".format(agentId), params['phase'])
        if 'naturalFrequency' in params:
            self.cf.param.set_value("app.{}_naturalFrequency".format(agentId), params['naturalFrequency'])


class snifferThread(threading.Thread):
    def __init__(self, *args, **kwargs):
        super(snifferThread, self).__init__(*args, **kwargs)
        self.daemon = True

        self._stop_thread = threading.Event()

    def stop_sniffer(self):
        self._stop_thread.set()

    def stopped(self):
        return self._stop_thread.is_set()

    def run(self):
        cflib.crtp.init_drivers(enable_debug_driver=False)

        context = zmq.Context()

        pub_socket = context.socket(zmq.PUSH)
        pub_socket.bind("tcp://*:5555")

        sub_socket = context.socket(zmq.PULL)
        # report_socket.connect("tcp://bitcrazeDemo:5556")
        sub_socket.connect("tcp://127.0.0.1:5556")
        sub_socket.setsockopt(zmq.RCVTIMEO, 1000)

        uri = "usb://0"
        sniffer = SnifferInterface(
            uri, report_socket=pub_socket, command_socket=sub_socket
        )

        while True:
            if self.stopped():
                print(Fore.RED + "Sniffer thread stopped", Fore.RESET)
                sniffer.disconnect()
                break

            sniffer.monitor()


def sniffer_interface_main():
    cflib.crtp.init_drivers(enable_debug_driver=False)

    context = zmq.Context()

    pub_socket = context.socket(zmq.PUSH)
    pub_socket.bind("tcp://*:5555")

    sub_socket = context.socket(zmq.PULL)
    # report_socket.connect("tcp://bitcrazeDemo:5556")
    sub_socket.connect("tcp://127.0.0.1:5556")
    sub_socket.setsockopt(zmq.RCVTIMEO, 1000)

    uri = "usb://0"
    sniffer = SnifferInterface(uri, report_socket=pub_socket, command_socket=sub_socket)

    while True:
        sniffer.monitor()


if __name__ == "__main__":
    sniffer_interface_main()
