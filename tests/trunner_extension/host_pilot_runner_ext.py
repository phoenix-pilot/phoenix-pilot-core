from trunner.target import HostPCGenericTarget
from trunner.host import EmulatorHost


class HostPilotTarget(HostPCGenericTarget):
    name = "host-generic-pilot"


def register_extension():
    return {
        "target": HostPilotTarget,
        "host": [EmulatorHost],
    }
