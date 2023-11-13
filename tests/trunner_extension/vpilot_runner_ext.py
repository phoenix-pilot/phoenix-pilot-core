from trunner.ctx import TestContext
from trunner.target.emulated import QemuTarget
from trunner.host import EmulatorHost


class VPilotTarget(QemuTarget):
    name = "armv7a9-zynq7000-vpilot"
    rootfs = True
    shell_prompt = "root@?:~ # "

    def __init__(self):
        super().__init__("armv7a9-zynq7000-vpilot.sh")
        # Start of the zynq target take around 45 seconds due to the slow filesystem initialization.
        # Iterate over harness chain to find a ShellHarness to increase prompt_timeout value.
        self.prompt_timeout = 60

    @classmethod
    def from_context(cls, _: TestContext):
        return cls()


def register_extension():
    return {
        "target": VPilotTarget,
        "host": [EmulatorHost],
    }
