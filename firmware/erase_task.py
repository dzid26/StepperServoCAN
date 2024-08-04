from SCons.Script import ARGUMENTS
from os.path import join
Import("env")

def do_erase(*args, **kwargs):
    board = env.BoardConfig()
    platform = env.PioPlatform()
    upload_protocol = env.subst("$UPLOAD_PROTOCOL")
    debug_tools = board.get("debug.tools", {})
    openocd_args = [
        join(platform.get_package_dir("tool-openocd"), "bin", "openocd"),
        "-d%d" % (2 if int(ARGUMENTS.get("PIOVERBOSE", 0)) else 1)
    ]
    # openocd_args.extend([
    #     "-c", "set CPUTAPID 0x2ba01477",
    # ])
    openocd_args.extend(
        debug_tools.get(upload_protocol).get("server").get("arguments", []))
    # actual erase command: erase bank 0, first sector to last sector.
    # assumes it's a 1-bank device. some STM32s have dual bank.
    openocd_args.extend([
        "-c",
        "init; halt; flash probe 0;  flash erase_sector 0 0 last; reset; shutdown;"
    ])
    # print("OpenOCD args", openocd_args)
    # string escape every argument
    openocd_args = ['"' + arg + '"' for arg in openocd_args]
    env.Execute(" ".join(openocd_args))

env.AddCustomTarget(
    name="erase",
    dependencies=None,
    actions=[
        do_erase
    ],
    title="Erase Flash",
    description="Erase eeprom (calibrations) and flash (program). Flash new program after this and perform motor calibration"
)