# After each build, write one flash-at-0x0 image (bootloader + partitions + app) next to firmware.bin:
#   .pio/build/<env>/cyd-touch-finder-<env>.bin
# so anyone can flash it with esptool / a web flasher without PlatformIO.
import os

Import("env")


def merge_bin(source, target, env):
    build = env.subst("$BUILD_DIR")
    out = os.path.join(build, "cyd-touch-finder-%s.bin" % env["PIOENV"])
    parts = []
    for addr, path in env.get("FLASH_EXTRA_IMAGES", []):
        parts += [env.subst(addr), env.subst(path)]
    parts += [env.subst("$ESP32_APP_OFFSET"), str(target[0])]
    esptool = os.path.join(env.PioPlatform().get_package_dir("tool-esptoolpy"), "esptool.py")
    cmd = ['"$PYTHONEXE"', '"%s"' % esptool, "--chip", "esp32", "merge_bin", "-o", '"%s"' % out,
           "--flash_mode", "dio", "--flash_size", "4MB"] + ['"%s"' % p for p in parts]
    env.Execute(" ".join(cmd))


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", merge_bin)
