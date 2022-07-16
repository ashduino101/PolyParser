import subprocess
import os

if __name__ == "__main__":
    subprocess.run(["cmake", "cmakelinux/CMakeLists.txt"])
    subprocess.run(["make"])
    os.remove("CMakeCache.txt")  # prevent cmake from caching

    subprocess.run(["cmake", "cmakewin/CMakeLists.txt"])
    subprocess.run(["make"])
    os.remove("CMakeCache.txt")
