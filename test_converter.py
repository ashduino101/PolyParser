import os
import subprocess


def test_converter():
    files = os.listdir('layouts')
    files = [f for f in files if f.endswith('.layout')]
    if not os.path.exists("test_json"):
        os.mkdir("test_json")

    for f in files:
        print("Converting " + f)
        subprocess.call(["cmake-build-debug/PolyParser", "-o", "test_json/" + f + ".json", "layouts/" + f])


if __name__ == "__main__":
    test_converter()
