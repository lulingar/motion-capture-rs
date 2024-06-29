#!/usr/bin/env python3

import sys
import time
from pathlib import Path

import pexpect

# Define the picocom command and arguments
# Adjust the device path as necessary
cmd = ["picocom", "-q", "-b", "115200", "/dev/ttyACM0"]


def main():
    assert len(sys.argv) == 2 and sys.argv[1] != "", "Must specify target output file!"
    out_file = Path(sys.argv[1])

    print(f"Remember to run this script in unbuffered mode (python -u {sys.argv[0]} target_file)")

    sample_time = 10
    samples = []
    while True:
        try:
            input(f"Press a key to capture a {sample_time}-second sample: ")
            samples.append(capture_lines(sample_time))
        except KeyboardInterrupt:
            break

    if samples:
        # Save the output to a file
        print(f"\n\nWill now save {len(samples)} collected samples")
        # FIXME remove picocom lines
        with out_file.open("w") as file:
            file.write('\n'.join(samples))
        print(f"Output has been saved to {out_file}")
    else:
        print("No samples collected, exiting.")


def capture_lines(sample_time_secs: int) -> str:
    # Start picocom with pexpect
    child = pexpect.spawn(" ".join(cmd), encoding='utf-8')

    # Send Ctrl-A followed by Ctrl-P to soft-reset the device
    child.sendcontrol('a')
    child.sendcontrol('p')

    # Wait for the given seconds to gather output,
    # plus one extra for the device reset time
    for elapsed in range(sample_time_secs + 1):
        time.sleep(1)
        print(f"{elapsed+1}..", end="")
    print("")

    # Send Ctrl-A followed by Ctrl-X to exit picocom
    child.sendcontrol('a')
    child.sendcontrol('x')

    # Wait for picocom to terminate after sending the exit sequence
    try:
        child.expect(pexpect.EOF, timeout=2)
    except expect.ExceptionPexpect as e:
        print("Abnormal termination!")
    
    # Capture all the output up to this point
    output = child.before
    return output

if __name__ == "__main__":
    main()
