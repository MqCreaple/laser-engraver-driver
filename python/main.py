import serial
import serial.tools.list_ports
import time
import re
import string

def preprocess(path: str) -> list[str]:
    supported_commands = re.compile(r"[MmLHhVvCcSsQqTtZz]")
    for c in string.ascii_letters:
        path = path.replace(c, "\n" + c + " ")   # Isolate every command into a new line
    path = path.replace(",", " ")
    whitespace = re.compile(r"[^\S\r\n]+")
    path = whitespace.sub(" ", path)                  # replace multiple whitespace with one
    commands = list(
        map(lambda s: s.strip() + "\n", filter(lambda s: s != "", path.splitlines()))
    )  # remove empty lines
    for i, command in enumerate(commands):
        if supported_commands.match(command) == None:
            print(f"Unsupported command at line {i + 1}: \"{command[:-1]}\"")
            return []
    return commands

def main():
    ports = [port.name for port in serial.tools.list_ports.comports()]
    print("Ports: ", ports)
    print("Please input the port you want to use:")
    comport = input()
    ser = serial.Serial(comport, 9600, timeout=1)
    print("Started serial at port " + comport)
    
    while True:
        if ser.in_waiting:
            data = ser.readline()
            print("Received: \"", data.decode().strip(), "\"")
            break
        else:
            time.sleep(0.01)
    if b"Homing completed" not in data:
        print("Homing failed. Abort.")
        return
    
    print("Homing completed")

    line = input("Please type a scaling factor (default: 1.0): ")
    scale = 1.0
    try:
        scale = float(line)
    except:
        scale = 1.0
    ser.write(f"scale {scale}\n".encode("utf-8"))
    print("Set scaling factor to ", scale)

    print("Press ENTER when you're ready to draw.")
    input()
    print("Start drawing the path.")

    with open("command-list.txt", "r") as file:
        commands = preprocess(file.read())
        for line in commands:
            print("Executing command \"" + line.strip() + "\"")
            ser.write((line).encode("utf-8"))

            # Read the "Finished command" feedback
            while True:
                if ser.in_waiting:
                    data = ser.readline()
                    print("Received: \"", data.decode().strip(), "\"")
                    break
                else:
                    time.sleep(0.01)
            if b"Finished command" not in data:
                print("Command failed. Abort.")
                return
            print("Command finished successfully.")
    ser.write(b"M 0 0")   # Move back to home
    ser.close()

if __name__ == "__main__":
    main()
