from main import *

if __name__ == "__main__":
    with open("command-list.txt", "r") as file:
        commands = preprocess(file.read())
        print(commands)