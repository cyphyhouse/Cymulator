#!/usr/bin/env python3


# TODO: Subject to later change to adapt to log files
def parse(filename):
    f = open(filename, "r")
    lines = f.readlines()
    path = []
    for x in lines:
        data = x.split(' ')
        point1 = (float(data[1][:-1]), float(data[2][:-1]))
        point2 = (float(data[4][:-1]), float(data[5][:-1]))
        path.append((point1, point2))

    f.close()
    return path

# def parse(filename):
#     f = open(filename, "r")
#     lines = f.readlines()
#     path = []
#     for x in lines:
#         data = x.split(' ')
#         waypoint = (float(data[0][:-1]), float(data[1][:-1]), float(data[2][:-1]))
#         for i in range(3, len(data)):
#             point = (float(data[i]), float[data[i+1]], float(data[i+2]))
#         point1 = (float(data[3][:-1]), float(data[2][:-1]))
#         point2 = (float(data[4][:-1]), float(data[5][:-1]))
#         path.append((point1, point2))
#
#     f.close()
#     return path


if __name__ == "__main__":
    parse("log3.txt")
