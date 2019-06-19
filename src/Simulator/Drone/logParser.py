#!/usr/bin/env python3


def parse():
    f = open("log3.txt", "r")
    lines = f.readlines()
    path = []
    for x in lines:
        data = x.split(' ')
        point1 = (float(data[1][:-1]), float(data[2][:-1]))
        point2 = (float(data[4][:-1]), float(data[5][:-1]))
        path.append((point1, point2))

    f.close()
    return path


if __name__ == "__main__":
    parse()