from math import sin, cos, tan, asin, acos, atan, pi

for i in range(65):
    input = i / 64.0
    if input >= 0.0 and input <= 1.0:
        r = asin(input) / (2 * pi)
        print(f"fixed<256, uint8_t>({r:.3f}f),")
    else:
        print("ARRRRRRGHHHH!")
