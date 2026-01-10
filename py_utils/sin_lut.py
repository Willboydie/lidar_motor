import math

N = (1 << 11)
values_per_row = 20

for i in range(N + 1):
    x = (i / N) * (math.pi / 2)
    v = int(round(math.sin(x) * 32767))

    end = ",\n" if (i + 1) % values_per_row == 0 else ", "
    print(f"{v}", end=end)

