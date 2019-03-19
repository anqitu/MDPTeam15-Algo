from Utils.utils import *

unexplored_coors = {}
y, x = get_matrix_coords(self._robot.center)
for i in range(NUM_ROWS):
    for j in range(NUM_COLS):
        if self._robot.discovered_map[i][j] == 2:
            unexplored_coors[(i, j)] = abs(y - i) + abs(x - j)
return sorted(unexplored_coors.items(), key=lambda kv: kv[1])


unexplored_coors[(4, 2)] = 1
unexplored_coors = sorted(unexplored_coors.items(), key=lambda kv: kv[1])
y, x = unexplored_coors[0][0]
