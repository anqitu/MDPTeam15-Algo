from Utils.utils import *
import re
num_sensor_readings = 11
regex_str = '^(\d,){%s}$' % (66)
_readings_regex = re.compile(regex_str)
'{"exploreMap":"{}","obstacleMap":"{}"}'.format('ff', '00')
'{"exploreMap":"%s","obstacleMap":"%s"}' % ('ff', '00')
import json
json.loads('{"exploreMap":"%s","obstacleMap":"%s"}' % ('ff', '11'))

data = 'P{"status":"explore done"}P{"status":"explore done"}'.split('P')
data[:] = [x for x in data if x != '']

a = [1]
a.extend(data)
json.loads(a[1])

str.encode('kill', "UTF-8")

from ast import literal_eval
(col, row) = literal_eval('waypoint1,3'[8:])

facing = 0
opposite = (facing + 2) % 4
arrow_map = [[[0, 0, 0, 0] for _ in range(ROW_LENGTH)] for _ in range(COL_LENGTH)]
arrow_map[0][0][0]=1
arrow_map[0][0][2]=1


readings = [int(x) for x in [1,1,1,1,1,1,2,2,2,2,2,2]]
# split readings list into len(sensors)-sized chunks
readings = [readings[i:i + 6] for i in range(0, len(readings), 6)]

list(range(0, len(readings), 6))
