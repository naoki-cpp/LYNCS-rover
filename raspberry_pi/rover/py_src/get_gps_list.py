# -*- coding: utf-8 -*-
#%%
import csv
import rover_module as gps
import  random
import time
import sys
args = sys.argv
"""
Usage
-------
例えばgpsのデータが100個欲しい場合, 
```bash
python3 get_gps_list.py 100
```
を実行するとgpsのデータが100個入ったcsvファイルが作成されます。
"""


data_num = int(args[1])
data_path = 'gps_data.csv'
with open(data_path, mode='w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['時間', '緯度', '経度', '速さ', '方位角', '総数', data_num])
    for i in range(data_num):
        lat_long = gps.lat_long_measurement()
        vel = gps.velocity_measurement()
        if lat_long[0] is not None and lat_long[1] is not None and vel[
                0] is not None and vel[1] is not None:
            writer.writerow([
                time.time(), lat_long[0], lat_long[1], vel[0] * 1852 / 3600,
                vel[1]
            ])
        time.sleep(0.5)
# knot から mへの換算
# 角度の単位は度

with open(data_path) as f:
    print(f.read())
# 0,1,2
# "a,b,c","x","y"
