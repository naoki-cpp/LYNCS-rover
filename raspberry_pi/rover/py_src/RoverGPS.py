import micropyGPS
import serial


def gps_measurement():
    my_gps = micropyGPS.MicropyGPS(9, 'dd')
    s = serial.Serial('/dev/serial0', 9600, timeout=10)
    s.readline()
    while my_gps.parsed_sentences < 5:
        sentence = s.readline().decode('utf-8') # GPSデーターを読み、文字列に変換する
        if sentence[0] == '$':
            for x in sentence: # 読んだ文字列を解析してGPSオブジェクトにデーターを追加、更新する
                my_gps.update(x)
    return [my_gps.latitude[0], my_gps.longitude[0]]