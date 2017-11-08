import dweepy
import os
import csv
import time
from copy import deepcopy
import datetime

filename = os.getcwd() + "/Station_Data.csv"
with open(filename, 'ab') as file:
    writer = csv.writer(file, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)

    order = ['UV_index', 'PM01', 'CO2', 'Temperature', 'PM10', 'IR', 'Visibility', 'Humidity', 'Organic_vapor', 'PM25']

    new_feed = []
    for i in range(7):
        try:
            newl = dweepy.get_latest_dweet_for('CrestResearchData-' + str(i))
        except:
            newl = None
        new_feed += [newl]
    while True:
        last_feed = deepcopy(new_feed)
        new_feed = []
        for i in range(7):
            try:
                newl = dweepy.get_latest_dweet_for('CrestResearchData-' + str(i))
            except:
                newl = None
            new_feed += [newl]    
        for i in range(7):
            if (new_feed[i] is not None) and (last_feed[i] is None or new_feed[i][0]['created'] != last_feed[i][0]['created']):
                    content = new_feed[i][0]['content']
                    station = new_feed[i][0]['thing']
                    time_rec = str(datetime.datetime.now())
                    writer.writerow([time_rec, station] + [content[sensor_read] for sensor_read in order])
                    file.flush()
        time.sleep(2)
