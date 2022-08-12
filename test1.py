from geopy import distance
import numpy as np

def belli_noktadan_uzaklik(coord,uzaklik, direction):
    #uzaklik = 1         #Metre
    belli_noktadan_uzaklik = distance.distance(meters=uzaklik).destination((coord),bearing=direction)  #0 – North, 90 – East, 180 – South, 270 or -90 – West
    return belli_noktadan_uzaklik

belli_noktadan_uzaklik((41.129564, 28.997433), 2, 0)

print(belli_noktadan_uzaklik((41.129564, 28.997433), 2.65, 0))