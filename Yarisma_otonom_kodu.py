from dronekit import connect, VehicleMode,LocationGlobalRelative
import time,math,serial
from geopy import distance
import numpy as np
import RPi.GPIO as GPIO

connection_string = "/dev/ttyACM0"
iha = connect(connection_string,wait_ready=True, timeout=100, baud=57600)
copter_horizontal_velocity=iha.parameters.get("WPNAV_SPEED")
copter_vertical_velocity=iha.parameters.get("WPNAV_SPEED_UP")

GPIO.setmode(GPIO.BOARD)
GPIO.setup(33, GPIO.OUT)

font=1.5
yazi=input("Lutfen bir kelime giriniz: ")
takeoff=5
sayac=0
iki_harf_arasi_bosluk=0.5
yaw=iha.heading
aci=yaw+90
if(aci>360):
    aci=aci-360

def nozzle_on():
    GPIO.output(33,GPIO.HIGH)
    print("nozzle açık")
def nozzle_off():
    GPIO.output(33,GPIO.LOW)
    print("nozzle kapalı")

def arm_ol_ve_yuksel(hedef_yukseklik):
    iha.mode = VehicleMode("GUIDED")
    while iha.mode != 'GUIDED':
        print('Guided moduna gecis yapiliyor')
        time.sleep(1.5)

    print("Guided moduna gecis yapildi")
    iha.armed = True
    time.sleep(3)

    print("Ihamiz arm olmustur")
    iha.simple_takeoff(hedef_yukseklik)
    while iha.location.global_relative_frame.alt <= hedef_yukseklik * 0.9:
        print("Su anki yukseklik{}".format(iha.location.global_relative_frame.alt))
        time.sleep(0.5)
    print("Takeoff gerceklesti")

def belli_noktadan_uzaklik(coord,uzaklik, direction):
    belli_noktadan_uzaklik = distance.distance(meters=uzaklik).destination((coord),bearing=direction)
    return belli_noktadan_uzaklik

def iki_nokta_arasi_uzaklik_hesaplama_2d(coord_1,coord_2):

    distance_2d = distance.distance(coord_1[:2], coord_2[:2]).m
    #print("2D - " +str(distance_2d))
    return distance_2d

def iki_nokta_arasi_uzaklik_hesaplama_3d(coord_1,coord_2):
    distance_3d = np.sqrt(iki_nokta_arasi_uzaklik_hesaplama_2d(coord_1,coord_2)**2 + (coord_1[2] - coord_2[2])**2)
    #print("3D - "+str(distance_3d))
    return distance_3d

konum_array=[[iha.location.global_relative_frame.lat,iha.location.global_relative_frame.lon,takeoff,0]]
def harf_ciz(array,sayac,nozzle):
        test_array = []
        if(array[0]!=0 and array[1]==0):
            konum_yeni=belli_noktadan_uzaklik( (konum_array[sayac][0],konum_array[sayac][1]),array[0],aci)
            test_array.append(konum_yeni.latitude)
            test_array.append(konum_yeni.longitude)
            test_array.append(konum_array[sayac][2])
            test_array.append(nozzle)
        elif(array[0]==0 and array[1]!=0):
            test_array.append(konum_array[sayac][0])
            test_array.append(konum_array[sayac][1])
            test_array.append(konum_array[sayac][2]-array[1])
            test_array.append(nozzle)
        elif(array[0]!=0 and array[1]!=0):
            konum_yeni=belli_noktadan_uzaklik((konum_array[sayac][0],konum_array[sayac][1]),array[0],aci)
            test_array.append(konum_yeni.latitude)
            test_array.append(konum_yeni.longitude)
            test_array.append(konum_array[sayac][2]-array[1])
            test_array.append(nozzle)
        konum_array.append(test_array)

position_array_A = [[iki_harf_arasi_bosluk,0, 0],
                    [0, font, 0],
                    [font / 2, -font, 1],
                    [font / 2, font, 1],
                    [-font * 3 / 4, -font / 2, 0],
                    [font / 2, 0, 1],
                    [font / 4,-font/2, 0]]

position_array_B = [[iki_harf_arasi_bosluk,0, 0],
                    [font / 2, 0, 1],
                    [0, font, 1],
                    [-font / 2, 0, 1],
                    [0, -font, 1],
                    [0, font/2, 0],
                    [font / 2, 0, 1],
                    [0, -font/2, 0]]

position_array_C = [[iki_harf_arasi_bosluk,0, 0],
                    [font / 2, 0, 0],
                    [-font / 2, 0, 1],
                    [0, font, 1],
                    [font / 2, 0, 1],
                    [0, -font, 0]]

position_array_D = [[iki_harf_arasi_bosluk,0, 0],
                    [font / 2, 0, 0],
                    [-font / 2, 0, 1],
                    [0, font, 1],
                    [font / 2, 0, 1],
                    [0, -font, 1]]

position_array_E = [[iki_harf_arasi_bosluk,0, 0],
                    [0, font, 1],
                    [font / 2, 0, 1],
                    [0, -font/2, 0],
                    [-font / 2, 0, 1],
                    [0, -font / 2, 0],
                    [font / 2, 0, 1]]

position_array_F = [[iki_harf_arasi_bosluk, 0, 0],
                    [0, font, 1],
                    [0, -font / 2, 0],
                    [font / 4, 0,1],
                    [-font / 4, -font/2, 0],
                    [font / 2, 0, 1]]

position_array_G = [[iki_harf_arasi_bosluk,0, 0],
                    [font / 2, 0, 0],
                    [-font / 2, 0, 1],
                    [0, font, 1],
                    [font / 2, 0, 1],
                    [0, -font / 2, 1],
                    [-font / 4, 0, 0],
                    [font / 4, -font / 2, 0]]

position_array_H = [[iki_harf_arasi_bosluk,0, 0],
                    [0, font, 1],
                    [0, -font/2, 0],
                    [font / 2, 0, 1],
                    [0, font/2, 0],
                    [0, -font, 1]]

position_array_I = [[iki_harf_arasi_bosluk,0, 0],
                    [0, font, 1],
                    [0, -font, 0]]

position_array_İ = [[iki_harf_arasi_bosluk,0, 0],
                    [0, font, 1],
                    [0, -font* 10/8, 0],
                    [0, font * 1/8, 1],
                    [0, font * 1/8, 0]]

position_array_J = [[iki_harf_arasi_bosluk,0, 0],
                    [0, font*3/4, 0],
                    [0, font*1/4, 1],
                    [font / 2, 0, 1],
                    [0, -font, 1]]

position_array_K = [[iki_harf_arasi_bosluk, 0, 0],
                    [0, font, 1],
                    [font/2, 0, 0],
                    [-font / 2, -font / 2, 1],
                    [font / 2, -font / 2, 1]]

position_array_L = [[iki_harf_arasi_bosluk,0, 0],
                    [0, font, 1],
                    [font / 2, 0, 1]]

position_array_M = [[iki_harf_arasi_bosluk, 0, 0],
                    [0, font, 0],
                    [0, -font, 1],
                    [font*3/8, font*5/8, 1],
                    [font*3/8, -font*5/8, 1],
                    [0, font, 1],
                    [0, -font, 0]]

position_array_N = [[iki_harf_arasi_bosluk, 0, 0],
                    [0, font, 0],
                    [0, -font, 1],
                    [font/2, font, 1],
                    [0, -font, 1]]

position_array_O = [[iki_harf_arasi_bosluk,0, 0],
                    [0, font, 1],
                    [font/2, 0, 1],
                    [0, -font, 1],
                    [-font/2, 0, 1],
                    [font/2, 0, 0]]

position_array_P = [[iki_harf_arasi_bosluk,0, 0],
                    [0, font, 0],
                    [0, -font, 1],
                    [font / 2, 0, 1],
                    [0, font/2, 1],
                    [-font / 2, 0, 1],
                    [font / 2, -font/2, 0]]

position_array_R = [[iki_harf_arasi_bosluk,0, 0],
                    [0, font, 0],
                    [0, -font, 1],
                    [font / 2, 0, 1],
                    [0, font/2, 1],
                    [-font / 2, 0, 1],
                    [font / 2, font/2, 0],
                    [0, -font, 0]]

position_array_S = [[iki_harf_arasi_bosluk, 0, 0],
                    [0, font, 0],
                    [font/2, 0, 1],
                    [0, -font / 2, 1],
                    [-font / 2, 0, 1],
                    [0,  -font / 2, 1],
                    [font/2, 0,  1]]

position_array_T = [[iki_harf_arasi_bosluk,0, 0],
                    [font, 0, 1],
                    [-font / 2, 0, 0],
                    [0,font,1],
                    [font / 2,-font, 0]]

position_array_U = [[iki_harf_arasi_bosluk,0, 0],
                    [0, font, 1],
                    [font, 0, 1],
                    [0, -font, 1]]


position_array_V = [[iki_harf_arasi_bosluk,0, 0],
                    [font*3/8, font, 1],
                    [font*3/8,-font, 1]]

position_array_Y = [[iki_harf_arasi_bosluk,0, 0],
                    [font/4, font/2, 1],
                    [0,font/2, 1],
                    [0,-font/2,0],
                    [font/4,-font/2,1]]

position_array_Z = [[iki_harf_arasi_bosluk,0, 0],
                    [font/2, 0, 1],
                    [-font/2,font, 1],
                    [font/2,0,1],
                    [0,-font,0]]

position_array_0 = [[iki_harf_arasi_bosluk,0, 0],
                    [0, font, 1],
                    [font/2, 0, 1],
                    [0, -font, 1],
                    [-font/2, 0, 1],
                    [font/2, 0, 0]]

position_array_1 = [[iki_harf_arasi_bosluk,0, 0],
                    [-font/4, font/4, 1],
                    [font/4,font/4, 0],
                    [0,font,1],
                    [0,-font,0]]

position_array_2 = [[iki_harf_arasi_bosluk,0, 0],
                    [font/2, 0, 1],
                    [0,font/2, 1],
                    [-font/2,0,1],
                    [0,font/2,1],
                    [font/2,0,1],
                    [0,-font,0]]

position_array_3 = [[iki_harf_arasi_bosluk,0, 0],
                    [font/2, 0, 1],
                    [0,font/2, 1],
                    [-font/2,0,1],
                    [0,font/2,0],
                    [font/2,0,1],
                    [0,-font,1]]

position_array_4 = [[iki_harf_arasi_bosluk,0, 0],
                    [-font/2, font/2, 1],
                    [font/2,0, 1],
                    [0,font/2,0],
                    [0,-font,1]]

position_array_5 = [[iki_harf_arasi_bosluk,0, 0],
                    [0, -font, 0],
                    [font/2,0, 1],
                    [0,-font/2,1],
                    [-font/2,0,1],
                    [0,-font/2,1],
                    [font/2,0,1]]

position_array_6 = [[iki_harf_arasi_bosluk,0, 0],
                    [0, font, 1],
                    [font/2,0, 1],
                    [0,-font/2,1],
                    [-font/2,0,1],
                    [0,-font/2,0],
                    [font/2,0,1]]

position_array_7 = [[iki_harf_arasi_bosluk,0, 0],
                    [font/2, 0, 1],
                    [0,font, 1],
                    [0,-font,0]]

position_array_8 = [[iki_harf_arasi_bosluk,0, 0],
                    [0, font, 1],
                    [font/2,0, 1],
                    [0,-font/2,1],
                    [-font/2,0,1],
                    [font/2,0,0],
                    [0,-font/2,1],
                    [-font/2,0,1],
                    [font/2,0,0]]

position_array_9 = [[iki_harf_arasi_bosluk,0, 0],
                    [font/2, 0, 1],
                    [0,font, 1],
                    [0,-font/2,0],
                    [-font/2,0,1],
                    [0,-font/2,1],
                    [font/2,0,0]]


for i in yazi:
    if i == "A" or i == "a":
        for i in position_array_A:
            harf_ciz(i,sayac,i[2])
            sayac+=1
    if i == "B" or i == "b":
        for i in position_array_B:
            harf_ciz(i,sayac,i[2])
            sayac+=1
    if i == "C" or i == "c":
        for i in position_array_C:
            harf_ciz(i,sayac,i[2])
            sayac+=1
    if i == "D" or i == "d":
        for i in position_array_D:
            harf_ciz(i,sayac,i[2])
            sayac+=1
    if i == "E" or i == "e":
        for i in position_array_E:
            harf_ciz(i,sayac,i[2])
            sayac+=1
    if i == "F" or i == "f":
        for i in position_array_F:
            harf_ciz(i,sayac,i[2])
            sayac+=1
    if i == "G" or i == "g":
        for i in position_array_G:
            harf_ciz(i,sayac,i[2])
            sayac+=1
    if i == "H" or i == "h":
        for i in position_array_H:
            harf_ciz(i,sayac,i[2])
            sayac+=1
    if i == "I" or i == "ı":
        for i in position_array_I:
            harf_ciz(i,sayac,i[2])
            sayac+=1
    if i == "İ" or i == "i":
        for i in position_array_İ:
            harf_ciz(i,sayac,i[2])
            sayac+=1
    if i == "J" or i == "j":
        for i in position_array_J:
            harf_ciz(i,sayac,i[2])
            sayac+=1
    if i == "K" or i == "k":
        for i in position_array_K:
            harf_ciz(i,sayac,i[2])
            sayac+=1
    if i == "L" or i == "l":
        for i in position_array_L:
            harf_ciz(i,sayac,i[2])
            sayac+=1
    if i == "M" or i == "m":
        for i in position_array_M:
            harf_ciz(i,sayac,i[2])
            sayac+=1
    if i == "N" or i == "n":
        for i in position_array_N:
            harf_ciz(i,sayac,i[2])
            sayac+=1
    if i == "O" or i == "o":
        for i in position_array_O:
            harf_ciz(i,sayac,i[2])
            sayac+=1
    if i == "P" or i == "p":
        for i in position_array_P:
            harf_ciz(i,sayac,i[2])
            sayac+=1
    if i == "R" or i == "r":
        for i in position_array_R:
            harf_ciz(i,sayac,i[2])
            sayac+=1
    if i == "S" or i == "s":
        for i in position_array_S:
            harf_ciz(i,sayac,i[2])
            sayac += 1
    if i == "T" or i == "t":
        for i in position_array_T:
            harf_ciz(i,sayac,i[2])
            sayac += 1
    if i == "U" or i == "u":
        for i in position_array_U:
            harf_ciz(i,sayac,i[2])
            sayac += 1
    if i == "V" or i == "v":
        for i in position_array_V:
            harf_ciz(i,sayac,i[2])
            sayac += 1
    if i == "Y" or i == "y":
        for i in position_array_Y:
            harf_ciz(i,sayac,i[2])
            sayac += 1
    if i == "Z" or i == "z":
        for i in position_array_Z:
            harf_ciz(i,sayac,i[2])
            sayac += 1
    if i == "0" :
        for i in position_array_0:
            harf_ciz(i,sayac,i[2])
            sayac += 1
    if i == "1":
        for i in position_array_1:
            harf_ciz(i,sayac,i[2])
            sayac += 1
    if i == "2" :
        for i in position_array_2:
            harf_ciz(i,sayac,i[2])
            sayac += 1
    if i == "3" :
        for i in position_array_3:
            harf_ciz(i,sayac,i[2])
            sayac += 1
    if i == "4" :
        for i in position_array_4:
            harf_ciz(i,sayac,i[2])
            sayac += 1
    if i == "5":
        for i in position_array_5:
            harf_ciz(i,sayac,i[2])
            sayac += 1
    if i == "6":
        for i in position_array_6:
            harf_ciz(i,sayac,i[2])
            sayac += 1
    if i == "7":
        for i in position_array_7:
            harf_ciz(i,sayac,i[2])
            sayac += 1
    if i == "8":
        for i in position_array_8:
            harf_ciz(i,sayac,i[2])
            sayac += 1
    if i == "9":
        for i in position_array_9:
            harf_ciz(i,sayac,i[2])
            sayac += 1

arm_ol_ve_yuksel(takeoff)
print(konum_array)
ekstra_time=2
def konuma_gitme():
    for i in range(1,len(konum_array)):
        mesafe=iki_nokta_arasi_uzaklik_hesaplama_3d( konum_array[i-1],konum_array[i])
        print ("iki konum arasi mesafe: {}".format(mesafe))
        print ("Belirtilen konuma gidiyorum...")
        konum=LocationGlobalRelative(konum_array[i][0],konum_array[i][1],konum_array[i][2])
        if(konum_array[i][3]==1):
            nozzle_on()
        time.sleep(1)
        iha.simple_goto(konum)
        if(konum_array[i][2]==konum_array[i][2]):
            wait_time = math.ceil(mesafe * 100 / copter_horizontal_velocity)
        wait_time = math.ceil(mesafe * 100 / copter_vertical_velocity)
        time.sleep(wait_time)
        nozzle_off()
        time.sleep(ekstra_time)
        print("Belirtilen konuma ulastim.")

konuma_gitme()
iha.mode="LAND"







