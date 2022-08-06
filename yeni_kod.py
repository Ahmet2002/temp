from dronekit import connect, VehicleMode,LocationGlobalRelative
import time,math,serial
from geopy import distance
import numpy as np
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(33,GPIO.OUT)

connection_string = "/dev/ttyUSB0"
iha = connect(connection_string,wait_ready=True, timeout=100, baud=57600)
copter_horizontal_velocity=iha.parameters.get("WPNAV_SPEED")
copter_vertical_velocity=iha.parameters.get("WPNAV_SPEED_UP")

font=4
yazi="TEK"
takeoff=6
sayac=0
iki_harf_arasi_bosluk=2

def nozzle_on(nozzle_status):
    if(nozzle_status==1):
        GPIO.output(33,GPIO.HIGH)
    if(nozzle_status==0):
        GPIO.outp4(33,GPIO.LOW)
def nozzle_off():
    GPIO.output(33,GPIO.LOW)
    GPIO.cleanup()

def arm_ol_ve_yuksel(hedef_yukseklik):
    iha.mode = VehicleMode("GUIDED")
    while iha.mode != 'GUIDED':
        print('Guided moduna gecis yapiliyor')
        time.sleep(1.5)

    print("Guided moduna gecis yapildi")
    iha.armed = True
    while iha.armed is False:
        print("Arm icin bekleniliyor")
        time.sleep(1)

    print("Ihamiz arm olmustur")

    iha.simple_takeoff(hedef_yukseklik)
    while iha.location.global_relative_frame.alt <= hedef_yukseklik * 0.99:
        print("Su anki yukseklik{}".format(iha.location.global_relative_frame.alt))
        time.sleep(0.5)
    print("Takeoff gerceklesti")



def belli_noktadan_uzaklik(coord,uzaklik, direction):
    #uzaklik = 1         #Metre
    belli_noktadan_uzaklik = distance.distance(meters=uzaklik).destination((coord),bearing=direction)  #0 – North, 90 – East, 180 – South, 270 or -90 – West
    return belli_noktadan_uzaklik

def iki_nokta_arasi_uzaklik_hesaplama_2d(coord_1,coord_2):

    distance_2d = distance.distance(coord_1[:2], coord_2[:2]).m
    #print("2D - " +str(distance_2d))
    return distance_2d

def iki_nokta_arasi_uzaklik_hesaplama_3d(coord_1,coord_2):
    distance_3d = np.sqrt(iki_nokta_arasi_uzaklik_hesaplama_2d(coord_1,coord_2)**2 + (coord_1[2] - coord_2[2])**2)
    #print("3D - "+str(distance_3d))
    return distance_3d

def get_distance_metres(aLocation1, aLocation2):  #konumun LocationGlobalRelative icinde gelmesi gerekiyor.
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

konum_array=[[iha.location.global_relative_frame.lat,iha.location.global_relative_frame.lon,takeoff]]#iha.location.global_relative_frame.lat,iha.location.global_relative_frame.lon

def harf_ciz(array,sayac):
        test_array = []
        if(array[0]!=0 and array[1]==0):
            lidar_calistir()
            konum_yeni=belli_noktadan_uzaklik( (konum_array[sayac][0],konum_array[sayac][1]),array[0],0)
            test_array.append(konum_yeni.latitude)
            test_array.append(konum_yeni.longitude)
            test_array.append(konum_array[sayac][2])
        elif(array[0]==0 and array[1]!=0):
            lidar_calistir()
            test_array.append(konum_array[sayac][0])
            test_array.append(konum_array[sayac][1])
            test_array.append(konum_array[sayac][2]-array[1])
        elif(array[0]!=0 and array[1]!=0):
            lidar_calistir()
            konum_yeni=belli_noktadan_uzaklik((konum_array[sayac][0],konum_array[sayac][1]),array[0],0)
            test_array.append(konum_yeni.latitude)
            test_array.append(konum_yeni.longitude)
            test_array.append(konum_array[sayac][2]-array[1])
        konum_array.append(test_array)

position_array_T = [[iki_harf_arasi_bosluk,0, 0],
                    [font, 0, 1],
                    [-font / 2, 0, 0],
                    [0,font,1],
                    [font / 2,-font, 0]]

position_array_E = [[iki_harf_arasi_bosluk,0, 0],
                    [0, font, 1],
                    [font / 2, 0, 1],
                    [0, -font/2, 0],
                    [-font / 2, 0, 1],
                    [0, -font / 2, 0],
                    [font / 2, 0, 1]]

position_array_K = [[iki_harf_arasi_bosluk, 0, 0],
                    [0, font, 1],
                    [font/2, 0, 0],
                    [-font / 2, -font / 2, 1],
                    [font / 2, -font / 2, 1]]

position_array_N = [[iki_harf_arasi_bosluk, 0, 0],
                    [0, font, 1],
                    [0, -font, 0],
                    [font/2, font, 1],
                    [0, -font, 1]]

position_array_O = [[iki_harf_arasi_bosluk,0, 0],
                    [0, font, 1],
                    [font/2, 0, 1],
                    [0, -font, 1],
                    [-font/2, 0, 1],
                    [font/2, 0, 0]]

position_array_F = [[iki_harf_arasi_bosluk, 0, 0],
                    [0, font, 1],
                    [0, -font / 2, 0],
                    [font / 4, 0,1],
                    [-font / 4, -font/2, 0],
                    [font / 2, 0, 1]]

position_array_S = [[iki_harf_arasi_bosluk, 0, 0],
                    [0, font, 0],
                    [font/2, 0, 1],
                    [0, -font / 2, 1],
                    [-font / 2, 0, 1],
                    [0,  -font / 2, 1],
                    [font/2, 0,  1]]

ser = serial.Serial("/dev/ttyUSB0", 115200)
def read_data():
    counter = ser.in_waiting # count the number of bytes of the serial port
    if counter > 8:
        bytes_serial = ser.read(9)
        ser.reset_input_buffer()
        if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59: # this portion is for python3
            print("Printing python3 portion")
            distance = bytes_serial[2] + bytes_serial[3]*256 # multiplied by 256, because the binary data is shifted by 8 to the left (equivalent to "<< 8").                                              # Dist_L, could simply be added resulting in 16-bit data of Dist_Total.
            strength = bytes_serial[4] + bytes_serial[5]*256
            temperature = bytes_serial[6] + bytes_serial[7]*256
            temperature = (temperature/8) - 256
            print("Distance:"+ str(distance))
            print("Strength:" + str(strength))
            if temperature != 0:
                print("Temperature:" + str(temperature))
            ser.reset_input_buffer()

        if bytes_serial[0] == "Y" and bytes_serial[1] == "Y":
            distL = int(bytes_serial[2].encode("hex"), 16)
            distH = int(bytes_serial[3].encode("hex"), 16)
            stL = int(bytes_serial[4].encode("hex"), 16)
            stH = int(bytes_serial[5].encode("hex"), 16)
            distance = distL + distH*256
            strength = stL + stH*256
            tempL = int(bytes_serial[6].encode("hex"), 16)
            tempH = int(bytes_serial[7].encode("hex"), 16)
            temperature = tempL + tempH*256
            temperature = (temperature/8) - 256
            print("Printing python2 portion")
            print("Distance:"+ str(distance) + "\n")
            print("Strength:" + str(strength) + "\n")
            print("Temperature:" + str(temperature) + "\n")
            ser.reset_input_buffer()

def lidar_calistir():
    if __name__ == "__main__":
        try:
            if ser.isOpen() == False:
                ser.open()
            read_data()
        except KeyboardInterrupt(): # ctrl + c in terminal.
            if ser != None:
                ser.close()
                print("program interrupted by the user")

for i in yazi:
    if i == "T" or i == "t":
        for i in position_array_T:
            nozzle_on(i[2])
            harf_ciz(i,sayac)
            sayac+=1
            nozzle_off()
    if i == "E" or i == "e":
        for i in position_array_E:
            nozzle_on(i[2])
            harf_ciz(i,sayac)
            sayac += 1
            nozzle_off()
    if i == "K" or i == "k":
        for i in position_array_K:
            nozzle_on(i[2])
            harf_ciz(i,sayac)
            sayac += 1
            nozzle_off()
    if i == "N" or i == "n":
        for i in position_array_N:
            nozzle_on(i[2])
            harf_ciz(i,sayac)
            sayac += 1
            nozzle_off()
    if i == "O" or i == "o" or i == "0":
        for i in position_array_O:
            nozzle_on(i[2])
            harf_ciz(i,sayac)
            sayac += 1
            nozzle_off()
    if i == "F" or i == "f":
        for i in position_array_F:
            nozzle_on(i[2])
            harf_ciz(i,sayac)
            sayac += 1
            nozzle_off()
    if i == "S" or i == "s":
        for i in position_array_S:
            nozzle_on(i[2])
            harf_ciz(i,sayac)
            sayac += 1
            nozzle_off()
arm_ol_ve_yuksel(takeoff)
print(konum_array)
def konuma_gitme():
    for i in range(1,len(konum_array)):
        mesafe=iki_nokta_arasi_uzaklik_hesaplama_3d( konum_array[i-1],konum_array[i])
        print ("iki konum arasi mesafe: {}".format(mesafe))
        print ("Belirtilen konuma gidiyorum...")
        konum=LocationGlobalRelative(konum_array[i][0],konum_array[i][1],konum_array[i][2])
        iha.simple_goto(konum)
        if(konum_array[i][2]==konum_array[i][2]):
            wait_time = math.ceil(mesafe * 100 / copter_horizontal_velocity) + 3
        wait_time = math.ceil(mesafe * 100 / copter_vertical_velocity) + 3
        time.sleep(wait_time)
        print("Belirtilen konuma ulastim.")

konuma_gitme()
iha.mode="RTL"






