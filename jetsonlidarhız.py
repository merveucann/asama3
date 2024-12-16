# -*- coding: utf-8 -*-
"""
Created on Wed Dec 11 22:28:56 2024

@author: merve
"""

import cv2
import numpy as np
import smbus2       #I2C haberleşmesi için kullanılan kütüphane
import time 
import threading


def gstreamer_pipeline(capture_width=640, capture_height=480, framerate=30, flip_method=0):  #Jetson platformunda GStreamer uzerinden kamera akışı sağlar
    return (
        "nvarguscamerasrc ! "          #Kamera modülünden görüntü yakalar
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "  #Kameradan gelen görüntü verisinin özelliklerini tanımlar.
        "nvvidconv flip-method=%d ! "  #Goruntuyu dondurur veya çevirir
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "  #goruntunun çözünürlüğünü ayarlar ve veriyi BGR formatında işler
        "videoconvert ! "              #video formatı dönüştürücüsü(örnğin renk formatı)
        "video/x-raw, format=(string)BGR ! appsink"       #Ham video verisinin formatını ve renk düzenini tanımlar.appsink ile veri uygulamaya(python) aktarılır
        % (
            capture_width,   #video çozunurluğu ayarlanır
            capture_height,  #video çozunurluğu ayarlanır
            framerate,       #video karesinin saniyedeki sayısını belirtir.(fps)
            flip_method,     #görüntünün döndürülmesi veya çevrilmesi
            capture_width,
            capture_height,  #bu fonksiyon goruntuyu BGR formatında döndürür
        )
    )
   
pipeline = gstreamer_pipeline()    #fonksiyondan alınan pipeline komutunu saklar
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER) #2.parametre OpenCV'ye GStreamer kullanarak video kaynağına erişmesi gerektiğini söyler.


LIDAR_ADDR = 0x62      #LİDAR cihazının I2C adresi(varsayılan:0x62)
bus = smbus2.SMBus(1)  

redlower = (0, 100, 100)  #Kırmızı renk için HSV renk uzayında alt ve üst eşik değerleri.
redupper = (10, 255, 255)
redlower2 = (160, 100, 100)
redupper2 = (180, 255, 255)

bluelower = (90, 120, 50) #mavi normalde tek aralıktadır ancak koyu ve açık mavi tespiti icin 2 renk araligi aliyoruz
blueupper = (110, 255, 255) 
bluelower2 = (110, 150, 50)
blueupper2 = (140, 255, 255)



def lidar_mesafe():
    bus.write_byte_data(LIDAR_ADDR, 0x00, 0x04)  #LIDAR'a ölçüm başlat komutu gönderir.
    time.sleep(0.02)                             #LIDAR ölçüm yapması için beklneir(20ms)
    high = bus.read_byte_data(LIDAR_ADDR, 0x0f)  #LIDAR'dan gelen ölçüm verisinin yüksek ve düşük baytlarını okur.
    low = bus.read_byte_data(LIDAR_ADDR, 0x10)
    distance_mm = (high << 8) + low              #İki baytı birleştirerek mm cinsinden mesafeyi hesaplar.
    return distance_mm/100                       #m'e cevirdim
def hiz_hesaplama():
    prev_dist = lidar_mesafe()   #ilk mesafe degeri
    prev_time = time.time()      #ilk zaman degeri aldık

    while True:
        curr_dist = lidar_mesafe()   #lidar sensorunden anlık mesafe degerini alir 
        curr_time = time.time()      #lidar sensorunden anlik zaman degisimini alir
        delta_dist = curr_dist - prev_dist #mesafe farkı alınıyor
        delta_time = curr_time - prev_time #zaman farki alınıyor

        if delta_time > 0:
            hiz = delta_dist / delta_time  #mesafe farkini zaman farkina bolerek hızı buluyoruz.metre cinsinden
            print(f"Mesafe: {curr_dist} m | Hız: {hiz:.2f} m/s")

        prev_dist, prev_time = curr_dist, curr_time
        time.sleep(0.1)        #dongunun her iterasyonunda 0.1 saniyelik bir bekleme süresi ekleyerek CPU kullanımını azaltır
def nesne_hareketi():
    fps_start_time = 0
    fps = 0
    while True:
        ret, frame = cap.read()
        fps_end_time = time.time()
        time_diff = fps_end_time - fps_start_time
        fps = 1/(time_diff)
        fps_start_time = fps_end_time
        fps_text = "FPS: {:.2f}".format(fps)
        
        if ret:
            blurred = cv2.GaussianBlur(frame, (3, 3), 0)    #Görüntüyü bulanıklaştırır,detaylar azaltılır.bu sayede küçük gürültülerin etkisini azaltır.
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)  #Görüntüyü BGR renk uzayından HSV renk uzayına çevirir. HSV, renk algılamada daha kararlıdır.
            
            mask_red1 = cv2.inRange(hsv, redlower, redupper)    #Kırmızı renk aralıklarına uyan pikselleri bulur. Maske, uygun pikselleri beyaz (255), diğerlerini siyah (0) yapar.
            mask_red2 = cv2.inRange(hsv, redlower2, redupper2)
            mask_red = mask_red1 | mask_red2                        #iki maskeyi birlestirir
            mask_red = cv2.erode(mask_red, None, iterations=2)      #Maskeyi küçültür, küçük gürültüleri yok eder.islem 2 kez tekrarlanir.None varsayılan bir çekirdek (kernel) 3x3
            dusman = cv2.bitwise_and(frame, frame, mask=mask_red)   #Maskeyi görüntüye uygular, sadece kırmızı renkli alanları çıkartır.Sonuç olarak, görüntüde yalnızca maskeye uyan bölgeler gösterilir.

            mask_blue1 = cv2.inRange(hsv, bluelower, blueupper)     #Mavi renkler icin maskeleme islemi yapiyoruz
            mask_blue2 = cv2.inRange(hsv, bluelower2, blueupper2)
            mask_blue = mask_blue1 | mask_blue2                     #İki maskeyi birlestirdim
            mask_blue = cv2.erode(mask_blue, None, iterations=2)
            dost = cv2.bitwise_and(frame, frame, mask=mask_blue)    #Bitwise ile birlestirdigim maskeyi dosta esitliyorum
            
            
            cv2.imshow("Dusman", dusman)
            cv2.imshow("Dost", dost)

            (contours_red, _) = cv2.findContours(mask_red.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  #Maskedeki beyaz alanların dış hatlarını (kontur) bulur.
            (contours_blue, _) = cv2.findContours(mask_blue.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours_red) > 0: #eger konturum kırmızı icinse
                c = max(contours_red, key=cv2.contourArea)
                rect = cv2.minAreaRect(c)
                ((x, y), (width, height), rotation) = rect
                box = np.int32(cv2.boxPoints(rect))

                M = cv2.moments(c)
                if M["m00"] != 0:
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)
                    cv2.circle(frame, center, 5, (0, 255, 0), -1)
                    cv2.putText(frame, "Dusman", (int(x - width / 2), int(y - height / 2) - 10), cv2.FONT_HERSHEY_COMPLEX, 0.8, (0, 255, 0), 2)

            if len(contours_blue) > 0:
                c = max(contours_blue, key=cv2.contourArea)
                rect = cv2.minAreaRect(c)
                ((x, y), (width, height), rotation) = rect
                s="x:{}, y:{}, width:{}, height:{}, rotation:{}".format(np.round(x), np.round(y), np.round(width), np.round(height), np.round(rotation))
                box = np.int32(cv2.boxPoints(rect))

                M = cv2.moments(c)
                if M["m00"] != 0:
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    
                    cv2.drawContours(frame, [box], 0, (255, 0, 0), 2)  
                    cv2.circle(frame, center, 5, (255, 0, 0), -1)
                    cv2.putText(frame, "Dost", (int(x - width / 2), int(y - height / 2) - 10), cv2.FONT_HERSHEY_COMPLEX, 0.8, (255, 0, 0), 2)          
                    
            cv2.imshow("Orijinal", frame)
            if cv2.waitKey(1)==ord("q"):
                break
            cap.release()
            cv2.destroyAllWindows()

def main():#iki fonksiyonu paralel olarak islemek icin main fonksiyonunu kullanıyorum.
       nesne_thread=threading.Thread(target=nesne_hareketi)
       lidar_thread=threading.Thread(target=hiz_hesaplama)
       
       nesne_thread.start()
       lidar_thread.start()
       
       nesne_thread.join()
       lidar_thread.join()
       
try :
    main()
except KeyboardInterrupt:
    print("program sonlandirildi")
    bus.close()