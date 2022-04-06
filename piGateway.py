import pymysql
import time
import serial
from datetime import datetime
#Configuration Values, left empty
endpoint = ''
username = ''
password = ''
database_name = ''
port = '3306'

conn = pymysql.connect(
	host = endpoint,
	user = username,
	password = password,
	db = database_name,
	)
#used to write to serial port, xbee s2c
ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1            
 )
counter=0

#Grab last entered info from plantChoice table in the db
count=1
while(count<=2):
      curs=conn.cursor()
      curs.execute("SET time_zone='Canada/Saskatchewan'")
      curs.execute("SELECT plant FROM plantchoice ORDER BY rDateTime DESC LIMIT 1")
      plantChoice = curs.fetchall()

      for plant in plantChoice:
        plantString = plant[0]
        print("User chose: ", plantString)
      time.sleep(2)
      count+=1

###############
#read plantChoice, send associated letter to STM32
while 1:
      x=ser.readline().strip()
#      print(x, len(x))
      if plantString == 'tomatoes outdoors':
        ser.write(str.encode('A'))
      elif plantString == 'sunflowers outdoors':
        ser.write(str.encode('B'))
      elif plantString == 'strawberries outdoors':
        ser.write(str.encode('C'))
      elif plantString == 'tomatoes indoors':
        ser.write(str.encode('D'))
      elif plantString == 'sunflowers indoors':
        ser.write(str.encode('E'))
      elif plantString == 'strawberries indoors':
        ser.write(str.encode('F'))
      else:
        print("No info on database")
#Receive data from STM32, count length of string, send to db
      if len(x) == 4:
       print("Humidity: ",x, len(x))
       curs.execute("INSERT INTO humidities (hum) VALUES (%s)",(x))
       conn.commit()
      elif len(x) == 5:
       print("Temperature: ",x, len(x))
       curs.execute("INSERT INTO temperatures (temp) VALUES (%s)", (x))
       conn.commit()
      elif len(x) == 6:
       print("Moisture: ",x, len(x))
       curs.execute("INSERT INTO moisture (moist) VALUES (%s)", (x))
       conn.commit()
      elif len(x) == 7:
       print("pH Val: ",x, len(x))
       curs.execute("INSERT INTO pH (pH) VALUES (%s)", (x))
       conn.commit()
      elif len(x) == 8:
       print("EC Val: ",x, len(x))
       curs.execute("INSERT INTO EC (EC) VALUES (%s)", (x))
       conn.commit()
      elif len(x) >= 9:
       print("Lux Val: ",x, len(x))
       curs.execute("INSERT INTO lux (lux) VALUES (%s)", (x))
       conn.commit()
      else:
       print("-----------")
      time.sleep(4)

