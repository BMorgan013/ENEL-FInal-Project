#######################################
# UNIVERSITY OF REGINA FACULTY OF APPLIED SCIENCE & ENGINEERING
# Title: Group 5 - Automated Greenhouse System Capstone
#######################################

#libraries
from flask import Flask
from flask import render_template, redirect, url_for, request,jsonify, flash

import sys
import pymysql
from datetime import datetime
import time
import serial
from time import gmtime, strftime


#Configuration Values for AWS RDS MySQL, username and password left blank
endpoint = 'enggcap.c1hu9mfpb5qx.us-east-1.rds.amazonaws.com'
username = ''
password = ''
database_name = 'plantAttributes'
port = '3306'

conn = pymysql.connect(
        host = endpoint,
        user = username,
        password = password,
        db = database_name,
        )

app = Flask(__name__)
app.debug = True

#takes to index, asking user what type of plant they are planting
@app.route("/")
def index():
	return render_template("plantChoice.html")

#takes user to information page, grabs all queries from the tables on the database
@app.route("/choice", methods = ["POST","GET"])
def choice():
	userPlantChoice = request.form
	plantChoice = userPlantChoice['plantchoice']
	curs=conn.cursor()
	curs.execute("SET time_zone='Canada/Saskatchewan'")
	curs.execute("INSERT INTO plantchoice (plant) VALUES (%s)", (plantChoice))
	conn.commit()

	#MySQL queries, pulling info from the cloud, showing information to the web interface
	curs.execute("SELECT date_format(rDateTime,'%Y-%m-%d %H:%i') AS dateToString, temp FROM temperatures")
	temperatures = curs.fetchall()
	curs.execute("SELECT date_format(rDateTime, '%Y-%m-%d %H:%i') AS dateToString, hum FROM humidities")
	humidities = curs.fetchall()
	curs.execute("SELECT date_format(rDateTime, '%Y-%m-%d %H:%i') AS dateToString, pH FROM pH")
	ph = curs.fetchall()
	curs.execute("SELECT date_format(rDateTime, '%Y-%m-%d %H:%i') AS dateToString, EC FROM EC")
	ec = curs.fetchall()
	curs.execute("SELECT date_format(rDateTime, '%Y-%m-%d %H:%i') AS dateToString, moist FROM moisture")
	moist = curs.fetchall()
	curs.execute("SELECT date_format(rDateTime, '%Y-%m-%d %H:%i') AS dateToString, lux FROM lux")
	lux = curs.fetchall()
	return render_template("lab_env_db.html",lux=lux, lux_items=len(lux),ph=ph,ph_items=len(ph),ec=ec,ec_items=len(ec),moist=moist,moist_items=len(moist),temp=temperatures,hum=humidities, plantChoice=plantChoice,temp_items=len(temperatures), hum_items=len(humidities))

#date range used to filter by date, but does not update google graphs
@app.route("/range", methods= ['GET', 'POST'])
def range():
	curs=conn.cursor()

	if request.method == 'POST':
	  From = request.form['From']
	  to = request.form['to']
	  print(From)
	  print(to)

	  curs.execute("SELECT * FROM temperatures WHERE rDateTime between (%s) AND (%s)",(From, to))
	  temperaturesRange = curs.fetchall()

	  curs.execute("SELECT * FROM humidities WHERE rDateTime between (%s)  AND (%s)", (From, to))
	  humiditiesRange = curs.fetchall()
	  curs.execute("SELECT * FROM pH WHERE rDateTime between (%s)  AND (%s)", (From, to))
	  phRange = curs.fetchall()
	  curs.execute("SELECT * FROM EC WHERE rDateTime between (%s)  AND (%s)", (From, to))
	  ecRange = curs.fetchall()
	  curs.execute("SELECT * FROM moisture WHERE rDateTime between (%s)  AND (%s)", (From, to))
	  moistRange = curs.fetchall()
	  curs.execute("SELECT * FROM lux WHERE rDateTime between (%s)  AND (%s)", (From, to))
	  luxRange = curs.fetchall()

	  return jsonify({'htmlresponse': render_template("response.html", tempRange=temperaturesRange, humRange=humiditiesRange, phRange=phRange, ecRange=ecRange, moistRange=moistRange, luxRange=luxRange)})

#main landing page used for testing
@app.route("/lab_env_db")
def lab_env_db():

	curs=conn.cursor()
	curs.execute("SELECT * FROM temperatures")
	temperatures = curs.fetchall()
	curs.execute("SELECT * FROM humidities")
	humidities = curs.fetchall()

	return render_template("lab_env_db.html",temp=temperatures,hum=humidities)

if __name__ == "__main__":
	app.run(host='0.0.0.0', port=8080)

