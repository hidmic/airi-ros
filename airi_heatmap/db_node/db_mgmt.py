#!/usr/bin/env python

import subprocess
import rospy
from std_msgs.msg import String
from utils import log
from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from db_functions import save_measure_in_db
from models import Base

#db_path = 'sqlite:///'
db_path = 'sqlite:////Users/juanignaciobattaglino/Documents/UTN2019/ProyectoFinal/proyectoheatmap.db'
measure_list = []

def measure_callback (data):
    measure_list = data.data

def ros_save_measure():
    engine = create_engine(db_path, echo=False)
    engine.execute('PRAGMA foreign_keys = ON')
    Base.metadata.create_all(engine)
    Session = sessionmaker(bind=engine)
    session = Session()
    rospy.init_node('db', anonymous=True)

    #while aca?
    rospy.Subscriber('measure_db', String, measure_callback)

    # Para recibir point me deberia suscribir a un topico y enviar timestamp del punto.
    save_measure_in_db(session, measure_list, point)
    rospy.spin()


if __name__ == "__main__":
    try:
        ros_save_measure()
    except rospy.RlsOSInterruptException as e:
        print(e)
