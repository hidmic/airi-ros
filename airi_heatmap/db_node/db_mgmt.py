#!/usr/bin/env python

import subprocess
import rospy
import tf2_ros
from std_msgs.msg import String
from utils import log
from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from db_functions import save_measure_in_db
from models import Base

#db_path = 'sqlite:///'
db_path = 'sqlite:////Users/juanignaciobattaglino/Documents/UTN2019/ProyectoFinal/proyectoheatmap.db'

def ros_save_measure():

    engine = create_engine(db_path, echo=False)
    engine.execute('PRAGMA foreign_keys = ON')
    Base.metadata.create_all(engine)
    Session = sessionmaker(bind=engine)
    session = Session()
    
    rospy.init_node('db', anonymous=True)
    rospy.init_node('tf2_point_listener', anonymous=True)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    def measure_callback (data):
        measure_list = data.data #Needs to be converted from string to list of lists
        trans = tfBuffer.lookup_transform(, rospy.Time()) #?
        point = (trans.transform.translation.x, trans.transform.translation.y)
        save_measure_in_db(session, measure_list, point)

    rospy.Subscriber('measure_db', String, measure_callback)

    # Para recibir point me deberia suscribir a un topico y enviar timestamp del punto.
    
    rospy.spin()


if __name__ == "__main__":
    try:
        ros_save_measure()
    except rospy.RlsOSInterruptException as e:
        print(e)
