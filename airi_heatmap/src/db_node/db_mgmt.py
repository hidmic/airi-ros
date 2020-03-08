#!/usr/bin/env python

import os
import subprocess
import rospy
import tf2_ros
import ast
from std_msgs.msg import String
from utils import log
from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from db_functions import save_measure_in_db
from models import Base

DB_PATH = os.environ('DB_PATH')

def ros_save_measure():

    engine = create_engine(DB_PATH, echo=False)
    engine.execute('PRAGMA foreign_keys = ON')
    Base.metadata.create_all(engine)
    Session = sessionmaker(bind=engine)
    session = Session()
    
    rospy.init_node('db_node', anonymous=True)
    rospy.init_node('tf2_point_listener', anonymous=True)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    def measure_callback (data):
        measure_list = ast.literal_eval(data.data)
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
