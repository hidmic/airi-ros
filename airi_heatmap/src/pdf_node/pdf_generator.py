#!/usr/bin/env python

import os
import subprocess
import rospy
from std_msgs.msg import String
from utils import log, pdf_generator
from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker

DB_PATH = os.environ('DB_PATH')
Base = declarative_base()

def ros_pdf_generator():
    #Change echo to True to troubleshoot
    engine = create_engine(DB_PATH, echo=False)
    Session = sessionmaker(bind=engine)
    session = Session()
    rospy.init_node('pdf_node', anonymous=True)

    def pdf_generator_callback(data):
        pdf_generator(session)

    rospy.Subscriber('pdf_gen', String, pdf_generator_callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        ros_pdf_generator()
    except rospy.ROSInterruptException as e:
        print(e)