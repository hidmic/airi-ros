import pyscreenshot as ImageGrab
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import pandas as pd
import csv
import os
import shutil
from pylab import imread, imshow
from mpl_toolkits.axes_grid1 import AxesGrid
from scipy.interpolate import Rbf
from reportlab.platypus import SimpleDocTemplate, Paragraph, PageBreak, Image
from reportlab.lib.units import mm, inch
from reportlab.lib.styles import getSampleStyleSheet
from sqlalchemy import distinct
from models import Ssid, Bssid, Measure, Security, Channel, Point, Draw_Point, Frequency, Rate
import PIL.Image
from PIL import Image as Img

GRID_WIDTH = 797
GRID_HEIGHT = 530
IMAGE_WIDTH = 2544
IMAGE_HEIGHT = 1691
BASEWIDTH = 500

def log(application, log_level, msg):
    print("[{}] - [{}] - {}".format(application, log_level, msg))


def pdf_generator(session):
    if os.path.exists('./output'):
        shutil.rmtree('./output', ignore_errors=True)
    os.makedirs('./output')
    pdf_buffer = []
    my_pdf = SimpleDocTemplate('heatmap_report.pdf')
    pdf_style_sheet = getSampleStyleSheet()
    paragraph = Paragraph('AIRI - Reporte de medición WiFi<br /><br /><br />', pdf_style_sheet['Heading1'])
    pdf_buffer.append(paragraph)
    paragraph = Paragraph('Proyecto Final - Heatmap<br /><br />', pdf_style_sheet['Heading2'])
    pdf_buffer.append(paragraph)
    paragraph = Paragraph('Alumnos:<br /><br />', pdf_style_sheet['Heading2'])
    pdf_buffer.append(paragraph)
    paragraph = Paragraph('Juan Ignacio Battaglino - juanibattaglino@gmail.com<br /><br />', pdf_style_sheet['Heading2'])
    pdf_buffer.append(paragraph)
    paragraph = Paragraph('Michel Hidalgo - hid.michel@gmail.com<br /><br />', pdf_style_sheet['Heading2'])
    pdf_buffer.append(paragraph)
    paragraph = Paragraph('Profesor: Silvio Tapino<br /><br />', pdf_style_sheet['Heading2'])
    pdf_buffer.append(paragraph)
    pdf_buffer.append(PageBreak())
    paragraph = Paragraph('Site analysis for floor plan draw:<br /><br /><br />', pdf_style_sheet['Heading1'])
    pdf_buffer.append(paragraph)
    my_img = PIL.Image.open('./input/my_floor_diagram.png')
    wpercent = BASEWIDTH / float(my_img.size[0])
    hsize = int((float(my_img.size[1]))*float(wpercent))
    img = my_img.resize((BASEWIDTH,hsize), Img.ANTIALIAS)
    img.save('./input/my_floor_diagram_resized.png', quality=85)
    pdf_buffer.append(Image('./input/my_floor_diagram_resized.png'))
    pdf_buffer.append(PageBreak())

    t1 = session.query(Ssid).filter(Ssid.id==Measure.ssid_id).order_by(Ssid.ssid_value).all()
    for ssid in t1:
        t2 = session.query(Bssid).join(Measure, Measure.bssid_id==Bssid.id).filter(Measure.ssid_id==ssid.id).all()
        for bssid in t2:
            paragraph_measure = Paragraph('SSID: {}'.format(ssid), pdf_style_sheet['Heading2'])
            pdf_buffer.append(paragraph_measure)

            #Security types:
            #SELECT DISTINCT ssid_value, security_type FROM measure INNER JOIN ssid ON measure.ssid_id = ssid.id INNER JOIN security ON measure.security_id = security.id ORDER BY ssid_value ASC;
            query = session.query(Security, Measure, Ssid).join(Ssid, Ssid.id==Measure.ssid_id).filter(Ssid.ssid_value==str(ssid)).filter(Bssid.bssid_value==str(bssid)).join(Security, Security.id==Measure.security_id).first()
            paragraph_sec = Paragraph('Security type: {}'.format(query[0]), pdf_style_sheet['Heading2'])
            
            #Bssids:
            #SELECT DISTINCT ssid_value, bssid_value FROM measure INNER JOIN ssid ON measure.ssid_id = ssid.id INNER JOIN bssid ON measure.bssid_id = bssid.id ORDER BY ssid_value ASC;
            query = session.query(Bssid, Measure, Ssid).join(Ssid, Ssid.id==Measure.ssid_id).filter(Ssid.ssid_value==str(ssid)).filter(Bssid.bssid_value==str(bssid)).join(Bssid, Bssid.id==Measure.bssid_id).all()
            bssid_list = remove_repeated_values(query)
            paragraph_bssid = Paragraph('Bssids detected: {}'.format(bssid_list), pdf_style_sheet['Heading2'])
            
            #Channels:
            #SELECT DISTINCT ssid_value, channel_number FROM measure INNER JOIN ssid ON measure.ssid_id = ssid.id INNER JOIN channel ON measure.channel_id = channel.id ORDER BY ssid_value ASC;
            query = session.query(Channel, Measure, Ssid).join(Ssid, Ssid.id==Measure.ssid_id).filter(Ssid.ssid_value==str(ssid)).filter(Bssid.bssid_value==str(bssid)).join(Channel, Channel.id==Measure.channel_id).join(Bssid, Bssid.id==Measure.bssid_id).all()
            channel_list = remove_repeated_values(query)
            paragraph_channel = Paragraph('Transmission channel: {}'.format(channel_list), pdf_style_sheet['Heading2'])
            
            #Frequency:
            #SELECT DISTINCT ssid_value, frequency FROM measure INNER JOIN ssid ON measure.ssid_id = ssid.id INNER JOIN frequency ON measure.frequency_id = frequency.id ORDER BY ssid_value ASC;
            query = session.query(Frequency, Measure, Ssid).join(Ssid, Ssid.id==Measure.ssid_id).filter(Ssid.ssid_value==str(ssid)).filter(Bssid.bssid_value==str(bssid)).join(Frequency, Frequency.id==Measure.frequency_id).join(Bssid, Bssid.id==Measure.bssid_id).all()
            frequency_list = remove_repeated_values(query)
            paragraph_frequency = Paragraph('Transmission frequency: {} MHz'.format(frequency_list), pdf_style_sheet['Heading2'])

            #Rate:
            #SELECT DISTINCT ssid_value, rate FROM measure INNER JOIN ssid ON measure.ssid_id = ssid.id INNER JOIN rate ON measure.rate_id = rate.id ORDER BY ssid_value ASC;
            query = session.query(Rate, Measure, Ssid).join(Ssid, Ssid.id==Measure.ssid_id).filter(Ssid.ssid_value==str(ssid)).filter(Bssid.bssid_value==str(bssid)).join(Rate, Rate.id==Measure.rate_id).join(Bssid, Bssid.id==Measure.bssid_id).all()
            rate_list = remove_repeated_values(query)
            paragraph_rate = Paragraph('Transmission rate: {} Mbps<br /><br /><br /><br />'.format(rate_list), pdf_style_sheet['Heading2'])
            
            pdf_buffer.append(paragraph_sec)
            pdf_buffer.append(paragraph_bssid)
            pdf_buffer.append(paragraph_channel)
            pdf_buffer.append(paragraph_frequency)
            pdf_buffer.append(paragraph_rate)

            #rssi:
            #SELECT DISTINCT ssid_value, x_location, y_location, rssi FROM measure INNER JOIN ssid ON measure.ssid_id = ssid.id INNER JOIN point ON measure.point_id = point.id ORDER BY ssid_value ASC;
            query = session.query(Point.id, Point.x_location, Point.y_location, Measure, Ssid, Bssid).join(Ssid, Ssid.id==Measure.ssid_id).filter(Ssid.ssid_value==str(ssid)).filter(Bssid.bssid_value==str(bssid)).join(Point, Point.id==Measure.point_id).join(Bssid, Bssid.id==Measure.bssid_id).all()
            query.insert(0, ['id','x_position', 'y_position', 'rssi', 'ssid', 'bssid'])
            plot_heatmap(query)

            plt.tight_layout()
            plt.savefig('./output/{}-{}-measure.png'.format(ssid,bssid))
            plt.close()
            pdf_buffer.append(Image('./output/{}-{}-measure.png'.format(ssid,bssid),width=3*200, height=1.5*200, kind='proportional'))
            pdf_buffer.append(PageBreak())
    my_pdf.build(pdf_buffer, onFirstPage=add_page_number, onLaterPages=add_page_number)


def add_page_number(canvas, doc):
     canvas.saveState()
     canvas.setFont('Times-Roman', 10)
     page_number_text = "%d" % (doc.page)
     canvas.drawCentredString(0.75 * inch, 0.75 * inch, page_number_text)
     canvas.restoreState()


def remove_repeated_values(query_list):
    my_list = []
    for my_query in query_list:
        my_list.append(my_query[0])
        my_set = set(my_list)
        my_set.union(my_set)
    return list(my_set)

def plot_heatmap(query):
    query_list = []
    for elem in query:
        query_list.append(list(elem))
    f = plt.figure()
    outfile = open('./output/my_csv.csv', 'w')
    outcsv = csv.writer(outfile)
    outcsv.writerows(query_list)
    outfile.close()
    df1 = pd.read_csv('./output/my_csv.csv', index_col=0)
    layout = imread('input/my_floor_diagram_resized.png')
    f.suptitle("Individual AP RSSI")
    f.subplots_adjust(hspace=0.1, wspace=0.1, left=0.05, right=0.95, top=0.85,bottom=0.15)
    rbf = Rbf(df1['x_position'], df1['y_position'], df1['rssi'], function='linear')
    num_x = int(IMAGE_WIDTH / 4)
    num_y = int(num_x / (IMAGE_WIDTH / IMAGE_HEIGHT))
    x = np.linspace(0, GRID_WIDTH, num_x)
    y = np.linspace(0, GRID_HEIGHT, num_y)
    gx, gy = np.meshgrid(x, y)
    gx, gy = gx.flatten(), gy.flatten()
    z = rbf(gx, gy)
    z = z.reshape((num_y, num_x))
    image = plt.imshow(z, vmin=0, vmax=100, extent=(0,
            IMAGE_WIDTH, IMAGE_HEIGHT, 0), cmap='RdYlBu_r', alpha=1)
    plt.colorbar(image)
    plt.imshow(layout, interpolation='bicubic', zorder=100)