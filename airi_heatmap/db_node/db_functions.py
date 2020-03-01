from models import Point, Channel, Security, Ssid, Measure, Bssid, Draw_Point, Frequency, Rate

def save_measure_in_db(session, data, point):
    objects = []
    value = data.pop(0)
    
    my_ssid = Ssid(value[0])
    my_bssid = Bssid(value[1])
    my_channel = Channel(value[2])
    my_freq = Frequency(value[3])
    my_rate = Rate(value[4])
    my_measure = Measure(value[5])
    my_security = Security(value[6])
    my_point = Point(point)
    
    entry = session.query(Ssid).filter(Ssid.ssid_value.like(value[0])).first()
    if entry is None:
        my_ssid.measure.append(my_measure)
        objects.append(my_ssid)
    else:
        entry.measure.append(my_measure)

    entry = session.query(Bssid).filter(Bssid.bssid_value.like(value[1])).first()
    if entry is None:
        my_bssid.measure.append(my_measure)
        objects.append(my_bssid)
    else:
        entry.measure.append(my_measure)

    entry = session.query(Channel).filter(Channel.channel_number.like(value[2])).first()
    if entry is None:
        my_channel.measure.append(my_measure)
        objects.append(my_channel)
    else:
        entry.measure.append(my_measure)

    entry = session.query(Frequency).filter(Frequency.frequency_value.like(value[3])).first()
    if entry is None:
        my_freq.measure.append(my_measure)
        objects.append(my_freq)
    else:
        entry.measure.append(my_measure)

    entry = session.query(Rate).filter(Rate.rate_value.like(value[4])).first()
    if entry is None:
        my_rate.measure.append(my_measure)
        objects.append(my_rate)
    else:
        entry.measure.append(my_measure)

    entry = session.query(Security).filter(Security.security_type.like(value[6])).first()
    if entry is None:
        my_security.measure.append(my_measure)
        objects.append(my_security)
    else:
        entry.measure.append(my_measure)
    
    entry = session.query(Point).filter(Point.x_location==point[0]).filter(Point.y_location==point[1]).first()
    if entry is None:
        my_point.measure.append(my_measure)
        objects.append(my_point)
    else:
        entry.measure.append(my_measure)

    objects.append(my_measure)

    session.add_all(objects)
    session.commit()
    if len(data) != 0:
        save_measure_in_db(session, data, point)
    else:
        return 'FINISHED'


def save_point_in_db(session, point):
    my_draw_point = Draw_Point(point)
    session.add(my_draw_point)
    session.commit()
