#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import csv
import sqlite3
from wifi_scanner import WiFiScanner

def create_csv():
    print("Creating CSV file for data logging...")
    with open('wifi_data.csv', 'w') as file:
        writer = csv.writer(file)
        writer.writerow(['x', 'y', 's1', 's2', 's3'])
    print("CSV file created.")

def create_db():
    print("Creating SQLite database for data logging...")
    conn = sqlite3.connect('wifi_data.db')
    cursor = conn.cursor()
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS wifi_data (
            x REAL,
            y REAL,
            s1 TEXT,
            s2 TEXT,
            s3 TEXT
        )
    ''')
    conn.commit()
    conn.close()
    rospy.loginfo("SQLite database created.")

def move_straight(cmd_vel_pub):
    move_cmd = Twist()
    move_cmd.linear.x = 0.1  # 0.1 m/s
    move_cmd.linear.y = 0.0
    move_cmd.linear.z = 0.0
    move_cmd.angular.x = 0.0
    move_cmd.angular.y = 0.0
    move_cmd.angular.z = 0.0

    # Définir la distance à parcourir
    distance = 0.5  # 50 cm

    # Calculer le temps nécessaire pour parcourir la distance
    time_needed = distance / move_cmd.linear.x

    # Définir la fréquence de publication
    rate = rospy.Rate(10)  # 10 Hz

    # Boucle pour envoyer les commandes de vitesse
    end_time = rospy.Time.now() + rospy.Duration(time_needed)
    while rospy.Time.now() < end_time:
        cmd_vel_pub.publish(move_cmd)
        rate.sleep()

    # Arrêter le robot après avoir parcouru la distance
    stop_cmd = Twist()
    cmd_vel_pub.publish(stop_cmd)

def rotate_robot(pub, angle, angular_speed=0.5):
    duration = abs(angle) / angular_speed
    angular_z = angular_speed if angle > 0 else -angular_speed
    move_robot(pub, angular_z=angular_z, duration=duration)

def average(values):
    # Calculer la moyenne en évitant les valeurs 'N/A'
    valid_values = [int(v) for v in values if v != 'N/A']
    if not valid_values:
        return 'N/A'
    print(valid_values)
    return sum(valid_values) / len(valid_values)

def collect_column(pub, scanner, steps=2, row=0, next_row=-1):
    x = row
    y = 0
    
    for i in range(steps):
        print(i)
        y = i if next_row == -1 else steps-i
        # collect data

        # Collecter les données trois fois pour améliorer la précision
        signals_list = [scanner.scan_and_get_data() for _ in range(3)]
        s1_values = [signals[0] if len(signals) > 0 else 'N/A' for signals in signals_list]
        s2_values = [signals[1] if len(signals) > 1 else 'N/A' for signals in signals_list]
        s3_values = [signals[2] if len(signals) > 2 else 'N/A' for signals in signals_list]

        # Calculer la moyenne des signaux
        s1_avg = average(s1_values)
        s2_avg = average(s2_values)
        s3_avg = average(s3_values)

        record_data(x, y, s1_avg, s2_avg, s3_avg)

        # signals = scanner.scan_and_get_data()
        # s1 = signals[0] if len(signals) > 0 else 'N/A'
        # s2 = signals[1] if len(signals) > 1 else 'N/A'
        # s3 = signals[2] if len(signals) > 2 else 'N/A'
        # record_data(x, y, s1, s2, s3)
        
        # move
        move_straight(pub)

    if next_row == -1:
        y += 1
    else:
        y -= 1
        
    signals = scanner.scan_and_get_data()
    s1 = signals[0] if len(signals) > 0 else 'N/A'
    s2 = signals[1] if len(signals) > 1 else 'N/A'
    s3 = signals[2] if len(signals) > 2 else 'N/A'
    record_data(x, y, s1, s2, s3)

    turn_right = True if next_row == -1 else False
    turn_robot(pub, right=turn_right)
    move_straight(pub)
    turn_robot(pub, right=turn_right)

def collect_space_data(pub, scanner, column_length=2, row_length=2):
    row_dir = -1
    for row in range(row_length+1):
        collect_column(pub, scanner, steps=column_length, row=row, next_row=row_dir)
        row_dir *= -1

def turn_robot(cmd_vel_pub, right=True):
    # Créer un message Twist pour tourner
    rotate_cmd = Twist()
    rotate_cmd.linear.x = 0.0
    rotate_cmd.linear.y = 0.0
    rotate_cmd.linear.z = 0.0
    rotate_cmd.angular.x = 0.0
    rotate_cmd.angular.y = 0.0
    rotate_cmd.angular.z = -0.5 if right else 0.5  # Négatif pour tourner à droite (horaire)

    # Définir la fréquence de publication
    rate = rospy.Rate(10)  # 10 Hz

    # Durée pendant laquelle envoyer les commandes pour tourner
    # Par exemple, pour tourner 90 degrés avec une vitesse angulaire de 0.5 rad/s
    angle_to_turn = 90  # degrés
    angular_speed = 0.5  # rad/s
    duration = (angle_to_turn * 3.14159 / 180) / angular_speed  # Convertir degrés en radians et calculer la durée

    # Boucle pour envoyer les commandes de rotation
    end_time = rospy.Time.now() + rospy.Duration(duration)
    while rospy.Time.now() < end_time:
        cmd_vel_pub.publish(rotate_cmd)
        rate.sleep()

    # Arrêter le robot après avoir tourné
    stop_cmd = Twist()  # Un message Twist avec toutes les valeurs à zéro
    cmd_vel_pub.publish(stop_cmd)

def record_data(x, y, s1, s2, s3):
    rospy.loginfo(f"Recording data: x={x}, y={y}, s1={s1}, s2={s2}, s3={s3}")
    with open('wifi_data.csv', 'a') as file:
        writer = csv.writer(file)
        writer.writerow([x, y, s1, s2, s3])

    conn = sqlite3.connect('wifi_data.db')
    cursor = conn.cursor()
    cursor.execute('INSERT INTO wifi_data (x, y, s1, s2, s3) VALUES (?, ?, ?, ?, ?)', (x, y, s1, s2, s3))
    conn.commit()
    conn.close()
    # rospy.loginfo("Data recorded.")

def main():
    rospy.init_node('robot_collector', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # create_csv()
    # create_db()

    # scanner = WiFiScanner()
    # collect_space_data(pub, scanner)
    i = 1
    while(True):
        move_straight(pub)
        rospy.loginfo(f'step {i}')
        i += 1
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
