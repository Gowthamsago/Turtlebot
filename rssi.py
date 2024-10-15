from scapy.all import *

# Définir les adresses MAC des points de référence
ap1 = "dc:08:56:28:01:32"
ap2 = "dc:08:56:30:a5:92"
ap3 = "dc:08:56:30:64:32"

# Définir la durée de l'analyse (en secondes)
duration = 2

# Lancer l'analyse des réseaux sans fil à portée
packets = sniff(iface="Wi-Fi", count=0, timeout=duration, prn=lambda x: x.summary())
print("\n\n\r==========================================================\n\n\r")
# Parcourir les paquets reçus et afficher la force du signal RSSI de chaque point de référence
for packet in packets:
    if packet.haslayer(Dot11Beacon):
        print("has Dit11Beacon")
        bssid = packet.addr2
        ssid = packet.info.decode("utf-8", "ignore")
        signal = packet.dBm_AntSignal

        # Vérifier si le réseau sans fil correspond à l'un des points de référence
        if bssid == ap1:
            print(f"Point d'accès 1 : {ssid}\tAdresse MAC : {bssid}\tForce du signal RSSI : {signal}")
        elif bssid == ap2:
            print(f"Point d'accès 2 : {ssid}\tAdresse MAC : {bssid}\tForce du signal RSSI : {signal}")
        elif bssid == ap3:
            print(f"Point d'accès 3 : {ssid}\tAdresse MAC : {bssid}\tForce du signal RSSI : {signal}")
