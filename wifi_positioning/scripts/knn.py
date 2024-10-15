import os
import pandas as pd
from sklearn.neighbors import KNeighborsClassifier
import joblib
from wifi_scanner import WiFiScanner

# Chemin pour sauvegarder le modèle
model_path = 'knn_model.pkl'

def train_and_save_model():
    # Charger les données depuis le fichier CSV
    data = pd.read_csv('wifi_data.csv')

    # Séparer les caractéristiques (RSSI) et les labels (positions x, y)
    X = data[['s1', 's2', 's3']]
    y = data[['x', 'y']]
    print(X)
    print(y)
    # Créer le modèle k-NN (k=3, par exemple)
    knn = KNeighborsClassifier(n_neighbors=5) # n_neighbors = 3

    # Entraîner le modèle avec les données
    knn.fit(X, y)

    # Sauvegarder le modèle entraîné
    joblib.dump(knn, model_path)
    print("Modèle entraîné et sauvegardé.")

def load_model():
    # Charger le modèle sauvegardé
    return joblib.load(model_path)

def main():
    # Entraîner et sauvegarder le modèle si ce n'est pas déjà fait
    if not os.path.exists(model_path):
        train_and_save_model()

    # Charger le modèle sauvegardé
    knn = load_model()

    # Initialiser le scanner WiFi
    scanner = WiFiScanner()

    # Scanner les réseaux WiFi pour une nouvelle observation
    wifi_data = scanner.scan_and_get_data()

    # Collecter les données trois fois pour améliorer la précision
    #signals_list = [scanner.scan_and_get_data() for _ in range(3)]
    #s1_values = [signals[0] if len(signals) > 0 else 'N/A' for signals in signals_list]
    #s2_values = [signals[1] if len(signals) > 1 else 'N/A' for signals in signals_list]
    #s3_values = [signals[2] if len(signals) > 2 else 'N/A' for signals in signals_list]
    
    # Calculer la moyenne des signaux
    #s1_avg = scanner.average(s1_values)
    #s2_avg = scanner.average(s2_values)
    #s3_avg = scanner.average(s3_values)
    
    #wifi_data = [s1_avg, s2_avg, s3_avg]
    print(wifi_data)

    # Vérifier que nous avons bien trois valeurs RSSI
    if len(wifi_data) == 3:
        new_observation = pd.DataFrame([wifi_data], columns=['s1', 's2', 's3'])

        # Prédire la position (x, y) pour la nouvelle observation
        predicted_position = knn.predict(new_observation)

        print("Position prédite:", predicted_position)
        print("Type Position prédite:", type(predicted_position))
    else:
        print("Erreur : Impossible de trouver les trois adresses WiFi d'intérêt")

if __name__ == '__main__':
    main()
