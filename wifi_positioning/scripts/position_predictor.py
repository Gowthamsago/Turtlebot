import os
import pandas as pd
from sklearn.neighbors import KNeighborsClassifier
import joblib

class PositionPredictor:
    def __init__(self):
        # Chemin pour sauvegarder le modèle
        self.model_path = 'knn_model.pkl'

        # Charger le modèle sauvegardé
        self.knn = self.load_model()

    def train_and_save_model(self):
        # Charger les données depuis le fichier CSV
        data = pd.read_csv('wifi_data.csv')

        # Séparer les caractéristiques (RSSI) et les labels (positions x, y)
        X = data[['s1', 's2', 's3']]
        y = data[['x', 'y']]

        # Créer le modèle k-NN (k=3, par exemple)
        knn = KNeighborsClassifier(n_neighbors=5) # n_neighbors = 3

        # Entraîner le modèle avec les données
        knn.fit(X, y)

        # Sauvegarder le modèle entraîné
        joblib.dump(knn, self.model_path)
        print("Modèle entraîné et sauvegardé.")

    def load_model(self):
        if not os.path.exists(self.model_path):
            self.train_and_save_model()
        # Charger le modèle sauvegardé
        return joblib.load(self.model_path)

    def predict_position(self, wifi_data):
        # Vérifier que nous avons bien trois valeurs RSSI
        if len(wifi_data) == 3:
            new_observation = pd.DataFrame([wifi_data], columns=['s1', 's2', 's3'])

            # Prédire la position (x, y) pour la nouvelle observation
            return self.knn.predict(new_observation)
        else:
            print("Erreur : Impossible de trouver les trois adresses WiFi d'intérêt")
            return None
