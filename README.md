# Pothole 3D Reconstruction & Volumetric Estimation

📌 **Language / Lingua:**  
🇬🇧 [English Version](#-english-version) | 🇮🇹 [Versione Italiana](#-versione-italiana)

---

## 🇬🇧 English Version

This project enables the estimation of the **volume** and the **deepest point** (expressed in centimeters) of a road pothole starting from a sequence of two-dimensional images. 

The system leverages **Structure from Motion (SfM)** techniques via COLMAP, geometric camera calibration using OpenCV (Zhang's Method), and three-dimensional segmentation algorithms such as **RANSAC** to identify the road plane. The conversion from the arbitrary units of the 3D space to real-world centimeters is achieved through an empirical scale factor calculated in MeshLab, using a coin as a known reference.

---

## 🛠️ Technologies and Tools Used

* **Python 3** (Main language)
* **OpenCV** (Camera calibration)
* **COLMAP** (CLI pipeline for sparse and dense 3D reconstruction)
* **Open3D / NumPy** (Point cloud processing and RANSAC)
* **MeshLab** (Metric analysis of the reference and scale factor calculation)

---

## 📁 Repository Structure

```text
├── HoleMoneta/                   # Photo dataset of the pothole with the reference coin
├── PotHoles/                     # Module dedicated to camera calibration
│   ├── Calibration_Images/       # Checkerboard frames from different angles
│   ├── Camera_Calibration.py     # OpenCV script for intrinsic parameters calculation
│   ├── Hole1/                    # [DEPRECATED] Initial reconstruction tests
│   └── Reconstruct.py            # [DEPRECATED] Legacy reconstruction script
├── calibration.npz               # Calibration output (Intrinsic matrix K)
├── Reconstruction.py             # CLI automation pipeline for COLMAP
└── Volume_Depth_Coin.py          # Point cloud processing and metrics estimation script
```

---

## 🚀 Execution Pipeline

### 1. Camera Calibration
Before performing the reconstruction, it is necessary to correct the geometric lens distortions and extract the intrinsic matrix \(K\). 
Inside the `PotHoles/` folder, the `Camera_Calibration.py` script processes the checkerboard images contained in `Calibration_Images/` by implementing **Zhang's Method**. 
* **Output:** Generates the `calibration.npz` file in the project root.

### 2. 3D Reconstruction (Structure from Motion)
The `Reconstruction.py` script automates the **COLMAP** command-line pipeline. It takes the pothole images in `HoleMoneta/` and the parameters extracted from `calibration.npz` as input.
* **Output:** Generates a dense point cloud saved in `HoleMoneta_reconstruction/dense/fused.ply`.

### 3. Scale Factor Calculation (MeshLab)
Since the SfM reconstruction is defined up to a scale factor (the units are arbitrary), **MeshLab** was used to measure the coin present in the scene. By comparing the software measurement with the known real diameter of the coin, the *unit-to-cm* conversion coefficient was derived.

### 4. Volumetric and Depth Analysis
The `Volume_Depth_Coin.py` script loads the `fused.ply` file and performs the following mathematical steps:
1. **RANSAC Segmentation:** Isolates the points belonging to the road plane (finding the plane equation \(ax + by + cz + d = 0\)).
2. **Depth Calculation:** Computes the maximum orthogonal distance between the points inside the pothole and the estimated road plane.
3. **Volume Estimation:** Applies a triangulation or a discrete integration of the empty space between the upper road plane and the internal surface of the pothole.
4. **Metric Conversion:** Applies the empirical scale factor to return the results directly in **centimeters** and **cubic centimeters / liters**.

---

## Development Notes
* The files and folders inside `PotHoles/Hole1/` and `PotHoles/Reconstruct.py` are to be considered **deprecated and obsolete**. They are kept in the repository solely for development history and are not required for operation.

---

## 🇮🇹 Versione Italiana

Questo progetto permette di stimare il **volume** e il **punto più profondo** (espressi in centimetri) di una buca stradale a partire da una sequenza di immagini bidimensionali. 

Il sistema sfrutta tecniche di **Structure from Motion (SfM)** tramite COLMAP, calibrazione geometrica della camera tramite OpenCV (Metodo di Zhang) e algoritmi di segmentazione tridimensionale come **RANSAC** per l'identificazione del piano stradale. Il passaggio dalle unità arbitrarie dello spazio 3D ai centimetri reali avviene tramite un fattore di scala empirico calcolato in MeshLab utilizzando una moneta come riferimento noto.

---

## 🛠️ Tecnologie e Strumenti Utilizzati

* **Python 3** (Linguaggio principale)
* **OpenCV** (Calibrazione della fotocamera)
* **COLMAP** (CLI pipeline per la ricostruzione 3D densa e sparsa)
* **Open3D / NumPy / SciPy** (Elaborazione della nuvola di punti e RANSAC)
* **MeshLab** (Analisi metrica del riferimento e calcolo del fattore di scala)

---

## 📁 Struttura del Repository

```text
├── HoleMoneta/                   # Dataset fotografico della buca con la moneta di riferimento
├── PotHoles/                     # Modulo dedicato alla calibrazione della camera
│   ├── Calibration_Images/       # Frame della scacchiera da diverse angolazioni
│   ├── Camera_Calibration.py     # Script OpenCV per il calcolo dei parametri intrinseci
│   ├── Hole1/                    # [DEPRECATED] Test iniziali di ricostruzione
│   └── Reconstruct.py            # [DEPRECATED] Script di ricostruzione legacy
├── calibration.npz               # Output di calibrazione (Matrice intrinseca K)
├── Reconstruction.py             # Pipeline di automazione CLI per COLMAP
└── Volume_Depth_Coin.py          # Script di elaborazione nuvola di punti e stima metriche
```

---

## 🚀 Pipeline di Esecuzione

### 1. Calibrazione della Camera
Prima di effettuare la ricostruzione, è necessario correggere le distorsioni geometriche della lente e ricavare la matrice intrinseca \(K\). 
All'interno della cartella `PotHoles/`, lo script `Camera_Calibration.py` elabora le immagini della scacchiera contenute in `Calibration_Images/` implementando il **Metodo di Zhang**. 
* **Output:** Genera il file `calibration.npz` nella root del progetto.

### 2. Ricostruzione 3D (Structure from Motion)
Lo script `Reconstruction.py` automatizza la pipeline da linea di comando di **COLMAP**. Prende in input le immagini della buca presenti in `HoleMoneta/` e i parametri estratti da `calibration.npz`.
* **Output:** Genera una nuvola di punti densa salvata in `HoleMoneta_reconstruction/dense/fused.ply`.

### 3. Calcolo del Fattore di Scala (MeshLab)
Poiché la ricostruzione SfM è definita a meno di un fattore di scala (le unità sono arbitrarie), è stato utilizzato **MeshLab** per misurare la moneta presente nella scena. Confrontando la misura software con il diametro reale noto della moneta, è stato ricavato il coefficiente di conversione *unità-to-cm*.

### 4. Analisi Volumetrica e Profondità
Lo script `Volume_Depth_Coin.py` carica il file `fused.ply` ed esegue i seguenti passaggi matematici:
1. **Segmentazione RANSAC:** Isola i punti appartenenti al piano stradale (trovando l'equazione del piano \(ax + by + cz + d = 0\)).
2. **Calcolo della Profondità:** Calcola la distanza ortogonale massima tra i punti interni alla buca e il piano stradale stimato.
3. **Stima del Volume:** Applica una triangolazione o un'integrazione discreta dello spazio vuoto compreso tra il piano stradale superiore e la superficie interna della buca.
4. **Conversione Metrica:** Applica il fattore di scala empirico per restituire i risultati direttamente in **centimetri** e **centimetri cubi / litri**.

---

## Note sullo Sviluppo
* I file e le cartelle all'interno di `PotHoles/Hole1/` e `PotHoles/Reconstruct.py` sono da considerarsi **deprecati e obsoleti**, mantenuti nel repository esclusivamente per storicità dello sviluppo ma non necessari al funzionamento.
