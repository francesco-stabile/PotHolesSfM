import os
import numpy as np
import open3d as o3d

# Percorso diretto al file di input
BASE_DIR    = os.getcwd()
PLY_FILE = os.path.join(BASE_DIR, r"HoleMoneta_reconstruction\dense", "fused.ply")
# Directory dove inserire l'output
output_dir = os.path.dirname(os.path.dirname(PLY_FILE))

# Carica la nuvola di punti
pcd = o3d.io.read_point_cloud(PLY_FILE)

# Segmentazione del piano usando RANSAC
plane_model, inliers = pcd.segment_plane(
    distance_threshold=0.06, # distanza massima dal piano
    ransac_n=3, #numero di punti per stimare il piano
    num_iterations=10000 # tentativi ransac
)

# Esrazione coefficienti del piano
[a, b, c, d] = plane_model
print(f"Piano stimato: {a:.4f}x + {b:.4f}y + {c:.4f}z + {d:.4f} = 0")

# Separiamo inlier e outlier
inlier_cloud = pcd.select_by_index(inliers) # punti che stanno sul piano
outlier_cloud = pcd.select_by_index(inliers, invert=True) # punti fuori dal piano (potenziale buca)

# Applichiamo il VERDE solo al manto stradale (inlier)
inlier_cloud.paint_uniform_color([0, 1, 0])

# Prepariamo i parametri matematici del piano
norm = np.sqrt(a**2 + b**2 + c**2)
a, b, c, d = a/norm, b/norm, c/norm, d/norm # normalizzazione coefficienti per calcolare le distanze corrette

# Pulizia statistica degli outlier, elimina i punti troppo isolati
outlier_cloud, ind = outlier_cloud.remove_statistical_outlier(nb_neighbors=30, std_ratio=1.5)
outlier_points = np.asarray(outlier_cloud.points) # conversione punti in array

# Calcolo distanze per trovare il punto più profondo
signed_dist = (a*outlier_points[:,0] + b*outlier_points[:,1] + c*outlier_points[:,2] + d) # Se positiva sopra il piano, se negativa sotto

# Controllo orientamento
if np.percentile(signed_dist, 5) < 0 and np.percentile(signed_dist, 95) <= 0.02:
    target_dist = signed_dist
else:
    target_dist = -signed_dist

threshold_depth = np.percentile(target_dist, 0.1) # Prende lo 0.1% dei punti più profondi
deep_indices = np.where(target_dist <= threshold_depth)[0] # Selezione di quei punti
deepest_point = np.mean(outlier_points[deep_indices], axis=0) # Scelta del punto più profondo usando la media
max_depth = np.abs(np.mean(target_dist[deep_indices])) # Profondità media dei punti più profondi

print(f"Profondità massima robusta: {max_depth:.4f}\n")

# Ora bisogna isolare la buca considerando parte del piano eventuali protuberanze o
# deformazioni del manto stradale lontane dalla buca

# RAGGIO_MASSIMO: i punti oltre questa distanza dal punto più profondo tornano verdi
RAGGIO_MASSIMO = 2

distanze_xy = np.linalg.norm(outlier_points[:, :2] - deepest_point[:2], axis=1)

# Indici dei punti che sono dentro la buca
indices_buca = np.where(distanze_xy <= RAGGIO_MASSIMO)[0]
# Indici dei punti che sono fuori dalla buca
indices_esclusi = np.where(distanze_xy > RAGGIO_MASSIMO)[0]

# Nuova nuvola della buca
hole_cloud = outlier_cloud.select_by_index(indices_buca)

# Rendiamo parte del piano e di colore VERDE i punti che sono fuori dalla buca (distanza > Raggio Massimo)
extra_plane_cloud = outlier_cloud.select_by_index(indices_esclusi)
extra_plane_cloud.paint_uniform_color([0, 1, 0])

# Unione finale del piano (inlier originali + outlier lontani)
plane_cloud_final = inlier_cloud + extra_plane_cloud

# Punto BLU per punto più profondo
deep_point_geom = o3d.geometry.PointCloud() # Creazione nuova nuvola
deep_point_geom.points = o3d.utility.Vector3dVector([deepest_point]) # Aggiungo il punto
deep_point_geom.paint_uniform_color([0, 0, 1]) # Lo coloro di BLU

# Salvataggio di strada, buca e punto più profondo
o3d.io.write_point_cloud(os.path.join(output_dir, "plane_points.ply"), plane_cloud_final)
o3d.io.write_point_cloud(os.path.join(output_dir,"hole_points.ply"), hole_cloud)
o3d.io.write_point_cloud(os.path.join(output_dir,"deep_point.ply"), deep_point_geom)

# Ora bisogna calcolare il volume della buca usando la scala trovata
# con il metodo della moneta da 2 euro (diamentro di 2,58)

# Per calcolare il volume dividiamo in una griglia la buca e per ogni cella calcoliamo la profondità media
# Infine sommiamo i vari valori di volume di ogni cella dati dalla formula volume = area x altezza

# Il valore di scala fornito (trasforma le unità COLMAP in unità reali)
SCALA_REALE = 14

# 1) Estraiamo i punti della buca filtrata
hole_pts = np.asarray(hole_cloud.points)

# Se esistono punti
if len(hole_pts) > 0:
    # Calcoliamo la distanza di ogni punto dal piano (profondità h in unità COLMAP)
    # Poiché a,b,c,d sono già normalizzati:
    distanze_colmap = np.abs(a * hole_pts[:, 0] + b * hole_pts[:, 1] + c * hole_pts[:, 2] + d)

    # 2) Trasformiamo le distanze in unità reali
    distanze_reali = distanze_colmap * SCALA_REALE

    # 3. Definiamo la risoluzione della griglia
    risoluzione = 1.0
    area_cella = risoluzione ** 2 # Dimensione celle

    # Portiamo anche le coordinate XY dei punti nella scala reale per la griglia
    coords_reali_xy = hole_pts[:, :2] * SCALA_REALE

    # Limiti area
    x_min, y_min = np.min(coords_reali_xy, axis=0)
    x_max, y_max = np.max(coords_reali_xy, axis=0)

    # Creazione griglia
    x_bins = np.arange(x_min, x_max + risoluzione, risoluzione)
    y_bins = np.arange(y_min, y_max + risoluzione, risoluzione)

    # Organizziamo i punti nelle celle della griglia
    x_indices = np.digitize(coords_reali_xy[:, 0], x_bins) - 1
    y_indices = np.digitize(coords_reali_xy[:, 1], y_bins) - 1

    # Dizionario delle celle
    griglia_prof = {}
    for i in range(len(distanze_reali)): # Per ogni punto
        idx = (x_indices[i], y_indices[i]) # calcola la rispettiva cella
        if idx not in griglia_prof: # per ogni nuova cella crea l'array delle profondità
            griglia_prof[idx] = []
        griglia_prof[idx].append(distanze_reali[i]) # e ne salva la profondità di quel punto

    # 4. Somma dei volumi parziali
    volume_totale = 0 # inizializzo volume a zero
    for idx in griglia_prof: # Per ogni cella
        # Calcolo della media delle profondità dei punti della cella
        h_media = np.mean(griglia_prof[idx])

        # Consideriamo solo profondità significative per evitare rumore del piano
        if h_media > 0.3:
            volume_totale += area_cella * h_media

    # Calcolo profondità massima scalata
    max_depth_reale = max_depth * SCALA_REALE

    # Outuput dei risultati
    print(f" RISULTATI SCALATI (Fattore: {SCALA_REALE:.4f})\n")
    print(f"Profondità Massima: {max_depth_reale:.2f} cm\n")
    print(f"Volume Totale:      {volume_totale:.2f} cm³\n")

    # Esempio: se la scala era in cm, il volume è in cm³ (ml). 1000 cm³ = 1 Litro.
    print(f"Volume in Litri: {volume_totale / 1000:.3f} L\n")

else:
    print("Errore: Nessun punto trovato per il calcolo del volume.")
