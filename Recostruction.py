import subprocess # per eseguire comandi esterni (quelli di colmap)
import os
import numpy as np


BASE_DIR    = os.getcwd() # Cartella del progetto
COLMAP_BAT  = os.path.join(BASE_DIR, "Colmap", "COLMAP.bat") # Path di Colmap.bat
IMAGE_DIR   = os.path.join(BASE_DIR, "HoleMoneta3") # Directory contenente le foto da cui ricostruire 3D
WORKSPACE   = os.path.join(BASE_DIR, "HoleMoneta3_reconstruction") # Directory per l'output
CALIB_FILE  = os.path.join(BASE_DIR, "calibration.npz") # Path file contenente i dati della calibrazione

# Creazione di database.db e cartelle sparse e dense
DATABASE    = os.path.join(WORKSPACE, "database.db")
SPARSE_DIR  = os.path.join(WORKSPACE, "sparse")
DENSE_DIR   = os.path.join(WORKSPACE, "dense")


# Funzione per eseguire un comando e verificare se ha funzionato
def run(cmd: list[str], step: str) -> None:
    print(f"\n{'='*60}")
    print(f"  STEP: {step}")
    print(f"{'='*60}")
    print("CMD:", " ".join(cmd))
    result = subprocess.run(cmd, check=False)
    if result.returncode != 0:
        raise RuntimeError(
            f"[ERRORE] Step '{step}' fallito con codice {result.returncode}"
        )
    print(f"{step} completato.")


# Funzione per leggere i parametri dal file calibration.npz
def load_calibration(npz_path: str) -> str:
    data = np.load(npz_path)
    mtx  = data["mtx"]   # estrazione matrice camera 3x3
    dist = data["dist"]  # estrazione valori distorsione k1 k2 p1 p2 k3

    fx, fy = float(mtx[0, 0]), float(mtx[1, 1]) #recupero lunghezze focali
    cx, cy = float(mtx[0, 2]), float(mtx[1, 2]) #recupero centri immagine
    k1, k2, p1, p2 = float(dist[0,0]), float(dist[0,1]), float(dist[0,2]), float(dist[0,3])
    # k1 e k2 distorsione radiale, p1 e p2 distorsione tangenziale

    camera_params = f"{fx},{fy},{cx},{cy},{k1},{k2},{p1},{p2}" # sistemare nel formato richiesto da colmap
    print(f"\n[Calibrazione caricata da {npz_path}]")
    print(f"  fx={fx:.4f}  fy={fy:.4f}  cx={cx:.4f}  cy={cy:.4f}")
    print(f"  k1={k1:.6f}  k2={k2:.6f}  p1={p1:.6f}  p2={p2:.6f}")
    return camera_params

# Creazione struttura richiesta da colmap
def prepare_workspace() -> None:
    for d in [WORKSPACE, SPARSE_DIR, DENSE_DIR]:
        os.makedirs(d, exist_ok=True) #Se non esistono già le crea
    # Se esiste rimuove il database precedente per evitare conflitti
    if os.path.isfile(DATABASE):
        os.remove(DATABASE)
        print(f"[Info] Database precedente rimosso: {DATABASE}")
    print(f"[Workspace pronto: {WORKSPACE}]")


def main() -> None:
    # 1) Preparazione
    prepare_workspace()
    camera_params = load_calibration(CALIB_FILE)

    # Avvio di Colmap
    def colmap(*args):
        return ["cmd", "/c", COLMAP_BAT] + list(args)

    # 2) Estrazione delle Feature

    run(colmap(
        "feature_extractor",
        "--database_path",              DATABASE,
        "--image_path",                 IMAGE_DIR,
        "--ImageReader.camera_model",   "OPENCV",
        "--ImageReader.camera_params",  camera_params, # Uso la mia calibrazione calcolata prima
        "--ImageReader.single_camera",  "1", # Impone che tutte le immagini provengono dalla stessa camera
        "--FeatureExtraction.use_gpu",  "1", # Flag per usare CPU
    ), "Feature Extraction")

    # 3) Exhaustive Matching
    run(colmap(
        "exhaustive_matcher",
        "--database_path",           DATABASE,
        "--FeatureMatching.use_gpu", "1",
    ), "Exhaustive Feature Matching")

    # 4) Ricostruzione sparsa
    run(colmap(
        "mapper",
        "--database_path", DATABASE,
        "--image_path",    IMAGE_DIR,
        "--output_path",   SPARSE_DIR,
    ), "Sparse Reconstruction (SfM)")  # Crea posizione camere e punti 3D base

    # 5) Non distorsione
    sparse_model = os.path.join(SPARSE_DIR, "0") # Ricerca del modello sparso creato in precedenza
    if not os.path.isdir(sparse_model):
        candidates = sorted([
            d for d in os.listdir(SPARSE_DIR)
            if os.path.isdir(os.path.join(SPARSE_DIR, d))
        ])
        if not candidates:
            raise RuntimeError(
                "Nessun modello sparse trovato in " + SPARSE_DIR +
                "\nVerifica che la fase di mapping abbia prodotto risultati."
            )
        sparse_model = os.path.join(SPARSE_DIR, candidates[0])
    print(f"[Info] Modello sparse usato: {sparse_model}")

    run(colmap(
        "image_undistorter",
        "--image_path",      IMAGE_DIR,
        "--input_path",      sparse_model,
        "--output_path",     DENSE_DIR,
        "--output_type",     "COLMAP",
        "--max_image_size",  "2000",
    ), "Image Undistortion") # Correggere le immagini usando la calibrazione

    # 6) Ricostruzione densa
    run(colmap(
        "patch_match_stereo",
        "--workspace_path",                    DENSE_DIR,
        "--workspace_format",                  "COLMAP",
        "--PatchMatchStereo.geom_consistency", "true",
    ), "Dense Stereo (PatchMatch)") # Genera tantissimi punti 3D, molto più dettagliato di sparse

    # 7) Fusione stereo
    fused_ply = os.path.join(DENSE_DIR, "fused.ply")
    run(colmap(
        "stereo_fusion",
        "--workspace_path",   DENSE_DIR,
        "--workspace_format", "COLMAP",
        "--input_type",       "geometric",
        "--output_path",      fused_ply,
    ), "Stereo Fusion → fused.ply") # Unisce tutti i punti 3D nel modello 3D finale


    # Output
    print("\n" + "="*60)
    print("  RICOSTRUZIONE COMPLETATA")
    print("="*60)
    print(f"  Sparse model : {sparse_model}")
    print(f"  Dense cloud  : {fused_ply}")
    print("Generato il file fused.ply\n")


if __name__ == "__main__":
    main()