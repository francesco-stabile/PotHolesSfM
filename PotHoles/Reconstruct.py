import os
import subprocess
import numpy as np
import open3d as o3d

PROJECT_PATH = os.getcwd()
IMAGE_PATH = os.path.join(PROJECT_PATH, "Hole1")
DATABASE_PATH = os.path.join(PROJECT_PATH, "database.db")
SPARSE_PATH = os.path.join(PROJECT_PATH, "sparse")

if os.path.exists(DATABASE_PATH):
    os.remove(DATABASE_PATH)

if not os.path.exists(SPARSE_PATH):
    os.makedirs(SPARSE_PATH)

calibration = np.load('calibration.npz')
mtx = calibration['mtx']
dist = calibration['dist']

fx, fy = mtx[0, 0], mtx[1, 1]
cx, cy = mtx[0, 2], mtx[1, 2]

k1, k2, p1, p2 = dist[0, 0], dist[0, 1], dist[0, 2], dist[0, 3]
params = f"{fx},{fy},{cx},{cy},{k1},{k2},{p1},{p2}"

def run_command(cmd):
    print(f"Eseguendo: {cmd}")
    subprocess.run(cmd, shell=True, check=True)

run_command(f"colmap feature_extractor "
            f"--database_path {DATABASE_PATH} "
            f"--image_path {IMAGE_PATH} "
            f"--ImageReader.camera_model OPENCV "
            f"--ImageReader.camera_params {params}")

run_command(f"colmap exhaustive_matcher --database_path {DATABASE_PATH}")

run_command(f"colmap mapper "
            f"--database_path {DATABASE_PATH} "
            f"--image_path {IMAGE_PATH} "
            f"--output_path {SPARSE_PATH}")

print("\n--- Ricostruzione Sparsa Completata ---")
print(f"Puoi visualizzare il risultato aprendo la GUI di COLMAP e andando su:")
print(f"File > Import Model > Seleziona la cartella: {SPARSE_PATH}/0")

run_command(f"colmap model_converter --input_path ./dense/sparse --output_path ./dense/sparse --output_type TXT")

inner_commands = (
    "InterfaceCOLMAP -i dense -o model.mvs && "
    "DensifyPointCloud model.mvs"
)

inner_commands_mesh = (
    "InterfaceCOLMAP -i dense -o model.mvs && "
    "DensifyPointCloud model.mvs && "
    "ReconstructMesh model_dense.mvs && "
    "TextureMesh model_dense_mesh.mvs"
)

docker_cmd = f'docker run --rm -v "$(pwd):/data" -w /data openmvs/openmvs-ubuntu /bin/bash -c "{inner_commands}"'

try:
    run_command(docker_cmd)
    print("\n--- Pipeline OpenMVS completata con successo! ---")
except subprocess.CalledProcessError as e:
    print(f"\n--- Errore durante l'esecuzione di Docker: {e} ---")

pcd = o3d.io.read_point_cloud("model_dense.ply")
plane_model, inliers = pcd.segment_plane(
    distance_threshold=0.01,
    ransac_n=3,
    num_iterations=10000
)

[a, b, c, d] = plane_model
print(f"Piano stimato: {a:.4f}x + {b:.4f}y + {c:.4f}z + {d:.4f} = 0")

inlier_cloud = pcd.select_by_index(inliers)
outlier_cloud = pcd.select_by_index(inliers, invert=True)

inlier_cloud.paint_uniform_color([0, 1, 0])   # verde
outlier_cloud.paint_uniform_color([1, 0, 0])  # rosso

points = np.asarray(pcd.points)

norm = np.sqrt(a**2 + b**2 + c**2)
a, b, c, d = a/norm, b/norm, c/norm, d/norm

outlier_cloud = pcd.select_by_index(inliers, invert=True)
outlier_cloud, ind = outlier_cloud.remove_statistical_outlier(nb_neighbors=30, std_ratio=1.5)
outlier_points = np.asarray(outlier_cloud.points)

signed_dist = (a*outlier_points[:,0] + b*outlier_points[:,1] + c*outlier_points[:,2] + d)

if np.percentile(signed_dist, 5) < 0 and np.percentile(signed_dist, 95) <= 0.02:
    target_dist = signed_dist
else:
    target_dist = -signed_dist

threshold_depth = np.percentile(target_dist, 0.5)
deep_indices = np.where(target_dist <= threshold_depth)[0]

deepest_point = np.mean(outlier_points[deep_indices], axis=0)
max_depth = np.abs(np.mean(target_dist[deep_indices]))

print(f"Profondità massima robusta: {max_depth:.4f}")
print(f"Punto più profondo stimato: {deepest_point}")

deep_point = o3d.geometry.PointCloud()
deep_point.points = o3d.utility.Vector3dVector([deepest_point])
deep_point.paint_uniform_color([0, 0, 1])  # blu

o3d.io.write_point_cloud("plane_points.ply", inlier_cloud)
o3d.io.write_point_cloud("hole_points.ply", outlier_cloud)
o3d.io.write_point_cloud("deep_point.ply", deep_point)