from __future__ import print_function
import numpy as np
import csv
import sys
import os

# Manual Quaternion to Rotation Matrix (Avoid Scipy dependency)
def quat2mat(q):
    # q = [qx, qy, qz, qw]
    x, y, z, w = q
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ])

def read_csv(path):
    data = []
    with open(path, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if row[0].startswith('#') or len(row) < 8:
                continue
            # timestamp, px, py, pz, qx, qy, qz, qw
            vals = [float(x) for x in row[:8]]
            vals[0] = vals[0] / 1e9 
            data.append(vals)
    return np.array(data)

def read_vins_csv(path):
    data = []
    with open(path, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row: continue
            # VINS: time, px, py, pz, qw, qx, qy, qz
            # Need to reorder to: time, px, py, pz, qx, qy, qz, qw
            # Filter empty strings (caused by trailing comma)
            vals = [float(x) for x in row if x.strip()]
            if len(vals) < 8: continue
            #                    t (ns->s)        px       py       pz       qx       qy       qz       qw
            data.append([vals[0]/1e9, vals[1], vals[2], vals[3], vals[5], vals[6], vals[7], vals[4]])
    return np.array(data)

def align_umeyama(model, data):
    """
    Aligns 'data' to 'model' using Umeyama algorithm.
    args:
        model: (N, 3) points
        data: (N, 3) points
    returns:
        R: 3x3 rotation matrix
        t: 3x1 translation vector
        s: scale factor
        aligned_data: (N, 3)
    """
    mu_m = model.mean(0)
    mu_d = data.mean(0)
    data_c = data - mu_d
    model_c = model - mu_m
    
    n = model.shape[0]
    sigma_m = (model_c**2).sum() / n
    sigma_d = (data_c**2).sum() / n
    
    H = data_c.T.dot(model_c) / n
    U, D, Vt = np.linalg.svd(H)
    
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
        
    # Correct Umeyama: R = V @ S @ U.T
    # numpy svd returns Vt (V transposed)
    R_opt = Vt.T.dot(S).dot(U.T)
    # VINS is metric, so we can force scale = 1.0 or estimate it.
    s_opt = 1.0 / sigma_d * np.trace(np.diag(D).dot(S))
    # s_opt = 1.0 # Force metric
    
    t_opt = mu_m - s_opt * R_opt.dot(mu_d)

    aligned_data = s_opt * data.dot(R_opt.T) + t_opt
    return R_opt, t_opt, s_opt, aligned_data

def evaluate(gt_path, est_path):
    print("Loading GT: {}".format(gt_path))
    gt_data = read_csv(gt_path) # [t, x, y, z, qx, qy, qz, qw]
    print("Loaded {} GT points".format(len(gt_data)))
    
    print("Loading Est: {}".format(est_path))
    est_data = read_vins_csv(est_path)
    print("Loaded {} Est points".format(len(est_data)))
    
    # 1. Compensate Prism Offset
    # P_prism = P_body + R_body * T_B_prism
    # T_B_prism = [-0.293656, -0.012288, -0.273095]
    off = np.array([-0.293656, -0.012288, -0.273095])
    
    # Apply offset
    new_est_pos = []
    for i in range(len(est_data)):
        p = est_data[i, 1:4]
        q = est_data[i, 4:8] # qx, qy, qz, qw
        # Manual Rotation Matrix
        r = quat2mat(q)
        p_new = p + r.dot(off)
        new_est_pos.append(p_new)
    
    est_pos = np.array(new_est_pos)
    est_t = est_data[:, 0]
    
    # 2. Sync (Interpolate GT to Est times)
    gt_t = gt_data[:, 0]
    gt_pos = gt_data[:, 1:4]

    # SKIP INITIALIZATION (Fix for high ATE due to takeoff drift)
    # The first few seconds of VINS are usually unstable.
    SKIP_SECONDS = 0.0
    # SKIP_SECONDS = 30.0
    start_t = est_t.min() + SKIP_SECONDS
    mask_skip = est_t > start_t
    
    est_t = est_t[mask_skip]
    est_pos = est_pos[mask_skip]
    
    # Simple linear interp
    gt_pos_sync = np.zeros_like(est_pos)
    for i in range(3):
        gt_pos_sync[:, i] = np.interp(est_t, gt_t, gt_pos[:, i])
        
    # Filter out-of-bounds
    valid_mask = (est_t >= gt_t.min()) & (est_t <= gt_t.max())
    est_pos = est_pos[valid_mask]
    gt_pos_sync = gt_pos_sync[valid_mask]
    
    print("Aligned {} points for evaluation".format(len(est_pos)))
    
    # Debug: Check bounds
    print("GT Bounds:")
    print("  X: {:.2f} to {:.2f}".format(gt_pos_sync[:,0].min(), gt_pos_sync[:,0].max()))
    print("  Y: {:.2f} to {:.2f}".format(gt_pos_sync[:,1].min(), gt_pos_sync[:,1].max()))
    print("  Z: {:.2f} to {:.2f}".format(gt_pos_sync[:,2].min(), gt_pos_sync[:,2].max()))
    
    print("Est Bounds (Before Align):")
    print("  X: {:.2f} to {:.2f}".format(est_pos[:,0].min(), est_pos[:,0].max()))
    print("  Y: {:.2f} to {:.2f}".format(est_pos[:,1].min(), est_pos[:,1].max()))
    print("  Z: {:.2f} to {:.2f}".format(est_pos[:,2].min(), est_pos[:,2].max()))

    # 3. Align (Sim3 or SE3)
    # Using SE3 (s=1)
    Rot, Trans, Scale, est_aligned = align_umeyama(gt_pos_sync, est_pos)
    
    print("Est Bounds (After Align):")
    print("  X: {:.2f} to {:.2f}".format(est_aligned[:,0].min(), est_aligned[:,0].max()))
    print("  Y: {:.2f} to {:.2f}".format(est_aligned[:,1].min(), est_aligned[:,1].max()))
    print("  Z: {:.2f} to {:.2f}".format(est_aligned[:,2].min(), est_aligned[:,2].max()))
    
    # 4. RMSE
    error = gt_pos_sync - est_aligned
    rmse = np.sqrt(np.mean(np.sum(error**2, axis=1)))
    
    # Print first 5 pairs
    print("First 5 entries (GT vs Aligned Est):")
    for i in range(5):
        diff = np.linalg.norm(gt_pos_sync[i] - est_aligned[i])
        print("  GT: {} | Est: {} | Diff: {:.2f}".format(gt_pos_sync[i], est_aligned[i], diff))
    
    print("-" * 30)
    print("Alignment Result (Full Alignment):")
    print("Scale: {:.4f}".format(Scale))
    print("Translation: {}".format(Trans))
    print("RMSE (ATE): {:.4f} m".format(rmse))
    print("-" * 30)

    # 4.5 Drift Analysis (Align using only start)
    # Align using first 10 seconds of overlap
    DRIFT_WIN = 10.0
    drift_mask = (est_t < est_t.min() + DRIFT_WIN)
    if np.sum(drift_mask) > 10:
        drift_gt = gt_pos_sync[drift_mask]
        drift_est = est_pos[drift_mask]
        
        # Align using start only
        R_d, t_d, s_d, _ = align_umeyama(drift_gt, drift_est)
        
        # Apply this START alignment to FULL trajectory
        est_drift_aligned = s_d * est_pos.dot(R_d.T) + t_d
        
        # Calc error
        error_d = gt_pos_sync - est_drift_aligned
        rmse_d = np.sqrt(np.mean(np.sum(error_d**2, axis=1)))
        
        print("Drift Analysis (Aligned by first {}s):".format(DRIFT_WIN))
        print("Scale: {:.4f}".format(s_d))
        print("RMSE (Drift): {:.4f} m".format(rmse_d))
        print("Final Error: {:.4f} m".format(np.linalg.norm(error_d[-1])))
        print("-" * 30)
    else:
        print("Not enough data for drift analysis.")

    try:
        import matplotlib
        matplotlib.use('Agg') # Non-interactive backend
        import matplotlib.pyplot as plt
        plt.figure(figsize=(10, 10))
        plt.plot(gt_pos_sync[:,0], gt_pos_sync[:,1], 'r-', label='Ground Truth')
        plt.plot(est_aligned[:,0], est_aligned[:,1], 'b-', label='Estimated (Aligned)')
        plt.legend()
        plt.title("Trajectory Comparison (XY Plane) ATE: {:.4f}m".format(rmse))
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.axis('equal')
        plt.savefig('/root/catkin_ws/vins_output/comparison.png')
        print("Saved plot to /root/catkin_ws/vins_output/comparison.png")
    except ImportError:
        print("matplotlib not found, skipping plot.")
    except Exception as e:
        print("Plotting failed: {}".format(e))
    
    if rmse < 0.2:
        print("SUCCESS: RMSE is below 0.2m!")
    else:
        print("WARNING: RMSE is high. Check calibration or synchronization.")

if __name__ == "__main__":
    gt_file = "/root/catkin_ws/vins_output/leica_pose.csv"
    est_file = "/root/catkin_ws/vins_output/vins_result_loop.csv"
    
if __name__ == "__main__":
    if len(sys.argv) >= 3:
        gt_file = sys.argv[1]
        est_file = sys.argv[2]
    else:
        # Default (Docker)
        gt_file = "/root/catkin_ws/vins_output/leica_pose.csv"
        est_file = "/root/catkin_ws/vins_output/vins_result_loop.csv"
    
    if not os.path.exists(gt_file):
        print("Error: {} not found".format(gt_file))
        sys.exit(1)
        
    if not os.path.exists(est_file):
        print("Error: {} not found".format(est_file))
        sys.exit(1)
        
    evaluate(gt_file, est_file)
