
import subprocess
import re
import numpy as np
import time
import sys

def run_benchmark(iteration):
    print(f"\n[Bench] Starting Run {iteration+1}...")
    start_time = time.time()
    
    try:
        # Run reproduce_ate.sh and capture output
        # cwd='/home/cheng/catkin_ws' is important
        result = subprocess.run(['./reproduce_ate.sh'], 
                                cwd='/home/cheng/catkin_ws',
                                stdout=subprocess.PIPE, 
                                stderr=subprocess.STDOUT, # Capture stderr too
                                text=True)
        
        output = result.stdout
        
        # Save log for debugging
        with open(f"bench_run_{iteration+1}.log", "w") as f:
            f.write(output)

        # Print Alignment info
        align_lines = [line for line in output.splitlines() if "[UWB Align]" in line]
        for l in align_lines:
            print(f"    {l}")

        # Check for success
        if result.returncode != 0:
            print(f"[Bench] Run {iteration+1} FAILED with exit code {result.returncode}")
            return None
            
        # Parse ATE
        match = re.search(r"RMSE \(ATE\): ([\d\.]+) m", output)
        if match:
            ate = float(match.group(1))
            duration = time.time() - start_time
            print(f"[Bench] Run {iteration+1} Success. ATE: {ate:.4f} m (Time: {duration:.1f}s)")
            return ate
        else:
            print(f"[Bench] Run {iteration+1} Finished but ATE not found.")
            return None
            
    except Exception as e:
        print(f"[Bench] Run {iteration+1} Exception: {e}")
        return None

def main():
    N_RUNS = 10
    ates = []
    
    print(f"==================================================")
    print(f"      Running VINS-Mono Benchmark {N_RUNS} times")
    print(f"==================================================")
    
    for i in range(N_RUNS):
        ate = run_benchmark(i)
        if ate is not None:
            ates.append(ate)
        else:
            print("[Bench] Warning: Skipping failed run.")
    
    print("\n==================================================")
    print(f"                 FINAL RESULTS                    ")
    print("==================================================")
    
    if len(ates) > 0:
        mean_ate = np.mean(ates)
        std_ate = np.std(ates)
        min_ate = np.min(ates)
        max_ate = np.max(ates)
        
        print(f"Total Successful Runs: {len(ates)}/{N_RUNS}")
        print(f"ATE Values: {[round(x, 4) for x in ates]}")
        print(f"MIN ATE : {min_ate:.4f} m")
        print(f"MAX ATE : {max_ate:.4f} m")
        print(f"AVG ATE : {mean_ate:.4f} m")
        print(f"STD DEV : {std_ate:.4f} m")
    else:
        print("No successful runs recorded.")

if __name__ == "__main__":
    main()
