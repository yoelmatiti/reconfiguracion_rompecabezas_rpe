import subprocess
import csv
import os

EXECUTABLE = "./bin/rpe_solver"
# Usa la instancia que probaste manualmente y funcionó:
INSTANCE = "instances/complex_test.rpe"
SEEDS = [42, 101, 2024, 777, 999]
ETA_VALUES = [0.0, 0.25, 0.50, 0.75, 1.0]
LAMBDA = 0.0
TIME_MS = 2000

# Guardar dentro de la carpeta results/
output_dir = "results"
os.makedirs(output_dir, exist_ok=True)
output_file = os.path.join(output_dir, "resultados_grasp.csv")

rows = []

print("=== Ejecutando experimentos GRASP (5 Semillas x 5 Etas) ===")

for eta in ETA_VALUES:
    for seed in SEEDS:
        cmd = [
            EXECUTABLE,
            "--instance", INSTANCE,
            "--eta", str(eta),
            "--lambda", str(LAMBDA),
            "--seed", str(seed),
            "--time_ms", str(TIME_MS)
        ]
        
        proc = subprocess.run(cmd, capture_output=True, text=True)
        
        # Leemos stdout y stderr por si acaso
        output_lines = proc.stdout.splitlines() + proc.stderr.splitlines()
        
        for line in output_lines:
            line = line.strip()
            if line.startswith("RESULT,"):
                parts = line.split(",")
                rows.append(parts[1:])  # Instancia, Eta, Lambda, Semilla, Costo, TiempoMs
                print(f"Eta: {eta:<4} | Seed: {seed:<5} | Costo: {parts[5]:<6} | Tiempo: {parts[6]} ms")

# Guardar en el directorio results/
with open(output_file, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["Instancia", "Eta", "Lambda", "Semilla", "Costo", "Tiempo_ms"])
    writer.writerows(rows)

print(f"\n¡Listo! Resultados guardados en: {output_file}")