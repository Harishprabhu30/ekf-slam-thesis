import pandas as pd

def normalize(in_csv, out_csv):
    df = pd.read_csv(in_csv).sort_values("timestamp")
    t0 = df["timestamp"].iloc[0]
    df["t_rel"] = (df["timestamp"] - t0) * 1e-9  # seconds
    df.to_csv(out_csv, index=False)
    print(f"Saved: {out_csv}  (t0={t0})  duration_s={df['t_rel'].iloc[-1]:.3f}")

if __name__ == "__main__":
    import sys
    normalize(sys.argv[1], sys.argv[2])
    
