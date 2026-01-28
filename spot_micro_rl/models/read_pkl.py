import pickle
import sys

if len(sys.argv) != 2:
    print("Usage: python read_pkl.py <file.pkl>")
    sys.exit(1)

pkl_file = sys.argv[1]

with open(pkl_file, "rb") as f:
    data = pickle.load(f)

print(f"File: {pkl_file}")
print(f"Type: {type(data)}")

if isinstance(data, dict):
    print("\nKeys:")
    for k, v in data.items():
        print(f"  {k} : {type(v)}")
else:
    print("\nContent:")
    print(data)
