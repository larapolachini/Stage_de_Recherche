# -----------------------------------------------------------------------------#
# 0. imports                                                                   #
# -----------------------------------------------------------------------------#
import sys, pathlib
scripts_dir = pathlib.Path(__file__).resolve().parent / "scripts"
sys.path.insert(0, str(scripts_dir))

import numpy as np
import pandas as pd
from pathlib import Path

import torch
from torch import nn
from torch.utils.data import Dataset, DataLoader

from sklearn.metrics import (
    accuracy_score, precision_recall_fscore_support, classification_report
)
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split, GroupShuffleSplit, GroupKFold

import matplotlib.pyplot as plt
from utils import load_data                       # ← your existing loader
from scripts.plots import _to_long                # ← helper already in plots.py
# -----------------------------------------------------------------------------#
# 1. user parameters                                                            #
# -----------------------------------------------------------------------------#

DATA_FILE   = "results/result.feather"   # Feather / CSV
WINDOW_LEN  = 50                        # time steps per slice
INPUT_DIM = 3 * WINDOW_LEN
SHIFT       = 50                        # slide window by this amount
BATCH_SIZE  = 128
LR          = 1e-3
EPOCHS      = 50
HIDDEN      = (8, )                  # MLP hidden sizes
TEST_SIZE   = 0.25                       # fraction of windows for validation
DEVICE      = "cuda" if torch.cuda.is_available() else "cpu"
RNG         = 42

# -----------------------------------------------------------------------------#
# 2. build one big  (n_samples × 3·WINDOW_LEN)  feature matrix                 #
# -----------------------------------------------------------------------------#
def make_windows(series: np.ndarray, wlen=WINDOW_LEN, shift=SHIFT):
    """Return list of sliding windows cut from a 1-D array."""
    if len(series) < wlen:
        return []
    idx0 = range(0, len(series) - wlen + 1, shift)
    return [series[i : i + wlen] for i in idx0]

df_raw, _ = load_data(DATA_FILE)
df_long   = _to_long(df_raw)              # adds 'session' and 's'

features, labels = [], []
groups = []


# --- NEW: group by arena/run/iteration ONLY -----------------------------------
for (arena, run, it), g_iter in df_long.groupby(
        ["arena_file", "run", "current_it"]):

    # Collect the three session arrays, sorted by time
    sess_arrays = []
    complete = True
    for sess_id in (0, 1, 2):
        g_sess = g_iter[g_iter["session"] == sess_id].sort_values("time")
        if g_sess.empty:
            complete = False
            break                        # skip if any session missing
        sess_arrays.append(g_sess["s"].to_numpy(dtype=float))

    if not complete:
        continue

    # Length of the *shortest* session controls windowing
    min_len = min(map(len, sess_arrays))
    if min_len < WINDOW_LEN:
        continue

    # Use any one array to compute cut points
    for i0 in range(0, min_len - WINDOW_LEN + 1, SHIFT):
        full_vec = np.hstack([sa[i0 : i0 + WINDOW_LEN] for sa in sess_arrays])
        features.append(full_vec)
        labels.append(arena)
        groups.append(run)

X = np.vstack(features).astype(np.float32)      # (n_samples, 3·WINDOW_LEN)
y = np.array(labels)
print(f"dataset: {X.shape[0]} samples × {X.shape[1]} features")


# X before scaling: n_windows × 150  (or 300/450)    
col_means = X.mean(axis=0)          # length 150
col_stds  = X.std(axis=0)

print('mean range ', col_means.min(), col_means.max())
print('std  range ', col_stds.min(),  col_stds.max())


# ------------------------------------------------------------------+
# 2-bis. DOWN-SAMPLE: keep the same #windows for every arena class  |
# ------------------------------------------------------------------+
rng = np.random.default_rng(RNG)       # reproducible shuffles

X       = np.vstack(features).astype(np.float32)   # (n_samples, 3·WINDOW_LEN)
y       = np.array(labels)                         # class labels (strings)
groups  = np.array(groups)                         # run-ids (ints or strs)

# gather the indices belonging to each class
class_to_idx = {c: np.where(y == c)[0] for c in np.unique(y)}

# smallest class size  →  target size for *all* classes
n_target = min(len(idx) for idx in class_to_idx.values())
print(f"Down-sampling every arena to {n_target} windows")

# randomly pick n_target indices from every class
sel_idx = np.concatenate([
    rng.choice(idx, size=n_target, replace=False)  # no replacement!
    for idx in class_to_idx.values()
])

# keep only the selected windows (order unimportant, so shuffle once)
rng.shuffle(sel_idx)               # optional but nice
X      = X[sel_idx]
y      = y[sel_idx]
groups = groups[sel_idx]
# ------------------------------------------------------------------+



# -----------------------------------------------------------------------------#
# 3. split + scale (scikit-learn)                                               #
# -----------------------------------------------------------------------------#
# 3-a.  Build a vector with one group-id per *window* you created
#       (⟂ same order as `features`, `labels`)
groups = np.array(groups)

# 3-b.  Group-aware split: all windows coming from the *same run* go to
#       either the train set or the test set, never both.
gss = GroupShuffleSplit(test_size=TEST_SIZE, random_state=RNG, n_splits=1)
(train_idx, test_idx), = gss.split(X, y, groups=groups)

X_tr, X_te = X[train_idx], X[test_idx]
y_tr, y_te = y[train_idx], y[test_idx]

# 3-c.  Standardise features (as before)
scaler = StandardScaler()
X_tr = scaler.fit_transform(X_tr).astype(np.float32)
X_te = scaler.transform(X_te).astype(np.float32)

class ArenaDataset(Dataset):
    def __init__(self, X, y, label2idx):
        self.X = torch.from_numpy(X)
        self.y = torch.tensor([label2idx[l] for l in y], dtype=torch.long)
    def __len__(self):  return len(self.X)
    def __getitem__(self, idx):
        return self.X[idx], self.y[idx]

label2idx = {lbl: i for i, lbl in enumerate(sorted(np.unique(y)))}
idx2label = {v: k for k, v in label2idx.items()}

train_ds = ArenaDataset(X_tr, y_tr, label2idx)
test_ds  = ArenaDataset(X_te, y_te, label2idx)
train_dl = DataLoader(train_ds, batch_size=BATCH_SIZE, shuffle=True)
test_dl  = DataLoader(test_ds,  batch_size=BATCH_SIZE, shuffle=False)
# -----------------------------------------------------------------------------#
# 4. define the MLP                                                             #
# -----------------------------------------------------------------------------#
class MLP(nn.Module):
    def __init__(self, d_in, hidden, d_out, p_drop: float = 0.20):
        super().__init__()
        layers = []
        prev = d_in
        for h in hidden:
            layers += [nn.Linear(prev, h), nn.BatchNorm1d(h), nn.ReLU(), nn.Dropout(p_drop)]
            prev = h
        layers.append(nn.Linear(prev, d_out))
        self.net = nn.Sequential(*layers)
    def forward(self, x):
        return self.net(x)

model = MLP(INPUT_DIM, HIDDEN, len(label2idx), p_drop = 0.20).to(DEVICE)
opt   = torch.optim.Adam(model.parameters(), lr=LR)
criterion = nn.CrossEntropyLoss()
# -----------------------------------------------------------------------------#
# 5. training loop                                                              #
# -----------------------------------------------------------------------------#
loss_curve = []
for epoch in range(1, EPOCHS + 1):
    model.train()
    running = 0.0
    for xb, yb in train_dl:
        xb, yb = xb.to(DEVICE), yb.to(DEVICE)
        opt.zero_grad()
        logits = model(xb)
        loss = criterion(logits, yb)
        loss.backward()
        opt.step()
        running += loss.item() * xb.size(0)
    epoch_loss = running / len(train_ds)
    loss_curve.append(epoch_loss)
    if epoch % 5 == 0 or epoch == 1:
        print(f"Epoch {epoch:3d}/{EPOCHS}  |  loss {epoch_loss:.4f}")

# -----------------------------------------------------------------------------#
# 6. evaluation                                                                 #
# -----------------------------------------------------------------------------#
model.eval()
y_true, y_pred = [], []
with torch.no_grad():
    for xb, yb in test_dl:
        logits = model(xb.to(DEVICE))
        preds = logits.argmax(dim=1).cpu().numpy()
        y_pred.extend(preds)
        y_true.extend(yb.numpy())

y_true = np.array(y_true)
y_pred = np.array(y_pred)
acc = accuracy_score(y_true, y_pred)
prec, rec, f1, _ = precision_recall_fscore_support(
    y_true, y_pred, labels=list(idx2label.keys()), zero_division=0)

print(f"\nOVERALL ACCURACY: {acc:.3f}\n")
print(classification_report(
    y_true, y_pred,
    target_names=[idx2label[i] for i in sorted(idx2label)],
    digits=3))

# -----------------------------------------------------------------------------#
# 7. plots                                                                      #
# -----------------------------------------------------------------------------#
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))

# learning curve
ax1.plot(loss_curve, lw=2)
ax1.set_xlabel("epoch"); ax1.set_ylabel("loss")
ax1.set_title("Training loss")

# per-class bar chart
x = np.arange(len(idx2label))
w = 0.25
ax2.bar(x - w, prec, width=w, label="prec")
ax2.bar(x,     rec,  width=w, label="recall")
ax2.bar(x + w, f1,   width=w, label="F1")
ax2.set_xticks(x); ax2.set_xticklabels([idx2label[i] for i in x], rotation=30)
ax2.set_ylim(0, 1); ax2.legend()
ax2.set_title("Metrics per class")
plt.tight_layout(); plt.show()
