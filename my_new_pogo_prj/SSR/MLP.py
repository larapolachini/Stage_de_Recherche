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
from sklearn.model_selection import train_test_split

import matplotlib.pyplot as plt                      
from plots import _to_long          
from utils import load_data   
from collections import Counter, defaultdict      
# -----------------------------------------------------------------------------#
# 1. user parameters                                                            #
# -----------------------------------------------------------------------------#
DATA_FILE   = "results/result.feather"   # Feather / CSV
WINDOW_LEN  = 100                        # time steps per slice
INPUT_DIM   = 3 * WINDOW_LEN
SHIFT       = 50                        # slide window by this amount
EARLY_ONLY = False
BURN_IN_ONLY = True
BURN_IN_STEPS = 50
BATCH_SIZE  = 128
LR          = 1e-3
EPOCHS      = 100
HIDDEN      = (64, 32)                  # MLP hidden sizes
DROPOUT_P = 0.25
TEST_SIZE   = 0.25                       # fraction of windows for validation
DEVICE      = "cuda" if torch.cuda.is_available() else "cpu"
RNG         = 42

# -----------------------------------------------------------------------------#
# 2. build one big (n_samples × 3×WINDOW_LEN) feature matrix                    #
# -----------------------------------------------------------------------------#

df_raw, _ = load_data(DATA_FILE)   
df_long   = _to_long(df_raw) 
features, labels, group_ids = [], [], [] 

def make_windows_three_sessions(sess_arrays):
    """
    sess_arrays = [s0_array, s1_array, s2_array]  (already time-sorted)
    Returns list of [s0|s1|s2] windows according to the chosen scheme.
    """
    # -- 1. ensure the 3 arrays are equally long (use the shortest) ----------
    min_len = min(map(len, sess_arrays))
    if EARLY_ONLY:
        if min_len < WINDOW_LEN:
            return []                       # drop if even the beginning is too short
        idx_starts = [0]                    # exactly one window: first WINDOW_LEN
    elif BURN_IN_ONLY:
        start0 = BURN_IN_STEPS

        # drop the iteration if it doesn't reach far enough past burn-in
        if min_len < start0 + WINDOW_LEN:
            return []

        # many windows, all starting AFTER the burn-in zone
        idx_starts = range(start0, min_len - WINDOW_LEN + 1, SHIFT)
    else:                                    # <── you lost this branch
        # FULL sliding-window behaviour
        idx_starts = range(0, min_len - WINDOW_LEN + 1, SHIFT)

    # -- 2. build the windows ------------------------------------------------
    windows = []
    for i0 in idx_starts:
        win = np.hstack([sa[i0 : i0 + WINDOW_LEN] for sa in sess_arrays])
        windows.append(win)
    return windows


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

    
    for full_vec in make_windows_three_sessions(sess_arrays):
        features.append(full_vec)
        labels.append(arena)
        group_ids.append((arena, run, it))

group_ids = np.array(group_ids, dtype=object)
X = np.vstack(features).astype(np.float32)      # (n_samples, 3·WINDOW_LEN)
y = np.array(labels)
print(f"dataset: {X.shape[0]} samples × {X.shape[1]} features")
# -----------------------------------------------------------------------------#
# 3. split + scale (scikit-learn)                                               #
# -----------------------------------------------------------------------------#
X_tr, X_te, y_tr, y_te, gid_tr, gid_te = train_test_split(
    X, y, group_ids, test_size=TEST_SIZE, stratify=y, random_state=RNG)

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
    def __init__(self, d_in, hidden, d_out, p_drop):
        super().__init__()
        layers = []
        prev = d_in
        for h in hidden:
            layers += [nn.Linear(prev, h), nn.ReLU(), nn.Dropout(p_drop)]
            prev = h
        layers.append(nn.Linear(prev, d_out))
        self.net = nn.Sequential(*layers)
    def forward(self, x):
        return self.net(x)

model = MLP(INPUT_DIM, HIDDEN, len(label2idx), DROPOUT_P).to(DEVICE)
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
y_true, y_pred, gid_seq = [], [], []
with torch.no_grad():
    start = 0
    for xb, yb in test_dl:
        bs = xb.size(0)
        logits = model(xb.to(DEVICE))
        preds = logits.argmax(dim=1).cpu().numpy()
        y_pred.extend(preds)
        y_true.extend(yb.numpy())
        gid_seq.extend(gid_te[start : start + bs])
        start += bs

y_true = np.array(y_true)
y_pred = np.array(y_pred)
acc = accuracy_score(y_true, y_pred)
prec, rec, f1, _ = precision_recall_fscore_support(
    y_true, y_pred, labels=list(idx2label.keys()), zero_division=0)

print("\n=====  Individual robot  =====")
print(f"ACCURACY (individual): {acc:.3f}\n")
print(classification_report(
    y_true, y_pred,
    target_names=[idx2label[i] for i in sorted(idx2label)],
    digits=3))

# 1. collect individual votes per swarm
votes   = defaultdict(list)
for gid, pred in zip(gid_seq, y_pred):
    votes[tuple(gid)].append(pred)

# 2. resolve the vote for every swarm
swarm_pred, swarm_true = [], []
for gid, pred_list in votes.items():
    # majority class (break ties by .most_common order)
    voted = Counter(pred_list).most_common(1)[0][0]

    swarm_pred.append(voted)
    # gid[0] is the arena label string, convert to the same integer coding
    swarm_true.append(label2idx[gid[0]])

# 3. swarm-level metrics
from sklearn.metrics import accuracy_score, classification_report
print("\n=====  Swarm-level vote  =====")
print(f"ACCURACY (swarm): {accuracy_score(swarm_true, swarm_pred):.3f}\n")

print(classification_report(
    swarm_true,
    swarm_pred,
    target_names=[idx2label[i] for i in sorted(idx2label)],
    digits=3))

sw_prec, sw_rec, sw_f1, _ = precision_recall_fscore_support(
    swarm_true, swarm_pred,
    labels=list(idx2label.keys()),
    zero_division=0)


# -----------------------------------------------------------------------------#
# 7. plots                                                                      #
# -----------------------------------------------------------------------------#
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 4))

# --- (a) learning curve -------------------------------------------------------
ax1.plot(loss_curve, lw=2)
ax1.set_xlabel("epoch"); ax1.set_ylabel("loss")
ax1.set_title("Training loss")

# --- (b) window-level bars ----------------------------------------------------
x = np.arange(len(idx2label))
w = 0.25
ax2.bar(x - w, prec, width=w, label="prec")
ax2.bar(x,     rec,  width=w, label="recall")
ax2.bar(x + w, f1,   width=w, label="F1")
ax2.set_xticks(x); ax2.set_xticklabels([idx2label[i] for i in x], rotation=30)
ax2.set_ylim(0, 1); ax2.set_title("Individual robot metrics")
ax2.legend()

# --- (c) swarm-level bars -----------------------------------------------------
ax3.bar(x - w, sw_prec, width=w, label="prec")
ax3.bar(x,     sw_rec,  width=w, label="recall")
ax3.bar(x + w, sw_f1,   width=w, label="F1")
ax3.set_xticks(x); ax3.set_xticklabels([idx2label[i] for i in x], rotation=30)
ax3.set_ylim(0, 1); ax3.set_title("Swarm-vote metrics")
ax3.legend()

# === 8. Accuracy bar plot ===
window_acc = accuracy_score(y_true, y_pred)
swarm_acc = accuracy_score(swarm_true, swarm_pred)

fig_acc, ax_acc = plt.subplots(figsize=(5, 5))

ax_acc.bar(["Individual robot", "Swarm"],
           [window_acc, swarm_acc],
           color=["skyblue", "orange"])

ax_acc.set_ylim(0, 1.08)
ax_acc.set_ylabel("Accuracy")
ax_acc.set_title("Classification Accuracy")
for i, v in enumerate([window_acc, swarm_acc]):
    ax_acc.text(i, v + 0.02, f"{v:.3f}", ha='center', fontweight='bold')

plt.tight_layout()
plt.savefig("accuracy_comparison.png", dpi=300)
plt.close(fig_acc) 


plt.tight_layout(); plt.show()

