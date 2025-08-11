#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# -------------------------------------------------------------- #
# 0. Imports                                                     #
# -------------------------------------------------------------- #
import sys, pathlib
scripts_dir = pathlib.Path(__file__).resolve().parent / "scripts"
sys.path.insert(0, str(scripts_dir))

import numpy as np
import torch
from torch import nn
from torch.utils.data import Dataset, DataLoader

from sklearn.metrics import (
    accuracy_score, precision_recall_fscore_support, classification_report
)
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA

import matplotlib.pyplot as plt

from plots import _to_long
from utils  import load_data
from collections import Counter, defaultdict

# -------------------------------------------------------------- #
# 1. User parameters                                             #
# -------------------------------------------------------------- #
DATA_FILE      = "results/result.feather"
SUBSTEP        = 3                 # keep 1 point every 3
WINDOW_LEN     = 100               # window length *after* sub-sampling
RAW_PER_WIN    = 3 * WINDOW_LEN    # 3 sessions × 100 → 300 raw features
SHIFT          = 50 // SUBSTEP     # slide stride in sub-sampled steps
BURN_IN_STEPS  = 50 // SUBSTEP
BURN_IN_ONLY   = True

BATCH_SIZE     = 128
LR             = 1e-3
EPOCHS         = 200
HIDDEN         = (16, 8)
DROPOUT_P      = 0.10
TEST_SIZE      = 0.25

DEVICE         = "cuda" if torch.cuda.is_available() else "cpu"
RNG            = 42
np.random.seed(RNG)
torch.manual_seed(RNG)

# -------------------------------------------------------------- #
# 2. Load data → windows → feature matrix (300 dims per window)  #
# -------------------------------------------------------------- #
df_raw, _ = load_data(DATA_FILE)
df_long   = _to_long(df_raw)

features, labels, group_ids = [], [], []

def windows_from_sessions(sess_arrays):
    """
    sess_arrays: list of 3 arrays (already sorted by time).
    Returns a list of 300-D windows: [s0|s1|s2] with sub-sampling applied.
    """
    # Sub-sample time
    sess_arrays = [sa[::SUBSTEP] for sa in sess_arrays]

    min_len = min(map(len, sess_arrays))
    if min_len < WINDOW_LEN:
        return []

    if BURN_IN_ONLY:
        start0 = BURN_IN_STEPS
        if min_len < start0 + WINDOW_LEN:
            return []
        idx_starts = range(start0, min_len - WINDOW_LEN + 1, SHIFT)
    else:
        idx_starts = range(0, min_len - WINDOW_LEN + 1, SHIFT)

    wins = []
    for i0 in idx_starts:
        # shape: (3, WINDOW_LEN) → transpose → (WINDOW_LEN, 3) → flatten → (300,)
        block = np.vstack([sa[i0:i0 + WINDOW_LEN] for sa in sess_arrays])
        feat  = block.T.reshape(-1)              # keep ALL 3×WINDOW_LEN = 300
        wins.append(feat)
    return wins

# Group by arena/run/iteration and extract windows
for (arena, run, it), g_iter in df_long.groupby(["arena_file", "run", "current_it"]):
    sess_arrays = []
    complete = True
    for sid in (0, 1, 2):
        arr = (g_iter[g_iter["session"] == sid]
               .sort_values("time")["s"]
               .to_numpy(dtype=float))
        if arr.size == 0:
            complete = False
            break
        sess_arrays.append(arr)
    if not complete:
        continue

    for win in windows_from_sessions(sess_arrays):
        features.append(win)
        labels.append(arena)
        group_ids.append((arena, run, it))

X = np.vstack(features).astype(np.float32)   # (N, 300)
y = np.array(labels)
group_ids = np.array(group_ids, dtype=object)

assert X.shape[1] == RAW_PER_WIN, f"Expected {RAW_PER_WIN} features, got {X.shape[1]}"
print(f"dataset: {X.shape[0]} samples × {X.shape[1]} features")

# -------------------------------------------------------------- #
# 3. Train / test split + scale + PCA→50                         #
# -------------------------------------------------------------- #
X_tr, X_te, y_tr, y_te, gid_tr, gid_te = train_test_split(
    X, y, group_ids, test_size=TEST_SIZE, stratify=y, random_state=RNG
)

# Scale (fit on train only)
scaler = StandardScaler()
X_tr = scaler.fit_transform(X_tr).astype(np.float32)
X_te = scaler.transform(X_te).astype(np.float32)

# PCA to 50 dims (fit on train only)
pca = PCA(n_components=50, whiten= False, random_state=RNG)
X_tr = pca.fit_transform(X_tr).astype(np.float32)
X_te = pca.transform(X_te).astype(np.float32)

print("after PCA:", X_tr.shape, X_te.shape)  # (*, 50), (*, 50)

INPUT_DIM = X_tr.shape[1]    # should be 50 now

# -------------------------------------------------------------- #
# 4. Datasets & loaders                                          #
# -------------------------------------------------------------- #
class ArenaDS(Dataset):
    def __init__(self, X, y, lbl2idx):
        self.X = torch.from_numpy(X)
        self.y = torch.tensor([lbl2idx[l] for l in y], dtype=torch.long)
    def __len__(self): return len(self.X)
    def __getitem__(self, i): return self.X[i], self.y[i]

lbl2idx = {lbl: i for i, lbl in enumerate(sorted(np.unique(y)))}
idx2lbl = {v: k for k, v in lbl2idx.items()}

train_dl = DataLoader(ArenaDS(X_tr, y_tr, lbl2idx), batch_size=BATCH_SIZE, shuffle=True)
test_dl  = DataLoader(ArenaDS(X_te, y_te, lbl2idx), batch_size=BATCH_SIZE)

# -------------------------------------------------------------- #
# 5. Tiny MLP (16, 8)                                            #
# -------------------------------------------------------------- #
class MLP(nn.Module):
    def __init__(self, d_in, hidden, d_out, p_drop):
        super().__init__()
        layers, prev = [], d_in
        for h in hidden:
            layers += [nn.Linear(prev, h), nn.ReLU(), nn.Dropout(p_drop)]
            prev = h
        layers.append(nn.Linear(prev, d_out))
        self.net = nn.Sequential(*layers)
    def forward(self, x): return self.net(x)

model = MLP(INPUT_DIM, HIDDEN, len(lbl2idx), DROPOUT_P).to(DEVICE)
opt    = torch.optim.Adam(model.parameters(), lr=LR, weight_decay=1e-4)  # add weight_decay=1e-4 if needed
classes = [lbl2idx[k] for k in sorted(lbl2idx)]
counts = np.bincount([lbl2idx[v] for v in y_tr], minlength=len(lbl2idx))
weights = counts.sum() / (len(lbl2idx) * np.maximum(counts, 1))  # inverse-freq
weights = torch.tensor(weights, dtype=torch.float32, device=DEVICE)
crit = nn.CrossEntropyLoss(weight=weights)

# -------------------------------------------------------------- #
# 6. Training                                                    #
# -------------------------------------------------------------- #
loss_curve = []
for epoch in range(1, EPOCHS + 1):
    model.train()
    run = 0.0
    for xb, yb in train_dl:
        xb, yb = xb.to(DEVICE), yb.to(DEVICE)
        opt.zero_grad()
        loss = crit(model(xb), yb)
        loss.backward()
        opt.step()
        run += loss.item() * xb.size(0)
    loss_curve.append(run / len(train_dl.dataset))
    if epoch % 10 == 0 or epoch == 1:
        print(f"Epoch {epoch:3d}/{EPOCHS}  |  loss {loss_curve[-1]:.4f}")

# -------------------------------------------------------------- #
# 7. Evaluation (window + swarm vote)                            #
# -------------------------------------------------------------- #
model.eval()
y_true, y_pred, gid_seq = [], [], []
with torch.no_grad():
    start = 0
    for xb, yb in test_dl:
        bs = xb.size(0)
        logits = model(xb.to(DEVICE))
        y_pred.extend(logits.argmax(1).cpu().numpy())
        y_true.extend(yb.numpy())
        gid_seq.extend(gid_te[start:start+bs])
        start += bs

y_true = np.array(y_true)
y_pred = np.array(y_pred)

print("\n=====  Window-level (individual)  =====")
print(f"ACCURACY (window): {accuracy_score(y_true, y_pred):.3f}\n")
print(classification_report(
    y_true, y_pred,
    target_names=[idx2lbl[i] for i in sorted(idx2lbl)],
    digits=3))

# Majority vote per swarm (keep key order explicit)
votes = defaultdict(list)
for gid, p in zip(gid_seq, y_pred):
    votes[tuple(gid)].append(p)

swarm_keys = list(votes.keys())
swarm_pred = [Counter(votes[k]).most_common(1)[0][0] for k in swarm_keys]
swarm_true = [lbl2idx[k[0]] for k in swarm_keys]

print("\n=====  Swarm-level vote  =====")
print(f"ACCURACY (swarm): {accuracy_score(swarm_true, swarm_pred):.3f}\n")
print(classification_report(
    swarm_true, swarm_pred,
    target_names=[idx2lbl[i] for i in sorted(idx2lbl)],
    digits=3))

# -------------------------------------------------------------- #
# 8. Plots (loss + window metrics + swarm metrics)               #
# -------------------------------------------------------------- #
prec, rec, f1, _ = precision_recall_fscore_support(
    y_true, y_pred, labels=list(idx2lbl.keys()), zero_division=0)

sw_prec, sw_rec, sw_f1, _ = precision_recall_fscore_support(
    swarm_true, swarm_pred, labels=list(idx2lbl.keys()), zero_division=0)

fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 4), dpi=110)

# (a) learning curve
ax1.plot(loss_curve, lw=2)
ax1.set_xlabel("epoch"); ax1.set_ylabel("loss"); ax1.set_title("Training loss")

# (b) window metrics
x = np.arange(len(idx2lbl)); w = 0.25
ax2.bar(x - w, prec, width=w, label="prec")
ax2.bar(x     , rec , width=w, label="recall")
ax2.bar(x + w , f1  , width=w, label="F1")
ax2.set_xticks(x)
ax2.set_xticklabels([idx2lbl[i] for i in x], rotation=30)
ax2.set_ylim(0, 1); ax2.set_title("Window metrics"); ax2.legend(frameon=False)

# (c) swarm metrics
ax3.bar(x - w, sw_prec, width=w, label="prec")
ax3.bar(x     , sw_rec , width=w, label="recall")
ax3.bar(x + w , sw_f1  , width=w, label="F1")
ax3.set_xticks(x)
ax3.set_xticklabels([idx2lbl[i] for i in x], rotation=30)
ax3.set_ylim(0, 1); ax3.set_title("Swarm-vote metrics"); ax3.legend(frameon=False)

plt.tight_layout(); plt.show()
