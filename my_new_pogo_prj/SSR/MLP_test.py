#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ------------------------------------------------------------------- #
# 0 . Imports                                                          #
# ------------------------------------------------------------------- #
import sys, pathlib
scripts_dir = pathlib.Path(__file__).resolve().parent / "scripts"
sys.path.insert(0, str(scripts_dir))

import numpy as np
import torch
from torch import nn
from torch.utils.data import Dataset, DataLoader
from sklearn.metrics import (accuracy_score,
                             precision_recall_fscore_support,
                             classification_report)
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt

from plots import _to_long          # your helper utilities
from utils import load_data
from collections import Counter, defaultdict


# ------------------------------------------------------------------- #
# 1 . User parameters                                                  #
# ------------------------------------------------------------------- #
DATA_FILE  = "results/result.feather"
SUBSTEP    = 3               # keep 1 every 3 points  →  down-sample ×3
WINDOW_LEN = 100             # *after* sub-sampling
INPUT_DIM  = WINDOW_LEN  # 3 sessions × 100 points = 300/3 = 100
SHIFT      = 50 // SUBSTEP   # slide window by 50 raw steps → 16 subs
BURN_IN_STEPS = 50 // SUBSTEP
BURN_IN_ONLY  = True         # use windows after burn-in zone
BATCH_SIZE = 128
LR, EPOCHS = 1e-3, 100
HIDDEN     = (8, 8)
DROPOUT_P  = 0.05
TEST_SIZE  = 0.25
DEVICE     = "cuda" if torch.cuda.is_available() else "cpu"
RNG        = 42
np.random.seed(RNG)
torch.manual_seed(RNG)

# ------------------------------------------------------------------- #
# 2 . Load data → window → feature matrix                              #
# ------------------------------------------------------------------- #
df_raw, _ = load_data(DATA_FILE)
df_long   = _to_long(df_raw)

features, labels, group_ids = [], [], []

def windows_from_sessions(sess_arrays):
    """Return list of flattened [s0|s1|s2] windows (already sub-sampled)."""
    # 0) sub-sample every SUBSTEP
    sess_arrays = [sa[::SUBSTEP] for sa in sess_arrays]

    min_len = min(map(len, sess_arrays))
    if min_len < WINDOW_LEN:
        return []

    # decide start indices
    if BURN_IN_ONLY:
        start0 = BURN_IN_STEPS
        if min_len < start0 + WINDOW_LEN:                 # too short
            return []
        idx_starts = range(start0,
                           min_len - WINDOW_LEN + 1,
                           SHIFT)
    else:  # full sliding mode
        idx_starts = range(0, min_len - WINDOW_LEN + 1, SHIFT)

    # build windows
    wins = []
    for i0 in idx_starts:
       block = np.vstack([sa[i0:i0 + WINDOW_LEN] for sa in sess_arrays])
       feat  = block.T.reshape(-1)[:WINDOW_LEN]             # (300,) → (100,)
       wins.append(feat)
    return wins


for (arena, run, it), g_iter in df_long.groupby(
        ["arena_file", "run", "current_it"]):

    sess_arrays = []
    for sid in (0, 1, 2):
        arr = (g_iter[g_iter["session"] == sid]
               .sort_values("time")["s"]
               .to_numpy(dtype=float))
        if arr.size == 0:
            break
        sess_arrays.append(arr)
    else:  # executed only if no break ⇒ we have 3 sessions
        for win in windows_from_sessions(sess_arrays):
            features.append(win)
            labels.append(arena)
            group_ids.append((arena, run, it))

X = np.vstack(features).astype(np.float32)          # (N, 100)
y = np.array(labels)
group_ids = np.array(group_ids, dtype=object)

print(f"dataset: {X.shape[0]} samples × {X.shape[1]} features")  # must be 100

# ------------------------------------------------------------------- #
# 3 . Train / test split                                              #
# ------------------------------------------------------------------- #
X_tr, X_te, y_tr, y_te, gid_tr, gid_te = train_test_split(
    X, y, group_ids, test_size=TEST_SIZE,
    stratify=y, random_state=RNG)

class ArenaDS(Dataset):
    def __init__(self, X, y, lbl2idx):
        self.X = torch.from_numpy(X)
        self.y = torch.tensor([lbl2idx[l] for l in y], dtype=torch.long)
    def __len__(self): return len(self.X)
    def __getitem__(self, i): return self.X[i], self.y[i]

lbl2idx = {lbl: i for i, lbl in enumerate(sorted(np.unique(y)))}
idx2lbl = {v: k for k, v in lbl2idx.items()}

train_dl = DataLoader(ArenaDS(X_tr, y_tr, lbl2idx),
                      batch_size=BATCH_SIZE, shuffle=True)
test_dl  = DataLoader(ArenaDS(X_te, y_te, lbl2idx),
                      batch_size=BATCH_SIZE)

# ------------------------------------------------------------------- #
# 4 . Tiny MLP                                                        #
# ------------------------------------------------------------------- #
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
opt    = torch.optim.Adam(model.parameters(), lr=LR)
crit   = nn.CrossEntropyLoss()

# ------------------------------------------------------------------- #
# 5 . Training                                                        #
# ------------------------------------------------------------------- #
loss_curve = []
for epoch in range(1, EPOCHS+1):
    model.train(); run = 0.0
    for xb, yb in train_dl:
        xb, yb = xb.to(DEVICE), yb.to(DEVICE)
        opt.zero_grad()
        loss = crit(model(xb), yb)
        loss.backward(); opt.step()
        run += loss.item() * xb.size(0)
    loss_curve.append(run / len(train_dl.dataset))
    if epoch % 5 == 0 or epoch == 1:
        print(f"Epoch {epoch:3d}/{EPOCHS}  |  loss {loss_curve[-1]:.4f}")

# ------------------------------------------------------------------- #
# 6 . Evaluation                                                      #
# ------------------------------------------------------------------- #
model.eval(); y_true, y_pred, gid_seq = [], [], []
with torch.no_grad():
    start = 0
    for xb, yb in test_dl:
        bs = xb.size(0)
        logits = model(xb.to(DEVICE))
        y_pred.extend(logits.argmax(1).cpu().numpy())
        y_true.extend(yb.numpy())
        gid_seq.extend(gid_te[start:start+bs]); start += bs

y_true = np.array(y_true);  y_pred = np.array(y_pred)
print("\nWindow-level accuracy:", np.round(accuracy_score(y_true, y_pred), 3))
print(classification_report(
        y_true, y_pred,
        target_names=[idx2lbl[i] for i in sorted(idx2lbl)], digits=3))

# Majority vote per swarm
votes = defaultdict(list)
for gid, p in zip(gid_seq, y_pred):
    votes[tuple(gid)].append(p)
swarm_pred = [Counter(v).most_common(1)[0][0] for v in votes.values()]
swarm_true = [lbl2idx[gid[0]] for gid in votes]

print("\n=====  Swarm-level vote  =====")
print(f"ACCURACY (swarm): {accuracy_score(swarm_true, swarm_pred):.3f}\n")

print(classification_report(
    swarm_true,
    swarm_pred,
    target_names=[idx2lbl[i] for i in sorted(idx2lbl)],
    digits=3))

sw_prec, sw_rec, sw_f1, _ = precision_recall_fscore_support(
    swarm_true, swarm_pred,
    labels=list(idx2lbl.keys()),
    zero_division=0)

# ------------------------------------------------------------------- #
# 7 . Quick plots                                                     #
# ------------------------------------------------------------------- #
# ------------------------------------------------------------------- #
# 7 . Plots – learning curve + window metrics + swarm metrics         #
# ------------------------------------------------------------------- #
import matplotlib.pyplot as plt
import numpy as np
from sklearn.metrics import precision_recall_fscore_support

# ── compute the metrics we want to display ──────────────────────────
# window-level
prec, rec, f1, _ = precision_recall_fscore_support(
        y_true, y_pred,
        labels=list(idx2lbl.keys()),
        zero_division=0)

# swarm-vote level
sw_prec, sw_rec, sw_f1, _ = precision_recall_fscore_support(
        swarm_true, swarm_pred,
        labels=list(idx2lbl.keys()),
        zero_division=0)

# ── build the 3-panel figure ────────────────────────────────────────
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 4), dpi=110)

# (a) learning curve -------------------------------------------------
ax1.plot(loss_curve, lw=2)
ax1.set_xlabel("epoch")
ax1.set_ylabel("loss")
ax1.set_title("Training loss")

# shared helpers for (b) & (c)
x = np.arange(len(idx2lbl))   # one bar per class
w = 0.25                        # bar width

# (b) window-level bars ---------------------------------------------
ax2.bar(x - w, prec, width=w, label="prec")
ax2.bar(x     , rec , width=w, label="recall")
ax2.bar(x + w , f1  , width=w, label="F1")
ax2.set_xticks(x)
ax2.set_xticklabels([idx2lbl[i] for i in x], rotation=30)
ax2.set_ylim(0, 1)
ax2.set_title("Window metrics")
ax2.legend(frameon=False)

# (c) swarm-vote bars -----------------------------------------------
ax3.bar(x - w, sw_prec, width=w, label="prec")
ax3.bar(x     , sw_rec , width=w, label="recall")
ax3.bar(x + w , sw_f1  , width=w, label="F1")
ax3.set_xticks(x)
ax3.set_xticklabels([idx2lbl[i] for i in x], rotation=30)
ax3.set_ylim(0, 1)
ax3.set_title("Swarm-vote metrics")
ax3.legend(frameon=False)

plt.tight_layout()
plt.show()


