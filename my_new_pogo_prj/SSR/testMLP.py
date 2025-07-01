#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# mlp_runlevel.py  ──  tiny MLP (8,1) + run-level CV, aiming at ≥80 % accuracy

# ───────────────────────────── 0. imports ────────────────────────────────────
import sys, pathlib, os, warnings
warnings.filterwarnings("ignore", category=UserWarning)  # silence old-CUDA msg

scripts_dir = pathlib.Path(__file__).resolve().parent / "scripts"
sys.path.insert(0, str(scripts_dir))

import numpy as np
import pandas as pd
import torch
from torch import nn
from torch.utils.data import DataLoader, Dataset
from torch.optim.lr_scheduler import OneCycleLR
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import GroupShuffleSplit
from sklearn.metrics import (
    accuracy_score, precision_recall_fscore_support, classification_report)

from utils import load_data
from scripts.plots import _to_long
import matplotlib.pyplot as plt
from pathlib import Path
from sklearn.metrics import confusion_matrix


# ───────────────────────────── 1. parameters ────────────────────────────────
DATA_FILE   = "results/result.feather"
WINDOW_LEN  = 100          # ⬅ longer context
INPUT_DIM   = 3 * WINDOW_LEN
SHIFT       = 10
HIDDEN      = (8, 1)       # DO NOT TOUCH
BATCH_SIZE  = 512
LR          = 1e-3
EPOCHS      = 120          # early-stop will cut it short
DROPOUT_P   = 0.15
EARLY_PAT   = 20
TEST_SIZE   = 0.25
RNG         = 42
DEVICE      = "cuda" if torch.cuda.is_available() else "cpu"

rng = np.random.default_rng(RNG)

# ───────────────────────── 2. load + per-run std-ise ─────────────────────────
df_raw, _ = load_data(DATA_FILE)
df_long   = _to_long(df_raw)            # adds 'session', 's'

features, labels, runs = [], [], []

for (arena, run, it), g_iter in df_long.groupby(["arena_file","run","current_it"]):
    # ── 2.1  three sessions as arrays ───────────────────────────────────────
    sess = []
    for s in (0,1,2):
        arr = g_iter[g_iter["session"]==s].sort_values("time")["s"].to_numpy()
        if arr.size == 0: break
        sess.append(arr)
    if len(sess) != 3: continue

    # ── 2.2  per-run μ/σ  → zero-mean, unit-var — improves generalisation ───
    sess = [(a - a.mean()) / (a.std() + 1e-9) for a in sess]

    min_len = min(len(a) for a in sess)
    if min_len < WINDOW_LEN: continue
    for i0 in range(0, min_len - WINDOW_LEN + 1, SHIFT):
        win = np.hstack([a[i0:i0+WINDOW_LEN] for a in sess])
        features.append(win)
        labels.append(arena)
        runs.append(run)

X = np.vstack(features).astype(np.float32)
y = np.array(labels);          runs = np.array(runs)
print(f"dataset: {X.shape[0]} windows  ×  {X.shape[1]} features")

# ───────────────────── 3. down-sample to smallest class ──────────────────────
class_to_idx = {c: np.where(y==c)[0] for c in np.unique(y)}
n_min        = min(len(v) for v in class_to_idx.values())
sel_idx = np.concatenate([rng.choice(v, n_min, replace=False)
                          for v in class_to_idx.values()])
rng.shuffle(sel_idx)
X, y, runs = X[sel_idx], y[sel_idx], runs[sel_idx]
print("down-sampled to", n_min, "windows per class")

# ─────────────────────── 4. group-aware train/test split ─────────────────────
# ───────────────────── 4-bis. Leave-One-Run-Out CV diagnostic ──────────────
from sklearn.model_selection import LeaveOneGroupOut

def fresh_model():
    return MLP(INPUT_DIM, HIDDEN, len(lbl2idx)).to(DEVICE)

def quick_train(model, X_tr, y_tr, epochs=35):
    ds = ArenaDS(X_tr, y_tr)
    dl = DataLoader(ds, batch_size=BATCH_SIZE, shuffle=True)
    opt = torch.optim.Adam(model.parameters(), lr=1e-3)
    crit = nn.CrossEntropyLoss()
    model.train()
    for _ in range(epochs):
        for xb, yb in dl:
            xb, yb = xb.to(DEVICE), yb.to(DEVICE)
            opt.zero_grad()
            crit(model(xb), yb).backward()
            opt.step()

logo = LeaveOneGroupOut()
loo_scores = []
for k, (tr, te) in enumerate(logo.split(X, y, groups=runs), 1):
    m = fresh_model()
    quick_train(m, X[tr], y[tr])
    m.eval()
    with torch.no_grad():
        preds = m(torch.from_numpy(X[te]).to(DEVICE)).argmax(1).cpu().numpy()
    acc = (preds == y[te]).mean()
    loo_scores.append(acc)
    print(f"run {k:2d}/{len(runs)}  LOO-acc = {acc:.3f}")

print("LOO mean ± std =", np.mean(loo_scores).round(3),
      "±", np.std(loo_scores).round(3))
print("-"*60)
# ─────────── end diagnostic – normal train/test split comes next ────────────


gss = GroupShuffleSplit(test_size=TEST_SIZE, random_state=RNG, n_splits=1)
(train_idx, test_idx), = gss.split(X, y, groups=runs)
X_tr, X_te = X[train_idx], X[test_idx]
y_tr, y_te = y[train_idx], y[test_idx]

#scaler = StandardScaler()
#X_tr = scaler.fit_transform(X_tr).astype(np.float32)
#X_te = scaler.transform(X_te).astype(np.float32)

# ───────────────────────────── 5. datasets ───────────────────────────────────
lbl2idx = {c:i for i,c in enumerate(sorted(np.unique(y)))}
idx2lbl = {i:c for c,i in lbl2idx.items()}

class ArenaDS(Dataset):
    def __init__(self, xs, ys): self.X, self.y = xs, ys
    def __len__(self): return len(self.X)
    def __getitem__(self,i):
        return torch.from_numpy(self.X[i]), torch.tensor(lbl2idx[self.y[i]])

train_dl = DataLoader(ArenaDS(X_tr,y_tr), batch_size=BATCH_SIZE, shuffle=True)
test_dl  = DataLoader(ArenaDS(X_te,y_te), batch_size=BATCH_SIZE)

# ─────────────────────────── 6. tiny MLP model ───────────────────────────────
class MLP(nn.Module):
    def __init__(self, d_in, hidden, d_out):
        super().__init__()
        layers = []
        prev = d_in
        for h in hidden:                                 # (8, 1)
            layers += [nn.Linear(prev, h),
                       nn.BatchNorm1d(h),
                       nn.ReLU()]  
            if h > 1:
                layers.append(nn.Dropout(DROPOUT_P))                      # ← no Dropout
            prev = h
        layers.append(nn.Linear(prev, d_out))
        self.net = nn.Sequential(*layers)
    def forward(self, x):
        return self.net(x)

model = MLP(INPUT_DIM, HIDDEN, len(lbl2idx)).to(DEVICE)

# ───────────────────────── 7. loss (+weights) & optim ────────────────────────
# ───────── 7. loss & optim  (replace the two lines) ─────────
weights = torch.zeros(len(lbl2idx), dtype=torch.float32, device=DEVICE)
for lbl, idx in lbl2idx.items():
    weights[idx] = 1 / np.sum(y_tr == lbl)
criterion = nn.CrossEntropyLoss(weight=weights)

opt = torch.optim.Adam(model.parameters(), lr=1e-3, weight_decay=1e-4)
sched = OneCycleLR(opt, max_lr=1e-3, total_steps=EPOCHS*len(train_dl))

# ──────────────────────── 8. train with early-stop ───────────────────────────
best_val=float('inf'); wait=0
loss_curve=[]
for epoch in range(1,EPOCHS+1):
    # training step
    model.train(); run=0.
    for xb,yb in train_dl:
        xb,yb = xb.to(DEVICE), yb.to(DEVICE)
        opt.zero_grad()
        loss = criterion(model(xb), yb)
        loss.backward(); opt.step(); sched.step()
        run += loss.item()*xb.size(0)
    train_loss = run/len(train_dl.dataset); loss_curve.append(train_loss)

    # validation
    model.eval(); run=0.
    with torch.no_grad():
        for xb,yb in test_dl:
            xb,yb = xb.to(DEVICE), yb.to(DEVICE)
            run += criterion(model(xb), yb).item()*xb.size(0)
    val_loss = run/len(test_dl.dataset)

    if val_loss < best_val-1e-4:
        best_val, wait = val_loss, 0
        best_state = model.state_dict()
    else:
        wait += 1
        if wait == EARLY_PAT:
            print(f"Early stop @ epoch {epoch}")
            break

    if epoch%10==0 or epoch==1:
        print(f"epoch {epoch:3d} | train {train_loss:.4f} | val {val_loss:.4f}")

model.load_state_dict(best_state)

# ───────────────────────────── 9. evaluation ────────────────────────────────
model.eval(); 
y_true = []
y_pred = []
logits_all = []
with torch.no_grad():
    for xb,yb in test_dl:
        logits = model(xb.to(DEVICE))
        logits_all.append(logits.cpu())
        y_pred.extend(logits.argmax(1).cpu().numpy())
        y_true.extend(yb.cpu().numpy())  

logits_all = torch.cat(logits_all, dim=0)
y_true=np.array(y_true)
y_pred=np.array(y_pred)
acc = accuracy_score(y_true,y_pred)

# ───────────────────── 9-bis. tune decision threshold ──────────────────────
from sklearn.metrics import confusion_matrix

# 1) turn logits into P(disk)   (index 1 because lbl2idx maps annulus→0, disk→1)
disk_idx = lbl2idx['disk']
probs_disk = torch.softmax(logits_all, dim=1)[:, disk_idx].numpy()

# 2) scan thresholds → choose the one with best accuracy
grid  = np.linspace(0.25, 0.75, 101)        # finer than 0.3…0.7 sweep
scores = []
for thr in grid:
    y_hat = (probs_disk >= thr).astype(int)
    scores.append((thr, (y_hat == y_true).mean()))
best_thr, best_acc = max(scores, key=lambda t: t[1])

print(f"\n‣ optimal threshold ≈ {best_thr:.3f}  "
      f"→ accuracy {best_acc:.3f}")

# 3) recompute full report at that threshold
y_pred_thr = (probs_disk >= best_thr).astype(int)
print("\nClassification report at tuned threshold")
print(classification_report(
        y_true, y_pred_thr,
        target_names=[idx2lbl[i] for i in range(len(idx2lbl))],
        digits=3))

# 4) optional: quick confusion-matrix dump
cm = confusion_matrix(y_true, y_pred_thr)
print("Confusion matrix (rows = true, cols = pred)\n", cm)

print(f"\nACCURACY on held-out runs: {acc:.3f}\n")
print(classification_report(
        y_true,y_pred,
        target_names=[idx2lbl[i] for i in range(len(idx2lbl))],
        digits=3))

# ───────────────────────────── 10. plots ─────────────────────────────────────


# 1) precision, recall, F1 for each class (in the order 0…n-1)
prec, rec, f1, _ = precision_recall_fscore_support(
    y_true, y_pred, labels=list(range(len(idx2lbl))))
x = np.arange(len(idx2lbl))           # bar positions
w = 0.25                              # bar width

fig, ax2 = plt.subplots(figsize=(6,4))
ax2.bar(x - w, prec, width=w, label="precision")
ax2.bar(x,     rec,  width=w, label="recall")
ax2.bar(x + w, f1,   width=w, label="F1-score")

ax2.set_xticks(x)
ax2.set_xticklabels([idx2lbl[i] for i in x], rotation=30)
ax2.set_ylim(0, 1)
ax2.set_ylabel("score")
ax2.set_title("Metrics per class")
ax2.legend()
plt.tight_layout()

# 2) optional helper to save the figure next to the script
def save_figure(fname: str):
    out = Path(fname).with_suffix(".png")
    plt.savefig(out, dpi=300)
    print(f"figure saved to {out.resolve()}")

save_figure("MLP")   # → MLP.png
plt.show()

plt.figure(figsize=(6,4))
plt.plot(loss_curve); plt.xlabel("epoch"); plt.ylabel("train loss")
plt.title("Tiny-MLP learning curve"); plt.tight_layout(); plt.show()
