# Spectral Swarm Robotics Simulator

Simulator for the **automatic discovery of spectral operators** for collective sensing in swarm robotics.
Agents move with a **run-and-tumble** policy while exploring arenas; you can analyze spatial distributions
and **optimize motion parameters**.

---

## Prerequisites
- **pogosim** – simulator backend: <https://github.com/Adacoma/pogosim>  
- **pogo-utils** – CLI tools (e.g., `pogobatch`): <https://github.com/Adacoma/pogo-utils> 

---

## Installation

```shell
git clone https://github.com/larapolachini/Stage_de_Recherche.git
cd Stage_de_Recherche/my_new_pogo_prj
```

---

## Quickstart: `run_and_tumble`

Build and run the example:
```shell
cd examples/run_and_tumble
make clean sim
```

Run one arena for 10 repeats:
```shell
pogobatch -c conf/simple.yaml -S ./examples/run_and_tumble/run_and_tumble -r 10 -t tmp -o results
```

Run multiple arenas (change the config):
```shell
pogobatch -c conf/simpleb.yaml -S ./examples/run_and_tumble/run_and_tumble -r 10 -t tmp -o results
```

---

## Analyzing results
Spatial distribution (single arena):
```bash
python3 vor.py
```
Multiple arenas:
```bash
python3 vorb.py
```
Figures are saved to `figures/`.

---

## Optimizing run-and-tumble
Tune the parameters  
(`run_duration_min`, `run_duration_max`, `tumble_duration_min`, `tumble_duration_max`):

Single arena:
```bash
python3 optimizer_vor.py
```
Multiple arenas:
```bash
python3 optimizer_vorb.py
```

---

## Spectral Swarm Robotics (SSR)

In the part of the Spectral Swarm robotics, we have the code that simulates how the robots discover the shape of the arena, 
and in this code we have the SSR code with the run_and_tumble algorithm

### SSR

If you want to launch the simulator where the robots are imobile all the time or if they move at the beginning of each iteration:

Build:
```shell
cd SSR
make clean sim
cd ..
```

Run:
```shell
pogobatch -c conf/ssr.yaml -S ./SSR -r 10 -t tmp -o results
```

### SSR_move

But if you want to launch the simulator where the robots move all the time:

Build:
```shell
cd SSR_move
make clean sim
cd ..
```

Run:
```shell
pogobatch -c conf/ssr.yaml -S ./SSR_move -r 10 -t tmp -o results
```

## Plotting and learning
Confusion matrix and time evolution of `s`:
```bash
python3 scripts/plots.py -i results/result.feather -o plots -a arenas
```

Predict arena shapes with a simple MLP:
```bash
python3 MLP.py
```

---