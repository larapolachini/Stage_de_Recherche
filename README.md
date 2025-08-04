# Internship 

Welcome to the simulator for an automatic discovery of spectral operators for collective sensing in Swarm Robotics, 
in this simulator we use the run_and_tumble algorithm to move the robots across the arena while trying to discover the spectral operators.

With this is possible to analyse how the agents are distributed across the arenas and also optimize the parameters of the run_and_tumble 
to chose the behavior of the robots.

## Installation

First you'll need the simulators of pogosim and pogo-utils.

So, you can install the pogosim following the instructions in this link: (https://github.com/Adacoma/pogosim)

And pogo-utils in: (https://github.com/Adacoma/pogo-utils)

Now you just need to clone the repository:
```shell
git clone https://github.com/larapolachini/Stage_de_Recherche.git
```

## Quickstart

To launch a code with the run_and_tumble algorithm you can use the following commands:

Inside the example/run_and_tumble:
```shell
make clean sim
```


Then to launch a code in one arena and with 10 runs:
```shell
pogobatch -c conf/simple.yaml -S ./examples/run_and_tumble/run_and_tumble -r 10 -t tmp -o results
```

And to launch a code with more than one arena and with 10 runs you just need to change the conf file to simpleb.yaml:
```shell
pogobatch -c conf/simpleb.yaml -S ./examples/run_and_tumble/run_and_tumble -r 10 -t tmp -o results
```

## Run and tumble

### Analysing data
After running an example of the run_and_tumble you can analyse how the agents are distributed across the arena. 
You just need to launch:
```shell
python3 vor.py
```

Or, if you're running examples with multiples arenas:
```shell
python3 vorb.py
```

With this all the graphics will be saved in the folder "figures/" 

### Optimizing

To change the parameters of run_and_tumble, which are: run_duration_min, run_duration_max, tumble_duration_min, tumble_duration_max
You just need to launch the optimizer:
```shell
python3 optimizer_vor.py
```

Or if you want to optimize the parameters across different arenas:
```shell
python3 optimizer_vorb.py
```

## SSR

In the part of the Spectral Swarm robotics, we have the code that simulates how the robots discover the shape of the arena, 
and in this code we have the SSR code with the run_and_tumble algorithm

If you want to launch the simulator where the robots are imobile all the time or if they move at the beginning of each iteration 
you can just launch:

To build it:

```shell
cd SSR
make clean sim
cd ..
```

Then launch:
```shell
pogobatch -c conf/ssr.yaml -S ./SSR -r 10 -t tmp -o results
```


## SSR_move

But if you want to launch the simulator where the robots move all the time 
you can just launch:

To build it:

```shell
cd SSR_move
make clean sim
cd ..
```

Then launch:
```shell
pogobatch -c conf/ssr.yaml -S ./SSR_move -r 10 -t tmp -o results
```

Then to analyse the confusion matrix and time evolution of s:
```shell
python3 scripts/plots.py -i results/result.feather -o plots -a arenas
```

And finally, to predict the shape of the arenas:
```shell
python3 MLP.py
```