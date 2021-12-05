# Complex Social Systems: Modeling Agents, Learning and Games: 

> * Group Name: *The walker's group*
> * Group participants names: 
>   * Battista, Piera;
>   * Biella, Alessandro;
>   * Ferradini, Carla Anais;
>   * Fioroni, Lorenzo;
>   * Guillemet, Vincent.
> * Project Title: *Modeling pedestrian flow in a crowded building*

## General Introduction



(States your motivation clearly: why is it important / interesting to solve this problem?)
(Add real-world examples, if any)
(Put the problem into a historical context, from what does it originate? Are there already some proposed solutions?)

## The Model

We implement numerically the agend-based model developped in the paper -Social force model for pedestrian dynam-
ics- by Helbing Dirk and Molnar Péter. The essential is that each agent (1rst class) evolves in an environment (2nd class), 
provided with rules that are defined by forces acting on each agent. The forces are will force, repulsion from environment structures, 
repulsion from other agents. These forces are integrated to get the speed of the agent, which integrated provides its position against time.
The simulation (3rd class) make several agents evolve in the same environment according to the previous scheme. We chose the hyperparameters 
according the advices provided at the end of the paper -Social force model for pedestrian dynam-
ics-.

We thus did not take into account attraction phenomenons, panic phenomenons, group phenomenons.


## Fundamental Questions
Provided that the theory is done (in the paper we worked on). Provided that the numerical implementation of the theory is done (what we did, 
the code is in the folder code). We asked the following questions.
>*-Is the model reality-like ? Does it shape well the behavior that people (agent) would have ?;
>     *-What happens when two or four opposite flows of agent encounter (ETH-GUESS/other/no_obstacles) ?;
>*-In a specific setting, is it possible to use this model to improve the structure / shape of a room ?
>     *-With two or four opposite flows of agent; Is it possible to shape the room in order to improve to mean time to arrival
>     (ETH-GUESS/other/pillar_infrontof_exit and ETH-GUESS/other/wedge) ? 
>     *-What is a best disposition of tables in a classroom (ETH-GUESS/other/classroom) ?
>*-What is the effect of having obstacles in a room (ETH-GUESS/other/grid and ETH-GUESS/other/grid_shifted and ETH-GUESS/other/random_obstacles)
  

## Expected Results

(What are the answers to the above questions that you expect to find before starting your research?)


## References 

(Add the bibliographic references you intend to use)
(Explain possible extension to the above models)
(Code / Projects Reports of the previous year)


## Research Methods

Agend-based model for pedestrian dynamics.
