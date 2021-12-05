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

Modeling the behaviour of pedestrians in a room with different environment configurations is crucial for the safe evacuation and movement inside of it. Building safety is one of the fundamental requirements when creating the blue print of an architectural project. This concept lead us to wanting to investigate the best configuration for a classroom with 12 desks and 24 agents, simulating a real-life high school. A large number of models have been used to investigate the movements of pedestrians and between them we chose to test the microscopic Social Force Model, to better simulate a small number of students evacuating the room.  

## The Model

We implement numerically the agend-based model developped in the paper -Social force model for pedestrian dynamics- by Helbing Dirk and Molnar Péter. The essential is that each agent (1rst class) evolves in an environment (2nd class), 
provided with rules that are defined by forces acting on each agent. The forces are will force, repulsion from environment structures, 
repulsion from other agents. These forces are integrated to get the speed of the agent, which integrated provides its position against time.
The simulation (3rd class) make several agents evolve in the same environment according to the previous scheme. We chose the hyperparameters 
according the advices provided at the end of the paper -Social force model for pedestrian dynamics-. For more information one can have a look to the report, available in this repositery.

We thus did not take into account attraction phenomenons, panic phenomenons, group phenomenons.


## Fundamental Questions
Provided that the theory is done (in the paper we worked on). Provided that the numerical implementation of the theory is done (what we did, 
the code is in the folder code). We asked the following questions.
 
> * Is the model reality-like ? Does it shape well the behavior that people (agent) would have ?
>   * What happens when two or four opposite flows of agent encounter (ETH-GUESS/other/no_obstacles) ?
> * In a specific setting, is it possible to use this model to improve the structure / shape of a room ?
>   * With two or four opposite flows of agent; Is it possible to shape the room in order to improve to mean time to arrival
>     (ETH-GUESS/other/pillar_infrontof_exit and ETH-GUESS/other/wedge) ? 
>   * What is a best disposition of tables in a classroom (ETH-GUESS/other/classroom) ?
> * What is the effect of having obstacles in a room (ETH-GUESS/other/grid and ETH-GUESS/other/grid_shifted and ETH-GUESS/other/random_obstacles) ?


## Expected Results
We expect that the model, provided with a good choice of hyper-parameters, is a good representation of a normal case in reality (no panic, no group behavior, no attractive devices). The model has already been tested in the paper -Social force model for pedestrian dynamics- and the authors referenced it as a good representation of reality for their case study. 

We expect that the model can indeed be used in specific settings, in order to improve the structure of a path / room, as it has been discussed in the lecture number 3 of the course -Complex Social Systems: Modeling Agents, Learning, and Games HS2021-. In particular we expect that adding pillars in front of exits will help "regularize" the flow of agent and thus reduce the mean optimal time to arrival, the measure of performance we used. Nevertheless we also expect our model to be very sensitive to the choice of hyperparameters and that some agents will act unaturally (eg. if there are two exits, one closest but cloged and the other far away but free).

Finally we expect that having obstacles in the room will make the mean optimal time to arrival increase. Moreover this augmentation should heavily depend on how the obstacles are displayed in the room.

## References 

The primary reference used in this project, from which the model is taken, is the following:
Helbing, Dirk; Molnar, Peter: Social force model for pedestrian dynamics. In:Physical Review E51 (1995), May, Nr. 5, S. 4282–4286. – ISSN 1095-3787.

A full list of all the references used can be found in the report.

## Research Methods

Agend-based model for pedestrian dynamics.
