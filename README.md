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

Modeling the behaviour of pedestrians in a room with different environment configurations is crucial for the safe evacuation and movement inside of it. Building safety is one of the fundamental requirements when creating the blue print of an architectural project. This concept lead us to investigate the best configuration for the evacuation of a classroom with 12 desks and 24 agents, simulating a real-life high school. A large number of models have been used to investigate the movements of pedestrians and between them we chose to test the microscopic Social Force Model, to better simulate a small number of students evacuating the room.  

## The Model

We implement numerically the agend-based model developped in the paper -Social force model for pedestrian dynamics- by Helbing Dirk and Molnar Péter. Essentially, each agent (1rst class) moves towards his/her goal in an environment (2nd class). The agent's movements are influenced by the environment and the presence of other agents, therefore he/she will experience a force resulting from his/her perceptions. In particular, he/she will feel a repulsion from the walls and obstacles in the environment as well as from other agents.
Once computed, these forces are integrated to get the speed of the agent, which integrated again gives his/her position as a function of time.
The simulation (3rd class) makes several agents evolve in the same environment according to the previous scheme. We chose the hyperparameters 
according to the advices provided at the end of the paper -Social force model for pedestrian dynamics-. For more information we suggest to read the report, available in this repository.


## Fundamental Questions
Given the theoretical model, as described in the paper -Social force model for pedestrian dynamics-, and once the numerical simulation was implemented (in the code situated in the folder code). Our goal was to answer the following questions:
 
> * Does the model describe correctly reality? Does it simulate realistically agents' behaviours?
>   * What happens when two or four opposite flows of agent encounter (ETH-GESS/other/no_obstacles)?
> * In a specific setting, is it possible to use this model in order to find the optimal configuration of objects in a room?
>   * With two or four opposite flows of agent, is it possible to move these objects in order to improve to mean time to goal
>     (ETH-GESS/other/pillar_infrontof_exit and ETH-GESS/other/wedge)? 
> * What is the effect of having obstacles in a room (ETH-GUESS/other/grid and ETH-GUESS/other/grid_shifted and ETH-GUESS/other/random_obstacles) ?
>   * What is the optimal configuation of tables in a classroom in an evacuation scenario (ETH-GUESS/other/classroom)?



## Expected Results
Assuming that the choice of hyper-parameters is good, we expect the model to describe well a normal scenarios (no panic, no group behaviours, no attractive devices). The model has already been tested in the paper -Social force model for pedestrian dynamics- and the authors referenced it as a good representation of reality for their case study. 

We expect that the model can indeed be used in specific settings, in order to improve the structure of a path / room, as it has been discussed in the lecture number 3 of the course -Complex Social Systems: Modeling Agents, Learning, and Games HS2021-. In particular we expect that adding pillars in front of exits will help "regularise" the flow of agent and thus reduce the mean optimal time to goal, the measure of performance we used. Nevertheless, we also expect our model to be very sensitive to the choice of hyperparameters and that some agents will act unaturally (eg. if there are two exits, one closest but cloged and the other far away but free).


## References 

The primary reference used in this project, from which the model is taken, is the following:
Helbing, Dirk; Molnar, Peter: Social force model for pedestrian dynamics. In:Physical Review E51 (1995), May, Nr. 5, S. 4282–4286. – ISSN 1095-3787.

A full list of all the references used can be found in the report.

## Research Methods

Agend-based model for pedestrian dynamics.
