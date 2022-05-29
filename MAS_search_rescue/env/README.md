# Case studies

We provide 4 case studies to show four different scenarios that you may meet. All case studies are offered with Jupyter Notebook.


## Case_study_1

Case Study 1 is the example we provided in the paper when there exists timed runs for all independent tasks and all the extracted independent requests can be accepted. 

Timed run synthesis by UPPAAL is implemented in cells
```ruby
"""--------------- Mission Decomposition ---------------"""
...
```
and 
```ruby
""" Start two round SWEEP """
```
In the end, we plot the trajectories for agile robot R0 and heavy-duty robots R1-R4.


## Case_study_2(Random)
In this case study, the whole environment and agents are randomly initialized.
Here we provide an example when there does not exist a timed run that satisfies the independent task after pre-computation (request extraction). Therefore, the execution stops.


## Case_study_3(Random)
In this case study, the whole environment and agents are randomly initialized.
Here we provide an example when there exists timed runs for all independent tasks but not all the extracted independent requests are accepted. Therefore, the heavy-duty robot that receives requests but cannot generates a timed run satisfying the requests follows the timed run based on its independent tasks.

## Case_study_4(Random)
In this case study, the whole environment and agents are randomly initialized.
Here we provide another example when there exists timed runs for all independent tasks and all the extracted independent requests can be accepted. 


# Video
A video shows the whole procedure can be generated if uncomment the following lines in *main.py*.
```ruby
animate_solution(map_obs_work, record_path_all, record_working_region, 
                record_task_left, r_0, r_i, v_light, fig_folder, video=True)

```

