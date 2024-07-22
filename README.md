# JEDAI Explains Decision-Making AI

This repository contains the code for the tool JEDAI. In-detail review of its architecture and features is given in paper at ArXiv, appearing as - 

Using Explainable AI and Hierarchical Planning for Outreach with Robots. <br/>
[Daksh Dobhal](https://github.com/DakshASU)\*,
[Jayesh Nagpal](https://github.com/jayesh59)\*,
[Rushang Karia](https://rushangkaria.github.io/),
[Pulkit Verma](https://pulkitverma.net),
[Rashmeet Kaur Nayyar](https://www.rashmeetnayyar.com/),
[Naman Shah](https://www.namanshah.net/), and
[Siddharth Srivastava](http://siddharthsrivastava.net/). <br/>
ArXiv 2404.00808, 2024 <br/>
[Paper](https://arxiv.org/pdf/2404.00808.pdf)

System Demonstration of JEDAI was published at AAMAS 2022 as:

JEDAI: A System for Skill-Aligned Explainable Robot Planning.<br/>
[Naman Shah](https://www.namanshah.net/)\*, 
[Pulkit Verma](https://pulkitverma.net)\*, 
[Trevor Angle](http://trevorangle.com), and 
[Siddharth Srivastava](http://siddharthsrivastava.net/). <br/>
21st International Conference on Autonomous Agents and MultiAgent Systems, 2022. <br/>

[Paper](https://aair-lab.github.io/Publications/svas_aamas22.pdf) | [Demo Talk](https://www.youtube.com/watch?v=MQdoikcnhbY) | [Tutorial](https://www.youtube.com/watch?v=57os8Ap1N5U)
<br />

##  Docker Image

<!-- The recommended way of using JEDAI is to use pre-configured Virtual Machine image that is available here: [https://bit.ly/2WccU4K](https://bit.ly/2WccU4K) -->
The recommended way of using JEDAI is to use the pre-configured docker image that is available here: 

Once all the files are downloaded, run the following command to load docker images - 

`docker load --input jedai_ed.tar` <br/>
`docker load --input novnc.tar`

## Running JEDAI

Now, to start a JEDAI instance, run the following script from the directory where all the files have been downloaded - 

`./start_instances.sh -n 1`

You can access the JEDAI interface over web by going to the link given after running the script.

To stop the JEDAI instance, run following - 

`./stop_instances.sh -n 1`

## Contributors
[Daksh Dobhal](https://github.com/DakshASU)<br/>
[Jayesh Nagpal](https://github.com/jayesh59)<br/>
[Rushang Karia](https://rushangkaria.github.io/)<br/>
[Pulkit Verma](https://pulkitverma.net)<br/>
[Rashmeet Kaur Nayyar](https://www.rashmeetnayyar.com/)<br/>
[Naman Shah](https://www.namanshah.net/)<br/>
[Trevor Angle](http://trevorangle.com) <br/>
[Kiran Prasad](https://github.com/kiranprasad)<br/>
[Amruta Tapadiya](https://github.com/amy88amy)<br/>
[Kyle Atkinson](https://github.com/KyleTheEpic)<br/>
[Chirav Dave](https://chiravdave.github.io/)<br/>
[Judith Rosenke](https://www.linkedin.com/in/judith-rosenke/)<br/>
[Siddharth Srivastava](https://siddharthsrivastava.net/)

# Citation
The latest version of this work can be cited as :

```
@article{dobhal2024using,
  title={Using Explainable AI and Hierarchical Planning for Outreach with Robots},
  author={Dobhal, Daksh and Nagpal, Jayesh and Karia, Rushang and Verma, Pulkit and Nayyar, Rashmeet Kaur and Shah, Naman and Srivastava, Siddharth},
  journal={arXiv preprint arXiv:2404.00808},
  year={2024}
}
```
The earlier version (AAMAS 2022) can be cited as :

```
@inproceedings{shah_2022_jedai,
    author = {Shah, Naman and Verma, Pulkit and Angle, Trevor and Srivastava, Siddharth},
    title = {{JEDAI: A System for Skill-Aligned Explainable Robot Planning}},
    booktitle = {Proceedings of the 21st International Conference on Autonomous Agents and MultiAgent Systems},
    year={2022}
}
```


