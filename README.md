# Decentralized Multi-agent Search and Rescue under MITL Specifications and Communication Constraints
This repository provides the implementation and case studies for the paper "Decentralized Multi-agent Search and Rescue under MITL Specifications and Communication Constraints".

## Installation

The implementation is performed on *Python3.9*, *Uppaal64-4.1.26*, make sure you have at least *Java 7* configured on your system

### [UPPAAL](https://www.it.uu.se/research/group/darts/uppaal/index.shtml)
We use [UPPAAL](https://www.it.uu.se/research/group/darts/uppaal/index.shtml) to translate MITL specification into timed automata and synthesize the timed run that satisfies the specification over given WTS. Check the website [here](https://www.it.uu.se/research/group/darts/uppaal/download.shtml) for installation instruction. 


#### [JAVA API](https://docs.uppaal.org/toolsandapi/javaapi/)
UPPAAL models can be created, loaded, saved, simulated, and checked using libs/model.jar library. 
We use the JAVA API of UPPAAL in order to repeatedly call UPPAAL for model timed runs synthesis and instance solving automatically. 

- File **MITL_verification.java** is the API code file, which is supposed to be generated/relocated at the following path after installing UPPAAL.

```ruby
[YOURPATH]/uppaal64-4.1.26/demo/ModelDemo/src[/MITL_verification.java]
```

The following variable in **MITL_verification.java** needs to rewrite as the absolute path where you download the repository

```ruby
public static void main(String[] args) throws Exception {

      String abspath = "[YOURPATH]/MAS_search_rescue/MAS_search_rescue";
```

- Bash file **deploy.sh** under the following direction is the used to call **MITL_verification.java** and envoke UPPAAL for instance solving.
```ruby
~/env
```

### Other Dependencies
Download [json-simple-1.1.1.jar](https://jar-download.com/artifacts/com.googlecode.json-simple/json-simple/1.1.1/source-code) and relocate the .jar file to the lib of UPPAAL. It is used to generate files that can be recognized and read by UPPAAL for timed run synthesis.
```ruby
[YOURPATH]/uppaal64-4.1.26/lib
```


## Running Code
You do not have to start UPPAAL while running the code. 
Once all the abovementioned steps are complete, run the main code under 

```ruby
MAS_search_rescue/env/main.py
```

as

```ruby
python3 main.py
```

All the results will be saved with date under **MAS_search_rescue/figures** for all figures and **MAS_search_rescue/files** for all .txt, .json files.


## Case studies
We provide several case studies through [Jupyter Notebook](https://jupyter.org/) under the following dictionary, please check the **README.md** file provided there.

```ruby
MAS_search_rescue/env
```


