# Mechanical Characters
Implementation Of Disney's Paper

This project is a python implementation of the main computations and logic of the paper ***"Computational Design Of Mechanical Characters"*** by Disney Research.

This project has 2 more projects associated with it that presents a UI for the process of designing a mechanical character.

# Usage
### To generate a DB of curves run:

```
python3 generate_db n < desired database size > --file_path < sampler destination path > --load_db < path to current database > --debug_mode < enable debug mode > 
```

For example
```
python3 generate_db 200 --file_path "C:\Users\a\Desktop\sampler_new" --load_db "C:\Users\a\Desktop\sampler_old"  --debug_mode True 
```
The arguments are:  
n - number of samples that will add to the database.  
file_path - the sampler will save in this path.  
load_db - if you already have database, you can continue sample to it, giving it's path here.  
debug_mode - change it to True will print progress notifications while the file is running.

### To plot an assembly and its tracing curve run:
```
[None]
```
# Dependancies
Some of the dependencies are installable via pip, and some via conda..

### We recommend:
1) Opening a new conda enviroment for the project (python 3.7).
2) pip install the requirements.
3) conda install for the packages that had issues with pip.

# Related Projects

For [Learning The Wights Of A](https://github.com/ofirbartal100/Mechanical_Characters_Learning_A)

For [Mechanical Characters UI](https://github.com/ofirbartal100/MechanicalCharactersUI)
